"""Live hardware sensor plots, using a bounded minute of board timestamps."""
from collections import deque
import math
from PySide6.QtCore import Qt,QPointF,QRectF
from PySide6.QtGui import QColor,QPainter,QPainterPath,QPen
from PySide6.QtWidgets import QDialog,QVBoxLayout,QGridLayout,QWidget
from .widgets import label,card,row


def finite(value): return type(value) in (int,float) and math.isfinite(value)


class SensorHistory:
    window_ms=60000

    def __init__(self): self.clear()

    def clear(self):
        self.samples=deque(); self.elapsed_ms=0; self._last_uptime=None

    def append(self,data):
        stamp=data.get('uptime_ms')
        if type(stamp) is not int or not 0<=stamp<=0xFFFFFFFF: return False
        if self._last_uptime is not None:
            delta=(stamp-self._last_uptime)&0xFFFFFFFF
            if not delta: return False
            if delta>0x7FFFFFFF: self.clear() # Reboot, rather than uint32 rollover.
            else: self.elapsed_ms+=delta
        self._last_uptime=stamp
        values={}
        if data.get('baro_ready') and data.get('baro_sample_valid'):
            for key in ('pressure_pa','temperature_c'):
                if finite(data.get(key)): values[key]=(float(data[key]),)
        if data.get('imu_ready') and data.get('imu_sample_valid'):
            for key in ('accel_mps2','gyro_radps'):
                vector=data.get(key)
                if isinstance(vector,list) and len(vector)==3 and all(finite(v) for v in vector): values[key]=tuple(vector)
        self.samples.append((self.elapsed_ms,values))
        cutoff=self.elapsed_ms-self.window_ms
        while self.samples and self.samples[0][0]<cutoff: self.samples.popleft()
        return True

    @property
    def extent(self): return max(0,self.elapsed_ms-self.window_ms)/1000,self.elapsed_ms/1000

    def series(self,key,count):
        return [[(stamp/1000,values[key][axis] if key in values else None) for stamp,values in self.samples] for axis in range(count)]

    def temperature_average(self):
        if not self.samples or 'temperature_c' not in self.samples[-1][1]: return None
        values=[values['temperature_c'][0] for stamp,values in self.samples
                if stamp>=self.elapsed_ms-5000 and 'temperature_c' in values]
        return sum(values)/len(values)


class SensorPlot(QWidget):
    colors=('#1d8b7e','#427ac0','#cf8735')

    def __init__(self):
        super().__init__(); self.setMinimumSize(320,170)
        self.series=[]; self.extent=(0,0); self.empty='In attesa delle misure'

    def set_data(self,series,extent,empty):
        self.series=series; self.extent=extent; self.empty=empty; self.update()

    def paintEvent(self,event):
        painter=QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        box=QRectF(80,23,max(1,self.width()-100),max(1,self.height()-58))
        for i in range(5):
            y=box.bottom()-box.height()*i/4
            painter.setPen(QPen(QColor('#edf1f5'),1)); painter.drawLine(QPointF(box.left(),y),QPointF(box.right(),y))
        x0,x1=self.extent
        if x1<=x0: x1=x0+1
        values=[value for series in self.series for _,value in series if value is not None]
        low=min(values) if values else 0; high=max(values) if values else 1
        padding=max(high-low,abs(low)*.000001,.001)*.15
        low-=padding; high+=padding
        def pos(x,y): return QPointF(box.left()+(x-x0)/(x1-x0)*box.width(),box.bottom()-(y-low)/(high-low)*box.height())
        painter.setPen(QColor('#7b8998'))
        for i in range(5):
            x=x0+(x1-x0)*i/4
            painter.drawText(QRectF(pos(x,low).x()-25,box.bottom()+7,50,20),Qt.AlignCenter,f'{x:.1f}')
            if values:
                value=low+(high-low)*i/4
                text=f'{value:.4g}' if abs(value)>=1e6 else f'{value:.3f}'.rstrip('0').rstrip('.')
                painter.drawText(QRectF(0,pos(x0,value).y()-9,70,18),Qt.AlignRight,text)
        if not values:
            if self.empty=='Non disponibile': painter.setPen(QColor('#c43d3d'))
            painter.drawText(box,Qt.AlignCenter,self.empty); return
        for index,series in enumerate(self.series):
            painter.setPen(QPen(QColor(self.colors[index]),2))
            if len(self.series)>1: painter.drawText(QPointF(box.right()-100+index*38,14),'XYZ'[index])
            path=QPainterPath(); segment=False
            painter.save(); painter.setClipRect(box.adjusted(-2,-2,2,2))
            for stamp,value in series:
                if value is None: segment=False; continue
                point=pos(stamp,value)
                if segment: path.lineTo(point)
                else: path.moveTo(point); painter.drawEllipse(point,2,2)
                segment=True
            painter.drawPath(path); painter.restore()


class SensorDetails(QDialog):
    channels=(('accel_mps2','Accelerazione · m/s²',3,'imu_ready','IMU'),
              ('gyro_radps','Velocità angolare · rad/s',3,'imu_ready','IMU'),
              ('pressure_pa','Pressione · Pa',1,'baro_ready','Barometro'))

    def __init__(self,controller,parent=None):
        super().__init__(parent); self.controller=controller; self.history=SensorHistory(); self._connected=False
        self.setWindowTitle('Perpetual · Dettagli sensori'); self.resize(1040,650)
        self.setMinimumSize(760,620)
        layout=QVBoxLayout(self); layout.setContentsMargins(22,18,22,18); layout.setSpacing(12)
        heading=QVBoxLayout(); heading.addWidget(label('Sensori in tempo reale','title'))
        self.message=label('','hint'); self.message.setWordWrap(True); heading.addWidget(self.message); self.message.hide()
        temperature=QVBoxLayout(); caption=label('TEMPERATURA','eyebrow'); caption.setAlignment(Qt.AlignRight)
        self.temperature=label('—','metric'); self.temperature.setAlignment(Qt.AlignRight)
        self.temperature.setToolTip('Media mobile degli ultimi 5 secondi')
        temperature.addWidget(caption); temperature.addWidget(self.temperature)
        top=row(); top.addLayout(heading,1); top.addLayout(temperature); layout.addLayout(top)
        grid=QGridLayout(); grid.setSpacing(12); layout.addLayout(grid,1); self.plots={}
        for index,(key,title,count,*_) in enumerate(self.channels):
            panel,body=card(title); plot=SensorPlot(); self.plots[key]=plot; body.addWidget(plot,1)
            grid.addWidget(panel,index//2,index%2,1,2 if count==1 else 1)
        self.summary=label('Intervallo crescente fino a 60 s · poi scorre sull’ultimo minuto','hint'); layout.addWidget(self.summary)
        controller.sensors_received.connect(self.receive); controller.connection_changed.connect(self.connection_changed)

    def showEvent(self,event):
        super().showEvent(event); self.history.clear(); self._render({})
        self.connection_changed(self.controller.state.connection); self.controller.stream_sensors(True)

    def hideEvent(self,event):
        self.controller.stream_sensors(False); super().hideEvent(event)

    def connection_changed(self,snapshot):
        connected=snapshot.get('state') in ('connected','waiting')
        if connected and not self._connected: self.history.clear(); self._render({})
        self._connected=connected
        if not self.isVisible(): return
        self.message.setText('' if connected else 'Disconnesso · acquisizione interrotta'); self.message.setVisible(not connected)
        if not connected: self.temperature.setText('—')

    def receive(self,data):
        if not self.isVisible() or not self.controller.state.connected: return
        if 'uptime_ms' not in data:
            self.message.setText('Aggiorna il firmware: manca il timestamp necessario ai grafici.'); self.message.show(); return
        if not self.history.append(data): return
        self.message.clear(); self.message.hide()
        self._render(data)

    def _render(self,data):
        value=self.history.temperature_average()
        self.temperature.setText(f'{value:.1f} °C' if value is not None else '—')
        for key,_,count,ready,name in self.channels:
            empty='Non disponibile' if data and not data.get(ready) else 'In attesa di campioni validi'
            self.plots[key].set_data(self.history.series(key,count),self.history.extent,empty)
        start,end=self.history.extent
        self.summary.setText(f'{end-start:.1f} / 60 s · {len(self.history.samples)} letture · secondi dalla prima lettura')
