"""Recorded flight curves and optional event overlays, using saved board timestamps."""
from math import ceil,floor,log10
from .i18n import t
from PySide6.QtCore import Qt,QPointF,QRectF
from PySide6.QtGui import QColor,QPainter,QPainterPath,QPen
from PySide6.QtWidgets import QWidget,QToolTip


PHASE_COLORS={'PAD':'#e7edf3','BOOST':'#fbe5da','COAST':'#e0edf9','DESCENT':'#dcf0e9','LANDED':'#ece7f4','BOOST 1':'#fbe5da','COAST 1':'#e0edf9','BOOST 2':'#ffd7c4','COAST 2':'#c9dff5','ABORT':'#f6cccc','Control enabled':'#dcf0e9','Control not active':'#edf1f5'}


def axis_scale(values,angular=False,include_zero=True):
    if angular: return -180,180,[-180,-90,0,90,180]
    low=min(values); high=max(values)
    if include_zero: low=min(0,low); high=max(0,high)
    padding=max(1,high-low)*.12
    target=(high-low+2*padding)/4; magnitude=10**floor(log10(target))
    step=next(magnitude*n for n in (1,2,2.5,5,10) if magnitude*n>=target)
    ticks=[n*step for n in range(floor((low-padding)/step),ceil((high+padding)/step)+1)]
    return ticks[0],ticks[-1],ticks


def axis_scales(values_by_axis,angular=False,include_zero=True):
    scales={axis:axis_scale(values,angular,include_zero) for axis,values in values_by_axis.items() if values}
    if len(scales)>1 and include_zero:
        # Common counts of tick intervals below/above zero align both axes exactly.
        steps={axis:ticks[1]-ticks[0] for axis,(_,_,ticks) in scales.items()}
        below=max(round(-low/steps[axis]) for axis,(low,_,_) in scales.items())
        above=max(round(high/steps[axis]) for axis,(_,high,_) in scales.items())
        for axis,step in steps.items():
            ticks=[n*step for n in range(-below,above+1)]
            scales[axis]=(ticks[0],ticks[-1],ticks)
    return scales


class FlightPlot(QWidget):
    # Each curve: (CSV row column, display name, unit, colour, axis).
    def __init__(self,curves,angular=False,include_zero=True,fixed_range=None):
        super().__init__(); self.fixed_range=fixed_range; self.curves=curves; self.angular=angular; self.include_zero=include_zero
        self.rows=[]; self.points=[]; self.markers=[]; self.phases=[]
        self._segments=[]; self._hover=None; self._geometry_key=None; self._paths={}; self._scales={}; self.setMouseTracking(True)
        self.show_actuators=True; self.show_states=True; self.setMinimumHeight(240)

    def set_data(self,rows,markers=(),phases=()):
        self._geometry_key=None; self._segments=[]; self._hover=None; QToolTip.hideText()
        self.rows=rows; self.markers=list(markers); self.phases=list(phases)
        self.points=[(r[0],r[self.curves[0][0]]) for r in rows if r[self.curves[0][0]] is not None]
        self.update()

    def set_overlays(self,actuators,states):
        if (self.show_actuators,self.show_states)==(actuators,states): return
        self.show_actuators=actuators; self.show_states=states; self.update()

    def _nearest_sample(self,position):
        closest=None; distance=9**2
        for a,b,first,last,curve in self._segments:
            dx=b.x()-a.x(); dy=b.y()-a.y(); length=dx*dx+dy*dy
            fraction=max(0,min(1,((position.x()-a.x())*dx+(position.y()-a.y())*dy)/length)) if length else 0
            squared=(position.x()-a.x()-fraction*dx)**2+(position.y()-a.y()-fraction*dy)**2
            if squared>distance: continue
            distance=squared; point,record=(a,first) if fraction<.5 else (b,last)
            col,name,unit,_,_=curve
            closest=(point,f'Tempo: {record[0]:.3f} s\n{name}: {record[col]:.3f} {unit}',col)
        return closest

    def mouseMoveEvent(self,event):
        hover=self._nearest_sample(event.position())
        if hover!=self._hover: self._hover=hover; self.update()
        if hover: QToolTip.showText(event.globalPosition().toPoint(),hover[1],self)
        else: QToolTip.hideText()

    def leaveEvent(self,event):
        self._hover=None; QToolTip.hideText(); self.update(); super().leaveEvent(event)

    def _prepare_geometry(self,box):
        key=(box.left(),box.top(),box.width(),box.height(),tuple(self.curves),self.angular,self.include_zero,self.fixed_range)
        if key==self._geometry_key: return
        self._geometry_key=key; self._segments=[]; self._paths={}
        dual=any(c[4]==1 for c in self.curves)
        values={axis:[r[c[0]] for c in self.curves if c[4]==axis for r in self.rows if r[c[0]] is not None]
                for axis in ((0,1) if dual else (0,))}
        self._scales=axis_scales(values,self.angular,self.include_zero)
        if self.fixed_range:
            low,high=self.fixed_range; self._scales={0:(low,high,[low+(high-low)*i/4 for i in range(5)])}
        x0=self.rows[0][0]; span=max(.001,self.rows[-1][0]-x0)
        for curve in self.curves:
            col,_,_,_,axis=curve
            if axis not in self._scales: continue
            low,high,_=self._scales[axis]; path=QPainterPath(); previous=None; angular_points=[]
            for r in self.rows:
                value=r[col]
                if value is None: previous=None; continue
                point=QPointF(box.left()+(r[0]-x0)/span*box.width(),box.bottom()-(value-low)/(high-low)*box.height())
                if previous is None or (self.angular and abs(value-previous[1][col])>=180):
                    path.moveTo(point)
                    if self.angular: self._segments.append((point,point,r,r,curve))
                else:
                    path.lineTo(point); self._segments.append((previous[0],point,previous[1],r,curve))
                if self.angular: angular_points.append(point)
                previous=(point,r)
            self._paths[col]=(path,angular_points)

    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        dual=any(c[4]==1 for c in self.curves)
        box=QRectF(68,48,max(10,self.width()-136),max(10,self.height()-82))
        if not self.rows:
            p.setPen(QColor('#7b8998')); p.drawText(box,Qt.AlignCenter,t('Open a flight to view the data'))
            return
        x0=self.rows[0][0]; x1=max(x0+.001,self.rows[-1][0])
        def xpos(stamp): return box.left()+(stamp-x0)/(x1-x0)*box.width()
        if self.show_states:
            legend_x=box.left(); seen=set()
            for i,(stamp,name) in enumerate(self.phases):
                end=self.phases[i+1][0] if i+1<len(self.phases) else x1
                left=xpos(max(x0,stamp)); right=xpos(min(x1,end))
                if right<=left or name is None: continue
                color=QColor(PHASE_COLORS.get(name,'#edf1f5'))
                p.fillRect(QRectF(left,box.top(),right-left,box.height()),color)
                if name not in seen:
                    p.fillRect(QRectF(legend_x,28,10,10),color); p.setPen(QColor('#65768b'))
                    p.drawText(QPointF(legend_x+15,38),t(name))
                    legend_x+=p.fontMetrics().horizontalAdvance(name)+30; seen.add(name)
        for i in range(5):
            x=box.left()+box.width()*i/4
            p.setPen(QColor('#7b8998'))
            p.drawText(QRectF(x-26,box.bottom()+8,52,18),Qt.AlignCenter,f'{x0+(x1-x0)*i/4:.1f}')
        p.drawText(QRectF(box.right()+31,box.bottom()+8,24,18),Qt.AlignLeft,'s')
        self._prepare_geometry(box)
        scales=self._scales
        for axis,(low,high,ticks) in scales.items():
            curves=[c for c in self.curves if c[4]==axis]
            color=curves[0][3] if dual else '#65768b'
            for value in ticks:
                y=box.bottom()-(value-low)/(high-low)*box.height()
                if not axis:
                    p.setPen(QPen(QColor('#aebdca' if value==0 else '#dce4ec'),1))
                    p.drawLine(QPointF(box.left(),y),QPointF(box.right(),y))
                p.setPen(QColor(color))
                area=QRectF(box.right()+7,y-9,58,18) if axis else QRectF(0,y-9,60,18)
                p.drawText(area,Qt.AlignLeft if axis else Qt.AlignRight,f'{value:g}')
            area=QRectF(box.right()+7,27,58,18) if axis else QRectF(0,27,60,18)
            p.drawText(area,Qt.AlignLeft if axis else Qt.AlignRight,curves[0][2])
        legend_x=box.left()
        selected=self._hover[2] if self._hover else None
        for col,name,unit,color,axis in self.curves:
            line_color=QColor(color)
            if selected is not None and col!=selected: line_color.setAlphaF(.22)
            p.setPen(line_color); text=f'{name} ({unit})'
            p.drawText(QPointF(legend_x,17),text); legend_x+=p.fontMetrics().horizontalAdvance(text)+24
            if axis not in scales: continue
            path,angular_points=self._paths[col]
            p.setPen(QPen(line_color,3 if col==selected else 2)); p.drawPath(path)
            if angular_points: p.drawPoints(angular_points)
        if not scales:
            p.setPen(QColor('#7b8998')); p.drawText(box,Qt.AlignCenter,t('Measurements not available in this flight'))
        if self.show_actuators:
            for index,(stamp,text) in enumerate(self.markers):
                if not x0<=stamp<=x1: continue
                x=xpos(stamp); p.setPen(QPen(QColor('#a55b28'),1,Qt.DashLine))
                p.drawLine(QPointF(x,box.top()),QPointF(x,box.bottom()))
                # Rotated labels keep simultaneous activations readable in separate lanes.
                p.save(); p.translate(x-8-(index%3)*15,box.bottom()-5); p.rotate(-90)
                p.drawText(QPointF(0,0),text); p.restore()
        if self._hover:
            color=next(c[3] for c in self.curves if c[0]==selected)
            p.setPen(QPen(QColor(color),1.5)); p.setBrush(QColor('#ffffff'))
            p.drawEllipse(self._hover[0],4,4)
