"""Board mounting selector and acknowledged device orientation."""
import math
from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap,QTransform
from PySide6.QtWidgets import QWidget,QVBoxLayout,QGridLayout,QComboBox,QSizePolicy,QGraphicsOpacityEffect
from .widgets import ASSETS,label,button,row

AXES=('+X','-X','+Y','-Y','+Z','-Z')


class OrientationPanel(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self._observed=None; self.setObjectName('orientationPanel')
        layout=QGridLayout(self); layout.setContentsMargins(0,0,0,0); layout.setHorizontalSpacing(32); layout.setVerticalSpacing(8)
        layout.setColumnStretch(1,1)
        self.direction=label('PUNTA DEL RAZZO  '+chr(8593),'eyebrow'); self.direction.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.direction,0,0)
        self.image=label(); self.image.setFixedSize(200,220); self.image.setAlignment(Qt.AlignCenter)
        self.image_opacity=QGraphicsOpacityEffect(self.image); self.image.setGraphicsEffect(self.image_opacity)
        self.artwork=QPixmap(str(ASSETS/'board-orientation-reference.jpg')).scaled(220,220,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.z_artwork=QPixmap(str(ASSETS/'board-orientation-z-reference.jpg')).scaled(200,220,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self._previews={}; layout.addWidget(self.image,1,0)
        details=QVBoxLayout(); details.setSpacing(12)
        text=label('Seleziona l'+chr(8217)+'asse del sensore rivolto verso la punta del razzo.','settingsDescription')
        text.setWordWrap(True); details.addWidget(text)
        self.axis=QComboBox(); self.axis.addItem(chr(8212),None)
        for axis in AXES: self.axis.addItem(axis,axis)
        self.axis.setAccessibleName('Asse del sensore rivolto verso la punta del razzo')
        self.axis.setFixedWidth(150); self.axis.currentIndexChanged.connect(self._axis_changed)
        self.save=button('Salva orientamento',lambda:controller.save_orientation(self.axis.currentData()),'saveAction')
        self.save.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed); self.save.setMinimumHeight(42)
        details.addLayout(row(self.axis,self.save,None))
        self.saved=label('','hint'); self.saved.setWordWrap(True); details.addWidget(self.saved)
        layout.addLayout(details,1,1,Qt.AlignVCenter)
        self.update_preview()

    def _axis_changed(self):
        self.controller.edit_setting('orientation',self.axis.currentData())
        self.update_preview()

    def update_preview(self):
        axis=self.axis.currentData()
        rotations={'+X':0,'-X':180,'+Y':90,'-Y':-90,'+Z':0,'-Z':180}
        illustrated=axis in rotations
        rotation=rotations.get(axis,0)
        key=(axis in ('+Z','-Z'),rotation)
        if key not in self._previews:
            artwork=self.z_artwork if key[0] else self.artwork
            self._previews[key]=artwork.transformed(QTransform().rotate(rotation)).scaled(200,220,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        self.image.setPixmap(self._previews[key])
        self.image.setAccessibleName('Anteprima orientamento '+axis if illustrated else 'Orientamento non selezionato')
        self.image_opacity.setOpacity(1 if illustrated else .25)
        self.render(self.controller.state,update_selection=False)

    def render(self,s,update_selection=True):
        orientation=s.orientation if s.connected else {}
        axis=s.local_setting('orientation')
        observed=(s.connected,axis,orientation.get('orientation_ready'))
        if update_selection and observed!=self._observed:
            self._observed=observed
            self.axis.blockSignals(True); self.axis.setCurrentIndex(self.axis.findData(axis) if axis in AXES else 0); self.axis.blockSignals(False)
            self.update_preview()
        supported=orientation.get('supported') is not False
        ready=orientation.get('orientation_ready',False)
        selected=self.axis.currentData()
        self.saved.setText(('Salvato sulla scheda: '+orientation['up_axis']) if ready else
                           'Orientamento: '+chr(8212)+' da impostare per armare.' if s.connected and orientation else 'Collega la scheda per leggere l'+chr(8217)+'orientamento.')
        if s.connected and selected in AXES and (not ready or selected!=orientation.get('up_axis')):
            self.saved.setText('Anteprima non salvata.'+(' Sulla scheda: '+orientation['up_axis'] if ready else ' Salva per impostare l'+chr(8217)+'orientamento.'))
        if s.connected and not supported: self.saved.setText('Aggiorna il firmware della scheda per impostare l'+chr(8217)+'orientamento.')
        editable=(supported and s.editable and not s.dirty and not s.staged and not s.sim.get('active') and
                  not s.status.get('sensor_hil_active'))
        self.axis.setEnabled(editable)
        self.save.setEnabled(editable and self.axis.currentData() in AXES and (not ready or self.axis.currentData()!=orientation.get('up_axis')))


def euler_x_up(roll, pitch, yaw):
    """Convert stored Z-up ZYX Euler degrees to X-up ZYX Euler degrees.

    R maps BODY to NAV. Change the reference, not the sensor axes:
    R_display = Ry(+90 deg) R, so NAV +Z becomes display +X,
    NAV +Y stays +Y and NAV +X becomes display -Z (right-handed).
    Thus R=Ry(-90 deg), an upright nose with the chosen heading, displays
    (0,0,0). This is a matrix composition, not a pitch offset. Gyro BODY
    components are unaffected and are not Euler-angle derivatives.
    """
    if any(v is None or not math.isfinite(v) for v in (roll,pitch,yaw)):
        return None,None,None
    r,p,y=map(math.radians,(roll,pitch,yaw))
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    # Rows of Ry(+90 deg) Rz(yaw) Ry(pitch) Rx(roll).
    m00,m01=-sp,cp*sr
    m10,m11=sy*cp,sy*sp*sr+cy*cr
    m20,m21,m22=-cy*cp,-(cy*sp*sr-sy*cr),-(cy*sp*cr+sy*sr)
    horizontal=math.hypot(m00,m10)
    p=math.atan2(-m20,horizontal)
    if horizontal>1e-8:
        r,y=math.atan2(m21,m22),math.atan2(m10,m00)
    else:
        # At the new Euler singularity choose roll=0; preserve the rotation.
        r,y=0.0,math.atan2(-m01,m11)
    return tuple(math.degrees(a) for a in (r,p,y))
