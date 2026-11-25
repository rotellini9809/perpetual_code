"""Board mounting selector and acknowledged device orientation."""
import math
from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap,QTransform
from PySide6.QtWidgets import QWidget,QVBoxLayout,QGridLayout,QComboBox,QSizePolicy,QGraphicsOpacityEffect
from .widgets import ASSETS,label,button,row
from .i18n import t

AXES=('+X','-X','+Y','-Y','+Z','-Z')
ROTATIONS={'+X':0,'-X':180,'+Y':90,'-Y':-90,'+Z':0,'-Z':180} # Board artwork rotation that puts that sensor axis up.
_ARTWORK={}; _PREVIEWS={}


def board_preview(axis,width,height):
    """Board artwork with the given sensor axis pointing up (side view for Z); shared by Settings and IMU calibration."""
    side=axis in ('+Z','-Z'); key=(side,ROTATIONS.get(axis,0),width,height)
    if key not in _PREVIEWS:
        box=(height,width) if key[1] in (90,-90) else (width,height) # Scale once, then only rotate: -Z is exactly +Z turned over.
        if (side,box) not in _ARTWORK:
            _ARTWORK[side,box]=QPixmap(str(ASSETS/('board-orientation-z-reference.jpg' if side else 'board-orientation-reference.jpg'))).scaled(*box,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        _PREVIEWS[key]=_ARTWORK[side,box].transformed(QTransform().rotate(key[1]))
    return _PREVIEWS[key]


class OrientationPanel(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self._observed=None; self.setObjectName('orientationPanel')
        layout=QGridLayout(self); layout.setContentsMargins(0,0,0,0); layout.setHorizontalSpacing(32); layout.setVerticalSpacing(8)
        layout.setColumnStretch(1,1)
        self.direction=label(t('ROCKET NOSE  ↑'),'eyebrow'); self.direction.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.direction,0,0)
        self.image=label(); self.image.setFixedSize(200,220); self.image.setAlignment(Qt.AlignCenter)
        self.image_opacity=QGraphicsOpacityEffect(self.image); self.image.setGraphicsEffect(self.image_opacity)
        layout.addWidget(self.image,1,0)
        details=QVBoxLayout(); details.setSpacing(12)
        text=label(t('Select the sensor axis that points toward the rocket nose.'),'settingsDescription')
        text.setWordWrap(True); details.addWidget(text)
        self.axis=QComboBox(); self.axis.addItem('—',None)
        for axis in AXES: self.axis.addItem(axis,axis)
        self.axis.setAccessibleName(t('Sensor axis pointing toward the rocket nose'))
        self.axis.setFixedWidth(150); self.axis.currentIndexChanged.connect(self._axis_changed)
        self.save=button(t('Save orientation'),lambda:controller.save_orientation(self.axis.currentData()),'saveAction')
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
        illustrated=axis in ROTATIONS
        self.image.setPixmap(board_preview(axis,200,220))
        self.image.setAccessibleName(t('Orientation preview {axis}',axis=axis) if illustrated else t('Orientation not selected'))
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
        ready=orientation.get('orientation_ready',False)
        selected=self.axis.currentData()
        self.saved.setText(t('Saved on the board: {axis}',axis=orientation['up_axis']) if ready else
                           t('Orientation: — to set before arming.') if s.connected and orientation else t('Connect the board to read the orientation.'))
        if s.connected and selected in AXES and (not ready or selected!=orientation.get('up_axis')):
            self.saved.setText(t('Unsaved preview.')+(t(' On the board: {axis}',axis=orientation['up_axis']) if ready else t(' Save to set the orientation.')))
        editable=(s.editable and not s.dirty and not s.staged and not s.sim.get('active') and
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
