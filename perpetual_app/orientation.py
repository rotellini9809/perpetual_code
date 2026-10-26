"""Display-only BODY orientation: ZYX Euler angles in an X-up reference."""
import math


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
