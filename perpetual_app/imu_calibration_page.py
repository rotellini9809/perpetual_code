"""Guided IMU acquisition; acceptance and saved approval belong to firmware."""
from PySide6.QtWidgets import QWidget,QVBoxLayout,QProgressBar
from .widgets import label,button,card,row
from .i18n import t

FACES=('+X','-X','+Y','-Y','+Z','-Z')
STATES={'invalid':'Recalibration required','identity':'Identity (default)','absent':'Missing','required':'Needs verification','verified':'Verified'}
CAUSES={'VERIFICATION_FAILED':'Validation failed: repeat the six-face calibration. Identity is permanently disabled.', 'IDENTITY':'No custom calibration saved: zero bias and unit scale.', 'MISSING':'No calibration saved.', 'AFTER_BOOST':'Verification required after the flight.',
        'FLIGHT_OR_RESET':'Verification required after a flight or a restart while armed.',
        'NOT_VERIFIED':'The saved coefficients must be verified.'}
ERRORS={'IMU_VERIFICATION_FAILED':'Validation failed. Repeat the calibration on all six faces.', 'POSITION_OR_MOTION':'Wrong position or board moving: the window restarts.', # Errors stay in English.
        'NOISY':'Measurements too noisy: the window restarts.',
        'IMU_CALIBRATION_TIMEOUT':'Timed out. Start the procedure again.',
        'IMU_CALIBRATION_INVALID':'Inconsistent result: repeat the calibration on all six faces.',
        'IMU_CALIBRATION_IO':'Saving failed: check the memory and repeat the procedure.'}


class ImuCalibrationPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self.next_face=0
        box=QVBoxLayout(self); box.setContentsMargins(0,14,0,0)
        frame,body=card(); box.addWidget(frame)
        body.addWidget(label(t('IMU calibration'),'cardTitle'))
        self.state_label=label(); self.state_label.setWordWrap(True); body.addWidget(self.state_label)
        explanation=label(t('The calibration measures bias and scale in six orientations: each face must point straight up (within about 2°; use a stand or a square block). Verification requires three new captures, +X, +Y and +Z; there each face only needs to point roughly up (within 10°). Keep the board still in both. '
                            'The axes are those of the sensor on the board; they do not depend on the rocket orientation.'),'hint')
        explanation.setWordWrap(True); body.addWidget(explanation)
        self.begin_button=button(t('Calibrate on six faces'),lambda:controller.command('imu begin',t('Starting IMU calibration')))
        self.verify_button=button(t('Verify calibration'),lambda:controller.command('imu verify',t('Starting IMU verification')))
        self.refresh_button=button(t('Refresh'),lambda:controller.command('imu status',t('Reading IMU calibration')))
        body.addLayout(row(self.begin_button,self.verify_button,self.refresh_button,None))
        self.instruction=label(); self.instruction.setWordWrap(True); body.addWidget(self.instruction)
        self.progress=QProgressBar(); body.addWidget(self.progress)
        self.capture_button=button(t('Capture position'),self.capture,'primary')
        self.cancel_button=button(t('Cancel procedure'),lambda:controller.command('imu cancel',t('Cancelling IMU calibration')))
        body.addLayout(row(self.capture_button,self.cancel_button,None))
        self.measurements=label('','hint'); self.measurements.setWordWrap(True); body.addWidget(self.measurements)
        self.detail=label('','hint'); self.detail.setWordWrap(True); body.addWidget(self.detail)
        box.addStretch()

    def capture(self):
        self.controller.command('imu capture '+FACES[self.next_face],t('Capturing {face}',face=FACES[self.next_face]))

    def render(self,s):
        c=s.imu_calibration; active=c.get('active',False); collecting=c.get('collecting',False)
        available=s.connected and c.get('supported',False)
        usable=available and s.owns_control and not s.foreground
        state=t(STATES.get(c.get('imu_calibration_state'),'Not read'))
        cause=t(CAUSES.get(c.get('imu_calibration_reason'),''))
        self.state_label.setText(t('IMU calibration: {state}. {cause}',state=state,cause=cause) if available else
                                 t('Connect the board to read the calibration.') if not s.connected else t('IMU procedure not available on this firmware.'))
        idle=usable and s.status.get('arming_state')=='disarmed' and not s.sim.get('active') and not s.status.get('sensor_hil_active') and not s.status.get('estimate_replay_active')
        self.begin_button.setEnabled(idle and not active and not s.dirty and not s.staged)
        self.verify_button.setEnabled(self.begin_button.isEnabled() and c.get('imu_calibration_state') in ('required','verified'))
        self.refresh_button.setEnabled(s.connected and not s.foreground)
        self.cancel_button.setEnabled(usable and active)
        self.capture_button.setEnabled(usable and active and not collecting)
        faces=(0,2,4) if c.get('verification') else range(6)
        mask=c.get('face_mask',0); done=sum(bool(mask & (1<<f)) for f in faces)
        self.next_face=next((f for f in faces if not mask & (1<<f)),0)
        face=c.get('face',0) if collecting else self.next_face
        self.capture_button.setText(t('Capture {face}',face=FACES[self.next_face]))
        self.instruction.setText((t('{done}/{total} positions completed. Point {face} up, rest the board and keep it still. ',done=done,total=len(faces),face=FACES[face])+
                                  (t('Capturing…') if collecting else t('Then press Capture.'))) if active else
                                 t('The coefficients stay saved. A physical flight that reaches Boost requires a new verification.'))
        self.progress.setVisible(active); self.progress.setRange(0,c.get('target',240)); self.progress.setValue(min(c.get('samples',0),c.get('target',240)))
        self.capture_button.setVisible(active); self.cancel_button.setVisible(active)
        values=[]
        if c.get('revision'): values.append(t('Calibration #{revision}',revision=c['revision']))
        if 'bias_mps2' in c: values.append(t('Bias XYZ: {values} m/s²',values=', '.join(f'{v:.4f}' for v in c['bias_mps2'])))
        if 'scale' in c: values.append(t('Scale XYZ: {values}',values=', '.join(f'{v:.5f}' for v in c['scale'])))
        if c.get('norm_mps2'): values.append(t('Last window: |a| {norm} m/s² · maximum noise {noise} m/s²',norm=f'{c["norm_mps2"]:.3f}',noise=f'{c.get("noise_mps2",0):.3f}'))
        self.measurements.setText('\n'.join(values))
        error=c.get('last_error')
        if error in (None,'None'): error=c.get('window_error') if collecting else None
        self.detail.setText(ERRORS.get(error,error if error not in (None,'None') else ''))
