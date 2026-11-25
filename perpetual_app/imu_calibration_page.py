"""Guided IMU acquisition; acceptance and saved approval belong to firmware."""
from PySide6.QtCore import Qt,QTimer
from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QGridLayout,QFrame,QProgressBar,QSizePolicy
from .widgets import label,button,card,row
from .orientation import board_preview
from .i18n import t

FACES=('+X','-X','+Y','-Y','+Z','-Z')
STATES={'invalid':'Recalibration required','identity':'Identity (default)','absent':'Missing','required':'Needs verification','verified':'Verified'}
CAUSES={'VERIFICATION_FAILED':'Validation failed: repeat the six-face calibration. Identity is permanently disabled.', 'IDENTITY':'No custom calibration saved: zero bias and unit scale.', 'MISSING':'No calibration saved.', 'AFTER_BOOST':'Verification required after the flight.',
        'FLIGHT_OR_RESET':'Verification required after a flight or a restart while armed.',
        'NOT_VERIFIED':'The saved coefficients must be verified.'}
TONES={'verified':'ok','required':'warn','identity':'warn','absent':'bad','invalid':'bad'} # Status colour, also used by the Dashboard row.
ERRORS={'IMU_VERIFICATION_FAILED':'Validation failed. Repeat the calibration on all six faces.', 'POSITION_OR_MOTION':'Wrong position or board moving: capture stopped.', # Errors stay in English.
        'NOISY':'Measurements too noisy: capture stopped.',
        'IMU_CALIBRATION_TIMEOUT':'Timed out. Start the procedure again.',
        'IMU_CALIBRATION_INVALID':'Inconsistent result: repeat the calibration on all six faces.',
        'IMU_CALIBRATION_IO':'Saving failed: check the memory and repeat the procedure.'}
RETRY_ERRORS=('POSITION_OR_MOTION','NOISY') # The firmware stops that capture: a warning, the face is captured again.
# Firmware verification limit (docs/technical/NAVIGATION.md), used only to scale the residual bars.
VERIFY_LIMIT=0.35


def restyle(widget,name,value):
    if widget.property(name)!=value: widget.setProperty(name,value); widget.style().unpolish(widget); widget.style().polish(widget)


class ImuCalibrationPage(QWidget):
    DONE_MS=1500 # How long a completed capture stays shown before the bar returns to idle.

    def __init__(self,controller):
        super().__init__(); self.controller=controller; self.next_face=0; self.last_state=None
        self.bias_before=None # (revision, bias) read before a six-face calibration, to show how much the bias moved.
        self.capturing=None; self.captured=None; self.reset_timer=QTimer(self,singleShot=True,interval=self.DONE_MS,timeout=self.clear_captured)
        self.window_samples=0
        box=QVBoxLayout(self); box.setContentsMargins(0,14,0,0); box.setSpacing(14)
        # Current calibration: a tinted panel coloured by the firmware state.
        self.status_panel=QFrame(); self.status_panel.setObjectName('imuStatus'); panel=QHBoxLayout(self.status_panel)
        panel.setContentsMargins(24,18,24,18); panel.setSpacing(32); box.addWidget(self.status_panel)
        self.status_panel.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Maximum)
        summary=QVBoxLayout(); summary.setSpacing(4); panel.addLayout(summary,1)
        summary.addWidget(label(t('CURRENT CALIBRATION'),'eyebrow'))
        self.state_label=label('—','imuState'); summary.addWidget(self.state_label)
        self.cause_label=label('','imuCause'); self.cause_label.setWordWrap(True); summary.addWidget(self.cause_label)
        summary.addStretch()
        # Per-axis bias and scale, each with a meter scaled to the firmware acceptance limit.
        self.coefficients=QWidget(); grid=QGridLayout(self.coefficients); grid.setContentsMargins(0,0,0,0)
        grid.setHorizontalSpacing(18); grid.setVerticalSpacing(3); panel.addWidget(self.coefficients,0,Qt.AlignVCenter)
        for column,axis in enumerate('XYZ',1):
            heading=label(axis,'eyebrow'); heading.setAlignment(Qt.AlignCenter); grid.addWidget(heading,0,column)
        self.values={}
        for line,(key,caption) in enumerate((('bias',t('BIAS · m/s²')),('scale',t('SCALE'))),1):
            grid.addWidget(label(caption,'eyebrow'),line,0)
            for column in range(3):
                value=label('—','imuValue'); value.setAlignment(Qt.AlignCenter); grid.addWidget(value,line,column+1); self.values[key,column]=value
        # Bias change after a recalibration in this session: new bias minus the one read before it.
        self.drift_caption=label(t('BIAS CHANGE · m/s²'),'eyebrow'); grid.addWidget(self.drift_caption,3,0)
        self.drift=[label('—','imuValue') for _ in range(3)]
        for column,value in enumerate(self.drift,1): value.setAlignment(Qt.AlignCenter); grid.addWidget(value,3,column)
        # Last verification (firmware history): corrected |a| minus g on +X, +Y, +Z, the real calibration quality.
        self.residual_caption=label(t('VERIFICATION · |a|−g m/s²'),'eyebrow'); grid.addWidget(self.residual_caption,4,0)
        self.residuals=[label('—','imuValue') for _ in range(3)]; self.residual_meters=[]
        for column,value in enumerate(self.residuals,1):
            value.setAlignment(Qt.AlignCenter); grid.addWidget(value,4,column)
            meter=QProgressBar(); meter.setObjectName('imuMeter'); meter.setRange(0,1000); meter.setTextVisible(False); meter.setFixedSize(88,5)
            grid.addWidget(meter,5,column,Qt.AlignHCenter|Qt.AlignTop); self.residual_meters.append(meter)
        self.meter_hint=label(t('Bars: share of the verification limit (±0.35 m/s²)'),'hint'); grid.addWidget(self.meter_hint,6,0,1,4)
        # Procedure.
        frame,body=card(); box.addWidget(frame); frame.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Maximum)
        body.addWidget(label(t('Procedure'),'cardTitle'))
        summary_text=label(t('Rest each face up on a stand or a square block, keep the board still and press Capture. Verification needs only +X, +Y and +Z, roughly up.'),'hint')
        summary_text.setWordWrap(True); body.addWidget(summary_text)
        self.begin_button=button(t('Calibrate on six faces'),self.begin)
        self.verify_button=button(t('Verify calibration'),lambda:controller.command('imu verify',t('Starting IMU verification')),'imuVerify')
        self.refresh_button=button(t('Refresh'),lambda:controller.command('imu status',t('Reading IMU calibration')))
        body.addLayout(row(self.begin_button,self.verify_button,self.refresh_button,None))
        self.instruction=label(); self.instruction.setWordWrap(True); body.addWidget(self.instruction)
        # Running procedure: which face to put up, the faces done so far and the capture window.
        self.session=QWidget(); session=QHBoxLayout(self.session); session.setContentsMargins(0,4,0,0); session.setSpacing(24); body.addWidget(self.session)
        board=QVBoxLayout(); board.setSpacing(6); session.addLayout(board)
        self.board_caption=label('','eyebrow'); self.board_caption.setAlignment(Qt.AlignCenter); board.addWidget(self.board_caption)
        self.board_image=label(); self.board_image.setObjectName('imuBoard'); self.board_image.setFixedSize(132,132); self.board_image.setAlignment(Qt.AlignCenter); board.addWidget(self.board_image)
        board.addStretch()
        steps=QVBoxLayout(); steps.setSpacing(12); session.addLayout(steps,1)
        self.chips=[label(face,'imuFace') for face in FACES]
        for chip in self.chips: chip.setAlignment(Qt.AlignCenter); chip.setFixedSize(54,30)
        steps.addLayout(row(*self.chips,None))
        self.progress=QProgressBar(); self.progress.setObjectName('imuProgress'); steps.addWidget(self.progress)
        self.progress_caption=label('','hint'); steps.addWidget(self.progress_caption)
        self.capture_button=button(t('Capture position'),self.capture,'primary')
        self.cancel_button=button(t('Cancel procedure'),lambda:controller.command('imu cancel',t('Cancelling IMU calibration')))
        steps.addLayout(row(self.capture_button,self.cancel_button,None)); steps.addStretch()
        self.measurements=label('','hint'); self.measurements.setWordWrap(True); body.addWidget(self.measurements)
        self.detail=label('','imuError'); self.detail.setWordWrap(True); self.detail.hide(); body.addWidget(self.detail)
        box.addStretch()

    def begin(self):
        imu=self.controller.state.imu_calibration; bias=imu.get('bias_mps2')
        # Only a measured calibration is worth comparing: the identity's zero bias was never measured.
        self.bias_before=(imu.get('revision'),bias) if bias and len(bias)==3 and not imu.get('identity_active') else None
        self.controller.command('imu begin',t('Starting IMU calibration'))

    def capture(self):
        self.controller.command('imu capture '+FACES[self.next_face],t('Capturing {face}',face=FACES[self.next_face]))

    def clear_captured(self):
        self.captured=None
        if self.last_state is not None: self.render(self.last_state)

    def render(self,state):
        self.last_state=state; imu=state.imu_calibration; active=imu.get('active',False); collecting=imu.get('collecting',False)
        available=state.connected and bool(imu); code=imu.get('imu_calibration_state') if available else None
        usable=available and state.owns_control and not state.foreground
        self.state_label.setText(t(STATES.get(code,'Not read')) if available else '—')
        self.cause_label.setText(t(CAUSES.get(imu.get('imu_calibration_reason'),'')) if available else
                                 t('Connect the board to read the calibration.') if not state.connected else '')
        tone=TONES.get(code,'neutral'); restyle(self.status_panel,'tone',tone); restyle(self.state_label,'tone',tone)
        self.coefficients.setVisible(available and ('bias_mps2' in imu or 'scale' in imu))
        for key,values,text in (('bias',imu.get('bias_mps2'),'{:+.4f}'),('scale',imu.get('scale'),'{:.5f}')):
            for axis in range(3):
                value=values[axis] if available and values and len(values)==3 else None
                self.values[key,axis].setText(text.format(value) if value is not None else '—')
        if not state.connected: self.bias_before=None # Another board may be connected next.
        bias=imu.get('bias_mps2'); bias_before=self.bias_before
        drift=bool(available and bias_before and bias and len(bias)==3 and imu.get('revision') and imu.get('revision')!=bias_before[0] and not imu.get('identity_active'))
        for widget in (self.drift_caption,*self.drift): widget.setVisible(drift)
        if drift:
            for axis,widget in enumerate(self.drift): widget.setText(f'{bias[axis]-bias_before[1][axis]:+.4f}')
            self.drift_caption.setToolTip(t('New bias minus the bias before this calibration.'))
        # Verification residuals, only once the calibration has been verified (passed or failed).
        residuals=imu.get('verification_residual_mps2') if available else None
        residuals=residuals if residuals and len(residuals)==3 and any(r is not None for r in residuals) else None
        for widget in (self.residual_caption,*self.residuals,*self.residual_meters,self.meter_hint): widget.setVisible(bool(residuals))
        for value,meter,residual in zip(self.residuals,self.residual_meters,residuals or [None]*3):
            share=min(abs(residual)/VERIFY_LIMIT,1) if residual is not None else 0
            value.setText(f'{residual:+.4f}' if residual is not None else '—'); meter.setValue(round(share*1000))
            meter.setToolTip(t('{percent} % of the acceptance limit',percent=f'{share*100:.1f}') if residual is not None else t('Face not measured: the verification stopped earlier.'))
        idle=usable and state.status.get('arming_state')=='disarmed' and not state.sim.get('active') and not state.status.get('sensor_hil_active') and not state.status.get('estimate_replay_active')
        self.begin_button.setEnabled(idle and not active and not state.dirty and not state.staged)
        self.verify_button.setEnabled(self.begin_button.isEnabled() and code in ('required','verified'))
        restyle(self.verify_button,'attention',self.verify_button.isEnabled() and code=='required')
        self.refresh_button.setEnabled(state.connected and not state.foreground)
        self.cancel_button.setEnabled(usable and active)
        self.capture_button.setEnabled(usable and active and not collecting)
        faces=(0,2,4) if imu.get('verification') else range(6)
        mask=imu.get('face_mask',0); done=sum(bool(mask & (1<<face_index)) for face_index in faces)
        self.next_face=next((face_index for face_index in faces if not mask & (1<<face_index)),0)
        face=imu.get('face',0) if collecting else self.next_face
        # Bar phases: idle (empty) -> collecting -> captured (full, green) for DONE_MS -> idle.
        # A capture the firmware stopped (wrong position, motion or noise) stays frozen in ochre until Capture.
        target=max(imu.get('target',240),1); samples=min(imu.get('samples',0),target); window_error=imu.get('window_error')
        if collecting:
            if self.capturing!=face: self.window_samples=0
            self.window_samples=max(self.window_samples,samples) if samples else 0; self.capturing=face; self.captured=None; self.reset_timer.stop()
        elif self.capturing is not None:
            if available and mask & (1<<self.capturing): self.captured=self.capturing; self.reset_timer.start()
            self.capturing=None
        if not available: self.captured=None; self.reset_timer.stop()
        stopped=available and active and not collecting and window_error in RETRY_ERRORS
        phase='loading' if collecting else 'stopped' if stopped else 'done' if self.captured is not None else 'idle'
        self.progress.setRange(0,target)
        self.progress.setValue(samples if phase=='loading' else self.window_samples if phase=='stopped' else target if phase=='done' else 0)
        restyle(self.progress,'phase',phase)
        self.progress_caption.setText(t('Capturing {face}: {samples}/{target} samples',face=FACES[face],samples=samples,target=target) if phase=='loading' else
                                      t('{face} captured',face=FACES[self.captured]) if phase=='done' else t('Waiting for the capture'))
        restyle(self.progress_caption,'phase',phase); self.progress_caption.setVisible(phase!='stopped') # The warning box below says why.
        self.session.setVisible(active or phase=='done')
        finished=not active or done==len(faces)
        for index,chip in enumerate(self.chips):
            chip.setVisible(index in faces)
            restyle(chip,'step','done' if mask & (1<<index) else 'current' if index==face and not finished else 'pending')
        self.board_image.setPixmap(board_preview(FACES[face],132,132)); self.board_image.setVisible(not finished)
        self.board_caption.setText(t('{face} UP',face=FACES[face]) if not finished else ''); self.board_caption.setVisible(not finished)
        self.capture_button.setText(t('Capture {face}',face=FACES[self.next_face]))
        self.instruction.setText((t('{done}/{total} positions completed. Point {face} up, rest the board and keep it still. ',done=done,total=len(faces),face=FACES[face])+
                                  (t('Capturing…') if collecting else t('Then press Capture.'))) if active else
                                 t('The coefficients stay saved. A physical flight that reaches Boost requires a new verification.'))
        self.capture_button.setVisible(active); self.cancel_button.setVisible(active)
        self.measurements.setText(t('Last window: |a| {norm} m/s² · maximum noise {noise} m/s²',norm=f'{imu["norm_mps2"]:.3f}',noise=f'{imu.get("noise_mps2",0):.3f}') if imu.get('norm_mps2') else '')
        error=imu.get('last_error')
        if error in (None,'None'): error=window_error if active else None
        message=ERRORS.get(error,error if error not in (None,'None') else '')
        self.detail.setText(message); self.detail.setVisible(bool(message))
        restyle(self.detail,'tone','warn' if error in RETRY_ERRORS else 'bad')
