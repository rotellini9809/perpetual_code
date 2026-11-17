"""Guided IMU acquisition; acceptance and saved approval belong to firmware."""
from PySide6.QtWidgets import QWidget,QVBoxLayout,QProgressBar
from .widgets import label,button,card,row

FACES=('+X','-X','+Y','-Y','+Z','-Z')
STATES={'absent':'Assente','required':'Da verificare','verified':'Verificata'}
CAUSES={'MISSING':'Nessuna calibrazione salvata.', 'AFTER_BOOST':'Verifica richiesta dopo il volo.',
        'FLIGHT_OR_RESET':'Verifica richiesta dopo un volo o un riavvio durante l’armaggio.',
        'NOT_VERIFIED':'I coefficienti salvati devono essere verificati.'}
ERRORS={'POSITION_OR_MOTION':'Posizione non corretta o scheda in movimento: la finestra riparte.',
        'NOISY':'Misure troppo rumorose: la finestra riparte.',
        'IMU_CALIBRATION_TIMEOUT':'Tempo scaduto. Avvia nuovamente la procedura.',
        'IMU_CALIBRATION_INVALID':'Risultato non coerente: ripeti la calibrazione sulle sei facce.',
        'IMU_CALIBRATION_IO':'Salvataggio non riuscito: verifica la memoria e ripeti la procedura.'}


class ImuCalibrationPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self.next_face=0
        box=QVBoxLayout(self); box.setContentsMargins(0,14,0,0)
        frame,body=card(); box.addWidget(frame)
        body.addWidget(label('Calibrazione IMU','cardTitle'))
        self.state_label=label(); self.state_label.setWordWrap(True); body.addWidget(self.state_label)
        explanation=label('La calibrazione misura bias e scala sui sei orientamenti. La verifica richiede due nuove acquisizioni, +X e +Y. '
                          'Gli assi sono quelli del sensore sulla scheda; sono indipendenti dall’orientamento del razzo.','hint')
        explanation.setWordWrap(True); body.addWidget(explanation)
        self.begin_button=button('Calibra sulle sei facce',lambda:controller.command('imu begin','Avvio calibrazione IMU'))
        self.verify_button=button('Verifica calibrazione',lambda:controller.command('imu verify','Avvio verifica IMU'))
        self.refresh_button=button('Aggiorna',lambda:controller.command('imu status','Lettura calibrazione IMU'))
        body.addLayout(row(self.begin_button,self.verify_button,self.refresh_button,None))
        self.instruction=label(); self.instruction.setWordWrap(True); body.addWidget(self.instruction)
        self.progress=QProgressBar(); body.addWidget(self.progress)
        self.capture_button=button('Acquisisci posizione',self.capture,'primary')
        self.cancel_button=button('Annulla procedura',lambda:controller.command('imu cancel','Annullamento calibrazione IMU'))
        body.addLayout(row(self.capture_button,self.cancel_button,None))
        self.measurements=label('','hint'); self.measurements.setWordWrap(True); body.addWidget(self.measurements)
        self.detail=label('','hint'); self.detail.setWordWrap(True); body.addWidget(self.detail)
        box.addStretch()

    def capture(self):
        self.controller.command('imu capture '+FACES[self.next_face],'Acquisizione '+FACES[self.next_face])

    def render(self,s):
        c=s.imu_calibration; active=c.get('active',False); collecting=c.get('collecting',False)
        available=s.connected and c.get('supported',False)
        usable=available and s.owns_control and not s.foreground
        state=STATES.get(c.get('imu_calibration_state'),'Non letta')
        cause=CAUSES.get(c.get('imu_calibration_reason'),'')
        self.state_label.setText(('Calibrazione IMU: '+state+'. '+cause) if available else
                                 'Collega la scheda per leggere la calibrazione.' if not s.connected else 'Procedura IMU non disponibile su questo firmware.')
        idle=usable and s.status.get('arming_state')=='disarmed' and not s.sim.get('active') and not s.status.get('sensor_hil_active') and not s.status.get('estimate_replay_active')
        self.begin_button.setEnabled(idle and not active and not s.dirty and not s.staged)
        self.verify_button.setEnabled(self.begin_button.isEnabled() and c.get('imu_calibration_state')!='absent')
        self.refresh_button.setEnabled(s.connected and not s.foreground)
        self.cancel_button.setEnabled(usable and active)
        self.capture_button.setEnabled(usable and active and not collecting)
        faces=(0,2) if c.get('verification') else range(6)
        mask=c.get('face_mask',0); done=sum(bool(mask & (1<<f)) for f in faces)
        self.next_face=next((f for f in faces if not mask & (1<<f)),0)
        face=c.get('face',0) if collecting else self.next_face
        self.capture_button.setText('Acquisisci '+FACES[self.next_face])
        self.instruction.setText((f'{done}/{len(faces)} posizioni completate. Orienta {FACES[face]} verso l’alto, appoggia la scheda e lasciala ferma. '+
                                  ('Acquisizione in corso…' if collecting else 'Poi premi Acquisisci.')) if active else
                                 'I coefficienti restano salvati. Dopo un volo fisico che raggiunge Boost è richiesta una nuova verifica.')
        self.progress.setVisible(active); self.progress.setRange(0,c.get('target',240)); self.progress.setValue(min(c.get('samples',0),c.get('target',240)))
        self.capture_button.setVisible(active); self.cancel_button.setVisible(active)
        values=[]
        if c.get('revision'): values.append('Calibrazione #'+str(c['revision']))
        if 'bias_mps2' in c: values.append('Bias XYZ: '+', '.join(f'{v:.4f}' for v in c['bias_mps2'])+' m/s²')
        if 'scale' in c: values.append('Scala XYZ: '+', '.join(f'{v:.5f}' for v in c['scale']))
        if c.get('norm_mps2'): values.append(f'Ultima finestra: |a| {c["norm_mps2"]:.3f} m/s² · rumore massimo {c.get("noise_mps2",0):.3f} m/s²')
        self.measurements.setText('\n'.join(values))
        error=c.get('last_error')
        if error in (None,'None'): error=c.get('window_error') if collecting else None
        self.detail.setText(ERRORS.get(error,error if error not in (None,'None') else ''))
