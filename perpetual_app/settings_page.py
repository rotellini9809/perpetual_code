"""Device settings UI; persistence and board operations belong to Controller."""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget,QFrame,QVBoxLayout,QCheckBox,QDoubleSpinBox,QMessageBox
from .orientation import OrientationPanel
from .widgets import create_page,card,label,row,button


def setting_card(title,action=None):
    frame,body=card(); heading=label(title,'settingsTitle')
    if action:
        action.setMinimumWidth(132); action.setMinimumHeight(42); action.setCursor(Qt.PointingHandCursor)
        body.addLayout(row(heading,None,action))
    else: body.addWidget(heading)
    return frame,body


class SettingsPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self._rendered_machine=None; self._rendered_airbrakes=None
        _,box=create_page('Impostazioni','Orientamento scheda, macchina a stati e gestione del dispositivo. Le operazioni sono disponibili con scheda collegata, disarmata e senza attività in corso.',self)
        frame,body=setting_card('Orientamento scheda'); box.addWidget(frame)
        self.orientation_panel=OrientationPanel(self.controller); body.addWidget(self.orientation_panel)
        self.machine_save=button('Salva',lambda:self.controller.save_machine(self.bistage_enabled.isChecked(),self.bistage_min_speed.value()),'saveAction')
        frame,body=setting_card('Macchina a stati:',self.machine_save); box.addWidget(frame)
        self.bistage_enabled=QCheckBox('Abilita il volo bistadio')
        self.flight_state_sequence=label('','flightStateSequence'); self.flight_state_sequence.setWordWrap(True); body.addWidget(self.flight_state_sequence)
        self.flight_state_hint=label('Lancio oltre 3 g → BOOST; fine spinta → COAST. Inizio discesa → DESCENT; 2 secondi di stabilità → LANDED.','hint')
        self.flight_state_hint.setWordWrap(True); body.addWidget(self.flight_state_hint)
        self.bistage_enabled.toggled.connect(self._update_machine_sequence); self._update_machine_sequence(False)
        self.bistage_parameters=QFrame(); self.bistage_parameters.setObjectName('bistageParameters')
        parameters=QVBoxLayout(self.bistage_parameters); parameters.setContentsMargins(16,14,16,14); parameters.setSpacing(10)
        parameters.addWidget(self.bistage_enabled)
        self.bistage_fields=QWidget(); fields=QVBoxLayout(self.bistage_fields); fields.setContentsMargins(0,0,0,0); fields.setSpacing(10)
        fields.addWidget(label('Velocit'+chr(224)+' verticale minima dopo BOOST 1','bistageLabel'))
        self.bistage_min_speed=QDoubleSpinBox(); self.bistage_min_speed.setRange(0,100000)
        self.bistage_min_speed.setDecimals(1); self.bistage_min_speed.setSingleStep(1); self.bistage_min_speed.setFixedWidth(160)
        self.bistage_min_speed.setKeyboardTracking(False)
        self.bistage_min_speed.setAccessibleName('Velocit'+chr(224)+' verticale minima per il volo bistadio')
        fields.addLayout(row(self.bistage_min_speed,label('m/s','bistageLabel'),None))
        explanation=label('All'+chr(8217)+'ingresso in COAST 1, la mediana dei primi 8 campioni validi deve superare questa soglia. Durante COAST 1 l'+chr(8217)+'inclinazione deve restare inferiore a 40'+chr(176)+'. Se uno dei controlli fallisce, il volo entra in ABORT: nessuna nuova attivazione; quelle gi'+chr(224)+' avviate vengono completate.','bistageHint')
        explanation.setWordWrap(True); fields.addWidget(explanation); parameters.addWidget(self.bistage_fields)
        self.bistage_fields.setEnabled(False)
        body.addWidget(self.bistage_parameters)
        self.machine_saved=label('','hint'); self.machine_saved.setWordWrap(True); self.machine_saved.hide(); body.addWidget(self.machine_saved)
        self.bistage_enabled.toggled.connect(self._edit_machine)
        self.bistage_min_speed.valueChanged.connect(self._edit_machine)
        self.airbrakes_save=button('Salva',lambda:self.controller.save_airbrakes(self.airbrakes_enabled.isChecked()),'saveAction')
        frame,body=setting_card('Aerofreni',self.airbrakes_save); box.addWidget(frame)
        self.airbrakes_enabled=QCheckBox('Abilita aerofreni'); body.addWidget(self.airbrakes_enabled)
        warning=label('Per usare gli aerofreni occorre implementare e caricare una legge di controllo nel firmware. La versione predefinita mantiene i servo chiusi allo 0%: attivare questa opzione da sola non li farà aprire.','controlWarning')
        warning.setWordWrap(True); body.addWidget(warning)
        hint=label('Canali dedicati e condizioni AND si impostano in Configurazione → Regole → Configura aerofreni.','hint')
        hint.setWordWrap(True); body.addWidget(hint)
        self.airbrakes_saved=label('','hint'); self.airbrakes_saved.setWordWrap(True); body.addWidget(self.airbrakes_saved)
        self.airbrakes_enabled.toggled.connect(lambda enabled:self.controller.edit_setting('airbrakes',enabled))
        actions=[('Ripristino dati di fabbrica','Elimina regole, posizioni servo e orientamento.',
                  'I voli registrati vengono conservati.','Ripristina',False),
                 ('Formatta memoria','Elimina tutti i voli registrati e libera lo spazio.',
                  'La configurazione viene conservata.','Formatta',True)]
        for title,description,preserved,caption,format_memory in actions:
            action=button(caption,lambda mode=format_memory:self.confirm_maintenance(mode),'settingsDestructive' if format_memory else 'settingsAction')
            frame,body=setting_card(title,action); box.addWidget(frame)
            text=label(description,'settingsDescription'); text.setWordWrap(True); body.addWidget(text)
            text=label(preserved,'settingsPreserved'); text.setWordWrap(True); body.addWidget(text)
            if format_memory: self.format_memory_button=action
            else: self.factory_reset_button=action
        box.addStretch()

    def _update_machine_sequence(self,bistage):
        states=('PAD','BOOST 1','COAST 1','BOOST 2','COAST 2','DESCENT','LANDED') if bistage else ('PAD','BOOST','COAST','DESCENT','LANDED')
        if bistage:
            cells=[]
            for i,state in enumerate(states):
                if i: cells.append('<td align="center">'+chr(8594)+'</td>')
                cells.append('<td align="center">'+state+'</td>')
            branch='<tr><td colspan="4"></td><td align="center" style="color: #000000">'+chr(8595)+'<br><span style="color: #c43d3d"><b>ABORT</b></span></td><td colspan="8"></td></tr>'
            self.flight_state_sequence.setText('<table cellspacing="6"><tr>'+''.join(cells)+'</tr>'+branch+'</table>')
        else: self.flight_state_sequence.setText((' '+chr(8594)+' ').join(states))

    def render(self,s):
        self.orientation_panel.render(s)
        self._render_machine(s)
        self._render_airbrakes(s)
        maintenance=s.editable and not s.sim.get('active') and not s.status.get('sensor_hil_active')
        self.factory_reset_button.setEnabled(maintenance)
        self.format_memory_button.setEnabled(maintenance and not s.dirty and not s.staged)

    def _edit_machine(self):
        self.controller.edit_setting('machine',(self.bistage_enabled.isChecked(),self.bistage_min_speed.value()))

    def _render_airbrakes(self,s):
        saved=s.airbrakes if s.connected else {}
        local=s.local_setting('airbrakes')
        observed=(s.connected,local,saved.get('ready'),saved.get('supported'))
        if observed!=self._rendered_airbrakes:
            self._rendered_airbrakes=observed
            self.airbrakes_enabled.blockSignals(True); self.airbrakes_enabled.setChecked(local); self.airbrakes_enabled.blockSignals(False)
        editable=s.editable and not s.dirty and not s.staged and not s.sim.get('active') and not s.status.get('sensor_hil_active') and saved.get('supported') is True
        self.airbrakes_enabled.setEnabled(editable)
        changed=not saved.get('ready',True) or s.setting_changed('airbrakes')
        self.airbrakes_save.setEnabled(editable and changed)
        self.airbrakes_save.setText('Salvato' if s.connected and saved.get('supported') and not changed else 'Salva')
        text='Collega la scheda per leggere questa impostazione.'
        if s.connected:
            if saved.get('supported') is False: text='Aggiorna il firmware per abilitare gli aerofreni.'
            elif not saved.get('ready',True): text='Impostazione non valida: salvala nuovamente.'
            elif 'enabled' in saved:
                text='Salvati sulla scheda: '+('abilitati' if saved['enabled'] else 'disabilitati')
                if changed: text+=' · Modifica non salvata: premi Salva.'
        self.airbrakes_saved.setText(text)

    def _render_machine(self,s):
        machine=s.machine if s.connected else {}
        local=s.local_setting('machine')
        observed=(s.connected,local,machine.get('supported'))
        if observed!=self._rendered_machine:
            self._rendered_machine=observed
            self.bistage_enabled.blockSignals(True); self.bistage_min_speed.blockSignals(True)
            self.bistage_enabled.setChecked(local[0]); self.bistage_min_speed.setValue(local[1])
            self.bistage_enabled.blockSignals(False); self.bistage_min_speed.blockSignals(False)
            self._update_machine_sequence(local[0])
        editable=s.editable and not s.dirty and not s.staged and not s.sim.get('active') and not s.status.get('sensor_hil_active') and machine.get('supported') is True
        self.bistage_parameters.setEnabled(editable)
        self.bistage_enabled.setEnabled(editable)
        self.bistage_fields.setEnabled(editable and self.bistage_enabled.isChecked())
        active=self.bistage_enabled.isChecked()
        if self.bistage_parameters.property('active')!=active:
            self.bistage_parameters.setProperty('active',active)
            self.bistage_parameters.style().unpolish(self.bistage_parameters); self.bistage_parameters.style().polish(self.bistage_parameters)
        changed=not machine.get('machine_ready',True) or s.setting_changed('machine')
        self.machine_save.setEnabled(editable and changed)
        self.machine_save.setText('Salvato' if s.connected and machine.get('supported') is True and not changed else 'Salva')
        text=''
        if s.connected:
            if machine.get('supported') is False: text='Aggiorna il firmware per impostare il bistadio.'
            elif not machine.get('machine_ready',True): text='Macchina a stati da salvare: impostazione non valida sulla scheda.'
        self.machine_saved.setText(text)
        self.machine_saved.setVisible(bool(text))

    def confirm_maintenance(self,format_memory):
        title='Formatta memoria' if format_memory else 'Ripristina dati di fabbrica'
        text=('Eliminare definitivamente tutti i voli registrati? La configurazione viene conservata.' if format_memory else
              'Eliminare regole e posizioni dei servo e azzerare l’orientamento, comprese le modifiche non applicate? La memoria dei voli non viene cancellata.')
        if QMessageBox.question(self,title,text,QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes:
            self.controller.maintenance(format_memory)

