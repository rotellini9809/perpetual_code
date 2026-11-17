"""Device settings UI; persistence and board operations belong to Controller."""
from PySide6.QtCore import Qt,Signal,QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import QWidget,QFrame,QVBoxLayout,QCheckBox,QComboBox,QDoubleSpinBox,QMessageBox,QLineEdit
from .orientation import OrientationPanel
from .widgets import create_page,card,label,row,button
from .i18n import t,language,LANGUAGES


def setting_card(title,action=None):
    frame,body=card(); heading=label(title,'settingsTitle')
    if action:
        action.setMinimumWidth(132); action.setMinimumHeight(42); action.setCursor(Qt.PointingHandCursor)
        body.addLayout(row(heading,None,action))
    else: body.addWidget(heading)
    return frame,body


class SettingsPage(QWidget):
    language_selected=Signal(str) # A local app preference: the window saves it and offers a restart.

    def __init__(self,controller):
        super().__init__(); self.controller=controller; self._rendered_machine=None; self._rendered_airbrakes=None
        _,box=create_page(t('Settings'),t('Board orientation, state machine and device management. Operations are available with the board connected, disarmed and idle.'),self)
        frame,body=setting_card(t('Board orientation')); box.addWidget(frame)
        self.orientation_panel=OrientationPanel(self.controller); body.addWidget(self.orientation_panel)
        self.machine_save=button(t('Save'),lambda:self.controller.save_machine(self.bistage_enabled.isChecked(),self.bistage_min_speed.value()),'saveAction')
        frame,body=setting_card(t('State machine:'),self.machine_save); box.addWidget(frame)
        self.bistage_enabled=QCheckBox(t('Enable bistage flight'))
        self.flight_state_sequence=label('','flightStateSequence'); self.flight_state_sequence.setWordWrap(True); body.addWidget(self.flight_state_sequence)
        self.flight_state_hint=label(t('Launch above 3 g → BOOST; end of thrust → COAST. Start of descent → DESCENT; 2 seconds of stability → LANDED.'),'hint')
        self.flight_state_hint.setWordWrap(True); body.addWidget(self.flight_state_hint)
        self.bistage_enabled.toggled.connect(self._update_machine_sequence); self._update_machine_sequence(False)
        self.bistage_parameters=QFrame(); self.bistage_parameters.setObjectName('bistageParameters')
        parameters=QVBoxLayout(self.bistage_parameters); parameters.setContentsMargins(16,14,16,14); parameters.setSpacing(10)
        parameters.addWidget(self.bistage_enabled)
        self.bistage_fields=QWidget(); fields=QVBoxLayout(self.bistage_fields); fields.setContentsMargins(0,0,0,0); fields.setSpacing(10)
        fields.addWidget(label(t('Minimum vertical speed after BOOST 1'),'bistageLabel'))
        self.bistage_min_speed=QDoubleSpinBox(); self.bistage_min_speed.setRange(0,100000)
        self.bistage_min_speed.setDecimals(1); self.bistage_min_speed.setSingleStep(1); self.bistage_min_speed.setFixedWidth(160)
        self.bistage_min_speed.setKeyboardTracking(False)
        self.bistage_min_speed.setAccessibleName(t('Minimum vertical speed for bistage flight'))
        fields.addLayout(row(self.bistage_min_speed,label('m/s','bistageLabel'),None))
        explanation=label(t('On entering COAST 1, the median of the first 8 valid samples must exceed this threshold. During COAST 1 the tilt must stay below 40°. If either check fails, the flight enters ABORT: no new activations; those already started are completed.'),'bistageHint')
        explanation.setWordWrap(True); fields.addWidget(explanation); parameters.addWidget(self.bistage_fields)
        self.bistage_fields.setEnabled(False)
        body.addWidget(self.bistage_parameters)
        self.machine_saved=label('','hint'); self.machine_saved.setWordWrap(True); self.machine_saved.hide(); body.addWidget(self.machine_saved)
        self.bistage_enabled.toggled.connect(self._edit_machine)
        self.bistage_min_speed.valueChanged.connect(self._edit_machine)
        self.airbrakes_save=button(t('Save'),lambda:self.controller.save_airbrakes(self.airbrakes_enabled.isChecked()),'saveAction')
        frame,body=setting_card(t('Airbrakes'),self.airbrakes_save); box.addWidget(frame)
        self.airbrakes_enabled=QCheckBox(t('Enable airbrakes')); body.addWidget(self.airbrakes_enabled)
        warning=label(t('To use the airbrakes, a control law must be implemented and loaded in the firmware. The default version keeps the servos closed at 0%: enabling this option alone will not open them.'),'controlWarning')
        warning.setWordWrap(True); body.addWidget(warning)
        hint=label(t('Dedicated channels and AND conditions are set in Configuration → Rules → Configure airbrakes.'),'hint')
        hint.setWordWrap(True); body.addWidget(hint)
        self.airbrakes_saved=label('','hint'); self.airbrakes_saved.setWordWrap(True); body.addWidget(self.airbrakes_saved)
        self.airbrakes_enabled.toggled.connect(lambda enabled:self.controller.edit_setting('airbrakes',enabled))
        self.bluetooth_save=button(t('Rename'),self._rename_bluetooth,'saveAction')
        frame,body=setting_card(t('Bluetooth name'),self.bluetooth_save); box.addWidget(frame)
        self.bluetooth_current=label('','settingsDescription'); self.bluetooth_current.setWordWrap(True); body.addWidget(self.bluetooth_current)
        # Board names are "Perpetual" or "Perpetual-" plus up to 4 characters (module limit with "_BLE").
        self.bluetooth_suffix=QLineEdit(); self.bluetooth_suffix.setMaxLength(4); self.bluetooth_suffix.setFixedWidth(110)
        self.bluetooth_suffix.setValidator(QRegularExpressionValidator(QRegularExpression('[A-Za-z0-9-]{1,4}'),self.bluetooth_suffix))
        self.bluetooth_suffix.setPlaceholderText('A1'); self.bluetooth_suffix.setAccessibleName(t('Bluetooth name suffix'))
        self.bluetooth_suffix.textChanged.connect(lambda _:self._render_bluetooth(self.controller.state))
        self.bluetooth_default=button(t('Restore "Perpetual"'),lambda:self.controller.rename_bluetooth('default'))
        body.addLayout(row(label('Perpetual-','settingsDescription'),self.bluetooth_suffix,self.bluetooth_default,None))
        hint=label(t('Up to 4 letters, digits or -. The module restarts to apply the name: over Bluetooth the app reconnects by itself. The BLE name gets "_BLE". Windows shows the old name until you remove and pair the board again; the connection keeps working.'),'hint')
        hint.setWordWrap(True); body.addWidget(hint)
        frame,body=setting_card(t('Language')); box.addWidget(frame)
        self.language=QComboBox(); self.language.setFixedWidth(200)
        for code,name in LANGUAGES.items(): self.language.addItem(name,code)
        self.language.setCurrentIndex(self.language.findData(language()))
        self.language.currentIndexChanged.connect(lambda _:self.language_selected.emit(self.language.currentData()))
        body.addLayout(row(self.language,None))
        hint=label(t('The new language is applied when Perpetual restarts. Error messages stay in English.'),'hint'); hint.setWordWrap(True); body.addWidget(hint)
        actions=[(t('Factory reset'),t('Deletes rules, servo positions and orientation.'),
                  t('Recorded flights are kept.'),t('Reset'),False),
                 (t('Format memory'),t('Deletes all recorded flights and frees the space.'),
                  t('The configuration is kept.'),t('Format'),True)]
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
                if i: cells.append('<td align="center">→</td>')
                cells.append('<td align="center">'+state+'</td>')
            branch='<tr><td colspan="4"></td><td align="center" style="color: #000000">↓<br><span style="color: #c43d3d"><b>ABORT</b></span></td><td colspan="8"></td></tr>'
            self.flight_state_sequence.setText('<table cellspacing="6"><tr>'+''.join(cells)+'</tr>'+branch+'</table>')
        else: self.flight_state_sequence.setText((' → ').join(states))

    def render(self,s):
        self.orientation_panel.render(s)
        self._render_machine(s)
        self._render_airbrakes(s)
        self._render_bluetooth(s)
        maintenance=s.editable and not s.sim.get('active') and not s.status.get('sensor_hil_active')
        self.factory_reset_button.setEnabled(maintenance)
        self.format_memory_button.setEnabled(maintenance and not s.dirty and not s.staged)

    def _rename_bluetooth(self):
        if self.bluetooth_suffix.hasAcceptableInput(): self.controller.rename_bluetooth(self.bluetooth_suffix.text())

    def _render_bluetooth(self,s):
        info=s.bluetooth if s.connected else {}
        supported='name' in info; name=info.get('name','')
        if not s.connected: text=t('Connect the board to read the Bluetooth name.')
        elif not supported: text=t('Update the board firmware to rename Bluetooth.')
        elif s.bluetooth_restart: text=t('Restarting the Bluetooth module to apply "{name}"…',name=s.bluetooth_restart)
        elif name: text=t('Current name: {name} · BLE: {name}_BLE',name=name)
        else: text=t('Bluetooth module not ready.')
        self.bluetooth_current.setText(text)
        editable=(s.editable and supported and bool(info.get('ready')) and not s.bluetooth_restart and
                  not s.sim.get('active') and not s.status.get('sensor_hil_active'))
        self.bluetooth_suffix.setEnabled(editable)
        suffix=self.bluetooth_suffix.text()
        self.bluetooth_save.setEnabled(editable and self.bluetooth_suffix.hasAcceptableInput() and 'Perpetual-'+suffix!=name)
        self.bluetooth_default.setEnabled(editable and name!='Perpetual')

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
        self.airbrakes_save.setText(t('Saved') if s.connected and saved.get('supported') and not changed else t('Save'))
        text=t('Connect the board to read this setting.')
        if s.connected:
            if saved.get('supported') is False: text=t('Update the firmware to enable the airbrakes.')
            elif not saved.get('ready',True): text=t('Invalid setting: save it again.')
            elif 'enabled' in saved:
                text=t('Unsaved change: press Save.') if changed else ''
        self.airbrakes_saved.setText(text)
        self.airbrakes_saved.setVisible(bool(text))

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
        self.machine_save.setText(t('Saved') if s.connected and machine.get('supported') is True and not changed else t('Save'))
        text=''
        if s.connected:
            if machine.get('supported') is False: text=t('Update the firmware to set bistage flight.')
            elif not machine.get('machine_ready',True): text=t('State machine to save: invalid setting on the board.')
        self.machine_saved.setText(text)
        self.machine_saved.setVisible(bool(text))

    def confirm_maintenance(self,format_memory):
        title=t('Format memory') if format_memory else t('Reset to factory settings')
        text=(t('Permanently delete all recorded flights? The configuration is kept.') if format_memory else
              t('Delete rules and servo positions and reset the orientation, including unapplied changes? The flight memory is not erased.'))
        if QMessageBox.question(self,title,text,QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes:
            self.controller.maintenance(format_memory)

