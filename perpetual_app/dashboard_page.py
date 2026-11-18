"""Dashboard: applied configuration, sensors, preflight checks and actuator preparation before arming."""
import time
from PySide6.QtCore import Qt,Signal
from PySide6.QtWidgets import QWidget,QFrame,QVBoxLayout,QHBoxLayout,QCheckBox,QHeaderView,QSizePolicy,QTableWidgetItem
from .controller import REASONS
from .arm_button import ArmButton
from .airbrakes import control_text
from .imu_calibration_page import STATES as IMU_STATES,CAUSES as IMU_CAUSES
from .widgets import label,button,card,row,table,fill_table,create_page
from .i18n import t

CONFIG_ERRORS={'Missing':'No configuration saved on the board','Io':'Configuration memory access error', # Shown with t().
               'Corrupt':'Configuration data corrupted','UnsupportedVersion':'Incompatible configuration format',
               'InvalidConfiguration':'The saved rules fail verification'}


class DashboardPage(QWidget):
    """Sends board commands itself; navigation to other pages is requested from the window through signals."""
    configuration_issue_requested=Signal()
    imu_calibration_requested=Signal()
    servo_editor_requested=Signal(int,bool) # Channel and whether to open its editor.
    sensor_details_requested=Signal()

    def __init__(self,controller):
        super().__init__(); self.controller=controller
        _,box=create_page(t('Flight preparation'),t('Configuration, sensors and checks required before arming.'),self)
        self.power_badge=label('POWER SAVE ACTIVE','powerSaveBadge')
        self.power_badge.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed); self.power_badge.hide()
        box.insertLayout(0,row(box.takeAt(0).widget(),None,self.power_badge))
        self.config_card,config=card()
        config.setContentsMargins(22,16,22,16); config.setSpacing(8)
        self.board_name=label('—','metric'); self.board_name.setTextFormat(Qt.PlainText)
        self.board_name.setWordWrap(True)
        self.config_value=label('—'); self.config_value.setWordWrap(True)
        self.bistage_badge=label(t('Bistage'),'featureBadge'); self.airbrake_badge=label(t('Airbrakes'),'featureBadge')
        for badge in (self.bistage_badge,self.airbrake_badge): badge.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed)
        summary=QVBoxLayout(); summary.setSpacing(6)
        summary.addLayout(row(self.board_name,None,self.bistage_badge,self.airbrake_badge))
        summary.addWidget(self.config_value)
        config.addLayout(summary); config.addStretch(1)
        self.arm_button=ArmButton(self.toggle_arming)
        config.addLayout(row(None,self.arm_button,None))
        hardware,hw=card(t('Sensors'))
        self.imu_label=label('—','sensorStatus'); self.baro_label=label('—','sensorStatus')
        for name,widget in (('IMU',self.imu_label),(t('Barometer'),self.baro_label)):
            widget.setWordWrap(True); hw.addLayout(row(label(name+' ·'),widget,None))
        self.sensor_refresh=button(t('Read sensor details'),self.sensor_details_requested.emit); hw.addWidget(self.sensor_refresh)
        overview=QHBoxLayout(); overview.setSpacing(18)
        overview.addWidget(self.config_card,3); overview.addWidget(hardware,2); box.addLayout(overview)
        columns=QHBoxLayout(); columns.setSpacing(18); box.addLayout(columns,1)
        readiness,body=card(); columns.addWidget(readiness,3)
        self.readiness_title=label(t('Preflight checks'),'cardTitle'); self.preflight_button=button(t('Recheck'),lambda:self.controller.command('preflight',t('Preflight check')))
        self.config_issue_button=button(t('Open configuration'),self.configuration_issue_requested.emit,'inlineLink'); self.config_issue_button.hide()
        self.config_issue_button.setCursor(Qt.PointingHandCursor)
        heading=QVBoxLayout(); heading.setSpacing(6); heading.addWidget(self.readiness_title); heading.addWidget(self.config_issue_button,0,Qt.AlignLeft)
        controls=QHBoxLayout(); controls.addLayout(heading,1); controls.addWidget(self.preflight_button,0,Qt.AlignTop); body.addLayout(controls)
        self.readiness_table=table([t('TO COMPLETE')]); self.readiness_table.horizontalHeader().hide()
        self.readiness_table.cellClicked.connect(self.open_preflight_item)
        self.readiness_table.setSelectionMode(self.readiness_table.SelectionMode.NoSelection)
        self.readiness_table.setWordWrap(True); self.readiness_table.setTextElideMode(Qt.ElideNone)
        self.readiness_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        body.addWidget(self.readiness_table,1)
        actuators,outputs=card(t('Pyro check')); columns.addWidget(actuators,2)
        self.pyro_status=label('','hint'); self.pyro_status.setWordWrap(True); outputs.addWidget(self.pyro_status)
        self.pyro_rows=[]; self.pyro_states=[]
        for i in range(4):
            entry=QWidget(); layout=row(label(f'PYRO {i+1} \u00b7','actuatorName'),label('','actuatorStatus'),None)
            layout.setContentsMargins(0,0,0,0); entry.setLayout(layout); outputs.addWidget(entry)
            self.pyro_rows.append(entry); self.pyro_states.append(layout.itemAt(1).widget())
        self.check_pyros_button=button(t('Check pyro continuity'),lambda:self.controller.command('pyro continuity',t('Checking continuity of the configured pyros')))
        self.check_pyros_button.setToolTip(t('Reads the continuity of the pyros assigned to the rules of the applied configuration.'))
        outputs.addWidget(self.check_pyros_button)
        divider=QFrame(); divider.setObjectName('sectionDivider'); divider.setFixedHeight(1); outputs.addWidget(divider)
        outputs.addWidget(label(t('Servo preparation'),'cardTitle'))
        self.servo_selection=[]; self.servo_states=[]
        for i in range(4):
            check=QCheckBox(f'SERVO {i+1} \u00b7'); check.setObjectName('actuatorName'); self.servo_selection.append(check)
            state=label('','actuatorStatus'); self.servo_states.append(state)
            outputs.addLayout(row(check,state,None))
            check.toggled.connect(lambda _:self.render(self.controller.state))
        self.airbrake_summary=label('','hint'); self.airbrake_summary.setWordWrap(True); outputs.addWidget(self.airbrake_summary)
        self.servo_empty=label('Nessun servo configurato.','hint'); self.servo_empty.setWordWrap(True); outputs.addWidget(self.servo_empty)
        self.prepare_servos_button=button(t('Prepare selected'),self.prepare_selected_servos,'primary')
        self.prepare_servos_button.setProperty('servoPreparation',True)
        self.release_servos_button=button(t('Release servos'),lambda:self.controller.command('servo release',t('Releasing servos')),'releaseServo')
        outputs.addLayout(row(self.prepare_servos_button,self.release_servos_button))
        hint=label(t('Moves the servos to the initial position and keeps the PWM active. Press Release servos to stop the PWM.'),'hint'); hint.setWordWrap(True); outputs.addWidget(hint)
        outputs.addStretch()

    def open_preflight_item(self,row,column):
        item=self.readiness_table.item(row,0)
        if item and item.data(Qt.ItemDataRole.UserRole)=='imu': self.imu_calibration_requested.emit()

    def toggle_arming(self):
        arming=self.controller.state.status.get('arming_state')
        if arming in ('armed','arming','preparing'): self.controller.command('disarm',t('Disarm request'))
        elif arming=='disarmed': self.controller.command('arm',t('Arm request'))

    def prepare_selected_servos(self):
        s=self.controller.state
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and c.isEnabled() and not c.isHidden()]
        if not selected or not s.base: return
        missing=[i for i in selected if not s.base.servos[i-1]['calibrated']]
        if missing:
            self.servo_editor_requested.emit(missing[0],bool(s.local and not s.local.servos[missing[0]-1]['calibrated']))
        else:
            prepared=s.preflight.get('prepared_servo_mask',0)
            pending=[channel for channel in selected if not prepared & (1<<(channel-1))]
            if pending: self.controller.prepare_servos(pending)

    def render(self,s):
        connected=s.connected; usable=connected and not s.foreground
        status=s.status; arming=status.get('arming_state'); sleeping=connected and status.get('power_mode')=='sleeping'
        self.power_badge.setVisible(sleeping)
        self.board_name.setText((s.bluetooth.get('name') or '—') if connected else '—')
        rules=s.base.rules if connected and s.base is not None else None
        count=len(rules) if rules is not None else None
        self.config_value.setText((t('{count} rule applied',count=count) if count==1 else t('{count} rules applied',count=count)) if count is not None else
                                 t('No applied configuration read') if connected else t('Connect the board to read the configuration'))
        for badge,saved,key,ready in ((self.bistage_badge,s.machine,'bistage','machine_ready'),(self.airbrake_badge,s.airbrakes,'enabled','ready')):
            known=connected and saved.get(ready,False)
            active=bool(known and saved.get(key))
            badge.setVisible(active)
            if badge.property('active')!=active:
                badge.setProperty('active',active); badge.style().unpolish(badge); badge.style().polish(badge)
            description=t('Enabled on the board') if active else t('Disabled on the board') if known else t('Setting not read from the board')
            badge.setToolTip(description); badge.setAccessibleDescription(description)
        reasons=s.preflight.get('reasons',[]) if connected else []
        config_error=status.get('config_error')
        settings_error=config_error in ('STATE_MACHINE_INVALID','AIRBRAKES_SETTING_INVALID')
        self.config_issue_button.setVisible('CONFIG_INVALID' in reasons)
        self.config_issue_button.setText(t('Open settings') if settings_error else t('Open configuration'))
        ready=connected and not sleeping and s.preflight.get('ready',False) and not reasons
        self.readiness_title.setText(t('Power saving') if sleeping else t('Ready to arm') if ready and connected else t('Arming blocked') if reasons else t('Preflight checks'))
        rows=[]
        for reason in reasons:
            text=t(REASONS.get(reason,reason))
            if reason=='CONFIG_INVALID':
                detail=CONFIG_ERRORS.get(config_error) or REASONS.get(config_error)
                if detail: text+=' · '+t(detail)
            if reason=='CONTINUITY_FAULT': text+=' · '+', '.join('PYRO '+str(i+1) for i in range(4) if s.preflight.get('failed_pyro_mask',0)&(1<<i))
            if reason=='SERVOS_NOT_PREPARED': text+=' · '+', '.join('SERVO '+str(i+1) for i in range(4) if s.preflight.get('unprepared_servo_mask',0)&(1<<i))
            if reason=='SERVOS_NOT_CONFIGURED': text+=' · '+', '.join('SERVO '+str(i+1) for i in range(4) if s.preflight.get('unconfigured_servo_mask',0)&(1<<i))
            rows.append((text,))
        if not rows:
            message=t('The board wakes up at the next command.') if sleeping else t('No blocker reported by the board.') if ready else t('Press Recheck to read the checks.') if connected else t('Connect the board to read the checks.')
            rows=[(message,)]
        fill_table(self.readiness_table,rows)
        for i,reason in enumerate(reasons):
            self.readiness_table.item(i,0).setToolTip(reason)
            if reason in ('IMU_CALIBRATION_MISSING','IMU_VERIFICATION_REQUIRED'):
                self.readiness_table.item(i,0).setData(Qt.ItemDataRole.UserRole,'imu')
        imu_state=s.preflight.get('imu_calibration_state') if connected else None
        if imu_state:
            # Keep the IMU state in the same preflight table, including successful approval.
            text=t('IMU calibration: {state}',state=t(IMU_STATES.get(imu_state,imu_state)))
            cause=t(IMU_CAUSES.get(s.preflight.get('imu_calibration_reason'),''))
            if cause: text+=' · '+cause
            text+=t(' · Open calibration')
            imu_row=next((i for i,r in enumerate(reasons) if r in ('IMU_CALIBRATION_MISSING','IMU_VERIFICATION_REQUIRED')),None)
            if imu_row is None:
                imu_row=self.readiness_table.rowCount(); self.readiness_table.insertRow(imu_row)
                self.readiness_table.setItem(imu_row,0,QTableWidgetItem(text))
            item=self.readiness_table.item(imu_row,0); item.setText(text); item.setData(Qt.ItemDataRole.UserRole,'imu')
            item.setToolTip(t('Open Configuration → IMU calibration'))
        self.preflight_button.setEnabled(usable)
        blocked=connected and bool(reasons)
        if self.arm_button.property('blocked')!=blocked:
            self.arm_button.setProperty('blocked',blocked)
            self.arm_button.style().unpolish(self.arm_button); self.arm_button.style().polish(self.arm_button)
        disarming=arming in ('armed','arming','preparing','closing')
        self.arm_button.set_disarming(disarming)
        self.arm_button.set_request_pending(bool(s.foreground))
        self.arm_button.setEnabled(usable and s.owns_control and (arming in ('armed','arming','preparing') if disarming else ready and not reasons and not s.dirty and not s.staged and arming=='disarmed'))
        self.arm_button.setToolTip(t('Hold for 1.5 s to disarm; the firmware verifies the request.') if disarming else
            t('Preflight blocked by the board. Press Recheck after solving the problems.') if blocked else
            t('Requires a ready preflight from the board. Hold for 1.5 s; the board checks again before arming.'))
        prepared=s.preflight.get('prepared_servo_mask',0)
        held_outputs=bool(prepared) or bool(status.get('outputs_active'))
        self.release_servos_button.setEnabled(usable and s.owns_control and arming=='disarmed' and held_outputs and
            not s.sim.get('active') and not status.get('sensor_hil_active') and not status.get('estimate_replay_active'))
        can_prepare=usable and s.owns_control and arming=='disarmed' and not s.dirty and not s.staged and bool(s.capabilities.get('physical_actions'))
        plan=s.base if connected else None
        assigned_servos=plan.servo_channels() if plan else set()
        air=s.airbrake_status if connected else {}; enabled=s.airbrakes.get('enabled',False)
        text=t('Disabled in Settings') if not enabled else t('No dedicated servo: configure the airbrakes in Rules.') if not plan or not plan.airbrakes['channel_mask'] else ''
        if enabled and air.get('active'): text=control_text(air.get('flags',0),air.get('failed_conditions',0),plan.airbrakes['conditions'] if plan else [],plan.rules if plan else [])
        self.airbrake_summary.setText(t('Airbrakes')+' · '+text if text else '')
        self.airbrake_summary.setVisible(bool(text) and (bool(plan and plan.airbrakes['channel_mask']) or enabled))
        for i,check in enumerate(self.servo_selection):
            assigned=i+1 in assigned_servos
            air_channel=bool(plan and plan.airbrake_channel(i+1))
            check.setText(f'SERVO {i+1} ·'+(' '+t('Airbrakes') if air_channel else ''))
            check.setVisible(assigned); check.setEnabled(can_prepare and assigned and (not air_channel or enabled))
            if not assigned and check.isChecked():
                check.blockSignals(True); check.setChecked(False); check.blockSignals(False)
            is_prepared=bool(prepared & (1<<i))
            state=self.servo_states[i]; state.setVisible(assigned)
            state.setText(t('disabled') if air_channel and not enabled else t('prepared') if is_prepared else t('not prepared'))
            preparation='disabled' if air_channel and not enabled else 'prepared' if is_prepared else 'unprepared'
            if state.property('preparation')!=preparation:
                state.setProperty('preparation',preparation)
                state.style().unpolish(state); state.style().polish(state)
        self.servo_empty.setVisible(not assigned_servos)
        self.servo_empty.setText(t('No servo assigned to the rules.') if plan else t('Configuration not loaded: press Reload from board in Configuration.') if connected else t('Connect the board and read the configuration.'))
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and c.isEnabled() and not c.isHidden()]
        missing=[i for i in selected if plan and not plan.servos[i-1]['calibrated']]
        self.prepare_servos_button.setText(t('Go to configuration') if missing else t('Prepare selected'))
        name='configureServo' if missing else 'primary'
        if self.prepare_servos_button.objectName()!=name:
            self.prepare_servos_button.setObjectName(name)
            self.prepare_servos_button.style().unpolish(self.prepare_servos_button); self.prepare_servos_button.style().polish(self.prepare_servos_button)
        pending=any(not prepared & (1<<(channel-1)) for channel in selected)
        self.prepare_servos_button.setEnabled(usable and s.editable if missing else can_prepare and pending)
        pyros=sorted({a['channel'] for r in plan.rules for a in r['outputs'] if a['kind']=='pyro'}) if plan else []
        self.check_pyros_button.setEnabled(usable and arming=='disarmed' and not s.dirty and not s.staged and bool(pyros))
        continuity=s.pyro_continuity
        failed=continuity.get('failed_pyro_mask')
        fresh=time.monotonic()<continuity.get('expires_at',0) # Manual checks expire after PYRO_CHECK_VALID_S.
        checked=connected and arming=='disarmed' and failed is not None and fresh and continuity.get('configured_pyro_mask')==sum(1<<(i-1) for i in pyros)
        # The board measured every rule pyro to accept the arm and again before flight.
        armed=connected and arming in ('arming','preparing','armed')
        self.pyro_status.setVisible(not pyros)
        self.pyro_status.setText(t('No pyro assigned to the rules.') if plan else t('Configuration not loaded: press Reload from board in Configuration.') if connected else t('Connect the board to read the configuration.'))
        for i,(entry,state) in enumerate(zip(self.pyro_rows,self.pyro_states)):
            entry.setVisible(i+1 in pyros)
            present=armed or (checked and not (failed & (1<<i)))
            state.setText(t('verified by the board at arming') if armed else (t('continuity present') if present else t('continuity missing')) if checked else t('continuity to check'))
            preparation='prepared' if present else 'unprepared'
            if state.property('preparation')!=preparation:
                state.setProperty('preparation',preparation)
                state.style().unpolish(state); state.style().polish(state)
        sensors=s.sensors if connected else {}
        available={key:status.get(key,sensors.get(key)) for key in ('imu_ready','baro_ready')} if connected else {}
        if sleeping: available=s.sensor_availability
        observed=connected and bool(status or sensors)
        self.sensor_refresh.setEnabled(usable and any(available.values()))
        for widget,key in ((self.imu_label,'imu'),(self.baro_label,'baro')):
            widget.setText(t('Available') if available.get(key+'_ready') else t('Not available') if observed else '—')
            availability='available' if available.get(key+'_ready') else 'unavailable' if observed else 'unknown'
            if widget.property('state')!=availability:
                widget.setProperty('state',availability)
                widget.style().unpolish(widget); widget.style().polish(widget)
            widget.setToolTip(t('ID: {id} · error: {error}',id=sensors.get(key+'_id','—'),error=sensors.get(key+'_error','—')))
