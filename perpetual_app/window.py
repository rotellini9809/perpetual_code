"""Main window: sidebar, connection bar, page navigation, Simulation and Console views."""
from datetime import datetime
from PySide6.QtCore import Qt,QTimer,QSettings
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (QMainWindow,QWidget,QFrame,QVBoxLayout,QHBoxLayout,QGridLayout,QStackedWidget,
    QComboBox,QLineEdit,QScrollArea,
    QPlainTextEdit,QCheckBox,QProgressBar,QMessageBox,QInputDialog,QSizePolicy)
from .controller import Controller,SIMULATION_START,DASHBOARD_PAGE,CONFIGURATION_PAGE,SETTINGS_PAGE
from .flight_data import phase_name
from .configuration import rule_label
from .configuration_page import ConfigurationPage
from .settings_page import SettingsPage
from .sensor_details import SensorDetails
from .recorded_flights_page import RecordedFlightsPage
from .dashboard_page import DashboardPage
from .widgets import ASSETS,Metric,label,button,card,row,table,fill_table,create_page
from .i18n import t,language,SETTING as LANGUAGE_SETTING


class MainWindow(QMainWindow):
    def __init__(self,controller=None):
        super().__init__(); self.controller=controller or Controller(); self.setWindowTitle('Perpetual · Flight Console')
        self.resize(1380,900); self.setMinimumSize(1120,730); self.setStyleSheet((ASSETS/'style.qss').read_text(encoding='utf-8'))
        self.settings=QSettings('Perpetual','Flight Console')
        self._closing=False; self._allow_close=False; self._skip_unsaved_prompt=False; self.restart_requested=False
        self._sensor_details=None
        root=QWidget(); self.setCentralWidget(root); outer=QHBoxLayout(root); outer.setContentsMargins(0,0,0,0); outer.setSpacing(0)
        self._sidebar(outer)
        main=QWidget(); main.setObjectName('canvas'); layout=QVBoxLayout(main); layout.setContentsMargins(0,0,0,0); layout.setSpacing(0); outer.addWidget(main,1)
        self._connection_bar(layout)
        body=QWidget(); content=QVBoxLayout(body); content.setContentsMargins(30,24,30,18); content.setSpacing(16); layout.addWidget(body,1)
        self.pages=QStackedWidget(); content.addWidget(self.pages,1)
        self.configuration_page=ConfigurationPage(self.controller)
        self.settings_page=SettingsPage(self.controller)
        self.recorded_flights_page=RecordedFlightsPage(self.controller)
        self.dashboard_page=DashboardPage(self.controller)
        self.dashboard_page.configuration_issue_requested.connect(self.show_configuration_issue)
        self.dashboard_page.imu_calibration_requested.connect(self.open_imu_calibration)
        self.dashboard_page.servo_editor_requested.connect(self.open_servo_editor)
        self.dashboard_page.sensor_details_requested.connect(self.show_sensor_details)
        self.settings_page.language_selected.connect(self.change_language)
        # Same order as the sidebar and the *_PAGE indices in controller.py.
        for page in (self.dashboard_page,self.configuration_page,self._simulation(),self.recorded_flights_page,self.settings_page,self._console()):
            scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(page); self.pages.addWidget(scroll)
        self.feedback=label(t('Connect the board to start.'),'hint'); self.feedback.setWordWrap(True); content.addWidget(self.feedback)
        self.controller.changed.connect(self.render)
        self.controller.connection_changed.connect(self.render_connection)
        self.controller.ports_changed.connect(self.ports_received)
        self.controller.port_choice.connect(self.choose_connection_port)
        self.controller.logged.connect(self.log)
        self.close_timer=QTimer(self); self.close_timer.setInterval(100); self.close_timer.timeout.connect(self._finish_close)
        self.render(); self.controller.scan_ports()

    def _sidebar(self,outer):
        side=QFrame(); side.setObjectName('sidebar'); side.setFixedWidth(206); outer.addWidget(side)
        layout=QVBoxLayout(side); layout.setContentsMargins(16,28,16,22); layout.setSpacing(8)
        logo=label(); logo.setAccessibleName('Perpetual'); logo.setAlignment(Qt.AlignCenter)
        artwork=QPixmap(str(ASSETS/'Clean Perpetual Infinity Logo on Navy-1.png')).scaled(
            348,282,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        artwork.setDevicePixelRatio(2); logo.setPixmap(artwork)
        logo.setFixedSize(174,141)
        layout.addWidget(logo)
        subtitle=label('FLIGHT CONSOLE','eyebrow'); subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle); layout.addSpacing(24)
        self.nav=[]
        for i,text in enumerate(['Dashboard',t('Configuration'),t('Test and simulation'),t('Recorded flights'),t('Settings'),'Console']):
            nav=button(text,lambda i=i:self.navigate(i),'nav'); nav.setCheckable(True); self.nav.append(nav); layout.addWidget(nav)
        self.nav[0].setChecked(True); layout.addStretch()
        for text,kind in [('Perpetual · Avionics','hint'),('GROUND STATION','eyebrow'),('Made by rotellini9809','credit')]:
            footer=label(text,kind); footer.setAlignment(Qt.AlignCenter); layout.addWidget(footer)

    def _connection_bar(self,layout):
        bar=QFrame(); bar.setObjectName('connectionBar'); box=QVBoxLayout(bar); box.setContentsMargins(26,14,26,12); box.setSpacing(8)
        self.link=label('●  '+t('Disconnected'),'pill'); self.link.setMaximumWidth(180)
        self.transport=label('USB / Bluetooth · —','hint'); self.transport.setMinimumWidth(180)
        self.manual_port=QCheckBox(t('Manual port')); self.manual_port.setToolTip(t('Advanced option: choose a port instead of the USB/Bluetooth search'))
        self.ports=QComboBox(); self.ports.setMinimumWidth(190); self.ports.setMaximumWidth(340)
        self.refresh_ports=button(t('Refresh'),self.controller.scan_ports); self.refresh_ports.setToolTip(t('Refresh the available ports'))
        self.connect_button=button(t('Connect'),self.toggle_connection,'primary')
        self.arming_state=label('—','armingState'); self.arming_state.setAlignment(Qt.AlignCenter)
        top=QGridLayout(); top.setColumnStretch(0,1); top.setColumnStretch(2,1)
        left=QWidget(); left.setLayout(row(self.link,self.transport,None)); left.layout().setContentsMargins(0,0,0,0)
        right=QWidget(); right.setLayout(row(None,self.manual_port,self.connect_button)); right.layout().setContentsMargins(0,0,0,0)
        top.addWidget(left,0,0); top.addWidget(self.arming_state,0,1); top.addWidget(right,0,2); box.addLayout(top)
        self.manual_controls=QWidget(); manual_box=QHBoxLayout(self.manual_controls); manual_box.setContentsMargins(0,0,0,0)
        manual_box.addStretch(); manual_box.addWidget(self.ports); manual_box.addWidget(self.refresh_ports)
        self.manual_controls.hide(); self.manual_port.toggled.connect(self.manual_controls.setVisible)
        box.addWidget(self.manual_controls)
        layout.addWidget(bar)

    def show_configuration_issue(self):
        error=self.controller.state.status.get('config_error')
        settings=error in ('STATE_MACHINE_INVALID','AIRBRAKES_SETTING_INVALID')
        self.navigate(SETTINGS_PAGE if settings else CONFIGURATION_PAGE)
        if settings:
            target=self.settings_page.bistage_parameters if error=='STATE_MACHINE_INVALID' else self.settings_page.airbrakes_enabled
            self.pages.widget(SETTINGS_PAGE).ensureWidgetVisible(target)

    def open_imu_calibration(self):
        self.navigate(CONFIGURATION_PAGE); self.configuration_page.tabs.setCurrentWidget(self.configuration_page.imu_page)

    def open_servo_editor(self,channel,edit):
        self.navigate(CONFIGURATION_PAGE); self.configuration_page.show_servo(channel,edit=edit)

    def show_sensor_details(self):
        if self._sensor_details is None: self._sensor_details=SensorDetails(self.controller,self)
        self._sensor_details.show(); self._sensor_details.raise_(); self._sensor_details.activateWindow()

    def _simulation(self):
        page,box=create_page(t('Test and simulation'),t('Simulate the flight with the state machine saved on the board: normal Hoppino or a bistage test with two BOOSTs.'))
        setup,body=card(); box.addWidget(setup)
        self.sim_physical=QCheckBox(t('Physical outputs')); self.sim_physical.setToolTip(t('During the simulation it really drives servos and pyros'))
        self.sim_record=QCheckBox(t('Record to memory')); self.sim_record.setToolTip(t('Requires memory preparation; existing flights are kept.'))
        self.sim_start=button(t('Start simulation'),self.start_simulation,'primary')
        self.sim_stop=button(t('Stop simulation'),lambda:self.controller.command('sim stop',t('Stopping simulation')),'danger')
        self.sim_preflight_notice=QWidget(); self.sim_preflight_notice.hide()
        self.sim_preflight_button=button(t('Go to checks'),self.review_simulation_preflight,'inlineLink')
        self.sim_preflight_button.setCursor(Qt.PointingHandCursor)
        self.sim_preflight_button.setToolTip(t('Open the Dashboard and run Recheck.'))
        error=label(t('Arming blocked'),'simPreflightError')
        notice=row(error,self.sim_preflight_button); notice.setSpacing(8)
        notice.setContentsMargins(16,0,0,0); self.sim_preflight_notice.setLayout(notice)
        self.sim_preflight_notice.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Preferred)
        controls=row(self.sim_record,self.sim_physical,self.sim_preflight_notice,None,self.sim_start,self.sim_stop)
        controls.setSpacing(8); body.addLayout(controls)
        self.sim_physical.toggled.connect(self.controller.clear_simulation_preflight)
        self.sim_blocker=label('','hint'); self.sim_blocker.setWordWrap(True); body.addWidget(self.sim_blocker)
        self.sim_progress=QProgressBar(); self.sim_progress.setRange(0,1); self.sim_progress.setValue(0); body.addWidget(self.sim_progress)
        self.sim_summary=label(t('No simulation started.'),'hint'); body.addWidget(self.sim_summary)
        self.sim_phase=Metric(t('Computed phase')); self.sim_altitude=Metric(t('Altitude'),t('Meters from the launch')); self.sim_samples=Metric(t('Samples'),t('Data processed by the board'))
        box.addLayout(row(self.sim_phase,self.sim_altitude,self.sim_samples))
        self.event_table=table([t('TIME'),t('EVENT'),t('CHANNEL / RULE')]); self.event_table.setMinimumHeight(140); box.addWidget(self.event_table)
        return page

    def start_simulation(self):
        # Disabling a focused Start otherwise transfers focus/scroll to the event table.
        self.sim_start.clearFocus()
        self.controller.simulate(self.sim_record.isChecked(),self.sim_physical.isChecked())

    def review_simulation_preflight(self):
        self.navigate(DASHBOARD_PAGE); self.dashboard_page.preflight_button.click()

    def _console(self):
        page,box=create_page('Console',t('Manual requests and board replies.'))
        self.command_input=QLineEdit(); self.command_input.setPlaceholderText(t('E.g. status, config show, sim timing…')); self.command_input.returnPressed.connect(self.send_console)
        self.command_send=button(t('Send'),self.send_console,'primary')
        box.addLayout(row(self.command_input,self.command_send,button(t('Clear'),lambda:self.console.clear())))
        self.console=QPlainTextEdit(); self.console.setReadOnly(True); self.console.document().setMaximumBlockCount(1200); box.addWidget(self.console,1)
        return page

    def navigate(self,index):
        if index!=DASHBOARD_PAGE: self.dashboard_page.arm_button.cancel_hold(immediate=True)
        self.pages.setCurrentIndex(index); self.controller.page=index
        for i,nav in enumerate(self.nav): nav.setChecked(i==index)
        self._render_feedback()

    def ports_received(self,ports):
        current=self.ports.currentData()
        previous=current.device if current else self.settings.value('lastPort','')
        self.ports.clear()
        for port in ports:
            self.ports.addItem(port.label,port)
            self.ports.setItemData(self.ports.count()-1,port.description,Qt.ItemDataRole.ToolTipRole)
        index=next((i for i,p in enumerate(ports) if p.device==previous),0)
        if ports: self.ports.setCurrentIndex(index)

    def choose_connection_port(self,ports):
        labels=[p.label for p in ports]
        chosen,ok=QInputDialog.getItem(self,t('Choose the board'),t('Several compatible ports are available. Which one do you want to use?'),labels,0,False)
        self.controller.choose_port(ports[labels.index(chosen)] if ok else None)

    def toggle_connection(self):
        if self.controller.connection_busy: return
        if self.controller.state.connected: self.controller.disconnect(); return
        if self.manual_port.isChecked():
            port=self.ports.currentData()
            if not port:
                self.feedback.setText(t('No port available. Press Refresh or use the automatic connection.')); self.feedback.show(); return
            self.settings.setValue('lastPort',port.device); self.controller.connect_port(port)
        else:
            self.connect_automatically()

    def connect_automatically(self, quiet=False):
        if self._closing or self.controller.connection is not None or self.controller.connection_busy: return
        self.controller.connect_auto({transport:self.settings.value('preferred_'+transport,'') for transport in ('usb','bluetooth')},quiet=quiet)

    def render_connection(self,snapshot):
        names={'disconnected':t('Disconnected'),'connecting':t('Connecting'),'connected':t('Connected'),'waiting':t('Connected'),
               'disconnecting':t('Disconnecting'),'unresponsive':t('No response'),'error':t('Connection error')}
        view=(snapshot.get('state'),snapshot.get('transport'),snapshot.get('port'),snapshot.get('identity'),self.controller.connection_busy,self.controller.connection is None,self._closing)
        if view==getattr(self,'_connection_view',None): return
        self._connection_view=view
        state=snapshot['state']; self.link.setText('●  '+names.get(state,state))
        link_up=state in ('connected','waiting'); color='#187f73' if link_up else '#ab6943' if state in ('connecting','disconnecting') else '#8190a2'
        self.link.setStyleSheet(f'color: {color}; background: {"#e7f3ef" if link_up else "#eef2f6"};')
        transport=snapshot.get('transport')
        self.transport.setText(({'usb':'USB','bluetooth':'Bluetooth','serial':'Seriale'}.get(transport,transport)+' · '+snapshot.get('port','')) if transport else 'USB / Bluetooth · —')
        busy=self.controller.connection_busy
        self.connect_button.setText(t('Disconnect') if link_up and not busy else t('Connect'))
        self.connect_button.setEnabled(not busy and state!='disconnecting' and not self._closing)
        idle=self.controller.connection is None and not busy
        self.manual_port.setEnabled(idle); self.ports.setEnabled(idle); self.refresh_ports.setEnabled(idle)
        identity=snapshot.get('identity')
        if link_up and identity and transport in ('usb','bluetooth') and self.settings.value('preferred_'+transport)!=identity:
            self.settings.setValue('preferred_'+transport,identity)

    def _render_feedback(self):
        state=self.controller.state
        message=state.error or ('' if self.pages.currentIndex()==SETTINGS_PAGE else state.message or ('' if state.connected else t('Connect the board to start.')))
        self.feedback.setText(message); self.feedback.setVisible(bool(message))
        style='color: #b14d40;' if state.error else 'color: #7b8998;'
        if self.feedback.styleSheet()!=style: self.feedback.setStyleSheet(style)

    def render(self):
        state=self.controller.state; connected=state.connected; usable=connected and not state.foreground
        self.render_connection(state.connection)
        arming=state.status.get('arming_state','—')
        arming_display=arming if connected else 'unknown'
        self.arming_state.setText({'disarmed':'DISARMED','armed':'ARMED','arming':'ARMING','preparing':t('Preparing memory'),'closing':'CLOSING'}.get(arming_display,'—'))
        if self.arming_state.property('state')!=arming_display:
            self.arming_state.setProperty('state',arming_display)
            self.arming_state.style().unpolish(self.arming_state); self.arming_state.style().polish(self.arming_state)
        self._render_feedback()
        self.dashboard_page.render(state)
        self.configuration_page.render(state)
        active=state.sim.get('active',False)
        blocker=self.controller.simulation_blocker()
        self.sim_start.setEnabled(not blocker); self.sim_start.setToolTip(blocker)
        self.sim_blocker.setText(blocker); self.sim_blocker.setVisible(bool(blocker) and not active)
        self.sim_stop.setEnabled(usable and state.owns_control and active); self.sim_record.setEnabled(usable and not active)
        self.sim_physical.setEnabled(usable and not active)
        if active: self.sim_physical.setChecked(state.sim.get('output_mode')=='PHYSICAL')
        self.sim_preflight_notice.setVisible(connected and self.sim_physical.isChecked() and not active and bool(state.sim_preflight))
        self.sim_preflight_button.setEnabled(usable and state.owns_control)
        self._render_sim(state)
        self.settings_page.render(state)
        self.command_input.setEnabled(usable); self.command_send.setEnabled(usable)
        self.recorded_flights_page.render(state)

    def _render_sim(self,state):
        sim=state.sim; elapsed_ms=sim.get('t_ms',0)
        starting=state.foreground_kind==SIMULATION_START
        if starting: elapsed_ms=0
        duration=sim.get('duration_ms')
        self.sim_progress.setRange(0,max(1,duration or 0))
        self.sim_progress.setValue(elapsed_ms)
        total=f'{duration/1000:.2f}' if duration else '—'
        profile=t('Bistage test') if sim.get('source')=='bistage_synthetic' else 'Hoppino'
        progress=t('Running') if sim.get('active') else sim.get('reason') or t('Waiting')
        self.sim_summary.setText(t('{profile} · {time} / {total} s · {state} · {starts} activations',profile=profile,time=f'{elapsed_ms/1000:.2f}',total=total,state=progress,starts=sim.get('starts',0)))
        if starting: self.sim_summary.setText(t('{operation}… Waiting for Perpetual to reply.',operation=state.foreground))
        self.sim_phase.set(phase_name(sim.get('phase'),sim.get('bistage',False)) if sim.get('samples') else '—')
        self.sim_altitude.set(f'{sim["z_est"]:.1f} m' if sim.get('samples') and sim.get('z_est') is not None else '—')
        self.sim_samples.set(f'{sim["samples"]:,}'.replace(',',' ') if 'samples' in sim else '—')
        events=[(at/1000,phase_name(phase,sim.get('bistage',False)),'') for phase,at in state.events.get('phases',[])]
        rules=state.base.rules if state.base else []
        prefix='' if state.events.get('output_mode')=='PHYSICAL' else 'VIRTUAL '
        events += [(start_ms/1000,prefix+kind.upper(),t('Channel {channel} · {rule}',channel=channel,rule=rule_label(rules,rule))) for rule,kind,channel,start_ms in state.events.get('starts',[])]
        fill_table(self.event_table,[(f'{at:.3f} s',event,detail) for at,event,detail in sorted(events)])

    def send_console(self):
        command=self.command_input.text().strip()
        if not command: return
        self.controller.command(command)

    def log(self,kind,text):
        mark={'tx':'→','rx':'←','error':'!'}.get(kind,'·')
        self.console.appendPlainText(f'{datetime.now():%H:%M:%S}  {mark}  {text}')

    def closeEvent(self,event):
        if self._allow_close:
            if self._sensor_details: self._sensor_details.close()
            self.controller.shutdown(); event.accept(); return
        if not self._closing and not self._skip_unsaved_prompt:
            self.settings_page.bistage_min_speed.interpretText()
            if self.controller.state.has_unsaved_changes:
                if QMessageBox.question(self,t('Unapplied draft'),t('Close and lose the unapplied local changes?'))!=QMessageBox.StandardButton.Yes:
                    self.restart_requested=False; event.ignore(); return
        event.ignore()
        if not self._closing:
            self._closing=True; self.controller.disconnect(); self.close_timer.start()

    def change_language(self,code):
        """Save the UI language; it applies at the next start, so offer to restart now."""
        self.settings.setValue(LANGUAGE_SETTING,code)
        if code==language(): return
        if QMessageBox.question(self,t('Restart required'),t('Restart Perpetual now to apply the new language?'))==QMessageBox.StandardButton.Yes:
            self.restart_requested=True; self.close() # main.py starts the new instance after a normal close.

    def close_from_terminal(self):
        self._skip_unsaved_prompt=True; self.close()

    def _finish_close(self):
        if self.controller.can_close():
            self.close_timer.stop(); self._allow_close=True; self.close()
