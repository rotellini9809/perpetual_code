"""Desktop views: acknowledged device state, local editing and flight inspection."""
from datetime import datetime
from PySide6.QtCore import Qt,QTimer,QSettings
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (QMainWindow,QWidget,QFrame,QVBoxLayout,QHBoxLayout,QGridLayout,QStackedWidget,
    QComboBox,QLineEdit,QScrollArea,
    QPlainTextEdit,QCheckBox,QProgressBar,QMessageBox,QHeaderView,QInputDialog,QSizePolicy)
from .controller import Controller,REASONS
from .arm_button import ArmButton
from .flight_data import phase_name
from .configuration import rule_label
from .airbrakes import control_text
from .configuration_page import ConfigurationPage
from .settings_page import SettingsPage
from .sensor_details import SensorDetails
from .recorded_flights_page import RecordedFlightsPage
from .widgets import ASSETS,Metric,label,button,card,row,table,fill_table,create_page


class MainWindow(QMainWindow):
    def __init__(self,controller=None):
        super().__init__(); self.controller=controller or Controller(); self.setWindowTitle('Perpetual · Flight Console')
        self.resize(1380,900); self.setMinimumSize(1120,730); self.setStyleSheet((ASSETS/'style.qss').read_text(encoding='utf-8'))
        self.settings=QSettings('Perpetual','Flight Console')
        self._closing=False; self._allow_close=False; self._skip_unsaved_prompt=False
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
        for page in (self._dashboard(),self.configuration_page,self._simulation(),self.recorded_flights_page,self.settings_page,self._console()):
            scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(page); self.pages.addWidget(scroll)
        self.feedback=label('Collega la scheda per iniziare.','hint'); self.feedback.setWordWrap(True); content.addWidget(self.feedback)
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
        for i,text in enumerate(['Dashboard','Configurazione','Test e simulazione','Voli registrati','Impostazioni','Console']):
            nav=button(text,lambda i=i:self.navigate(i),'nav'); nav.setCheckable(True); self.nav.append(nav); layout.addWidget(nav)
        self.nav[0].setChecked(True); layout.addStretch()
        for text,kind in [('Perpetual · Avionics','hint'),('GROUND STATION','eyebrow'),('Made by rotellini9809','credit')]:
            footer=label(text,kind); footer.setAlignment(Qt.AlignCenter); layout.addWidget(footer)

    def _connection_bar(self,layout):
        bar=QFrame(); bar.setObjectName('connectionBar'); box=QVBoxLayout(bar); box.setContentsMargins(26,14,26,12); box.setSpacing(8)
        self.link=label('●  Disconnesso','pill'); self.link.setMaximumWidth(180)
        self.transport=label('USB / Bluetooth · —','hint'); self.transport.setMinimumWidth(180)
        self.manual_port=QCheckBox('Porta manuale'); self.manual_port.setToolTip('Opzione avanzata: scegli una porta anziché usare la ricerca USB/Bluetooth')
        self.ports=QComboBox(); self.ports.setMinimumWidth(190); self.ports.setMaximumWidth(340)
        self.refresh_ports=button('Aggiorna',self.controller.scan_ports); self.refresh_ports.setToolTip('Aggiorna le porte disponibili')
        self.connect_button=button('Connetti',self.toggle_connection,'primary')
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

    def _dashboard(self):
        page,box=create_page('Preparazione del volo','Configurazione, sensori e controlli richiesti prima dell’armamento.')
        self.power_badge=label('POWER SAVE ACTIVE','powerSaveBadge')
        self.power_badge.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed); self.power_badge.hide()
        box.insertLayout(0,row(box.takeAt(0).widget(),None,self.power_badge))
        self.config_card,config=card()
        config.setContentsMargins(22,16,22,16); config.setSpacing(8)
        self.config_value=label('—','metric'); self.config_detail=label('Nessuna lettura disponibile')
        self.config_detail.setWordWrap(True); self.config_detail.setTextFormat(Qt.PlainText)
        self.bistage_badge=label('Bistadio','featureBadge'); self.airbrake_badge=label('Aerofreni','featureBadge')
        for badge in (self.bistage_badge,self.airbrake_badge): badge.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed)
        summary=QVBoxLayout(); summary.setSpacing(6)
        summary.addLayout(row(self.config_value,None,self.bistage_badge,self.airbrake_badge)); summary.addWidget(self.config_detail)
        config.addLayout(summary); config.addStretch(1)
        self.arm_button=ArmButton(self.toggle_arming)
        config.addLayout(row(None,self.arm_button,None))
        hardware,hw=card('Sensori')
        self.imu_label=label('—','sensorStatus'); self.baro_label=label('—','sensorStatus')
        for name,widget in (('IMU',self.imu_label),('Barometro',self.baro_label)):
            widget.setWordWrap(True); hw.addLayout(row(label(name+' ·'),widget,None))
        self.sensor_refresh=button('Leggi dettagli sensori',self.show_sensor_details); hw.addWidget(self.sensor_refresh)
        overview=QHBoxLayout(); overview.setSpacing(18)
        overview.addWidget(self.config_card,3); overview.addWidget(hardware,2); box.addLayout(overview)
        columns=QHBoxLayout(); columns.setSpacing(18); box.addLayout(columns,1)
        readiness,body=card(); columns.addWidget(readiness,3)
        self.readiness_title=label('Controlli preflight','cardTitle'); self.preflight_button=button('Ricontrolla',lambda:self.controller.command('preflight','Verifica preflight'))
        self.config_issue_button=button('Apri configurazione',self.show_configuration_issue,'inlineLink'); self.config_issue_button.hide()
        self.config_issue_button.setCursor(Qt.PointingHandCursor)
        heading=QVBoxLayout(); heading.setSpacing(6); heading.addWidget(self.readiness_title); heading.addWidget(self.config_issue_button,0,Qt.AlignLeft)
        controls=QHBoxLayout(); controls.addLayout(heading,1); controls.addWidget(self.preflight_button,0,Qt.AlignTop); body.addLayout(controls)
        self.readiness_table=table(['DA COMPLETARE']); self.readiness_table.horizontalHeader().hide()
        self.readiness_table.setSelectionMode(self.readiness_table.SelectionMode.NoSelection)
        self.readiness_table.setWordWrap(True); self.readiness_table.setTextElideMode(Qt.ElideNone)
        self.readiness_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        body.addWidget(self.readiness_table,1)
        actuators,outputs=card('Controllo pyro'); columns.addWidget(actuators,2)
        self.pyro_status=label('','hint'); self.pyro_status.setWordWrap(True); outputs.addWidget(self.pyro_status)
        self.pyro_rows=[]; self.pyro_states=[]
        for i in range(4):
            entry=QWidget(); layout=row(label(f'PYRO {i+1} \u00b7','actuatorName'),label('','actuatorStatus'),None)
            layout.setContentsMargins(0,0,0,0); entry.setLayout(layout); outputs.addWidget(entry)
            self.pyro_rows.append(entry); self.pyro_states.append(layout.itemAt(1).widget())
        self.check_pyros_button=button('Verifica continuità pyro',lambda:self.controller.command('pyro continuity','Verifica continuità pyro configurati'))
        self.check_pyros_button.setToolTip('Legge la continuità dei pyro assegnati alle regole della configurazione applicata.')
        outputs.addWidget(self.check_pyros_button)
        divider=QFrame(); divider.setObjectName('sectionDivider'); divider.setFixedHeight(1); outputs.addWidget(divider)
        outputs.addWidget(label('Preparazione servo','cardTitle'))
        self.servo_selection=[]; self.servo_states=[]
        for i in range(4):
            check=QCheckBox(f'SERVO {i+1} \u00b7'); check.setObjectName('actuatorName'); self.servo_selection.append(check)
            state=label('','actuatorStatus'); self.servo_states.append(state)
            outputs.addLayout(row(check,state,None))
            check.toggled.connect(lambda _:self.render())
        self.airbrake_summary=label('','hint'); self.airbrake_summary.setWordWrap(True); outputs.addWidget(self.airbrake_summary)
        self.servo_empty=label('Nessun servo configurato.','hint'); self.servo_empty.setWordWrap(True); outputs.addWidget(self.servo_empty)
        self.prepare_servos_button=button('Prepara selezionati',self.prepare_selected_servos,'primary')
        self.prepare_servos_button.setProperty('servoPreparation',True)
        self.release_servos_button=button('Rilascia servo',lambda:self.controller.command('servo release','Rilascio servo'),'releaseServo')
        outputs.addLayout(row(self.prepare_servos_button,self.release_servos_button))
        hint=label('Porta i servo alla posizione iniziale e mantiene il PWM attivo. Premi Rilascia servo per interrompere il PWM.','hint'); hint.setWordWrap(True); outputs.addWidget(hint)
        outputs.addStretch()
        return page

    def show_configuration_issue(self):
        error=self.controller.state.status.get('config_error')
        settings=error in ('STATE_MACHINE_INVALID','AIRBRAKES_SETTING_INVALID')
        self.navigate(4 if settings else 1)
        if settings:
            target=self.settings_page.bistage_parameters if error=='STATE_MACHINE_INVALID' else self.settings_page.airbrakes_enabled
            self.pages.widget(4).ensureWidgetVisible(target)

    def toggle_arming(self):
        arming=self.controller.state.status.get('arming_state')
        if arming in ('armed','arming'): self.controller.command('disarm','Richiesta di disarmo')
        elif arming=='disarmed': self.controller.command('arm','Richiesta di armamento')

    def prepare_selected_servos(self):
        s=self.controller.state
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and c.isEnabled() and not c.isHidden()]
        if not selected or not s.base: return
        missing=[i for i in selected if not s.base.servos[i-1]['calibrated']]
        if missing:
            self.navigate(1)
            self.configuration_page.show_servo(missing[0],edit=bool(s.local and not s.local.servos[missing[0]-1]['calibrated']))
        else:
            prepared=s.preflight.get('prepared_servo_mask',0)
            pending=[channel for channel in selected if not prepared & (1<<(channel-1))]
            if pending: self.controller.prepare_servos(pending)

    def show_sensor_details(self):
        if self._sensor_details is None: self._sensor_details=SensorDetails(self.controller,self)
        self._sensor_details.show(); self._sensor_details.raise_(); self._sensor_details.activateWindow()

    def _simulation(self):
        page,box=create_page('Test e simulazione','Simula il volo con la macchina a stati salvata sulla scheda: Hoppino normale oppure test bistadio con due BOOST.')
        setup,body=card(); box.addWidget(setup)
        self.sim_physical=QCheckBox('Uscite fisiche'); self.sim_physical.setToolTip('Durante la simulazione comanda realmente servo e pyro')
        self.sim_record=QCheckBox('Registra in memoria'); self.sim_record.setToolTip('Richiede preparazione della memoria; i voli esistenti vengono mantenuti.')
        self.sim_start=button('Avvia simulazione',self.start_simulation,'primary')
        self.sim_stop=button('Arresta simulazione',lambda:self.controller.command('sim stop','Arresto simulazione'),'danger')
        self.sim_preflight_notice=QWidget(); self.sim_preflight_notice.hide()
        self.sim_preflight_button=button('Vai ai controlli',self.review_simulation_preflight,'inlineLink')
        self.sim_preflight_button.setCursor(Qt.PointingHandCursor)
        self.sim_preflight_button.setToolTip('Apri la Dashboard ed esegui Ricontrolla.')
        error=label('Armamento bloccato','simPreflightError')
        notice=row(error,self.sim_preflight_button); notice.setSpacing(8)
        notice.setContentsMargins(16,0,0,0); self.sim_preflight_notice.setLayout(notice)
        self.sim_preflight_notice.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Preferred)
        controls=row(self.sim_record,self.sim_physical,self.sim_preflight_notice,None,self.sim_start,self.sim_stop)
        controls.setSpacing(8); body.addLayout(controls)
        self.sim_physical.toggled.connect(self.controller.clear_simulation_preflight)
        self.sim_blocker=label('','hint'); self.sim_blocker.setWordWrap(True); body.addWidget(self.sim_blocker)
        self.sim_progress=QProgressBar(); self.sim_progress.setRange(0,1); self.sim_progress.setValue(0); body.addWidget(self.sim_progress)
        self.sim_summary=label('Nessuna simulazione avviata.','hint'); body.addWidget(self.sim_summary)
        self.sim_phase=Metric('Fase calcolata'); self.sim_altitude=Metric('Quota','Metri rispetto al lancio'); self.sim_samples=Metric('Campioni','Dati elaborati dalla scheda')
        box.addLayout(row(self.sim_phase,self.sim_altitude,self.sim_samples))
        self.event_table=table(['TEMPO','EVENTO','CANALE / REGOLA']); self.event_table.setMinimumHeight(140); box.addWidget(self.event_table)
        return page

    def start_simulation(self):
        # Disabling a focused Start otherwise transfers focus/scroll to the event table.
        self.sim_start.clearFocus()
        self.controller.simulate(self.sim_record.isChecked(),self.sim_physical.isChecked())

    def review_simulation_preflight(self):
        self.navigate(0); self.preflight_button.click()

    def _console(self):
        page,box=create_page('Console','Richieste manuali e risposte della scheda.')
        self.command_input=QLineEdit(); self.command_input.setPlaceholderText('Es. status, config show, sim timing…'); self.command_input.returnPressed.connect(self.send_console)
        self.command_send=button('Invia',self.send_console,'primary')
        box.addLayout(row(self.command_input,self.command_send,button('Pulisci',lambda:self.console.clear())))
        self.console=QPlainTextEdit(); self.console.setReadOnly(True); self.console.document().setMaximumBlockCount(1200); box.addWidget(self.console,1)
        return page

    def navigate(self,index):
        if index!=0: self.arm_button.cancel_hold(immediate=True)
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
        chosen,ok=QInputDialog.getItem(self,'Scegli la scheda','Sono disponibili più porte compatibili. Quale vuoi usare?',labels,0,False)
        self.controller.choose_port(ports[labels.index(chosen)] if ok else None)

    def toggle_connection(self):
        if self.controller.connection_busy: return
        if self.controller.state.connected: self.controller.disconnect(); return
        if self.manual_port.isChecked():
            port=self.ports.currentData()
            if not port:
                self.feedback.setText('Nessuna porta disponibile. Premi Aggiorna oppure usa la connessione automatica.'); self.feedback.show(); return
            self.settings.setValue('lastPort',port.device); self.controller.connect_port(port)
        else:
            self.connect_automatically()

    def connect_automatically(self, quiet=False):
        if self._closing or self.controller.connection is not None or self.controller.connection_busy: return
        self.controller.connect_auto({t:self.settings.value('preferred_'+t,'') for t in ('usb','bluetooth')},quiet=quiet)

    def render_connection(self,snapshot):
        names={'disconnected':'Disconnesso','connecting':'Connessione…','connected':'Connesso','waiting':'Connesso',
               'disconnecting':'Disconnessione…','unresponsive':'Nessuna risposta','error':'Errore di connessione'}
        view=(snapshot.get('state'),snapshot.get('transport'),snapshot.get('port'),snapshot.get('identity'),self.controller.connection_busy,self.controller.connection is None,self._closing)
        if view==getattr(self,'_connection_view',None): return
        self._connection_view=view
        state=snapshot['state']; self.link.setText('●  '+names.get(state,state))
        good=state in ('connected','waiting'); color='#187f73' if good else '#ab6943' if state in ('connecting','disconnecting') else '#8190a2'
        self.link.setStyleSheet(f'color: {color}; background: {"#e7f3ef" if good else "#eef2f6"};')
        transport=snapshot.get('transport')
        self.transport.setText(({'usb':'USB','bluetooth':'Bluetooth','serial':'Seriale'}.get(transport,transport)+' · '+snapshot.get('port','')) if transport else 'USB / Bluetooth · —')
        busy=self.controller.connection_busy
        self.connect_button.setText('Disconnetti' if good and not busy else 'Connetti')
        self.connect_button.setEnabled(not busy and state!='disconnecting' and not self._closing)
        idle=self.controller.connection is None and not busy
        self.manual_port.setEnabled(idle); self.ports.setEnabled(idle); self.refresh_ports.setEnabled(idle)
        identity=snapshot.get('identity')
        if good and identity and transport in ('usb','bluetooth') and self.settings.value('preferred_'+transport)!=identity:
            self.settings.setValue('preferred_'+transport,identity)

    def _render_feedback(self):
        s=self.controller.state
        message=s.error or ('' if self.pages.currentIndex()==4 else s.message or ('' if s.connected else 'Collega la scheda per iniziare.'))
        self.feedback.setText(message); self.feedback.setVisible(bool(message))
        style='color: #b14d40;' if s.error else 'color: #7b8998;'
        if self.feedback.styleSheet()!=style: self.feedback.setStyleSheet(style)

    def render(self):
        s=self.controller.state; connected=s.connected; usable=connected and not s.foreground
        self.render_connection(s.connection)
        status=s.status; arming=status.get('arming_state','—'); sleeping=connected and status.get('power_mode')=='sleeping'
        self.power_badge.setVisible(sleeping)
        state=arming if connected else 'unknown'
        self.arming_state.setText({'disarmed':'DISARMED','armed':'ARMED','arming':'ARMING','closing':'CLOSING'}.get(state,'—'))
        if self.arming_state.property('state')!=state:
            self.arming_state.setProperty('state',state)
            self.arming_state.style().unpolish(self.arming_state); self.arming_state.style().polish(self.arming_state)
        self._render_feedback()
        self._render_dashboard(s)
        self.configuration_page.render(s)
        active=s.sim.get('active',False)
        blocker=self.controller.simulation_blocker()
        self.sim_start.setEnabled(not blocker); self.sim_start.setToolTip(blocker)
        self.sim_blocker.setText(blocker); self.sim_blocker.setVisible(bool(blocker) and not active)
        self.sim_stop.setEnabled(usable and s.owns_control and active); self.sim_record.setEnabled(usable and not active)
        self.sim_physical.setEnabled(usable and not active)
        if active: self.sim_physical.setChecked(s.sim.get('output_mode')=='PHYSICAL')
        self.sim_preflight_notice.setVisible(connected and self.sim_physical.isChecked() and not active and bool(s.sim_preflight))
        self.sim_preflight_button.setEnabled(usable and s.owns_control)
        self._render_sim(s)
        self.settings_page.render(s)
        self.command_input.setEnabled(usable); self.command_send.setEnabled(usable)
        self.recorded_flights_page.render(s)

    def _render_dashboard(self,s):
        connected=s.connected; usable=connected and not s.foreground
        status=s.status; arming=status.get('arming_state'); sleeping=connected and status.get('power_mode')=='sleeping'
        rules=s.base.rules if connected and s.base is not None else None
        count=len(rules) if rules is not None else None
        self.config_value.setText(f'{count} '+('regola applicata' if count==1 else 'regole applicate') if count is not None else '—')
        names=' · '.join(rule_label(rules,r['id']) for r in rules) if rules is not None else ''
        self.config_detail.setText(names or ('Nessuna regola di deployment' if rules is not None else 'Nessuna configurazione applicata letta' if connected else 'Collega la scheda per leggere la configurazione'))
        self.config_detail.setToolTip(names)
        for badge,saved,key,ready in ((self.bistage_badge,s.machine,'bistage','machine_ready'),(self.airbrake_badge,s.airbrakes,'enabled','ready')):
            known=connected and saved.get(ready,False)
            active=bool(known and saved.get(key))
            if badge.property('active')!=active:
                badge.setProperty('active',active); badge.style().unpolish(badge); badge.style().polish(badge)
            description='Abilitato sulla scheda' if active else 'Disabilitato sulla scheda' if known else 'Impostazione non letta dalla scheda'
            badge.setToolTip(description); badge.setAccessibleDescription(description)
        reasons=s.preflight.get('reasons',[]) if connected else []
        config_error=status.get('config_error')
        settings_error=config_error in ('STATE_MACHINE_INVALID','AIRBRAKES_SETTING_INVALID')
        self.config_issue_button.setVisible('CONFIG_INVALID' in reasons)
        self.config_issue_button.setText('Apri impostazioni' if settings_error else 'Apri configurazione')
        ready=connected and not sleeping and s.preflight.get('ready',False) and not reasons
        self.readiness_title.setText('Risparmio energetico' if sleeping else 'Pronto all’armamento' if ready and connected else 'Armamento bloccato' if reasons else 'Controlli preflight')
        rows=[]
        for reason in reasons:
            text=REASONS.get(reason,reason)
            if reason=='CONFIG_INVALID':
                detail={'Missing':'Nessuna configurazione salvata sulla scheda','Io':'Errore di accesso alla memoria della configurazione',
                        'Corrupt':'Dati della configurazione danneggiati','UnsupportedVersion':'Formato della configurazione non compatibile',
                        'InvalidConfiguration':'Le regole salvate non superano la verifica'}.get(config_error) or REASONS.get(config_error)
                if detail: text+=' · '+detail
            if reason=='CONTINUITY_FAULT': text+=' · '+', '.join('PYRO '+str(i+1) for i in range(4) if s.preflight.get('failed_pyro_mask',0)&(1<<i))
            if reason=='SERVOS_NOT_PREPARED': text+=' · '+', '.join('SERVO '+str(i+1) for i in range(4) if s.preflight.get('unprepared_servo_mask',0)&(1<<i))
            if reason=='SERVOS_NOT_CONFIGURED': text+=' · '+', '.join('SERVO '+str(i+1) for i in range(4) if s.preflight.get('unconfigured_servo_mask',0)&(1<<i))
            rows.append((text,))
        if not rows:
            message='La scheda si riattiva al prossimo comando.' if sleeping else 'Nessun impedimento segnalato dalla scheda.' if ready else 'Premi Ricontrolla per leggere i controlli.' if connected else 'Collega la scheda per leggere i controlli.'
            rows=[(message,)]
        fill_table(self.readiness_table,rows)
        for i,reason in enumerate(reasons):
            self.readiness_table.item(i,0).setToolTip(reason)
        self.preflight_button.setEnabled(usable)
        blocked=connected and bool(reasons)
        if self.arm_button.property('blocked')!=blocked:
            self.arm_button.setProperty('blocked',blocked)
            self.arm_button.style().unpolish(self.arm_button); self.arm_button.style().polish(self.arm_button)
        disarming=arming in ('armed','arming','closing')
        self.arm_button.set_disarming(disarming)
        self.arm_button.set_request_pending(bool(s.foreground))
        self.arm_button.setEnabled(usable and s.owns_control and (arming in ('armed','arming') if disarming else ready and not reasons and not s.dirty and not s.staged and arming=='disarmed'))
        self.arm_button.setToolTip('Disarma la scheda; la richiesta viene verificata dal firmware.' if disarming else
            'Preflight bloccato dalla scheda. Premi Ricontrolla dopo aver risolto i problemi.' if blocked else
            'Richiede preflight pronto dalla scheda. Tieni premuto per 1,5 s; la scheda ricontrolla prima di armare.')
        prepared=s.preflight.get('prepared_servo_mask',0)
        held_outputs=bool(prepared) or bool(status.get('outputs_active'))
        self.release_servos_button.setEnabled(usable and s.owns_control and arming=='disarmed' and held_outputs and
            not s.sim.get('active') and not status.get('sensor_hil_active') and not status.get('estimate_replay_active'))
        can_prepare=usable and s.owns_control and arming=='disarmed' and not s.dirty and not s.staged and bool(s.capabilities.get('physical_actions'))
        plan=s.base if connected else None
        assigned_servos=plan.servo_channels() if plan else set()
        air=s.airbrake_status if connected else {}; enabled=s.airbrakes.get('enabled',False)
        text='Disabilitati in Impostazioni' if not enabled else 'Nessun servo dedicato: configura gli aerofreni in Regole.' if not plan or not plan.airbrakes['channel_mask'] else ''
        if enabled and air.get('active'): text=control_text(air.get('flags',0),air.get('failed_conditions',0),plan.airbrakes['conditions'] if plan else [],plan.rules if plan else [])
        self.airbrake_summary.setText('Aerofreni '+chr(183)+' '+text if text else '')
        self.airbrake_summary.setVisible(bool(text) and (bool(plan and plan.airbrakes['channel_mask']) or enabled))
        for i,check in enumerate(self.servo_selection):
            assigned=i+1 in assigned_servos
            air_channel=bool(plan and plan.airbrake_channel(i+1))
            check.setText(f'SERVO {i+1} '+chr(183)+(' Aerofreni' if air_channel else ''))
            check.setVisible(assigned); check.setEnabled(can_prepare and assigned and (not air_channel or enabled))
            if not assigned and check.isChecked():
                check.blockSignals(True); check.setChecked(False); check.blockSignals(False)
            is_prepared=bool(prepared & (1<<i))
            state=self.servo_states[i]; state.setVisible(assigned)
            state.setText('disabilitato' if air_channel and not enabled else 'preparato' if is_prepared else 'non preparato')
            preparation='disabled' if air_channel and not enabled else 'prepared' if is_prepared else 'unprepared'
            if state.property('preparation')!=preparation:
                state.setProperty('preparation',preparation)
                state.style().unpolish(state); state.style().polish(state)
        self.servo_empty.setVisible(not assigned_servos)
        self.servo_empty.setText('Nessun servo assegnato alle regole.' if plan else 'Configurazione non caricata: premi Ricarica dalla scheda in Configurazione.' if connected else 'Collega la scheda e leggi la configurazione.')
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and c.isEnabled() and not c.isHidden()]
        missing=[i for i in selected if plan and not plan.servos[i-1]['calibrated']]
        self.prepare_servos_button.setText('Vai alla configurazione' if missing else 'Prepara selezionati')
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
        checked=connected and arming=='disarmed' and failed is not None and continuity.get('configured_pyro_mask')==sum(1<<(i-1) for i in pyros)
        self.pyro_status.setVisible(not pyros)
        self.pyro_status.setText('Nessun pyro assegnato alle regole.' if plan else 'Configurazione non caricata: premi Ricarica dalla scheda in Configurazione.' if connected else 'Collega la scheda per leggere la configurazione.')
        for i,(entry,state) in enumerate(zip(self.pyro_rows,self.pyro_states)):
            entry.setVisible(i+1 in pyros)
            present=checked and not (failed & (1<<i))
            state.setText(('continuità presente' if present else 'continuità assente') if checked else 'continuità da verificare')
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
            widget.setText('Disponibile' if available.get(key+'_ready') else 'Non disponibile' if observed else '—')
            availability='available' if available.get(key+'_ready') else 'unavailable' if observed else 'unknown'
            if widget.property('state')!=availability:
                widget.setProperty('state',availability)
                widget.style().unpolish(widget); widget.style().polish(widget)
            widget.setToolTip(f'ID: {sensors.get(key+"_id","—")} · errore: {sensors.get(key+"_error","—")}')
    def _render_sim(self,s):
        data=s.sim; stamp=data.get('t_ms',0)
        duration=data.get('duration_ms')
        self.sim_progress.setRange(0,max(1,duration or 0))
        self.sim_progress.setValue(stamp)
        total=f'{duration/1000:.2f}' if duration else '—'
        profile='Test bistadio' if data.get('source')=='bistage_synthetic' else 'Hoppino'
        self.sim_summary.setText(f'{profile} · {stamp/1000:.2f} / {total} s · {"In esecuzione" if data.get("active") else data.get("reason","In attesa")} · {data.get("starts",0)} attivazioni')
        self.sim_phase.set(phase_name(data.get('phase'),data.get('bistage',False)) if data.get('samples') else '—')
        self.sim_altitude.set(f'{data["z_est"]:.1f} m' if data.get('samples') and data.get('z_est') is not None else '—')
        self.sim_samples.set(f'{data["samples"]:,}'.replace(',',' ') if 'samples' in data else '—')
        events=[(t/1000,phase_name(p,data.get('bistage',False)),'') for p,t in s.events.get('phases',[])]
        rules=s.base.rules if s.base else []
        prefix='' if s.events.get('output_mode')=='PHYSICAL' else 'VIRTUAL '
        events += [(stamp/1000,prefix+kind.upper(),f'Canale {channel} · {rule_label(rules,rule)}') for rule,kind,channel,stamp in s.events.get('starts',[])]
        fill_table(self.event_table,[(f'{t:.3f} s',event,detail) for t,event,detail in sorted(events)])

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
                if QMessageBox.question(self,'Bozza non applicata','Chiudere e perdere le modifiche locali non applicate?')!=QMessageBox.StandardButton.Yes:
                    event.ignore(); return
        event.ignore()
        if not self._closing:
            self._closing=True; self.controller.disconnect(); self.close_timer.start()

    def close_from_terminal(self):
        self._skip_unsaved_prompt=True; self.close()

    def _finish_close(self):
        if self.controller.can_close():
            self.close_timer.stop(); self._allow_close=True; self.close()
