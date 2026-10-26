"""Desktop views: acknowledged device state, local editing and flight inspection."""
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from math import ceil
from PySide6.QtCore import Qt,QTimer,QSettings
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (QMainWindow,QWidget,QFrame,QVBoxLayout,QHBoxLayout,QGridLayout,QStackedWidget,
    QComboBox,QDoubleSpinBox,QLineEdit,QScrollArea,QTreeWidget,QTreeWidgetItem,
    QPlainTextEdit,QCheckBox,QProgressBar,QFileDialog,QMessageBox,QHeaderView,QInputDialog,QSizePolicy)
from .controller import Controller,PHASES,REASONS
from .configuration import rule_label,decode_condition,condition_text
from .configuration_page import ConfigurationPage
from .orientation import OrientationPanel
from .sensor_details import SensorDetails
from .flight_plot import FlightPlot
from .widgets import ASSETS,Metric,label,button,card,row,table,fill_table

def phase_name(value,bistage=False): return (PHASES[value]+' 1' if bistage and value in (1,2) else PHASES[value]) if type(value) is int and 0<=value<len(PHASES) else '—'


class MainWindow(QMainWindow):
    def __init__(self,controller=None):
        super().__init__(); self.controller=controller or Controller(); self.setWindowTitle('Perpetual · Flight Console')
        self.resize(1380,900); self.setMinimumSize(1120,730); self.setStyleSheet((ASSETS/'style.qss').read_text(encoding='utf-8'))
        self.settings=QSettings('Perpetual','Flight Console')
        self._flight_path=None; self._flight_entries=None; self._rendered_machine=None
        self._closing=False; self._allow_close=False
        self._sensor_details=None
        root=QWidget(); self.setCentralWidget(root); outer=QHBoxLayout(root); outer.setContentsMargins(0,0,0,0); outer.setSpacing(0)
        self._sidebar(outer)
        main=QWidget(); main.setObjectName('canvas'); layout=QVBoxLayout(main); layout.setContentsMargins(0,0,0,0); layout.setSpacing(0); outer.addWidget(main,1)
        self._connection_bar(layout)
        body=QWidget(); content=QVBoxLayout(body); content.setContentsMargins(30,24,30,18); content.setSpacing(16); layout.addWidget(body,1)
        self.pages=QStackedWidget(); content.addWidget(self.pages,1)
        self.configuration_page=ConfigurationPage(self.controller)
        for page in (self._dashboard(),self.configuration_page,self._simulation(),self._flights(),self._additional_settings(),self._console()):
            scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(page); self.pages.addWidget(scroll)
        self.configuration_page.attach_footer(content)
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

    def page(self,title,subtitle):
        page=QWidget(); page.setObjectName('canvas'); box=QVBoxLayout(page); box.setContentsMargins(0,0,0,0); box.setSpacing(18)
        box.addWidget(label(title,'title')); sub=label(subtitle,'subtitle'); sub.setWordWrap(True); box.addWidget(sub)
        return page,box

    def _dashboard(self):
        page,box=self.page('Preparazione del volo','Configurazione, sensori e controlli richiesti prima dell’armamento.')
        self.power_badge=label('POWER SAVE ACTIVE','powerSaveBadge')
        self.power_badge.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Fixed); self.power_badge.hide()
        box.insertLayout(0,row(box.takeAt(0).widget(),None,self.power_badge))
        self.config_metric=Metric('Configurazione','Nessuna lettura disponibile')
        hardware,hw=card('Sensori')
        self.imu_label=label('—','sensorStatus'); self.baro_label=label('—','sensorStatus')
        for name,widget in (('IMU',self.imu_label),('Barometro',self.baro_label)):
            widget.setWordWrap(True); hw.addLayout(row(label(name+' ·'),widget,None))
        self.sensor_refresh=button('Leggi dettagli sensori',self.show_sensor_details); hw.addWidget(self.sensor_refresh)
        overview=QHBoxLayout(); overview.setSpacing(18)
        overview.addWidget(self.config_metric,3); overview.addWidget(hardware,2); box.addLayout(overview)
        columns=QHBoxLayout(); columns.setSpacing(18); box.addLayout(columns,1)
        readiness,body=card(); columns.addWidget(readiness,3)
        self.readiness_title=label('Controlli preflight','cardTitle'); self.preflight_button=button('Ricontrolla',lambda:self.controller.command('preflight','Verifica preflight'))
        body.addLayout(row(self.readiness_title,None,self.preflight_button))
        self.readiness_table=table(['DA COMPLETARE']); self.readiness_table.horizontalHeader().hide()
        self.readiness_table.setSelectionMode(self.readiness_table.SelectionMode.NoSelection)
        self.readiness_table.setWordWrap(True); self.readiness_table.setTextElideMode(Qt.ElideNone)
        self.readiness_table.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        body.addWidget(self.readiness_table,1)
        self.arm_button=button('ARM',self.toggle_arming,'armFlight')
        body.addLayout(row(self.arm_button,None))
        preparation=QVBoxLayout(); preparation.setSpacing(18); columns.addLayout(preparation,2)
        pyros,py=card('Controllo pyro'); preparation.addWidget(pyros)
        self.pyro_status=label('','hint'); self.pyro_status.setWordWrap(True); py.addWidget(self.pyro_status)
        self.pyro_rows=[]; self.pyro_states=[]
        for i in range(4):
            entry=QWidget(); layout=row(label(f'PYRO {i+1} \u00b7','actuatorName'),label('','actuatorStatus'),None)
            layout.setContentsMargins(0,0,0,0); entry.setLayout(layout); py.addWidget(entry)
            self.pyro_rows.append(entry); self.pyro_states.append(layout.itemAt(1).widget())
        self.check_pyros_button=button('Verifica continuità pyro',lambda:self.controller.command('pyro continuity','Verifica continuità pyro configurati'))
        self.check_pyros_button.setToolTip('Legge la continuità dei pyro assegnati alle regole della configurazione applicata.')
        py.addWidget(self.check_pyros_button)
        servos,sv=card('Preparazione servo'); preparation.addWidget(servos,1)
        self.servo_selection=[]; self.servo_states=[]
        for i in range(4):
            check=QCheckBox(f'SERVO {i+1} \u00b7'); check.setObjectName('actuatorName'); self.servo_selection.append(check)
            state=label('','actuatorStatus'); self.servo_states.append(state)
            sv.addLayout(row(check,state,None))
            check.toggled.connect(lambda _:self.render())
        self.servo_empty=label('Nessun servo configurato.','hint'); sv.addWidget(self.servo_empty)
        self.prepare_servos_button=button('Prepara selezionati',self.prepare_selected_servos,'primary')
        self.prepare_servos_button.setProperty('servoPreparation',True)
        self.release_servos_button=button('Rilascia servo',lambda:self.controller.command('servo release','Rilascio servo'),'releaseServo')
        sv.addLayout(row(self.prepare_servos_button,self.release_servos_button))
        hint=label('Porta i servo alla posizione iniziale e mantiene il PWM attivo. Premi Rilascia servo per interrompere il PWM.','hint'); hint.setWordWrap(True); sv.addWidget(hint)
        sv.addStretch()
        return page

    def toggle_arming(self):
        arming=self.controller.state.status.get('arming_state')
        if arming in ('armed','arming'): self.controller.command('disarm','Richiesta di disarmo')
        elif arming=='disarmed': self.controller.command('arm','Richiesta di armamento')

    def prepare_selected_servos(self):
        s=self.controller.state
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and not c.isHidden()]
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
        page,box=self.page('Test e simulazione','Simula il volo con la macchina a stati salvata sulla scheda: Hoppino normale oppure test bistadio con due BOOST.')
        setup,body=card(); box.addWidget(setup)
        self.sim_physical=QCheckBox('Uscite fisiche'); self.sim_physical.setToolTip('Durante la simulazione comanda realmente servo e pyro')
        self.sim_record=QCheckBox('Registra in memoria'); self.sim_record.setToolTip('Richiede preparazione della memoria; i voli esistenti vengono mantenuti.')
        self.sim_start=button('Avvia simulazione',self.start_simulation,'primary')
        self.sim_stop=button('Arresta simulazione',lambda:self.controller.command('sim stop','Arresto simulazione'),'danger')
        body.addLayout(row(self.sim_physical,self.sim_record,None,self.sim_start,self.sim_stop))
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

    def _flights(self):
        page,box=self.page('Voli registrati','Leggi i voli conservati sulla scheda oppure apri un CSV già scaricato.')
        self.list_flights_button=button('Leggi voli dalla scheda',self.controller.list_flights,'primary')
        self.download_button=button('Scarica selezionato',self.download_selected)
        self.open_flight_button=button('Apri CSV locale',self.open_local)
        self.download_feedback=QStackedWidget(); self.download_feedback.setMinimumWidth(140)
        self.download_feedback.addWidget(QWidget())
        self.download_progress=QProgressBar(); self.download_progress.setObjectName('downloadProgress')
        self.download_progress.setRange(0,100); self.download_progress.setValue(0); self.download_progress.setFormat('%p%')
        self.download_feedback.addWidget(self.download_progress)
        self.download_result=label('','hint'); self.download_result.setWordWrap(True)
        self.download_result.setTextInteractionFlags(Qt.TextSelectableByMouse|Qt.TextSelectableByKeyboard)
        self.download_result.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        self.download_feedback.addWidget(self.download_result)
        controls=row(self.list_flights_button,self.download_button); controls.addWidget(self.download_feedback,1)
        controls.addWidget(self.open_flight_button); box.addLayout(controls)
        frame,body=card(); self.flights_table=table(['ID VOLO','STATO','DATI CONFERMATI','SPAZIO RISERVATO']); self.flights_table.setMaximumHeight(205); self.flights_table.setMinimumHeight(130)
        body.addWidget(self.flights_table); box.addWidget(frame)
        self.flight_duration=Metric('Durata'); self.flight_peak=Metric('Quota massima'); self.flight_count=Metric('Campioni salvati')
        box.addLayout(row(self.flight_duration,self.flight_peak,self.flight_count))
        self.flight_actuators=QCheckBox('Attuatori'); self.flight_states=QCheckBox('Stati di volo')
        for check in (self.flight_actuators,self.flight_states):
            check.setChecked(True); check.toggled.connect(self._flight_overlays)
        box.addLayout(row(self.flight_actuators,self.flight_states,None))
        frame,body=card('Quota'); self.flight_plot=FlightPlot([(1,'Quota','m','#1d8b7e',0)]); body.addWidget(self.flight_plot)
        self.flight_file=label('Nessun volo aperto.','hint'); self.flight_file.setWordWrap(True); body.addWidget(self.flight_file); box.addWidget(frame,1)
        frame,body=card('Velocità e accelerazione verticali')
        self.flight_motion=FlightPlot([(2,'Velocità','m/s','#3c70b8',0),(3,'Accelerazione','m/s²','#c47539',1)])
        body.addWidget(self.flight_motion); box.addWidget(frame)
        frame,body=card('Velocità angolari BODY · °/s')
        self.flight_rates=FlightPlot([(7,'X','°/s','#3c70b8',0),(8,'Y','°/s','#1d8b7e',0),(9,'Z','°/s','#a36db1',0)])
        body.addWidget(self.flight_rates); box.addWidget(frame)
        frame,body=card('Orientamento Euler · riferimento X verso l’alto')
        self.flight_orientation=FlightPlot([(4,'Roll','°','#3c70b8',0),(5,'Pitch','°','#1d8b7e',0),(6,'Yaw','°','#a36db1',0)],angular=True)
        self.flight_orientation.setToolTip('Euler ZYX in un riferimento con +X verticale. Zero: muso +X verso l’alto con orientamento trasversale di riferimento. Non è un azzeramento sul primo campione.')
        self.flight_rates.setToolTip('Componenti del giroscopio attorno agli assi BODY X/Y/Z, convertite da rad/s a °/s. Non sono le derivate degli angoli Euler.')
        body.addWidget(self.flight_orientation)
        orientation_note=label('Angoli Euler ZYX convertiti nel riferimento con +X verso l’alto. Lo zero corrisponde al muso verticale con orientamento trasversale di riferimento.','hint')
        orientation_note.setWordWrap(True); orientation_note.setStyleSheet('color: #929da8; font-size: 12px;')
        body.addWidget(orientation_note); box.addWidget(frame)
        frame,body=card('Tensione batteria')
        self.flight_battery=FlightPlot([(10,'Tensione batteria','V','#b58a26',0)],include_zero=False)
        body.addWidget(self.flight_battery); box.addWidget(frame)
        self.flight_plots=(self.flight_plot,self.flight_motion,self.flight_rates,self.flight_orientation,self.flight_battery)
        self.flight_details=QTreeWidget(); self.flight_details.setHeaderLabels(['DATI DEL VOLO','VALORE']); self.flight_details.setMinimumHeight(270); self.flight_details.header().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.flight_details.setSelectionMode(QTreeWidget.SelectionMode.NoSelection); self.flight_details.setFocusPolicy(Qt.NoFocus)
        box.addWidget(self.flight_details)
        return page

    def _additional_settings(self):
        page,box=self.page('Impostazioni','Gestione della configurazione e della memoria dei voli.')
        frame,body=card(); box.addWidget(frame)
        self.orientation_panel=OrientationPanel(self.controller); body.addWidget(self.orientation_panel)
        frame,body=card('Macchina a stati:'); box.addWidget(frame)
        self.bistage_enabled=QCheckBox('Usa macchina a stati bistadio')
        self.flight_state_sequence=label('','flightStateSequence'); self.flight_state_sequence.setWordWrap(True); body.addWidget(self.flight_state_sequence)
        def update_sequence(bistage):
            states=('PAD','BOOST 1','COAST 1','BOOST 2','COAST 2','DESCENT','LANDED') if bistage else ('PAD','BOOST','COAST','DESCENT','LANDED')
            if bistage:
                cells=[]
                for i,state in enumerate(states):
                    if i: cells.append('<td align="center">'+chr(8594)+'</td>')
                    cells.append('<td align="center">'+state+'</td>')
                branch='<tr><td colspan="4"></td><td align="center" style="color: #000000">'+chr(8595)+'<br><span style="color: #c43d3d"><b>ABORT</b></span></td><td colspan="8"></td></tr>'
                self.flight_state_sequence.setText('<table cellspacing="6"><tr>'+''.join(cells)+'</tr>'+branch+'</table>')
            else: self.flight_state_sequence.setText((' '+chr(8594)+' ').join(states))
        self._update_machine_sequence=update_sequence
        self.bistage_enabled.toggled.connect(update_sequence); update_sequence(False)
        body.addWidget(self.bistage_enabled)
        self.bistage_parameters=QFrame(); self.bistage_parameters.setObjectName('bistageParameters')
        parameters=QVBoxLayout(self.bistage_parameters); parameters.setContentsMargins(16,14,16,14); parameters.setSpacing(10)
        parameters.addWidget(label('Velocit'+chr(224)+' verticale minima all'+chr(8217)+'uscita da BOOST 1','bistageLabel'))
        self.bistage_min_speed=QDoubleSpinBox(); self.bistage_min_speed.setRange(0,100000)
        self.bistage_min_speed.setDecimals(1); self.bistage_min_speed.setSingleStep(1); self.bistage_min_speed.setFixedWidth(160)
        self.bistage_min_speed.setKeyboardTracking(False)
        self.bistage_min_speed.setAccessibleName('Velocit'+chr(224)+' verticale minima per il volo bistadio')
        parameters.addLayout(row(self.bistage_min_speed,label('m/s','bistageLabel'),None))
        explanation=label('Mediana dei primi 8 campioni validi in COAST 1 e inclinazione continua < 40'+chr(176)+'. In ABORT le nuove attivazioni sono bloccate.','bistageHint')
        explanation.setWordWrap(True); parameters.addWidget(explanation)
        self.bistage_parameters.setEnabled(False); self.bistage_enabled.toggled.connect(self.bistage_parameters.setEnabled)
        body.addWidget(self.bistage_parameters)
        self.machine_saved=label('Collega la scheda per leggere la macchina a stati.','hint'); self.machine_saved.setWordWrap(True); body.addWidget(self.machine_saved)
        self.machine_save=button('Salva macchina a stati',lambda:self.controller.save_machine(self.bistage_enabled.isChecked(),self.bistage_min_speed.value()),'settingsAction')
        body.addLayout(row(self.machine_save,None))
        self.bistage_enabled.toggled.connect(lambda _:self.render_machine_settings(self.controller.state))
        self.bistage_min_speed.valueChanged.connect(lambda _:self.render_machine_settings(self.controller.state))
        box.addWidget(label('GESTIONE DISPOSITIVO','eyebrow'))
        frame,body=card(); body.setContentsMargins(0,0,0,0); body.setSpacing(0); box.addWidget(frame)
        actions=[('Configurazione','Ripristino dati di fabbrica','Elimina regole, posizioni servo e orientamento.',
                  'I voli registrati vengono conservati.','Ripristina',False),
                 ('Archiviazione','Formatta memoria','Elimina tutti i voli registrati e libera lo spazio.',
                  'La configurazione viene conservata.','Formatta',True)]
        for category,title,description,preserved,caption,format_memory in actions:
            if format_memory:
                divider=QFrame(); divider.setObjectName('settingsDivider'); divider.setFixedHeight(1); body.addWidget(divider)
            entry=QWidget(); entry.setObjectName('settingsEntry'); line=QHBoxLayout(entry)
            line.setContentsMargins(26,24,26,24); line.setSpacing(28)
            details=QVBoxLayout(); details.setSpacing(7)
            details.addWidget(label(category.upper(),'eyebrow')); details.addWidget(label(title,'settingsTitle'))
            text=label(description,'settingsDescription'); text.setWordWrap(True); details.addWidget(text)
            text=label(preserved,'settingsPreserved'); text.setWordWrap(True); details.addWidget(text)
            line.addLayout(details,1)
            action=button(caption,lambda mode=format_memory:self.confirm_maintenance(mode),'settingsDestructive' if format_memory else 'settingsAction')
            action.setFixedWidth(132); action.setMinimumHeight(42); action.setCursor(Qt.PointingHandCursor)
            line.addWidget(action,0,Qt.AlignVCenter); body.addWidget(entry)
            if format_memory: self.format_memory_button=action
            else: self.factory_reset_button=action
        note=label('Disponibili con scheda collegata, disarmata e senza attivit'+chr(224)+' in corso.','hint')
        note.setWordWrap(True); box.addWidget(note)
        box.addStretch()
        return page

    def render_machine_settings(self,s):
        machine=s.machine if s.connected else {}
        observed=(s.connected,machine.get('bistage'),machine.get('minimum_coast_velocity'),machine.get('supported'))
        if observed!=self._rendered_machine:
            self._rendered_machine=observed
            if 'bistage' in machine:
                self.bistage_enabled.blockSignals(True); self.bistage_min_speed.blockSignals(True)
                self.bistage_enabled.setChecked(machine['bistage']); self.bistage_min_speed.setValue(machine['minimum_coast_velocity'])
                self.bistage_enabled.blockSignals(False); self.bistage_min_speed.blockSignals(False)
                self._update_machine_sequence(machine['bistage'])
        editable=s.editable and not s.dirty and not s.staged and not s.sim.get('active') and not s.status.get('sensor_hil_active') and machine.get('supported') is True
        self.bistage_enabled.setEnabled(editable)
        self.bistage_parameters.setEnabled(editable and self.bistage_enabled.isChecked())
        changed=not machine.get('machine_ready',True) or self.bistage_enabled.isChecked()!=machine.get('bistage') or (self.bistage_enabled.isChecked() and self.bistage_min_speed.value()!=machine.get('minimum_coast_velocity'))
        self.machine_save.setEnabled(editable and changed)
        text='Collega la scheda per leggere la macchina a stati.'
        if s.connected:
            if machine.get('supported') is False: text='Aggiorna il firmware per impostare il bistadio.'
            elif not machine.get('machine_ready',True): text='Macchina a stati da salvare: impostazione non valida sulla scheda.'
            elif 'bistage' in machine: text='Salvata sulla scheda: '+('Bistadio' if machine['bistage'] else 'Normale')
        self.machine_saved.setText(text)

    def confirm_maintenance(self,format_memory):
        title='Formatta memoria' if format_memory else 'Ripristina dati di fabbrica'
        text=('Eliminare definitivamente tutti i voli registrati? La configurazione viene conservata.' if format_memory else
              'Eliminare regole e posizioni dei servo e azzerare l’orientamento, comprese le modifiche non applicate? La memoria dei voli non viene cancellata.')
        if QMessageBox.question(self,title,text,QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes:
            self.controller.maintenance(format_memory)

    def _console(self):
        page,box=self.page('Console','Richieste manuali e risposte della scheda.')
        self.command_input=QLineEdit(); self.command_input.setPlaceholderText('Es. status, config show, sim timing…'); self.command_input.returnPressed.connect(self.send_console)
        self.command_send=button('Invia',self.send_console,'primary')
        box.addLayout(row(self.command_input,self.command_send,button('Pulisci',lambda:self.console.clear())))
        self.console=QPlainTextEdit(); self.console.setReadOnly(True); self.console.document().setMaximumBlockCount(1200); box.addWidget(self.console,1)
        return page

    def navigate(self,index):
        self.pages.setCurrentIndex(index); self.controller.page=index
        for i,nav in enumerate(self.nav): nav.setChecked(i==index)

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
        message=s.error or s.message or ('' if connected else 'Collega la scheda per iniziare.')
        self.feedback.setText(message); self.feedback.setVisible(bool(message))
        self.feedback.setStyleSheet('color: #b14d40;' if s.error else 'color: #7b8998;')
        self.config_metric.set('Valida' if status.get('config_valid') else 'Da configurare' if status else '—',
                               f'{len(s.base.rules)} regole applicate' if s.base else 'Nessuna configurazione applicata letta')
        reasons=s.preflight.get('reasons',[]) if connected else []
        ready=connected and not sleeping and s.preflight.get('ready',False) and not reasons
        self.readiness_title.setText('Risparmio energetico' if sleeping else 'Pronto all’armamento' if ready and connected else 'Armamento bloccato' if reasons else 'Controlli preflight')
        rows=[]
        for reason in reasons:
            text=REASONS.get(reason,reason)
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
        self.arm_button.setText('DISARM' if disarming else 'ARM')
        self.arm_button.setEnabled(usable and s.owns_control and (arming in ('armed','arming') if disarming else ready and not reasons and not s.dirty and not s.staged and arming=='disarmed'))
        self.arm_button.setToolTip('Disarma la scheda; la richiesta viene verificata dal firmware.' if disarming else
            'Preflight bloccato dalla scheda. Premi Ricontrolla dopo aver risolto i problemi.' if blocked else
            'Richiede preflight pronto dalla scheda; la scheda ricontrolla prima di armare')
        prepared=s.preflight.get('prepared_servo_mask',0)
        held_outputs=bool(prepared) if 'prepared_servo_mask' in s.preflight else bool(status.get('outputs_active'))
        self.release_servos_button.setEnabled(usable and s.owns_control and arming=='disarmed' and held_outputs and
            not s.sim.get('active') and not status.get('sensor_hil_active') and not status.get('estimate_replay_active'))
        can_prepare=usable and s.owns_control and arming=='disarmed' and not s.dirty and not s.staged and bool(s.capabilities.get('physical_actions'))
        plan=s.base if connected else None
        assigned_servos={a['channel'] for r in plan.rules for a in r['outputs'] if a['kind']=='servo'} if plan else set()
        for i,check in enumerate(self.servo_selection):
            assigned=i+1 in assigned_servos
            check.setVisible(assigned); check.setEnabled(can_prepare and assigned)
            if not assigned and check.isChecked():
                check.blockSignals(True); check.setChecked(False); check.blockSignals(False)
            is_prepared=bool(prepared & (1<<i))
            state=self.servo_states[i]; state.setVisible(assigned)
            state.setText('preparato' if is_prepared else 'non preparato')
            preparation='prepared' if is_prepared else 'unprepared'
            if state.property('preparation')!=preparation:
                state.setProperty('preparation',preparation)
                state.style().unpolish(state); state.style().polish(state)
        self.servo_empty.setVisible(not assigned_servos)
        self.servo_empty.setText('Nessun servo assegnato alle regole.' if plan else 'Configurazione non caricata: premi Ricarica dalla scheda in Configurazione.' if connected else 'Collega la scheda e leggi la configurazione.')
        selected=[i+1 for i,c in enumerate(self.servo_selection) if c.isChecked() and not c.isHidden()]
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
        self.configuration_page.render(s)
        active=s.sim.get('active',False)
        blocker=self.controller.simulation_blocker()
        self.sim_start.setEnabled(not blocker); self.sim_start.setToolTip(blocker)
        self.sim_blocker.setText(blocker); self.sim_blocker.setVisible(bool(blocker) and not active)
        self.sim_stop.setEnabled(usable and s.owns_control and active); self.sim_record.setEnabled(usable and not active)
        self.sim_physical.setEnabled(usable and not active)
        if active: self.sim_physical.setChecked(s.sim.get('output_mode')=='PHYSICAL')
        self._render_sim(s)
        self.list_flights_button.setEnabled(usable and arming=='disarmed')
        download_state=s.download.get('state'); downloading=download_state in ('downloading','cancelling')
        self.download_button.setText('Annullamento…' if download_state=='cancelling' else 'Annulla' if downloading else 'Scarica selezionato')
        name='downloadCancel' if downloading else ''
        if self.download_button.objectName()!=name:
            self.download_button.setObjectName(name); self.download_button.style().unpolish(self.download_button); self.download_button.style().polish(self.download_button)
        self.download_button.setEnabled((connected and download_state=='downloading') if downloading else usable and bool(s.flights) and arming=='disarmed')
        self.open_flight_button.setEnabled(not s.foreground)
        maintenance=s.editable and not s.sim.get('active') and not status.get('sensor_hil_active')
        self.orientation_panel.render(s)
        self.render_machine_settings(s)
        self.factory_reset_button.setEnabled(maintenance)
        self.format_memory_button.setEnabled(maintenance and not s.dirty and not s.staged)
        self.command_input.setEnabled(usable); self.command_send.setEnabled(usable)
        self._render_flights(s)

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

    def _render_flights(self,s):
        download=s.download; state=download.get('state')
        if state in ('downloading','cancelling'):
            total=download.get('total')
            self.download_progress.setRange(0,100 if total else 0)
            if total: self.download_progress.setValue(min(99,download.get('done',0)*100//total))
            eta=download.get('eta_seconds')
            if state=='cancelling': text='Annullamento…'
            elif total is None: text='Preparazione…'
            elif download.get('done',0)>=total: text='Finalizzazione…'
            elif eta is None: text='Calcolo tempo…'
            else:
                hours,remainder=divmod(max(1,ceil(eta)),3600); minutes,seconds=divmod(remainder,60)
                text=(f'~{hours} h {minutes:02d} min' if hours else f'~{minutes} min {seconds:02d} s' if minutes else f'~{seconds} s')+' rimanenti'
            self.download_progress.setFormat('%p% · '+text)
            self.download_feedback.setCurrentWidget(self.download_progress)
        elif state in ('completed','failed','interrupted','cancelled'):
            text='Downloaded at '+download['path'] if state=='completed' else 'Download annullato' if state=='cancelled' else 'Download interrotto' if state=='interrupted' else 'Volo corrotto' if download.get('error_code')=='Corrupt' else 'Download non completato'
            self.download_result.setText(text); self.download_result.setToolTip(text)
            self.download_result.setStyleSheet('color: #7b8998;' if state in ('completed','cancelled') else 'color: #b14d40;')
            self.download_feedback.setCurrentWidget(self.download_result)
        else: self.download_feedback.setCurrentIndex(0)
        if self._flight_entries != s.flights:
            selected=self.flights_table.currentRow(); previous=self._flight_entries or []
            sequence=previous[selected]['sequence'] if 0<=selected<len(previous) else None
            self._flight_entries=deepcopy(s.flights)
            names={1:'Interrotto / aperto',2:'Completato',3:'Interrotto',4:'Memoria piena',6:'Corrotto'}
            fill_table(self.flights_table,[(e['sequence'],names.get(e['status'],str(e['status'])),f'{e["committed_bytes"]/1024:.1f} KiB',f'{e["allocated_bytes"]/1024:.1f} KiB') for e in s.flights])
            if s.flights: self.flights_table.selectRow(next((i for i,e in enumerate(s.flights) if e['sequence']==sequence),0))
        data=s.flight
        if not data or self._flight_path==data['path']: return
        self._flight_path=data['path']; self.flight_duration.set(f'{data["duration"]:.2f} s'); self.flight_peak.set(f'{data["max_altitude"]:.1f} m'); self.flight_count.set(str(data['samples']))
        meta=data['metadata']; markers=[(e['t_ms']/1000,f'{e["kind"].upper()} {e["channel"]}') for e in meta.get('output_starts',[]) if e.get('state')=='recorded']
        self.flight_battery.curves=[(10,'Tensione batteria','V','#b58a26',0)]
        if data.get('simulated_battery'):
            self.flight_battery.curves=[(11,'Tensione batteria','V','#8b67bc',0)]
        for plot in self.flight_plots:
            plot.set_data(data['rows'],markers,data['phases'])
        self.flight_file.setText(data['path'])
        self.flight_details.clear()
        summary=meta.get('recording_summary',{})
        summary_states={'Valid':'Verificato','Missing':'Non disponibile','Incomplete':'Incompleto','Corrupt':'Corrotto','Unsupported':'Formato non supportato'}
        for key,value,hint in [
            ('ID volo',meta.get('sequence','—'),'Identificativo originale assegnato dalla scheda.'),
            ('Tipo di volo',{0:'Reale',1:'Simulato'}.get(meta.get('sensor_mode'),'Non disponibile'),'Tipo salvato dalla scheda: una simulazione resta tale anche con uscite fisiche abilitate.'),
            ('Riepilogo registrazione',summary_states.get(summary.get('state'),'—'),'Controllo di integrità del riepilogo della registrazione.'),
            ('Picco velocità verticale',f'{data["max_speed"]:.1f} m/s','Massimo valore assoluto della velocità verticale, considerando salita e discesa.'),
            ('Campioni non salvati',summary.get('dropped_samples','—'),'Campioni scartati dal registratore.')]:
            item=QTreeWidgetItem(self.flight_details,[key,str(value)]); item.setToolTip(0,hint); item.setToolTip(1,hint)
        calibration=meta.get('calibration') or {}
        def biases(key,unit):
            values=calibration.get(key)
            if not isinstance(values,list) or len(values)!=3: return 'Non applicati (dati simulati)' if calibration else 'Non disponibile'
            return '  /  '.join(f'{axis}: {value:.6g}' for axis,value in zip('XYZ',values))+' '+unit
        sensor_mode=meta.get('sensor_mode'); output_mode=meta.get('output_mode')
        mode=('Volo reale' if sensor_mode==0 else 'Simulazione' if sensor_mode==1 else 'Non disponibile')
        if output_mode in (0,1): mode+=' '+chr(183)+' uscite '+('fisiche' if output_mode==0 else 'virtuali')
        gyro=biases('gyro_bias_radps','rad/s')
        reference=calibration.get('reference_altitude_m')
        for key,value,hint in [
            ('Bias giroscopio',gyro,'Correzioni X/Y/Z negli assi del sensore, usate in questa registrazione.'),
            ('Riferimento barometrico',f'{reference:.3f} m' if reference is not None else 'Non disponibile','Quota barometrica di riferimento sottratta per ottenere la quota relativa del volo.'),
            ('Bias accelerometro',biases('accel_bias_mps2','m/s'+chr(178)),'Correzioni X/Y/Z applicate negli assi del sensore. Zero significa nessuna correzione; non certifica una calibrazione eseguita.'),
            ('Orientamento scheda',calibration.get('up_axis') or ('Non applicato (dati simulati)' if calibration else 'Non disponibile'),'Asse del sensore rivolto verso la punta, salvato con questo volo.'),
            ('Modalit'+chr(224),mode,'Sorgente dei sensori e modalit'+chr(224)+' delle uscite usate nel volo registrato.')]:
            item=QTreeWidgetItem(self.flight_details,[key,value]); item.setToolTip(0,hint); item.setToolTip(1,hint)
        if calibration.get('bistage'):
            QTreeWidgetItem(self.flight_details,['Macchina a stati','Bistadio'])
            QTreeWidgetItem(self.flight_details,['Velocit'+chr(224)+' minima in COAST 1',str(calibration.get('minimum_coast_velocity'))+' m/s'])
        abort=meta.get('abort')
        if abort: QTreeWidgetItem(self.flight_details,['ABORT',{'Velocity':'Velocit'+chr(224)+' insufficiente','Tilt':'Inclinazione oltre il limite'}.get(abort.get('reason'),'Non disponibile')+f' a {abort["t_ms"]/1000:.3f} s'])
        rules=meta.get('configuration',{}).get('rules',[])
        starts=meta.get('output_starts')
        recorded={(e.get('rule_id'),e.get('kind'),e.get('channel')) for e in starts or [] if e.get('state')=='recorded'}
        activated={r['id'] for r in rules if any((r['id'],a['kind'],a['channel']) in recorded for a in r.get('outputs',[]))}
        count=QTreeWidgetItem(self.flight_details,['Regole azionate',f'{len(activated)} / {len(rules)}' if starts is not None and meta.get('configuration') else 'Non disponibile'])
        count.setToolTip(0,'Regole con almeno un’attivazione di uscita registrata nel volo. Una regola è contata una sola volta.')
        config=QTreeWidgetItem(self.flight_details,['Regole del volo',f'{len(rules)} '+('regola' if len(rules)==1 else 'regole') if meta.get('configuration') else 'Non disponibili'])
        for r in rules:
            timing=f'Ritardo di attivazione: {r["delay_ms"]} ms' if 'delay_ms' in r else 'Ritardo non disponibile'
            rule=QTreeWidgetItem(config,[rule_label(rules,r['id']),timing])
            for condition in r.get('conditions',[]):
                decoded=decode_condition(condition)
                if decoded['type']=='phase':
                    decoded['phase']=' o '.join(phase_name(i,calibration.get('bistage',False)) for i in range(len(PHASES)) if condition['phase_mask'] & (1<<i)) or '—'
                QTreeWidgetItem(rule,['Condizione · AND',condition_text(decoded,rules)])
            for output in r.get('outputs',[]):
                value=f'Ritardo canale {output["delay_ms"]} ms' if 'delay_ms' in output else 'Ritardo non disponibile'
                item=QTreeWidgetItem(rule,[f'Uscita · {output["kind"].upper()} {output["channel"]}',value])
                item.setToolTip(1,'Ritardo aggiuntivo rispetto a quello della regola, dalla configurazione salvata nel volo.')

    def _flight_overlays(self):
        for plot in self.flight_plots:
            plot.set_overlays(self.flight_actuators.isChecked(),self.flight_states.isChecked())

    def download_selected(self):
        if self.controller.state.download.get('state') in ('downloading','cancelling'):
            self.controller.cancel_download(); return
        i=self.flights_table.currentRow(); entries=self.controller.state.flights
        if i<0 or i>=len(entries): return
        entry=entries[i]
        if entry.get('corrupt'):
            QMessageBox.information(self,'Volo corrotto','Questo volo è segnalato come corrotto. Gli altri voli restano scaricabili.'); return
        path,_=QFileDialog.getSaveFileName(self,'Scarica volo',str(Path.home()/'Documents'/f'flight_{entry["sequence"]}.csv'),'CSV (*.csv)')
        if path: self.controller.download(entry['sequence'],path)

    def open_local(self):
        path,_=QFileDialog.getOpenFileName(self,'Apri volo registrato',str(Path.home()/'Documents'),'CSV (*.csv)')
        if path: self.controller.open_flight(path)

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
        if not self._closing and self.controller.state.dirty:
            if QMessageBox.question(self,'Bozza non applicata','Chiudere e perdere le modifiche locali non applicate?')!=QMessageBox.StandardButton.Yes:
                event.ignore(); return
        event.ignore()
        if not self._closing:
            self._closing=True; self.controller.disconnect(); self.close_timer.start()

    def _finish_close(self):
        if self.controller.can_close():
            self.close_timer.stop(); self._allow_close=True; self.close()
