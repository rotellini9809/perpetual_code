"""Recorded-flight page: inventory, export controls, plots and flight details."""
from copy import deepcopy
from math import ceil
from pathlib import Path
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget,QVBoxLayout,QStackedWidget,QProgressBar,QSizePolicy,QCheckBox,QTreeWidget,QTreeWidgetItem,QHeaderView,QFileDialog,QMessageBox
from .configuration import rule_label,decode_condition,condition_text
from .airbrakes import COLORS,channels,recorded_conditions
from .flight_data import PHASES,phase_name
from .flight_plot import FlightPlot
from .widgets import create_page,button,label,row,table,card,Metric,fill_table
from .i18n import t


class RecordedFlightsPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller
        self._flight_path=None; self._flight_entries=None
        _,box=create_page(t('Recorded flights'),t('Read the flights stored on the board or open a CSV you already downloaded.'),self)
        self.list_flights_button=button(t('Read flights from board'),self.controller.list_flights,'primary')
        self.download_button=button(t('Download selected'),self.download_selected)
        self.open_flight_button=button(t('Open local CSV'),self.open_local)
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
        frame,body=card(); self.flights_table=table([t('FLIGHT ID'),t('STATUS'),t('COMMITTED DATA'),t('RESERVED SPACE')]); self.flights_table.setMaximumHeight(205); self.flights_table.setMinimumHeight(130)
        self.flights_empty=label(t('No flights in memory.'),'emptyState'); self.flights_empty.setAlignment(Qt.AlignCenter)
        self.flights_empty.setAttribute(Qt.WA_TransparentForMouseEvents); self.flights_empty.hide()
        empty=QVBoxLayout(self.flights_table.viewport()); empty.setContentsMargins(0,0,0,0); empty.addWidget(self.flights_empty)
        body.addWidget(self.flights_table); box.addWidget(frame)
        self.flight_duration=Metric(t('Duration')); self.flight_peak=Metric(t('Maximum altitude')); self.flight_count=Metric(t('Saved samples'))
        box.addLayout(row(self.flight_duration,self.flight_peak,self.flight_count))
        self.flight_actuators=QCheckBox(t('Actuators')); self.flight_states=QCheckBox(t('Flight states'))
        for check in (self.flight_actuators,self.flight_states):
            check.setChecked(True); check.toggled.connect(self._flight_overlays)
        box.addLayout(row(self.flight_actuators,self.flight_states,None))
        frame,body=card(t('Altitude')); self.flight_plot=FlightPlot([(1,t('Altitude'),'m','#1d8b7e',0)]); body.addWidget(self.flight_plot)
        self.flight_file=label(t('No flight open.'),'hint'); self.flight_file.setWordWrap(True); body.addWidget(self.flight_file); box.addWidget(frame,1)
        frame,body=card(t('Vertical speed and acceleration'))
        self.flight_motion=FlightPlot([(2,t('Speed'),'m/s','#3c70b8',0),(3,t('Acceleration'),'m/s²','#c47539',1)])
        body.addWidget(self.flight_motion); box.addWidget(frame)
        self.airbrake_card,body=card(t('Commanded airbrake opening'))
        self.flight_airbrakes=FlightPlot([(i,f'SERVO {i}','%',COLORS[i-1],0) for i in range(1,5)],fixed_range=(0,100))
        body.addWidget(self.flight_airbrakes)
        self.airbrake_note=label(t('Commanded openings, not measured.'),'hint'); self.airbrake_note.setWordWrap(True); body.addWidget(self.airbrake_note)
        self.airbrake_conditions=table([t('RULE CONDITIONS · AND')]); self.airbrake_conditions.setWordWrap(True)
        self.airbrake_conditions.setTextElideMode(Qt.ElideNone)
        self.airbrake_conditions.setSelectionMode(self.airbrake_conditions.SelectionMode.NoSelection); self.airbrake_conditions.setFocusPolicy(Qt.NoFocus)
        self.airbrake_conditions.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.airbrake_conditions.setMaximumHeight(240); body.addWidget(self.airbrake_conditions); box.addWidget(self.airbrake_card)
        self.airbrake_card.hide()
        frame,body=card(t('BODY angular rates · °/s'))
        self.flight_rates=FlightPlot([(7,'X','°/s','#3c70b8',0),(8,'Y','°/s','#1d8b7e',0),(9,'Z','°/s','#a36db1',0)])
        body.addWidget(self.flight_rates); box.addWidget(frame)
        frame,body=card(t('Euler orientation'))
        self.flight_orientation=FlightPlot([(4,'Roll','°','#3c70b8',0),(5,'Pitch','°','#1d8b7e',0),(6,'Yaw','°','#a36db1',0)],angular=True)
        self.flight_rates.setToolTip(t('Gyroscope components around the BODY X/Y/Z axes, converted from rad/s to °/s. They are not the derivatives of the Euler angles.'))
        body.addWidget(self.flight_orientation)
        orientation_note=label(t('ZYX Euler angles in the frame with +X up. Zero means the rocket nose up with a reference rotation around its own axis.'),'hint')
        orientation_note.setWordWrap(True); orientation_note.setStyleSheet('color: #929da8; font-size: 12px;')
        body.addWidget(orientation_note); box.addWidget(frame)
        frame,body=card(t('Battery voltage'))
        self.flight_battery=FlightPlot([(10,t('Battery voltage'),'V','#b58a26',0)],include_zero=False)
        body.addWidget(self.flight_battery); box.addWidget(frame)
        self.flight_plots=(self.flight_plot,self.flight_motion,self.flight_rates,self.flight_orientation,self.flight_battery)
        self.flight_details=QTreeWidget(); self.flight_details.setHeaderLabels([t('FLIGHT DATA'),t('VALUE')]); self.flight_details.setMinimumHeight(270); self.flight_details.header().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.flight_details.setSelectionMode(QTreeWidget.SelectionMode.NoSelection); self.flight_details.setFocusPolicy(Qt.NoFocus)
        box.addWidget(self.flight_details)

    def render(self,state):
        connected=state.connected; usable=connected and not state.foreground; arming=state.status.get('arming_state')
        self.flights_empty.setVisible(connected and state.flights_loaded and not state.flights)
        self.list_flights_button.setEnabled(usable and arming=='disarmed')
        download_state=state.download.get('state'); downloading=download_state in ('downloading','cancelling')
        self.download_button.setText(t('Cancelling…') if download_state=='cancelling' else t('Cancel') if downloading else t('Download selected'))
        name='downloadCancel' if downloading else ''
        if self.download_button.objectName()!=name:
            self.download_button.setObjectName(name); self.download_button.style().unpolish(self.download_button); self.download_button.style().polish(self.download_button)
        self.download_button.setEnabled((connected and download_state=='downloading') if downloading else usable and bool(state.flights) and arming=='disarmed')
        self.open_flight_button.setEnabled(not state.foreground)
        download=state.download
        if download_state in ('downloading','cancelling'):
            total=download.get('total')
            self.download_progress.setRange(0,100 if total else 0)
            if total: self.download_progress.setValue(min(99,download.get('done',0)*100//total))
            eta=download.get('eta_seconds')
            if download_state=='cancelling': text=t('Cancelling…')
            elif total is None: text=t('Preparing…')
            elif download.get('done',0)>=total: text=t('Finalizing…')
            elif eta is None: text=t('Estimating time…')
            else:
                hours,remainder=divmod(max(1,ceil(eta)),3600); minutes,seconds=divmod(remainder,60)
                text=t('{time} remaining',time=f'~{hours} h {minutes:02d} min' if hours else f'~{minutes} min {seconds:02d} s' if minutes else f'~{seconds} s')
            self.download_progress.setFormat('%p% · '+text)
            self.download_feedback.setCurrentWidget(self.download_progress)
        elif download_state in ('completed','failed','interrupted','cancelled'):
            text=t('Downloaded to {path}',path=download['path']) if download_state=='completed' else t('Download cancelled') if download_state=='cancelled' else t('Download interrupted') if download_state=='interrupted' else 'Corrupted flight' if download.get('error_code')=='Corrupt' else 'Download not completed'
            self.download_result.setText(text); self.download_result.setToolTip(text)
            self.download_result.setStyleSheet('color: #7b8998;' if download_state in ('completed','cancelled') else 'color: #b14d40;')
            self.download_feedback.setCurrentWidget(self.download_result)
        else: self.download_feedback.setCurrentIndex(0)
        if self._flight_entries != state.flights:
            selected=self.flights_table.currentRow(); previous=self._flight_entries or []
            sequence=previous[selected]['sequence'] if 0<=selected<len(previous) else None
            self._flight_entries=deepcopy(state.flights)
            names={1:t('Interrupted / open'),2:t('Completed'),3:t('Interrupted'),4:t('Memory full'),6:t('Corrupted')}
            fill_table(self.flights_table,[(entry['sequence'],names.get(entry['status'],str(entry['status'])),f'{entry["committed_bytes"]/1024:.1f} KiB',f'{entry["allocated_bytes"]/1024:.1f} KiB') for entry in state.flights])
            if state.flights: self.flights_table.selectRow(next((i for i,entry in enumerate(state.flights) if entry['sequence']==sequence),0))
        flight=state.flight
        if not flight:
            self.airbrake_card.hide(); return
        if self._flight_path==flight['path'] and getattr(self,'_rendered_flight',None) is flight: return
        self._rendered_flight=flight
        self._flight_path=flight['path']; self.flight_duration.set(f'{flight["duration"]:.2f} s'); self.flight_peak.set(f'{flight["max_altitude"]:.1f} m'); self.flight_count.set(str(flight['samples']))
        meta=flight['metadata']; markers=[(start['t_ms']/1000,f'{start["kind"].upper()} {start["channel"]}') for start in meta.get('output_starts',[]) if start.get('state')=='recorded']
        self.flight_battery.curves=[(10,t('Battery voltage'),'V','#b58a26',0)]
        if flight.get('simulated_battery'):
            self.flight_battery.curves=[(11,t('Battery voltage'),'V','#8b67bc',0)]
        for plot in self.flight_plots:
            plot.set_data(flight['rows'],markers,flight['phases'])
        airbrakes=flight.get('airbrakes',{}); mask=airbrakes.get('channel_mask',0)
        enabled=(meta.get('calibration') or {}).get('airbrakes_enabled') is True
        self.airbrake_card.setVisible(enabled)
        self.flight_airbrakes.curves=[(i,f'SERVO {i}','%',COLORS[i-1],0) for i in channels(mask)]
        if enabled:
            self.flight_airbrakes.setVisible(bool(mask))
            self.flight_airbrakes.set_data(airbrakes.get('rows',[]),markers,airbrakes.get('phases',[]))
            config=meta.get('configuration',{}); bistage=(meta.get('calibration') or {}).get('bistage',False)
            conditions=recorded_conditions(config,bistage); rules=config.get('rules',[])
            fill_table(self.airbrake_conditions,[(condition_text(condition,rules),) for condition in conditions])
            height=self.airbrake_conditions.horizontalHeader().sizeHint().height()+sum(self.airbrake_conditions.rowHeight(i) for i in range(len(conditions)))+2*self.airbrake_conditions.frameWidth()
            self.airbrake_conditions.setFixedHeight(min(240,max(90,height)))
            self.airbrake_note.setText(t('Commanded openings, not measured.') if mask else t('No servo dedicated to the airbrakes in this flight.'))
        self.flight_file.setText(flight['path'])
        self.flight_details.clear()
        summary=meta.get('recording_summary',{})
        for key,value,hint in [
            (t('Flight ID'),meta.get('sequence','—'),t('Original identifier assigned by the board.')),
            (t('Flight type'),{0:t('Real'),1:t('Simulated')}.get(meta.get('sensor_mode'),t('Not available')),t('Type saved by the board: a simulation stays a simulation even with physical outputs enabled.')),
            (t('Peak vertical speed'),f'{flight["max_speed"]:.1f} m/s',t('Maximum absolute vertical speed, considering ascent and descent.')),
            (t('Unsaved samples'),summary.get('dropped_samples','—'),t('Samples discarded by the recorder.'))]:
            item=QTreeWidgetItem(self.flight_details,[key,str(value)]); item.setToolTip(0,hint); item.setToolTip(1,hint)
        calibration=meta.get('calibration') or {}
        def biases(key,unit):
            values=calibration.get(key)
            if not isinstance(values,list) or len(values)!=3: return t('Not applied (simulated data)') if calibration else t('Not available')
            return '  /  '.join(f'{axis}: {value:.6g}' for axis,value in zip('XYZ',values))+' '+unit
        sensor_mode=meta.get('sensor_mode'); output_mode=meta.get('output_mode')
        mode=(t('Real flight') if sensor_mode==0 else t('Simulation') if sensor_mode==1 else t('Not available'))
        if output_mode in (0,1): mode+=' · '+(t('physical outputs') if output_mode==0 else t('virtual outputs'))
        gyro=biases('gyro_bias_radps','rad/s')
        reference=calibration.get('reference_altitude_m')
        for key,value,hint in [
            (t('Gyroscope bias'),gyro,t('X/Y/Z corrections in the sensor axes, used in this recording.')),
            (t('Barometric reference'),f'{reference:.3f} m' if reference is not None else t('Not available'),t('Reference barometric altitude subtracted to get the flight relative altitude.')),
            (t('Accelerometer bias'),biases('accel_bias_mps2','m/s²'),t('X/Y/Z corrections applied in the sensor axes. Zero means no correction; it does not certify a performed calibration.')),
            (t('Accelerometer scale'),biases('accel_scale',''),t('Multiplicative factors applied after subtracting the bias.')),
            (t('IMU calibration'),str(calibration.get('accel_calibration_revision') or t('Not available')),t('Coefficient revision saved with this flight.')),
            (t('Board orientation'),calibration.get('up_axis') or (t('Not applied (simulated data)') if calibration else t('Not available')),t('Sensor axis pointing toward the nose, saved with this flight.')),
            (t('Mode'),mode,t('Sensor source and output mode used in the recorded flight.'))]:
            item=QTreeWidgetItem(self.flight_details,[key,value]); item.setToolTip(0,hint); item.setToolTip(1,hint)
        if calibration.get('bistage'):
            QTreeWidgetItem(self.flight_details,[t('State machine'),t('Bistage')])
            QTreeWidgetItem(self.flight_details,[t('Minimum speed in COAST 1'),str(calibration.get('minimum_coast_velocity'))+' m/s'])
        abort=meta.get('abort')
        if abort: QTreeWidgetItem(self.flight_details,['ABORT',t('{reason} at {time} s',reason={'Velocity':t('Insufficient speed'),'Tilt':t('Tilt beyond the limit')}.get(abort.get('reason'),t('Not available')),time=f'{abort["t_us"]/1e6:.3f}')])
        air_config=meta.get('configuration',{}).get('airbrakes',{})
        if air_config.get('channel_mask'):
            group=QTreeWidgetItem(self.flight_details,[t('Airbrakes'),t('Enabled') if calibration.get('airbrakes_enabled') else t('Disabled')])
            QTreeWidgetItem(group,[t('Channels'),', '.join(f'SERVO {i}' for i in channels(air_config['channel_mask']))])
            QTreeWidgetItem(group,[t('Control law version'),str(calibration.get('airbrake_law_version',t('Not available')))])
            for condition in recorded_conditions(meta.get('configuration',{})): QTreeWidgetItem(group,[t('Condition · AND'),condition_text(condition,meta.get('configuration',{}).get('rules',[]))])
        rules=meta.get('configuration',{}).get('rules',[])
        starts=meta.get('output_starts')
        recorded={(start.get('rule_id'),start.get('kind'),start.get('channel')) for start in starts or [] if start.get('state')=='recorded'}
        activated={rule['id'] for rule in rules if any((rule['id'],output['kind'],output['channel']) in recorded for output in rule.get('outputs',[]))}
        count=QTreeWidgetItem(self.flight_details,[t('Rules triggered'),f'{len(activated)} / {len(rules)}' if starts is not None and meta.get('configuration') else t('Not available')])
        count.setToolTip(0,t('Rules with at least one output activation recorded in the flight. A rule is counted once.'))
        rules_item=QTreeWidgetItem(self.flight_details,[t('Flight rules'),(t('{count} rule',count=len(rules)) if len(rules)==1 else t('{count} rules',count=len(rules))) if meta.get('configuration') else t('Not available')])
        for saved_rule in rules:
            timing=t('Activation delay: {delay} ms',delay=saved_rule['delay_ms']) if 'delay_ms' in saved_rule else t('Delay not available')
            rule_item=QTreeWidgetItem(rules_item,[rule_label(rules,saved_rule['id']),timing])
            for condition in saved_rule.get('conditions',[]):
                decoded=decode_condition(condition)
                if decoded['type']=='phase':
                    decoded['phase']=t(' or ').join(phase_name(i,calibration.get('bistage',False)) for i in range(len(PHASES)) if condition['phase_mask'] & (1<<i)) or '—'
                QTreeWidgetItem(rule_item,[t('Condition · AND'),condition_text(decoded,rules)])
            for output in saved_rule.get('outputs',[]):
                value=t('Channel delay {delay} ms',delay=output['delay_ms']) if 'delay_ms' in output else t('Delay not available')
                item=QTreeWidgetItem(rule_item,[t('Output · {channel}',channel=f'{output["kind"].upper()} {output["channel"]}'),value])
                item.setToolTip(1,t('Delay added to the rule delay, from the configuration saved with the flight.'))

    def _flight_overlays(self):
        for plot in (*self.flight_plots,self.flight_airbrakes):
            plot.set_overlays(self.flight_actuators.isChecked(),self.flight_states.isChecked())

    def download_selected(self):
        if self.controller.state.download.get('state') in ('downloading','cancelling'):
            self.controller.cancel_download(); return
        i=self.flights_table.currentRow(); entries=self.controller.state.flights
        if i<0 or i>=len(entries): return
        entry=entries[i]
        if entry.get('corrupt'):
            QMessageBox.information(self,'Corrupted flight','This flight is marked as corrupted. The other flights can still be downloaded.'); return
        path,_=QFileDialog.getSaveFileName(self,t('Download flight'),str(Path.home()/'Documents'/f'flight_{entry["sequence"]}.csv'),'CSV (*.csv)',options=QFileDialog.DontConfirmOverwrite)
        if not path: return
        # The download writes a CSV and a JSON pair; confirm here so both files are named.
        files=[Path(path),Path(path).with_suffix('.json')]; existing=[file.name for file in files if file.exists()]
        if existing and QMessageBox.question(self,t('Overwrite the files?'),
                t('These files already exist and will be replaced when the download completes:')+'\n\n'+'\n'.join('• '+name for name in existing)+
                '\n\n'+t('If the download is cancelled or fails, the current files stay unchanged.'),
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes: return
        self.controller.download(entry['sequence'],path)

    def open_local(self):
        path,_=QFileDialog.getOpenFileName(self,t('Open recorded flight'),str(Path.home()/'Documents'),'CSV (*.csv)')
        if path: self.controller.open_flight(path)

