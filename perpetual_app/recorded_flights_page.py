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


class RecordedFlightsPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller
        self._flight_path=None; self._flight_entries=None
        _,box=create_page('Voli registrati','Leggi i voli conservati sulla scheda oppure apri un CSV già scaricato.',self)
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
        self.flights_empty=label('Nessun volo in memoria.','emptyState'); self.flights_empty.setAlignment(Qt.AlignCenter)
        self.flights_empty.setAttribute(Qt.WA_TransparentForMouseEvents); self.flights_empty.hide()
        empty=QVBoxLayout(self.flights_table.viewport()); empty.setContentsMargins(0,0,0,0); empty.addWidget(self.flights_empty)
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
        self.airbrake_card,body=card('Apertura comandata aerofreni')
        self.flight_airbrakes=FlightPlot([(i,f'SERVO {i}','%',COLORS[i-1],0) for i in range(1,5)],fixed_range=(0,100))
        body.addWidget(self.flight_airbrakes)
        self.airbrake_note=label('Aperture comandate, non misurate.','hint'); self.airbrake_note.setWordWrap(True); body.addWidget(self.airbrake_note)
        self.airbrake_conditions=table(['CONDIZIONI DELLA REGOLA · AND']); self.airbrake_conditions.setWordWrap(True)
        self.airbrake_conditions.setTextElideMode(Qt.ElideNone)
        self.airbrake_conditions.setSelectionMode(self.airbrake_conditions.SelectionMode.NoSelection); self.airbrake_conditions.setFocusPolicy(Qt.NoFocus)
        self.airbrake_conditions.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.airbrake_conditions.setMaximumHeight(240); body.addWidget(self.airbrake_conditions); box.addWidget(self.airbrake_card)
        self.airbrake_card.hide()
        frame,body=card('Velocità angolari BODY · °/s')
        self.flight_rates=FlightPlot([(7,'X','°/s','#3c70b8',0),(8,'Y','°/s','#1d8b7e',0),(9,'Z','°/s','#a36db1',0)])
        body.addWidget(self.flight_rates); box.addWidget(frame)
        frame,body=card('Orientamento Euler')
        self.flight_orientation=FlightPlot([(4,'Roll','°','#3c70b8',0),(5,'Pitch','°','#1d8b7e',0),(6,'Yaw','°','#a36db1',0)],angular=True)
        self.flight_rates.setToolTip('Componenti del giroscopio attorno agli assi BODY X/Y/Z, convertite da rad/s a °/s. Non sono le derivate degli angoli Euler.')
        body.addWidget(self.flight_orientation)
        orientation_note=label('Angoli Euler ZYX espressi nel riferimento con +X verso l’alto. Lo zero indica il razzo con la punta verso l’alto e una rotazione di riferimento attorno al proprio asse.','hint')
        orientation_note.setWordWrap(True); orientation_note.setStyleSheet('color: #929da8; font-size: 12px;')
        body.addWidget(orientation_note); box.addWidget(frame)
        frame,body=card('Tensione batteria')
        self.flight_battery=FlightPlot([(10,'Tensione batteria','V','#b58a26',0)],include_zero=False)
        body.addWidget(self.flight_battery); box.addWidget(frame)
        self.flight_plots=(self.flight_plot,self.flight_motion,self.flight_rates,self.flight_orientation,self.flight_battery)
        self.flight_details=QTreeWidget(); self.flight_details.setHeaderLabels(['DATI DEL VOLO','VALORE']); self.flight_details.setMinimumHeight(270); self.flight_details.header().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.flight_details.setSelectionMode(QTreeWidget.SelectionMode.NoSelection); self.flight_details.setFocusPolicy(Qt.NoFocus)
        box.addWidget(self.flight_details)

    def render(self,s):
        connected=s.connected; usable=connected and not s.foreground; arming=s.status.get('arming_state')
        self.flights_empty.setVisible(connected and s.flights_loaded and not s.flights)
        self.list_flights_button.setEnabled(usable and arming=='disarmed')
        download_state=s.download.get('state'); downloading=download_state in ('downloading','cancelling')
        self.download_button.setText('Annullamento…' if download_state=='cancelling' else 'Annulla' if downloading else 'Scarica selezionato')
        name='downloadCancel' if downloading else ''
        if self.download_button.objectName()!=name:
            self.download_button.setObjectName(name); self.download_button.style().unpolish(self.download_button); self.download_button.style().polish(self.download_button)
        self.download_button.setEnabled((connected and download_state=='downloading') if downloading else usable and bool(s.flights) and arming=='disarmed')
        self.open_flight_button.setEnabled(not s.foreground)
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
        if not data:
            self.airbrake_card.hide(); return
        if self._flight_path==data['path'] and getattr(self,'_rendered_flight',None) is data: return
        self._rendered_flight=data
        self._flight_path=data['path']; self.flight_duration.set(f'{data["duration"]:.2f} s'); self.flight_peak.set(f'{data["max_altitude"]:.1f} m'); self.flight_count.set(str(data['samples']))
        meta=data['metadata']; markers=[(e['t_ms']/1000,f'{e["kind"].upper()} {e["channel"]}') for e in meta.get('output_starts',[]) if e.get('state')=='recorded']
        self.flight_battery.curves=[(10,'Tensione batteria','V','#b58a26',0)]
        if data.get('simulated_battery'):
            self.flight_battery.curves=[(11,'Tensione batteria','V','#8b67bc',0)]
        for plot in self.flight_plots:
            plot.set_data(data['rows'],markers,data['phases'])
        air=data.get('airbrakes',{}); mask=air.get('channel_mask',0)
        enabled=(meta.get('calibration') or {}).get('airbrakes_enabled') is True
        self.airbrake_card.setVisible(enabled)
        self.flight_airbrakes.curves=[(i,f'SERVO {i}','%',COLORS[i-1],0) for i in channels(mask)]
        if enabled:
            self.flight_airbrakes.setVisible(bool(mask))
            self.flight_airbrakes.set_data(air.get('rows',[]),markers,air.get('phases',[]))
            config=meta.get('configuration',{}); bistage=(meta.get('calibration') or {}).get('bistage',False)
            conditions=recorded_conditions(config,bistage); rules=config.get('rules',[])
            fill_table(self.airbrake_conditions,[(condition_text(c,rules),) for c in conditions])
            height=self.airbrake_conditions.horizontalHeader().sizeHint().height()+sum(self.airbrake_conditions.rowHeight(i) for i in range(len(conditions)))+2*self.airbrake_conditions.frameWidth()
            self.airbrake_conditions.setFixedHeight(min(240,max(90,height)))
            self.airbrake_note.setText('Aperture comandate, non misurate.' if mask else 'Nessun servo dedicato agli aerofreni in questo volo.')
        self.flight_file.setText(data['path'])
        self.flight_details.clear()
        summary=meta.get('recording_summary',{})
        for key,value,hint in [
            ('ID volo',meta.get('sequence','—'),'Identificativo originale assegnato dalla scheda.'),
            ('Tipo di volo',{0:'Reale',1:'Simulato'}.get(meta.get('sensor_mode'),'Non disponibile'),'Tipo salvato dalla scheda: una simulazione resta tale anche con uscite fisiche abilitate.'),
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
            ('Scala accelerometro',biases('accel_scale',''),'Fattori moltiplicativi applicati dopo aver sottratto il bias.'),
            ('Calibrazione IMU',str(calibration.get('accel_calibration_revision') or 'Non disponibile'),'Revisione dei coefficienti salvata con questo volo.'),
            ('Orientamento scheda',calibration.get('up_axis') or ('Non applicato (dati simulati)' if calibration else 'Non disponibile'),'Asse del sensore rivolto verso la punta, salvato con questo volo.'),
            ('Modalit'+chr(224),mode,'Sorgente dei sensori e modalit'+chr(224)+' delle uscite usate nel volo registrato.')]:
            item=QTreeWidgetItem(self.flight_details,[key,value]); item.setToolTip(0,hint); item.setToolTip(1,hint)
        if calibration.get('bistage'):
            QTreeWidgetItem(self.flight_details,['Macchina a stati','Bistadio'])
            QTreeWidgetItem(self.flight_details,['Velocit'+chr(224)+' minima in COAST 1',str(calibration.get('minimum_coast_velocity'))+' m/s'])
        abort=meta.get('abort')
        if abort: QTreeWidgetItem(self.flight_details,['ABORT',{'Velocity':'Velocit'+chr(224)+' insufficiente','Tilt':'Inclinazione oltre il limite'}.get(abort.get('reason'),'Non disponibile')+f' a {abort["t_ms"]/1000:.3f} s'])
        air_config=meta.get('configuration',{}).get('airbrakes',{})
        if air_config.get('channel_mask'):
            group=QTreeWidgetItem(self.flight_details,['Aerofreni','Abilitati' if calibration.get('airbrakes_enabled') else 'Disabilitati'])
            QTreeWidgetItem(group,['Canali',', '.join(f'SERVO {i}' for i in channels(air_config['channel_mask']))])
            QTreeWidgetItem(group,['Versione legge di controllo',str(calibration.get('airbrake_law_version','Non disponibile'))])
            for c in recorded_conditions(meta.get('configuration',{})): QTreeWidgetItem(group,['Condizione '+chr(183)+' AND',condition_text(c,meta.get('configuration',{}).get('rules',[]))])
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
        for plot in (*self.flight_plots,self.flight_airbrakes):
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

