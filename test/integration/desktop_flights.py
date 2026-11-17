"""Recorded flights: native recording, download, cancel/overwrite and plots."""
from desktop_support import session
import csv
import json
import math
import tempfile
import threading
from pathlib import Path
from unittest.mock import patch
from PySide6.QtWidgets import QApplication,QMessageBox
from device_client import CommandError
from perpetual_app.flight_data import load_flight
from perpetual_app.flight_plot import axis_scale,axis_scales
from perpetual_app.orientation import euler_x_up


def check_flight_plots(window,controller,app):
    def matrix(angles):
        r,p,y=map(math.radians,angles)
        def multiply(a,b): return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        rx=[[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]]
        ry=[[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]]
        rz=[[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]]
        return multiply(multiply(rz,ry),rx)
    assert all(abs(a)<1e-10 for a in euler_x_up(0,-90,0))
    assert abs(euler_x_up(30,-90,0)[0]-30)<1e-10
    assert euler_x_up(None,-90,0)==(None,None,None)
    assert euler_x_up(0,float('nan'),0)==(None,None,None)
    # Independent rotation matrices check composition, including both old/new
    # singularities and equivalent old Euler triplets at the upright attitude.
    for angles in [(r,p,y) for r in (-170,0,37) for p in (-90,-40,0,60,90) for y in (-180,-75,0,50,180)]:
        before=matrix(angles); after=matrix(euler_x_up(*angles))
        expected=[before[2],before[1],[-v for v in before[0]]]
        assert max(abs(after[i][j]-expected[i][j]) for i in range(3) for j in range(3))<1e-8,angles
    assert max(abs(a-b) for a,b in zip(euler_x_up(0,-90,0),euler_x_up(120,-90,-120)))<1e-10
    for values in ((0,),(1,12),(-12,-1),(-3,110),(0,.000003),(-124.4,97.2)):
        low,high,ticks=axis_scale(values)
        assert 0 in ticks and low<min(values) and high>max(values) and ticks==sorted(set(ticks))
    for values in ({0:[-100,100],1:[-9.5]},{0:[1,120],1:[-300,-1]},{0:[0],1:[0]},{0:[-.00003],1:[120000]}):
        scales=axis_scales(values)
        zero_positions=[-low/(high-low) for low,high,_ in scales.values()]
        assert abs(zero_positions[0]-zero_positions[1])<1e-12
        for axis,(low,high,ticks) in scales.items():
            assert 0 in ticks and low<min(values[axis]) and high>max(values[axis])
    assert axis_scales({0:[-10,2],1:[]})=={0:axis_scale([-10,2])}
    with tempfile.TemporaryDirectory() as folder:
        target=Path(folder)/'curves.csv'
        with target.open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream); writer.writerow(['t_ms','z_est','vz_est','az_est','roll','pitch','yaw','state','gyroX','gyroY','gyroZ','battery_v'])
            for i in range(2401):
                writer.writerow([i*5,i/2,100-i/12,-9.5,'nan' if i==4 else 12,3,-40,0 if i==0 else 1 if i==1 else 2,math.pi/2,'nan' if i==4 else -math.pi,math.pi/4,8.4-(i//40)*.01])
        saved_rules=[dict(id=10,name='Regola salvata',delay_ms=50,conditions=[dict(kind=1),dict(kind=0,phase_mask=8),dict(kind=8,elapsed_ms=250,reference=20)],outputs=[dict(kind='pyro',channel=1,delay_ms=75),dict(kind='servo',channel=1,delay_ms=200)]),
                     dict(id=20,name='Riferimento salvato',delay_ms=0,conditions=[dict(kind=2,comparison=3,value=80)],outputs=[dict(kind='pyro',channel=2,delay_ms=0)])]
        starts=[dict(rule_id=10,state='recorded',t_ms=5,kind='pyro',channel=1),dict(rule_id=10,state='recorded',t_ms=150,kind='servo',channel=1),dict(rule_id=20,state='invalid',t_ms=None,kind='pyro',channel=2)]
        target.with_suffix('.json').write_text(json.dumps(dict(configuration=dict(rules=saved_rules),output_starts=starts)),encoding='utf-8')
        data=load_flight(target)
        assert data['samples']==2401 and len(data['rows'])<=1201
        assert data['rows'][0][:4]==(0,0,100,-9.5)
        before=matrix((12,3,-40)); after=matrix(data['rows'][0][4:7])
        expected=[before[2],before[1],[-v for v in before[0]]]
        assert max(abs(after[i][j]-expected[i][j]) for i in range(3) for j in range(3))<1e-8
        assert data['rows'][0][7:]==(90,-180,45,8.4) # Exactly one rad/s -> deg/s conversion; battery in volts.
        assert data['phases']==[(0,'PAD'),(.005,'BOOST'),(.010,'COAST')] # Before plot decimation.
        assert next(r for r in data['rows'] if r[0]==.030)[4] is None # A skipped invalid sample still breaks the curve.
        assert next(r for r in data['rows'] if r[0]==.030)[8] is None
        legacy=Path(folder)/'minimal.csv'; legacy.write_text('t_ms,z_est,vz_est\n0,0,0\n5,1,2\n',encoding='utf-8')
        assert load_flight(legacy)['rows'][0][3:]==(None,)*8 # Missing optional values are never fake zeroes.
        original=controller.state.flight; controller.state.flight=data; window.recorded_flights_page.render(controller.state)
        details=window.recorded_flights_page.flight_details
        group=next(details.topLevelItem(i) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Regole del volo')
        assert group.child(0).text(0)=='Regola salvata' and group.child(0).childCount()==5
        assert [group.child(0).child(i).text(1) for i in range(3)]==['Apogeo rilevato','Fase: DESCENT','Da regola «Riferimento salvato»: 250 ms']
        assert group.child(1).child(0).text(1)=='Quota >= 80 m'
        def detail_values(): return {details.topLevelItem(i).text(0):details.topLevelItem(i).text(1) for i in range(details.topLevelItemCount())}
        values=detail_values(); assert values['Bias giroscopio']=='Non disponibile' and values['Orientamento scheda']=='Non disponibile'
        data['metadata'].update(sensor_mode=0,output_mode=0,calibration=dict(gyro_calibrated=True,sensor_axes=True,
            reference_altitude_m=123.25,up_axis='-Z',gyro_bias_radps=[.03125,-.0625,.125],accel_bias_mps2=[.25,0,-.5]))
        window.recorded_flights_page._flight_path=None; window.recorded_flights_page.render(controller.state)
        values=detail_values()
        assert values['Bias giroscopio']=='X: 0.03125  /  Y: -0.0625  /  Z: 0.125 rad/s'
        assert values['Riferimento barometrico']=='123.250 m' and values['Orientamento scheda']=='-Z'
        assert 'X: 0.25' in values['Bias accelerometro']
        assert values['Modalit'+chr(224)]=='Volo reale '+chr(183)+' uscite fisiche'
        group=next(details.topLevelItem(i) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Regole del volo')
        assert [(group.child(0).child(i).text(0),group.child(0).child(i).text(1)) for i in (3,4)]==[('Uscita · PYRO 1','Ritardo canale 75 ms'),('Uscita · SERVO 1','Ritardo canale 200 ms')]
        assert group.child(1).child(1).text(1)=='Ritardo canale 0 ms'
        count=next(details.topLevelItem(i) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Regole azionate')
        assert count.text(1)=='1 / 2'
        assert not any(details.topLevelItem(i).text(0)=='Attivazioni delle uscite' for i in range(details.topLevelItemCount()))
        plots=window.recorded_flights_page.flight_plots
        assert [[c[0] for c in p.curves] for p in plots]==[[1],[2,3],[7,8,9],[4,5,6],[10]]
        assert window.recorded_flights_page.flight_motion.curves[0][4]!=window.recorded_flights_page.flight_motion.curves[1][4] # Separate m/s and m/s² scales.
        with patch.object(controller,'_job') as job:
            window.recorded_flights_page.flight_actuators.setChecked(False)
            assert all(not p.show_actuators and p.show_states for p in plots)
            window.recorded_flights_page.flight_states.setChecked(False)
            assert all(not p.show_actuators and not p.show_states for p in plots)
            window.recorded_flights_page.flight_actuators.setChecked(True)
            assert all(p.show_actuators and not p.show_states for p in plots)
            window.recorded_flights_page.flight_states.setChecked(True)
            job.assert_not_called()
        for plot in plots:
            assert plot.rows is data['rows'] and plot.phases==data['phases'] and plot.markers==[(.005,'PYRO 1'),(.150,'SERVO 1')]
            assert not plot.grab().isNull()
            paths,segments=plot._paths,plot._segments
            plot.grab() # Repainting unchanged data must reuse inspected geometry.
            assert plot._paths is paths and plot._segments is segments
            # Hover uses painted geometry, including the acceleration's independent axis.
            from PySide6.QtCore import QEvent,QPointF,Qt
            from PySide6.QtGui import QMouseEvent
            for curve in plot.curves:
                a,b,first,last,_=next(segment for segment in plot._segments[100:] if segment[4]==curve)
                position=(a+b)/2; hover=plot._nearest_sample(position)
                assert curve[1]+':' in hover[1] and hover[1].endswith(curve[2]) and 'Tempo:' in hover[1]
                assert hover[2]==curve[0]
                with patch('perpetual_app.flight_plot.QToolTip.showText') as show:
                    event=QMouseEvent(QEvent.MouseMove,position,QPointF(plot.mapToGlobal(position.toPoint())),Qt.NoButton,Qt.NoButton,Qt.NoModifier)
                    QApplication.sendEvent(plot,event)
                    assert plot._hover==hover and show.call_args.args[1]==hover[1]
            assert plot._nearest_sample(QPointF(5,5)) is None
            with patch('perpetual_app.flight_plot.QToolTip.hideText') as hide:
                QApplication.sendEvent(plot,QEvent(QEvent.Leave))
                assert plot._hover is None and hide.called
        # Real resize/data/curve changes must invalidate the paint cache and hover.
        from perpetual_app.flight_plot import FlightPlot
        probe=FlightPlot([(1,'Altitude','m','#123456',0)])
        probe.resize(400,240); probe.set_data([(0,0,10),(1,1,20)]); probe.grab()
        previous_x=probe._segments[-1][1].x()
        probe.resize(700,240); probe.grab()
        assert probe._segments[-1][1].x()>previous_x
        probe.set_data([(0,5,10),(1,7,20)]); probe.grab()
        assert '7.000 m' in probe._nearest_sample(probe._segments[-1][1])[1]
        probe.curves=[(2,'Speed','m/s','#123456',0)]; probe.grab()
        assert 'Speed: 20.000 m/s' in probe._nearest_sample(probe._segments[-1][1])[1]
        probe.close()
        orientation=window.recorded_flights_page.flight_orientation
        yaw=(178,179,-179,-178,2,0,180,180,None,180)
        orientation.set_data([(i,0,0,0,None,None,value) for i,value in enumerate(yaw)])
        orientation.grab()
        # Do not connect either a 360-degree wrap, an exact half-turn, or a missing sample.
        connections=[(a[0],b[0]) for _,_,a,b,_ in orientation._segments if a is not b]
        assert connections==[(0,1),(2,3),(4,5),(6,7)]
        point=next(a for a,_,r,_,_ in orientation._segments if r[0]==9)
        assert '180.000' in orientation._nearest_sample(point)[1] # Isolated samples remain visible/inspectable.
        # The illustrative 2S trace is separate from real/missing CSV telemetry.
        legacy.with_suffix('.json').write_text(json.dumps(dict(sensor_mode=1)),encoding='utf-8')
        simulated=load_flight(legacy)
        assert simulated['simulated_battery'] and [r[11] for r in simulated['rows']]==[8.4,8.2]
        assert all(r[10] is None for r in simulated['rows'])
        controller.state.flight=simulated; window.recorded_flights_page.render(controller.state)
        assert [c[1] for c in window.recorded_flights_page.flight_battery.curves]==['Tensione batteria']
        def flight_kind():
            return next(details.topLevelItem(i).text(1) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Tipo di volo')
        assert flight_kind()=='Simulato'
        assert not window.recorded_flights_page.flight_battery.grab().isNull()
        legacy.with_suffix('.json').write_text(json.dumps(dict(sensor_mode=0)),encoding='utf-8')
        hardware=load_flight(legacy)
        assert not hardware['simulated_battery'] and len(hardware['rows'][0])==11
        window.recorded_flights_page._flight_path=None; controller.state.flight=hardware; window.recorded_flights_page.render(controller.state)
        assert flight_kind()=='Reale' and [c[0] for c in window.recorded_flights_page.flight_battery.curves]==[10]
        data['metadata'].pop('sensor_mode') # Restore missing metadata after the calibration-details check.
        controller.state.flight=data; window.recorded_flights_page.render(controller.state)
        assert len(window.recorded_flights_page.flight_battery.curves)==1 and flight_kind()=='Non disponibile'
        app.processEvents(); controller.state.flight=original; window.recorded_flights_page.render(controller.state)



def check_recorded_flights(app,wait):
    """Real flash-format/download path inside the PC firmware fixture, not fabricated CSV."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
        # Real flash-format/download path inside the PC firmware fixture, not fabricated CSV.
        backend.request('flights init'); window.navigate(3)
        assert window.recorded_flights_page.flights_empty.isHidden()
        gate=threading.Event(); backend.gates['flights list 0']=gate
        window.recorded_flights_page.list_flights_button.click(); wait(lambda:'flights list 0' in backend.commands)
        assert controller.state.foreground and window.recorded_flights_page.flights_empty.isHidden()
        gate.set(); wait(lambda:not controller.state.foreground); backend.gates.pop('flights list 0')
        assert not controller.state.flights and not controller.state.error
        assert controller.state.flights_loaded and window.recorded_flights_page.flights_empty.isVisible()
        assert window.recorded_flights_page.flights_empty.text()=='Nessun volo in memoria.' and not window.feedback.isVisible()
        assert window.recorded_flights_page.flights_table.rowCount()==0 and not window.recorded_flights_page.download_button.isEnabled()
        with patch.object(backend,'request',side_effect=CommandError('flights recover',dict(code='FLASH_WAKE_FAILED',ok=False))):
            window.recorded_flights_page.list_flights_button.click(); wait(lambda:not controller.state.foreground)
        assert controller.state.error and not controller.state.flights_loaded and window.recorded_flights_page.flights_empty.isHidden()
        backend.request('hil begin 0 200 logged_body record 4096')
        for i in range(22): backend.request(f'hil sample {i*5} 0 0 9.80665 0 0 0 101325')
        backend.request('hil stop completed')
        controller.list_flights(); wait(lambda:not controller.state.foreground)
        assert len(controller.state.flights)==1
        assert window.recorded_flights_page.flights_empty.isHidden()
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'flight.csv'
            window.navigate(3)
            assert window.recorded_flights_page.download_feedback.currentIndex()==0 and not window.recorded_flights_page.download_progress.isVisible()
            with patch('perpetual_app.recorded_flights_page.QFileDialog.getSaveFileName',return_value=('','CSV (*.csv)')):
                window.recorded_flights_page.download_button.click()
            assert not controller.state.download and not window.recorded_flights_page.download_progress.isVisible()
            gate=threading.Event(); backend.gates['flights packed 1 0 0']=gate
            with patch('perpetual_app.recorded_flights_page.QFileDialog.getSaveFileName',return_value=(str(target),'CSV (*.csv)')):
                window.recorded_flights_page.download_button.click()
            wait(lambda:'flights packed 1 0 0' in backend.commands and controller.state.download.get('total')==22)
            assert window.recorded_flights_page.download_progress.isVisible() and window.recorded_flights_page.download_result.isHidden()
            assert window.recorded_flights_page.download_progress.maximum()==100 and window.recorded_flights_page.download_progress.value()<100
            assert window.recorded_flights_page.download_progress.text()=='0% · Calcolo tempo…'
            controller._on_progress(controller._epoch-1,11,22,10) # Results from a previous connection cannot change ETA.
            assert controller.state.download['eta_seconds'] is None
            controller._on_progress(controller._epoch,11,22,.5)
            assert controller.state.download['eta_seconds'] is None
            controller._on_progress(controller._epoch,11,22,10)
            assert controller.state.download['eta_seconds']==10 and window.recorded_flights_page.download_progress.text()=='50% · ~10 s rimanenti'
            controller._on_progress(controller._epoch,22,22,20)
            assert window.recorded_flights_page.download_progress.text()=='99% · Finalizzazione…'
            gate.set(); wait(lambda:not controller.state.foreground); backend.gates.pop('flights packed 1 0 0')
            assert not controller.state.error and controller.state.flight['samples']==22
            assert target.with_suffix('.json').exists() and len(window.recorded_flights_page.flight_plot.points)==22
            assert controller.state.download['state']=='completed' and controller.state.download['done']==22
            assert not window.recorded_flights_page.download_progress.isVisible() and window.recorded_flights_page.download_result.isVisible()
            assert window.recorded_flights_page.download_result.text()=='Scaricato in '+str(target.resolve())
            # Cancel during a real pending reply, after a chunk has been saved.
            cancelled=Path(folder)/'cancelled.csv'; marker=len(backend.commands)
            gate=threading.Event(); backend.gates['flights packed 1 1 0']=gate
            controller.download('1',cancelled)
            wait(lambda:'flights packed 1 1 0' in backend.commands[marker:] and controller.state.download.get('done')==9)
            # The app stages the export beside the destination and moves it only after completion.
            assert not cancelled.exists() and list(Path(folder).glob('.flight-download-*/cancelled.csv'))
            assert window.recorded_flights_page.download_button.text()=='Annulla' and window.recorded_flights_page.download_button.isEnabled()
            assert window.recorded_flights_page.download_button.objectName()=='downloadCancel'
            window.recorded_flights_page.download_button.click()
            assert controller.state.download['state']=='cancelling' and controller.state.foreground
            assert window.recorded_flights_page.download_button.text()=='Annullamento…' and not window.recorded_flights_page.download_button.isEnabled()
            gate.set(); wait(lambda:not controller.state.foreground); backend.gates.pop('flights packed 1 1 0')
            assert controller.state.download['state']=='cancelled' and not controller.state.error and controller.state.connected
            assert window.recorded_flights_page.download_result.text()=='Download annullato' and not window.recorded_flights_page.download_progress.isVisible()
            assert window.recorded_flights_page.download_button.text()=='Scarica selezionato' and window.recorded_flights_page.download_button.isEnabled()
            assert not cancelled.exists() and not cancelled.with_suffix('.json').exists()
            assert not list(Path(folder).glob('.flight-download-*')) # Cancelling removes the staging folder too.
            assert [command for command in backend.commands[marker:] if command.startswith('flights packed ')]==['flights packed 1 0 0','flights packed 1 1 0']
            assert backend.request('flights info 1')['samples']==22 # Reply drained; remote recording and connection are intact.
            controller.download('1',cancelled); wait(lambda:not controller.state.foreground)
            assert controller.state.download['state']=='completed' and cancelled.with_suffix('.json').exists()
            for page in range(6):
                window.navigate(page); app.processEvents(); assert window.grab().width()>=1120
            check_flight_plots(window,controller,app)
            # The save dialog confirms overwrites; the app replaces existing files only after a complete staged export.
            target.write_text('old'); target.with_suffix('.json').write_text('old')
            controller.download('1',target); wait(lambda:not controller.state.foreground)
            assert controller.state.download['state']=='completed' and window.recorded_flights_page.download_result.text()=='Scaricato in '+str(target.resolve())
            assert target.read_text()!='old' and target.with_suffix('.json').read_text()!='old'
            assert not list(Path(folder).glob('.flight-download-*')) and not window.recorded_flights_page.download_progress.isVisible()
            # From the page, one app confirmation names both files replaced by the download.
            dialog='perpetual_app.recorded_flights_page.QFileDialog.getSaveFileName'; question='perpetual_app.recorded_flights_page.QMessageBox.question'
            marker=len(backend.commands)
            with patch(dialog,return_value=(str(target),'CSV (*.csv)')), patch(question,return_value=QMessageBox.No) as ask:
                window.recorded_flights_page.download_button.click()
            assert 'flight.csv' in ask.call_args[0][2] and 'flight.json' in ask.call_args[0][2]
            assert not controller.state.foreground and len(backend.commands)==marker # Declining starts no download.
            with patch(dialog,return_value=(str(target),'CSV (*.csv)')), patch(question,return_value=QMessageBox.Yes):
                window.recorded_flights_page.download_button.click()
            wait(lambda:not controller.state.foreground); assert controller.state.download['state']=='completed'
            target.with_suffix('.json').unlink() # Only the metadata file left behind still asks.
            fresh=Path(folder)/'fresh.csv'; fresh.with_suffix('.json').write_text('old')
            with patch(dialog,return_value=(str(fresh),'CSV (*.csv)')), patch(question,return_value=QMessageBox.No) as ask:
                window.recorded_flights_page.download_button.click()
            assert 'fresh.json' in ask.call_args[0][2] and 'fresh.csv' not in ask.call_args[0][2] and not fresh.exists()
            fresh.with_suffix('.json').unlink()
            with patch(dialog,return_value=(str(fresh),'CSV (*.csv)')), patch(question) as ask:
                window.recorded_flights_page.download_button.click()
            wait(lambda:not controller.state.foreground)
            assert not ask.called and fresh.exists() # New files need no extra confirmation.
            with patch('download_flight.download',side_effect=CommandError('flights info 1',dict(ok=False,code='Corrupt'))):
                controller.download('1',Path(folder)/'corrupt.csv'); wait(lambda:not controller.state.foreground)
            assert window.recorded_flights_page.download_result.text()=='Corrupted flight' and controller.state.error=='Corrupted flight'
            assert not window.recorded_flights_page.download_progress.isVisible()
            backend.request('hil begin 0 200 logged_body record 4096'); backend.request('hil stop completed')
            empty=Path(folder)/'empty_flight.csv'; controller.download('2',empty); wait(lambda:not controller.state.foreground)
            assert empty.exists() and empty.with_suffix('.json').exists()
            assert controller.state.download['state']=='completed' and window.recorded_flights_page.download_result.text()=='Scaricato in '+str(empty.resolve())
            assert 'preview not available' in controller.state.error # An empty flight still downloads successfully.
            controller.list_flights(); wait(lambda:not controller.state.foreground)
            assert [e['sequence'] for e in controller.state.flights]==['2','1']
            assert window.recorded_flights_page.flights_table.item(0,0).text()=='2'
            assert window.recorded_flights_page.flights_table.currentRow()==1 # The selected flight 1 moved down; preserve its identity.
            selected=Path(folder)/'selected.csv' # New file: this checks flight identity, not the overwrite prompt.
            with patch.object(controller,'download') as download, patch('perpetual_app.recorded_flights_page.QFileDialog.getSaveFileName',return_value=(str(selected),'CSV (*.csv)')):
                window.recorded_flights_page.download_selected()
                download.assert_called_once_with('1',str(selected))
