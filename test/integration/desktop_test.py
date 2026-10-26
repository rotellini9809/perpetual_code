"""Optional Qt tests: actual firmware configuration/storage plus asynchronous views."""
from concurrent.futures import Future
from copy import deepcopy
import csv
import json
import math
import os
from pathlib import Path
from unittest.mock import patch
import subprocess
import sys
import tempfile
import threading
import time

os.environ['QT_QPA_PLATFORM']='offscreen'
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT/'tools/replay')]
from PySide6.QtWidgets import QApplication,QMessageBox
from PySide6.QtGui import QFontDatabase
from device_client import Client,CommandError,ConnectionLost
from perpetual_app.configuration import Plan,read_plan,stage_plan,removal_order,plan_commands,next_rule_id,condition_text,rule_label
from perpetual_app.controller import Controller,error_text,load_flight
from perpetual_app.dialogs import RuleDialog,ConditionDialog,OutputDialog,ChannelDialog
from perpetual_app.window import MainWindow
from perpetual_app.sensor_details import SensorHistory
from perpetual_app.flight_plot import axis_scale,axis_scales
from perpetual_app.orientation import euler_x_up
from perpetual_app.ports import Port,describe_port,candidates


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
        original=controller.state.flight; controller.state.flight=data; window._render_flights(controller.state)
        details=window.flight_details
        group=next(details.topLevelItem(i) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Regole del volo')
        assert group.child(0).text(0)=='Regola salvata' and group.child(0).childCount()==5
        assert [group.child(0).child(i).text(1) for i in range(3)]==['Apogeo rilevato','Fase: DESCENT','Da regola «Riferimento salvato»: 250 ms']
        assert group.child(1).child(0).text(1)=='Quota >= 80 m'
        def detail_values(): return {details.topLevelItem(i).text(0):details.topLevelItem(i).text(1) for i in range(details.topLevelItemCount())}
        values=detail_values(); assert values['Bias giroscopio']=='Non disponibile' and values['Orientamento scheda']=='Non disponibile'
        data['metadata'].update(sensor_mode=0,output_mode=0,calibration=dict(gyro_calibrated=True,sensor_axes=True,
            reference_altitude_m=123.25,up_axis='-Z',gyro_bias_radps=[.03125,-.0625,.125],accel_bias_mps2=[.25,0,-.5]))
        window._flight_path=None; window._render_flights(controller.state)
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
        plots=window.flight_plots
        assert [[c[0] for c in p.curves] for p in plots]==[[1],[2,3],[7,8,9],[4,5,6],[10]]
        assert window.flight_motion.curves[0][4]!=window.flight_motion.curves[1][4] # Separate m/s and m/s² scales.
        with patch.object(controller,'_job') as job:
            window.flight_actuators.setChecked(False)
            assert all(not p.show_actuators and p.show_states for p in plots)
            window.flight_states.setChecked(False)
            assert all(not p.show_actuators and not p.show_states for p in plots)
            window.flight_actuators.setChecked(True)
            assert all(p.show_actuators and not p.show_states for p in plots)
            window.flight_states.setChecked(True)
            job.assert_not_called()
        for plot in plots:
            assert plot.rows is data['rows'] and plot.phases==data['phases'] and plot.markers==[(.005,'PYRO 1'),(.150,'SERVO 1')]
            assert not plot.grab().isNull()
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
        orientation=window.flight_orientation
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
        controller.state.flight=simulated; window._render_flights(controller.state)
        assert [c[1] for c in window.flight_battery.curves]==['Tensione batteria']
        def flight_kind():
            return next(details.topLevelItem(i).text(1) for i in range(details.topLevelItemCount()) if details.topLevelItem(i).text(0)=='Tipo di volo')
        assert flight_kind()=='Simulato'
        assert not window.flight_battery.grab().isNull()
        legacy.with_suffix('.json').write_text(json.dumps(dict(sensor_mode=0)),encoding='utf-8')
        hardware=load_flight(legacy)
        assert not hardware['simulated_battery'] and len(hardware['rows'][0])==11
        window._flight_path=None; controller.state.flight=hardware; window._render_flights(controller.state)
        assert flight_kind()=='Reale' and [c[0] for c in window.flight_battery.curves]==[10]
        data['metadata'].pop('sensor_mode') # Restore missing metadata after the calibration-details check.
        controller.state.flight=data; window._render_flights(controller.state)
        assert len(window.flight_battery.curves)==1 and flight_kind()=='Non disponibile'
        app.processEvents(); controller.state.flight=original; window._render_flights(controller.state)


def check_connections(app,wait):
    from types import SimpleNamespace
    from PySide6.QtCore import QSettings
    usb=Port('COM43','usb','usb:test','RP2350',True)
    radio=Port('COM44','bluetooth','bluetooth:test','JDY-34',True)
    def info(hwid,vid=None,pid=None):
        return SimpleNamespace(device='COM50',hwid=hwid,vid=vid,pid=pid,serial_number='ABC',description='port')
    incoming=describe_port(info(r'BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}\A&000000000000_00000000'),lambda _: 'Perpetual')
    outgoing=describe_port(info(r'BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}\A&20211125130A_C00000000'),lambda _: 'Perpetual')
    assert not incoming.automatic and outgoing.automatic
    assert candidates([incoming,outgoing],{'bluetooth':''})==[outgoing]
    assert describe_port(info('USB',0x2E8A,0x000F)).automatic
    assert not describe_port(info('USB',0x1234,0x5678)).automatic
    assert not describe_port(info('ACPI')).automatic
    assert candidates([radio,usb])==[usb,radio]

    class Link:
        def __init__(self,port):
            self.port=port; self.transport='usb' if port==usb.device else 'bluetooth'
            self.state='connecting'; self.ready=Future(); self.closed=Future(); self.close_called=False
        def connect(self): return self.ready
        def succeed(self):
            self.state='connected'; self.ready.set_result(dict(device='Perpetual',protocol_version=1))
        def snapshot(self): return dict(state=self.state,transport=self.transport,port=self.port,reply_age_s=0,error=None)
        def disconnect(self,wait=False): self.close_called=True; self.state='disconnecting'; return self.closed
        def finish_close(self): self.state='disconnected'; self.closed.set_result(None)

    links=[]
    def make(port):
        link=Link(port); links.append(link); return link
    controller=Controller(make,lambda:[radio,usb])
    controller.refresh=lambda:None
    controller._last_status=controller._last_sensors=time.monotonic()+1000000
    window=MainWindow(controller)
    with tempfile.TemporaryDirectory() as folder:
        window.settings=QSettings(str(Path(folder)/'preferences.ini'),QSettings.Format.IniFormat)
        window.show(); app.processEvents()
        try:
            assert not window.ports.isEditable() and not window.manual_controls.isVisible()
            window.connect_button.click()
            assert not window.connect_button.isEnabled() and window.connect_button.text()=='Connetti'
            assert 'Connessione in corso' in window.feedback.text()
            wait(lambda:len(links)==1); assert links[0].port==usb.device
            links[0].ready.set_exception(PermissionError('Access denied'))
            wait(lambda:links[0].close_called)
            assert len(links)==1 and not window.connect_button.isEnabled()
            links[0].finish_close(); wait(lambda:len(links)==2)
            assert links[1].port==radio.device
            links[1].succeed(); wait(lambda:controller.state.connected)
            assert window.transport.text()=='Bluetooth · COM44' and window.connect_button.text()=='Disconnetti'
            # Lost sessions close themselves. No manual Disconnect and no replay/reconnect.
            links[1].state='unresponsive'; controller.tick()
            assert not window.connect_button.isEnabled() and window.connect_button.text()=='Connetti'
            links[1].finish_close(); wait(lambda:controller.connection is None)
            assert window.connect_button.isEnabled() and len(links)==2
            window.connect_button.click(); wait(lambda:len(links)==3)
            assert links[2].port==usb.device # USB wins even after Bluetooth was last used.
            links[2].succeed(); wait(lambda:controller.state.connected)
            assert window.transport.text()=='USB · COM43'
            window.connect_button.click(); links[2].finish_close(); wait(lambda:controller.can_close())
            # Both candidates fail; the next click starts a fresh search directly.
            window.connect_button.click(); wait(lambda:len(links)==4)
            links[3].ready.set_exception(PermissionError('busy')); wait(lambda:links[3].close_called)
            links[3].finish_close(); wait(lambda:len(links)==5)
            links[4].ready.set_exception(TimeoutError('no reply')); wait(lambda:links[4].close_called)
            links[4].finish_close(); wait(lambda:not controller.connection_busy)
            assert controller.connection is None and window.connect_button.isEnabled()
            assert 'Porta occupata' in controller.state.error and 'Nessuna risposta' in controller.state.error
            # A pending open/handshake can still be stopped by closing the app.
            window.connect_button.click(); wait(lambda:len(links)==6)
            controller.disconnect(); links[5].finish_close(); links[5].succeed()
            wait(lambda:controller.can_close()); assert not controller.state.connected and len(links)==6
            controller.session.port_scanner=lambda:[]
            window.connect_button.click(); wait(lambda:not controller.connection_busy)
            assert 'associa' in controller.state.error and window.connect_button.isEnabled()
        finally:
            if controller.connection:
                controller.disconnect()
                if not links[-1].closed.done(): links[-1].finish_close()
                wait(lambda:controller.can_close())
            window._allow_close=True; window.close(); app.processEvents()

    # Only the initial background connection failure is silent, including console logging.
    links.clear(); errors=[]
    controller=Controller(make,lambda:[usb]); controller.refresh=lambda:None
    controller._last_status=controller._last_sensors=time.monotonic()+1000000
    controller.logged.connect(lambda kind,text:errors.append(text) if kind=='error' else None)
    try:
        controller.connect_auto(quiet=True); wait(lambda:len(links)==1)
        links[0].ready.set_exception(PermissionError('startup busy')); wait(lambda:links[0].close_called)
        links[0].finish_close(); wait(lambda:controller.can_close())
        assert controller.state.connection['state']=='disconnected' and not controller.state.error and not errors
        controller.connect_auto(); wait(lambda:len(links)==2)
        links[1].ready.set_exception(PermissionError('manual busy')); wait(lambda:links[1].close_called)
        links[1].finish_close(); wait(lambda:controller.can_close())
        assert controller.state.error and errors and controller.state.connection['state']=='error'
        controller.connect_auto(quiet=True); wait(lambda:len(links)==3)
        links[2].succeed(); wait(lambda:controller.state.connected)
        links[2].state='unresponsive'; controller.tick()
        links[2].finish_close(); wait(lambda:controller.can_close())
        assert controller.state.error and controller.state.connection['state']=='error' # Established-session loss stays visible.
        controller.session.port_scanner=lambda:[]
        controller.connect_auto(quiet=True); wait(lambda:controller.can_close())
        assert not controller.state.error and controller.state.connection['state']=='disconnected'
    finally:
        if controller.connection:
            controller.disconnect()
            if not links[-1].closed.done(): links[-1].finish_close()
            wait(controller.can_close)
        controller.shutdown()

    # Ambiguous boards need an explicit choice; no random first port is opened.
    other=Port('COM45','usb','usb:other','RP2350',True); choices=[]; links.clear()
    controller=Controller(make,lambda:[usb,other,radio]); controller.refresh=lambda:None
    controller._last_status=controller._last_sensors=time.monotonic()+1000000
    controller.port_choice.connect(lambda values:choices.extend(values))
    try:
        controller.connect_auto(); wait(lambda:bool(choices)); assert len(choices)==2 and not links
        controller.choose_port(other); assert links[0].port==other.device
        controller.disconnect(); links[0].finish_close(); wait(lambda:controller.can_close())
    finally: controller.shutdown()

    # Closing while discovery runs invalidates its result, without opening a late port.
    entered=threading.Event(); release=threading.Event(); links.clear()
    def discover_late(): entered.set(); release.wait(3); return [usb]
    controller=Controller(make,discover_late); controller.refresh=lambda:None
    try:
        controller.connect_auto(); wait(entered.is_set)
        controller.disconnect(); assert not controller.can_close()
        release.set(); wait(controller.can_close)
        assert not links and not controller.state.connected
    finally: release.set(); controller.shutdown()


def check_sensor_history():
    reading=dict(imu_ready=True,imu_sample_valid=True,accel_mps2=[1,2,3],gyro_radps=[.1,.2,.3],baro_ready=True,baro_sample_valid=True,pressure_pa=101325,temperature_c=24.5)
    history=SensorHistory()
    for stamp in range(0,60250,250): assert history.append(dict(reading,uptime_ms=stamp))
    assert history.extent==(0,60) and len(history.samples)==241
    assert history.append(dict(reading,uptime_ms=60250))
    assert history.extent==(.25,60.25) and history.samples[0][0]==250 and len(history.samples)==241
    assert not history.append(dict(reading,uptime_ms=60250)) # No duplicate points for the same response timestamp.
    assert history.append(dict(reading,uptime_ms=60500,imu_sample_valid=False,baro_sample_valid=False))
    assert history.series('pressure_pa',1)[0][-1][1] is None
    assert all(series[-1][1] is None for series in history.series('accel_mps2',3)) # Break curves across invalid readings.
    assert history.append(dict(reading,uptime_ms=130000)) and len(history.samples)==1
    assert not history.append(reading) # Old firmware must not get fabricated local timestamps.
    assert history.append(dict(reading,uptime_ms=100)) and history.extent==(0,0) and len(history.samples)==1 # Reboot.
    history.clear(); history.append(dict(reading,uptime_ms=0xFFFFFFF0)); history.append(dict(reading,uptime_ms=0x20))
    assert history.elapsed_ms==48 and len(history.samples)==2 # uint32 rollover is continuous.
    history.clear()
    for stamp,value in ((0,20),(1000,30),(6000,40)):
        history.append(dict(reading,uptime_ms=stamp,temperature_c=value))
    assert history.temperature_average()==35 # The 20-degree sample is older than five seconds.
    assert history.samples[-1][1]['temperature_c']==(40.0,) # Display smoothing does not change raw data.
    history.append(dict(reading,uptime_ms=6500,temperature_c=float('nan')))
    assert history.temperature_average() is None # Do not reuse an average for an invalid current reading.
    history.append(dict(reading,uptime_ms=7000,temperature_c=50))
    assert history.temperature_average()==45
    history.append(dict(reading,uptime_ms=10,temperature_c=10))
    assert history.temperature_average()==10 # No average survives a board reboot.


class Native:
    """PC firmware stdin endpoint. Qt UI tests supplement hardware-only status APIs."""
    def __init__(self):
        self.process=subprocess.Popen([str(ROOT/'.pio/tests/bin/command_device.exe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.client=Client(self); self.state='disconnected'; self.active=False; self.commands=[]; self.threads=[]; self.gates={}
        self.output_mode='VIRTUAL'; self.reject_physical=False; self.ready_to_arm=False; self.prepared_mask=0
        self.sensor_time=0; self.transport='usb'; self.failed_pyros=0; self.power_mode='active'
        self.client.request('config orientation +X')
    def write(self,data): self.process.stdin.write(data); self.process.stdin.flush(); return len(data)
    def read(self,count): return self.process.stdout.read(count)
    def connect(self):
        self.state='connected'; f=Future(); f.set_result(dict(device='Perpetual',protocol_version=1)); return f
    def snapshot(self): return dict(state=self.state,transport=self.transport,port='PC_FIXTURE',reply_age_s=0,error=None)
    def request(self,command,timeout=5):
        self.commands.append(command); self.threads.append(threading.get_ident())
        if command in self.gates: self.gates[command].wait(3)
        if self.state=='disconnected': raise ConnectionLost('Fixture closed')
        if command in ('preflight','sensors'): self.power_mode='active'
        if command=='arm': raise CommandError(command,dict(code='IMU_UNAVAILABLE',reasons=['IMU_UNAVAILABLE'],ready=False,prepared_servo_mask=self.prepared_mask,ok=False))
        if command=='preflight':
            orientation_ready=self.client.request('config orientation')['orientation_ready']
            return dict(ready=self.ready_to_arm and not self.failed_pyros and orientation_ready,
                        reasons=([] if self.ready_to_arm else ['IMU_UNAVAILABLE'])+(['CONTINUITY_FAULT'] if self.failed_pyros else [])+([] if orientation_ready else ['ORIENTATION_NOT_READY']),
                        failed_pyro_mask=self.failed_pyros,unprepared_servo_mask=1 & ~self.prepared_mask,prepared_servo_mask=self.prepared_mask,requested_mode='physical')
        if command=='preflight simulation': return dict(ready=True,reasons=[],requested_mode='simulation')
        if command=='pyro continuity': return dict(configured_pyro_mask=3,connected_pyro_mask=3 & ~self.failed_pyros,failed_pyro_mask=self.failed_pyros & 3,all_connected=not (self.failed_pyros & 3))
        if command=='sensors':
            self.sensor_time+=250
            return dict(imu_ready=False,baro_ready=False,baro_sample_valid=False,uptime_ms=self.sensor_time)
        if command.startswith('servo arm'):
            self.prepared_mask|=1; return dict(initial_position_commanded=True,prepared_servo_mask=self.prepared_mask)
        if command=='servo release':
            self.prepared_mask=0; return dict(released=True,prepared_servo_mask=0)
        if command in ('disarm','ds'):
            self.prepared_mask=0; self.active=False; return dict(arming_state='disarmed')
        if command.startswith('sim begin'):
            if 'physical' not in command and self.prepared_mask:
                raise CommandError(command,dict(code='PHYSICAL_SERVOS_PREPARED',ok=False))
            if 'physical' in command and self.reject_physical:
                raise CommandError(command,dict(code='CONTINUITY_FAULT',reasons=['CONTINUITY_FAULT'],failed_pyro_mask=1,ok=False))
            self.active=True; self.prepared_mask=0; self.output_mode='PHYSICAL' if 'physical' in command else 'VIRTUAL'
        if command=='sim stop': self.active=False
        if command.startswith('sim begin') or command in ('sim stop','sim status'):
            return dict(active=self.active,arming_state='armed' if self.active else 'disarmed',output_mode=self.output_mode,
                        source='hoppino',duration_ms=32995,total_samples=6600,
                        samples=1 if self.active else 0,t_ms=0,phase=0,z_est=0,starts=0,reason='running' if self.active else 'stopped')
        if command=='sim events': return dict(starts=[],phases=[],output_mode=self.output_mode)
        response=self.client.request(command,timeout)
        if command=='capabilities': response.update(physical_actions=True,physical_simulation=True)
        if command=='status': response.update(power_mode=self.power_mode,config_valid=self.client.request('config show')['available'],arming_state='armed' if self.active else 'disarmed',simulation_active=self.active,
                                             flight_mode='simulation' if self.active else 'none',output_mode=self.output_mode,outputs_active=bool(self.prepared_mask) or (self.active and self.output_mode=='PHYSICAL'))
        return response
    def close(self):
        for gate in self.gates.values(): gate.set()
        if self.process.poll() is None:
            self.process.stdin.close(); self.process.wait(timeout=3)
        self.process.stdout.close()
    def disconnect(self,wait=False):
        self.state='disconnected'; self.close(); f=Future(); f.set_result(None); return f


def run():
    app=QApplication([])
    check_sensor_history()
    for name in ('segoeui.ttf','segoeuib.ttf','seguisym.ttf'):
        path=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'/name
        if path.exists(): QFontDatabase.addApplicationFont(str(path))
    def wait(test):
        until=time.monotonic()+5
        while not test():
            assert time.monotonic()<until,'Qt operation did not finish'
            app.processEvents(); time.sleep(.003)
        app.processEvents()
    def raises(kind,operation):
        try: operation()
        except kind as error: return error
        raise AssertionError(f'Expected {kind.__name__}')

    # A real SIGINT goes through Qt close/cleanup, including repeated interrupts.
    import signal
    from PySide6.QtTest import QTest
    from perpetual_app.main import terminal_interrupt
    interrupt_controller=Controller(port_scanner=lambda:[]); interrupt_window=MainWindow(interrupt_controller); interrupt_window.show()
    previous=signal.getsignal(signal.SIGINT)
    try:
        with patch.object(interrupt_controller,'disconnect') as disconnect, patch.object(interrupt_controller,'can_close',return_value=False) as can_close:
            with terminal_interrupt(interrupt_window):
                signal.raise_signal(signal.SIGINT)
                wait(lambda:interrupt_window._closing)
                assert interrupt_window.isVisible() and disconnect.call_count==1
                signal.raise_signal(signal.SIGINT); QTest.qWait(150)
                assert interrupt_window.isVisible() and disconnect.call_count==1
                can_close.return_value=True
                wait(lambda:not interrupt_window.isVisible())
        assert signal.getsignal(signal.SIGINT)==previous
    finally:
        interrupt_window._allow_close=True; interrupt_window.close(); app.processEvents()

    check_connections(app,wait)

    backend=Native(); backend.state='connected'
    try:
        base,meta,view=read_plan(backend)
        assert view=='applied' and meta['available'] and len(base.rules)==2
        local=base.clone(); local.servos[0]['final_us']=1500
        local.rules[0]['outputs'].append(dict(kind='pyro',channel=3,delay_ms=125)) # No manual pyro settings required.
        local.rules[0]['outputs'].append(dict(kind='servo',channel=2,delay_ms=0)) # Rules can persist before servo setup.
        local.rules[0]['name']='New "label" \\ x'
        local.rules[0]['conditions'].append(dict(type='since_rule_ms',reference=20,elapsed_ms=50))
        stage_plan(backend,base,local)
        staged,_,view=read_plan(backend); assert staged==local and view=='draft'
        saved=backend.request('config apply'); assert all(saved[k] for k in ('applied','persisted','verified'))
        current,_,view=read_plan(backend); assert current==local and view=='applied'
        assert removal_order(current).index(10)<removal_order(current).index(20)
        error=raises(RuntimeError,lambda:stage_plan(backend,base,base.clone()))
        assert 'cambiata' in str(error) and not backend.request('config draft')['available']
        bad=current.clone(); bad.rules[0]['name']='bad\ncommand'
        start=len(backend.commands); raises(ValueError,lambda:stage_plan(backend,current,bad))
        assert len(backend.commands)==start # Local invalid input never creates a remote draft.
        duplicate=current.clone()
        duplicate.rules[1]['outputs'].append(deepcopy(duplicate.rules[0]['outputs'][0]))
        start=len(backend.commands); error=raises(ValueError,lambda:stage_plan(backend,current,duplicate))
        assert 'PYRO 1' in str(error) and current.rules[0]['name'] in str(error)
        assert len(backend.commands)==start # Duplicate ownership is explained before any board mutation.
        empty=current.clone(); empty.rules=[]
        error=raises(CommandError,lambda:stage_plan(backend,current,empty))
        assert error.code=='CONFIG_INVALID' and backend.request('config show')['available']
        stage_plan(backend,current,current.clone()) # Repairs an own partial/invalid draft safely.
        backend.request('config discard')
        backend.request('config begin')
        class Foreign:
            def request(self,command,**kwargs):
                if command=='control status': return dict(owner='bluetooth',transport='usb')
                return backend.request(command,**kwargs)
        start=len(backend.commands)
        raises(RuntimeError,lambda:stage_plan(Foreign(),current,current.clone()))
        assert 'config discard' not in backend.commands[start:] and backend.request('config draft')['available']
        backend.request('config discard')
    finally: backend.close()

    # IDs remain internal and immutable; named references preserve full uint32 identity.
    output=OutputDialog(value=dict(kind='pyro',channel=2))
    assert output.result_value()==dict(kind='pyro',channel=2,delay_ms=0)
    output.delay.setValue(150)
    assert output.result_value()['delay_ms']==150
    output.close()
    empty_servo=dict(calibrated=False,initial_us=0,final_us=0)
    servo=ChannelDialog(None,'servo',1,empty_servo,dict(servo_min_us=1000,servo_max_us=2000))
    servo.setStyleSheet((ROOT/'perpetual_app/assets/style.qss').read_text(encoding='utf-8'))
    servo.show(); app.processEvents()
    assert set(servo.pwm_labels)=={'initial_us','final_us'}
    for display in servo.pwm_labels.values():
        assert display.isVisible() and '1500</b>' in display.text()
        assert abs(display.geometry().center().x()-display.parentWidget().rect().center().x())<=1
    servo.sliders['final_us'].setValue(1800); assert '1800</b>' in servo.pwm_labels['final_us'].text()
    servo.sliders['initial_us'].setValue(1100)
    from PySide6.QtWidgets import QPushButton
    decrement=next(b for b in servo.findChildren(QPushButton) if b.text()=='−10')
    decrement.click(); assert '1090</b>' in servo.pwm_labels['initial_us'].text()
    servo.sliders['initial_us'].setValue(1000); decrement.click(); assert servo.sliders['initial_us'].value()==1000
    assert servo.result_value()==dict(calibrated=True,initial_us=1000,final_us=1800)
    assert empty_servo==dict(calibrated=False,initial_us=0,final_us=0)
    servo.reject()
    saved_servo=ChannelDialog(None,'servo',1,dict(calibrated=True,initial_us=1200,final_us=1800),{})
    assert saved_servo.result_value()==dict(calibrated=True,initial_us=1200,final_us=1800)
    saved_servo.reject()
    # One live request in flight; intermediate slider positions are replaced, never queued.
    from types import SimpleNamespace
    from PySide6.QtCore import QObject,Signal
    class Preview(QObject):
        servo_previewed=Signal(object)
        changed=Signal()
        def __init__(self):
            super().__init__(); self.state=SimpleNamespace(editable=True,connected=True); self.sent=[]; self.released=[]
        def preview_servo(self,channel,pwm): self.sent.append((channel,pwm)); return True
        def release_servo(self,channel): self.released.append(channel)
    preview=Preview()
    servo=ChannelDialog(None,'servo',2,empty_servo,{},preview)
    servo.apply_buttons[0].click(); wait(lambda:preview.sent==[(2,1500)])
    servo.live_buttons['final_us'].click()
    assert servo.live_buttons['final_us'].isChecked() and not servo.live_buttons['initial_us'].isEnabled()
    assert servo.sliders['final_us'].isEnabled() and not servo.sliders['initial_us'].isEnabled()
    assert not any(b.isEnabled() for b in servo.apply_buttons+sum(servo.step_buttons.values(),[]))
    for pwm in (1600,1700,1800): servo.sliders['final_us'].setValue(pwm)
    servo._send_position(); assert preview.sent==[(2,1500)] # Still waiting for the board.
    preview.servo_previewed.emit(dict(channel=2,pwm=1500))
    wait(lambda:preview.sent==[(2,1500),(2,1800)])
    preview.servo_previewed.emit(dict(channel=2,pwm=1800))
    assert not servo.feedback.text()
    servo.sliders['final_us'].setValue(1900); servo.live_buttons['final_us'].click()
    servo._send_position(); assert len(preview.sent)==2
    assert all(b.isEnabled() for b in servo.apply_buttons+list(servo.live_buttons.values()))
    servo.apply_buttons[1].click(); wait(lambda:len(preview.sent)==3)
    preview.servo_previewed.emit(dict(error='BUSY'))
    assert servo._live_key is None and servo.feedback.text()=='BUSY'
    servo.live_buttons['initial_us'].click()
    assert servo._live_key=='initial_us' and not servo.live_buttons['final_us'].isEnabled()
    assert servo.pwm_labels['initial_us'].isEnabled() and not servo.pwm_labels['final_us'].isEnabled()
    preview.state.editable=False; preview.state.connected=False; preview.changed.emit()
    assert not any(b.isEnabled() for b in list(servo.live_buttons.values())+servo.apply_buttons)
    servo.reject(); assert preview.released==[2] and not servo.preview_timer.isActive()
    rule=dict(id=0xFFFFFFFF,name='Stable ID',delay_ms=0,conditions=[],outputs=[])
    other=dict(id=7,name='Second rule',delay_ms=0,conditions=[],outputs=[])
    dialog=RuleDialog(None,rule,[rule,other])
    assert not hasattr(dialog,'identity')
    assert hasattr(dialog,'delay') and not hasattr(dialog,'confirmation')
    dialog.delay.setValue(250)
    dialog.name.setText('Renamed'); dialog.finish()
    assert dialog.rule['id']==0xFFFFFFFF and dialog.rule['name']=='Renamed' and dialog.rule['delay_ms']==250; dialog.close()
    reference=dict(type='since_rule_ms',reference=0xFFFFFFFF,elapsed_ms=1)
    c=ConditionDialog(value=reference,rules=[rule,other],exclude_id=other['id'])
    assert not c.reference.isEditable() and c.reference.findData(other['id'])<0
    assert c.reference.currentText()=='Stable ID' and c.result_value()['reference']==0xFFFFFFFF
    assert '4294967295' not in condition_text(reference,[rule,other]); c.close()
    c=ConditionDialog(value=reference,rules=[other])
    assert c.reference.currentData() is None
    raises(ValueError,c.result_value); c.close()
    assert next_rule_id(Plan(rules=[rule,other]))==1
    # Removed saved IDs and dangling references cannot be silently rebound to a new rule.
    local=Plan(rules=[dict(other,conditions=[dict(reference,reference=8)])])
    assert next_rule_id(local,Plan(rules=[dict(other,id=9)]))==10
    duplicates=[dict(rule,name='Same'),dict(other,name='Same')]
    assert rule_label(duplicates,rule['id'])!=rule_label(duplicates,other['id'])
    error=CommandError('config validate',dict(code='CONFIG_INVALID',validation='InvalidPyroSettings',rule_id=0,index=1,field=4))
    assert 'PYRO 2' in error_text(error) and 'regola 0' not in error_text(error)
    error=CommandError('config validate',dict(code='CONFIG_INVALID',validation='InvalidCondition',rule_id=rule['id'],index=0,field=2))
    assert 'Stable ID' in error_text(error,Plan(rules=[rule])) and '4294967295' not in error_text(error,Plan(rules=[rule]))

    backends=[]
    def factory(_): backend=Native(); backends.append(backend); return backend
    controller=Controller(factory); window=MainWindow(controller); window.show()
    main_thread=threading.get_ident()
    try:
        assert not window.arm_button.isEnabled() and not window.sim_start.isEnabled()
        assert not window.bistage_enabled.isChecked() and not window.bistage_min_speed.isEnabled()
        assert window.flight_state_sequence.text()==(' '+chr(8594)+' ').join(('PAD','BOOST','COAST','DESCENT','LANDED'))

        controller.connect_port('PC_FIXTURE')
        wait(lambda:controller.state.local is not None and not controller.state.foreground)
        backend=backends[-1]
        window.bistage_enabled.setChecked(True); assert window.bistage_min_speed.isEnabled()
        assert 'BOOST 2' in window.flight_state_sequence.text() and 'COAST 2' in window.flight_state_sequence.text()
        assert 'ABORT' in window.flight_state_sequence.text() and chr(8595) in window.flight_state_sequence.text()
        window.bistage_min_speed.setValue(42.5)
        assert window.bistage_min_speed.value()==42.5 and not window.bistage_min_speed.suffix()
        window.bistage_enabled.setChecked(False); assert not window.bistage_min_speed.isEnabled()
        assert window.bistage_min_speed.value()==42.5
        window.bistage_enabled.setChecked(True); assert window.machine_save.isEnabled()
        window.machine_save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.machine['bistage']
        assert controller.state.machine['minimum_coast_velocity']==42.5
        assert not window.machine_save.isEnabled()
        window.bistage_enabled.setChecked(False); window.machine_save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.machine['bistage']

        assert window.configuration_page.channels_table.rowCount()==8 and window.configuration_page.rules_tree.topLevelItemCount()==2
        for i,r in enumerate(controller.state.local.rules):
            assert window.configuration_page.rules_tree.topLevelItem(i).text(0)==r['name']
        assert not window.arm_button.isEnabled() and window.arm_button.property('blocked') and window.sim_start.isEnabled()
        assert window.arm_button.text()=='ARM' and window.arm_button.objectName()=='armFlight'
        assert not hasattr(window,'phase_metric') and not hasattr(window,'sensor_metric')
        assert [not check.isHidden() for check in window.servo_selection]==[True,False,False,False]
        assert window.check_pyros_button.isEnabled() and 'da verificare' in window.pyro_states[0].text()
        # Older firmware may lack orientation; its valid plan must still load on initial sync.
        old_request=backend.request
        for code in ('NO_DRAFT','INVALID_ARGUMENT','UNKNOWN_COMMAND'):
            def unsupported_orientation(command,timeout=5,code=code):
                if command=='config orientation': raise CommandError(command,dict(ok=False,code=code))
                return old_request(command,timeout)
            controller.state.base=None; controller.state.local=None
            with patch.object(backend,'request',side_effect=unsupported_orientation):
                controller.refresh(); wait(lambda:not controller.state.foreground)
            assert not controller.state.error and controller.state.base is not None
            assert controller.state.orientation==dict(supported=False)
            assert not window.servo_selection[0].isHidden() and window.check_pyros_button.isEnabled()
            assert 'Aggiorna il firmware' in window.orientation_panel.saved.text() and not window.orientation_panel.save.isEnabled()
        controller.refresh(); wait(lambda:not controller.state.foreground)
        assert controller.state.orientation['supported'] and controller.state.orientation['up_axis']=='+X'
        backend.failed_pyros=1
        preflight=deepcopy(controller.state.preflight); before=backend.commands.count('preflight')
        window.check_pyros_button.click(); wait(lambda:not controller.state.foreground)
        assert backend.commands.count('preflight')==before and 'pyro continuity' in backend.commands
        assert window.pyro_states[0].property('preparation')=='unprepared' and controller.state.preflight==preflight
        window.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert 'CONTINUITY_FAULT' in controller.state.preflight['reasons'] # Preflight still checks independently.
        backend.failed_pyros=0; window.check_pyros_button.click(); wait(lambda:not controller.state.foreground)
        assert all(state.property('preparation')=='prepared' for state in window.pyro_states[:2]) and window.pyro_rows[2].isHidden()
        assert 'CONTINUITY_FAULT' in controller.state.preflight['reasons'] # A manual pass does not approve arming.
        window.preflight_button.click(); wait(lambda:not controller.state.foreground)
        applied=controller.state.base
        assert [b.text() for b in window.configuration_page.channel_buttons]==['Modifica']
        assert all(window.configuration_page.channels_table.cellWidget(i,3) is None for i in (5,6,7))
        # Missing endpoints lead to the local editor without sending a preparation command.
        unconfigured=applied.clone(); unconfigured.servos[0]=deepcopy(empty_servo)
        controller.state.base=unconfigured; controller.state.local=unconfigured.clone(); window.render()
        window.servo_selection[0].setChecked(True)
        assert window.prepare_servos_button.text()=='Vai alla configurazione' and window.prepare_servos_button.objectName()=='configureServo'
        before=len(backend.commands)
        with patch('perpetual_app.configuration_page.ChannelDialog') as editor:
            editor.return_value.exec.return_value=0
            window.prepare_servos_button.click()
            assert editor.call_args.args[2]==1
        assert len(backend.commands)==before and window.pages.currentIndex()==1
        assert window.configuration_page.tabs.currentIndex()==1 and window.configuration_page.channels_table.currentRow()==4
        with patch('perpetual_app.configuration_page.ChannelDialog') as editor:
            editor.return_value.exec.return_value=1
            editor.return_value.result_value.return_value=dict(calibrated=True,initial_us=1200,final_us=1800)
            window.configuration_page.channel_buttons[0].click()
        assert controller.state.dirty and controller.state.local.servos[0]['initial_us']==1200
        assert not controller.state.base.servos[0]['calibrated'] and len(backend.commands)==before
        assert window.configuration_page.channel_buttons[0].text()=='Modifica'
        with patch('perpetual_app.configuration_page.ChannelDialog') as editor:
            window.prepare_servos_button.click(); editor.assert_not_called() # Already edited: guide to Apply instead.
        controller.state.base=applied; controller.state.local=applied.clone(); window.servo_selection[0].setChecked(False); window.render(); window.navigate(0)
        # Preparation uses applied channels; an unsaved rule edit cannot expose a new servo.
        draft=applied.clone(); draft.rules[0]['outputs'].append(dict(kind='servo',channel=4,delay_ms=0))
        controller.edit_plan(draft)
        assert window.servo_selection[3].isHidden() and not window.check_pyros_button.isEnabled()
        assert window.configuration_page.channels_table.cellWidget(7,3) is not None
        controller.edit_plan(applied.clone())
        assert window.configuration_page.channels_table.cellWidget(7,3) is None
        window.servo_selection[0].setChecked(True)
        without_servo=applied.clone()
        for r in without_servo.rules: r['outputs']=[a for a in r['outputs'] if a['kind']!='servo']
        controller.state.base=without_servo; controller.state.local=without_servo.clone(); window.render()
        assert all(check.isHidden() and not check.isChecked() for check in window.servo_selection)
        assert not window.prepare_servos_button.isEnabled() and not window.servo_empty.isHidden()
        controller.state.base=applied; controller.state.local=applied.clone(); window.render()
        assert not window.imu_label.isHidden() and not window.baro_label.isHidden()
        assert window.imu_label.palette().color(window.imu_label.foregroundRole()).name()=='#c43d3d'
        assert not window.sensor_refresh.isEnabled()
        sensor_state=controller.state.sensors
        sensor_status={key:controller.state.status.get(key) for key in ('imu_ready','baro_ready')}
        controller.state.status.update(imu_ready=True,baro_ready=True)
        controller.state.sensors=dict(imu_ready=True,imu_sample_valid=True,accel_mps2=[1.25,-2,9.8],gyro_radps=[.1,.2,.3],baro_ready=True,baro_sample_valid=True,pressure_pa=101325,temperature_c=24.5)
        window.render()
        assert window.imu_label.text().endswith('Disponibile') and window.baro_label.text().endswith('Disponibile')
        assert window.imu_label.palette().color(window.imu_label.foregroundRole()).name()=='#187f73'
        assert window.sensor_refresh.isEnabled()
        controller.state.sensors['imu_ready']=False; controller.state.status['imu_ready']=False; window.render()
        assert window.sensor_refresh.isEnabled() # Barometer alone is enough to open details.
        window.sensor_refresh.click(); details=window._sensor_details
        controller.state.sensors=sensor_state; controller.state.status.update(sensor_status); window.render()
        wait(lambda:details.history.samples)
        assert details.isVisible() and controller._sensor_stream and len(details.plots)==3
        assert details.message.isHidden()
        controller.timer.stop()
        before=backend.commands.count('sensors'); controller._last_status=time.monotonic()
        controller._last_sensors=time.monotonic()-.3; controller.tick(); wait(lambda:not controller._jobs)
        assert backend.commands.count('sensors')==before+1 # USB updates faster than 1 Hz.
        backend.transport='bluetooth'; controller._last_sensors=time.monotonic()-.5; controller.tick()
        assert backend.commands.count('sensors')==before+1 and not controller._jobs
        controller._last_sensors=time.monotonic()-1.1; controller.tick(); wait(lambda:not controller._jobs)
        assert backend.commands.count('sensors')==before+2 # Bluetooth is limited to 1 Hz.
        backend.transport='usb'; controller.session.poll()
        details.receive(dict(uptime_ms=backend.sensor_time+1000,imu_ready=True,imu_sample_valid=True,accel_mps2=[1.25,-2,9.8],gyro_radps=[.1,.2,.3],baro_ready=True,baro_sample_valid=True,pressure_pa=101325,temperature_c=24.5))
        assert details.temperature.text()=='24.5 °C' and details.plots['pressure_pa'].series[0][-1][1]==101325
        assert details.plots['accel_mps2'].series[2][-1][1]==9.8
        details.receive(dict(uptime_ms=backend.sensor_time+1500,baro_ready=True,baro_sample_valid=True,temperature_c=26.5))
        assert details.temperature.text()=='25.5 °C'
        details.receive(dict(uptime_ms=backend.sensor_time+2000,imu_ready=False,baro_ready=False))
        assert details.temperature.text()=='—' and details.plots['pressure_pa'].series[0][-1][1] is None
        details.close(); assert not controller._sensor_stream
        before=backend.commands.count('sensors'); controller._last_sensors=0; controller.tick()
        assert backend.commands.count('sensors')==before # Dashboard uses passive status; only live details acquire sensors.
        awake_status=dict(controller.state.status,imu_ready=False,baro_ready=True)
        controller._complete(controller._epoch,0,'poll',dict(status=awake_status),None)
        backend.power_mode='sleeping'; controller._last_status=0; controller.tick(); wait(lambda:not controller._jobs)
        assert controller.state.status['power_mode']=='sleeping' and not controller.state.preflight
        assert not window.power_badge.isHidden() and window.power_badge.text()=='POWER SAVE ACTIVE'
        assert window.readiness_title.text()=='Risparmio energetico' and not window.arm_button.isEnabled()
        assert window.imu_label.text()=='Non disponibile' and window.baro_label.text()=='Disponibile'
        assert controller.state.status['baro_ready'] is False and window.sensor_refresh.isEnabled()
        assert backend.commands.count('sensors')==before and backend.power_mode=='sleeping'
        window.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert backend.power_mode=='active' and controller.state.status['power_mode']=='active'
        assert window.power_badge.isHidden()
        assert window.baro_label.text()=='Non disponibile' # Wake uses the new probe, including actual failures.
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE']
        controller.timer.start()
        assert window.arming_state.text()=='DISARMED' and window.arming_state.property('state')=='disarmed'
        assert not window.sim_physical.isChecked()
        assert backend.threads and all(t!=main_thread for t in backend.threads)
        # No UI state is changed to Armed while a request is merely pending.
        gate=threading.Event(); backend.gates['sim begin']=gate
        from PySide6.QtTest import QTest
        from PySide6.QtCore import Qt
        previous_size=window.size(); window.resize(1120,730); window.navigate(2); app.processEvents()
        scroll=window.pages.widget(2).verticalScrollBar()
        scroll.setValue(min(40,scroll.maximum())); position=scroll.value()
        QTest.mouseClick(window.sim_start,Qt.LeftButton); wait(lambda:'sim begin' in backend.commands)
        assert scroll.value()==position # Qt must not scroll to the table when Start loses its enabled state.
        assert not controller.state.sim.get('active') and controller.state.status['arming_state']=='disarmed'
        ticks=[]; from PySide6.QtCore import QTimer
        timer=QTimer(); timer.setInterval(5); timer.timeout.connect(lambda:ticks.append(1)); timer.start()
        for _ in range(15): app.processEvents(); time.sleep(.005)
        assert len(ticks)>5 # The UI event loop stays alive during serial/business waits.
        gate.set(); wait(lambda:controller.state.sim.get('active') and not controller.state.foreground); timer.stop()
        assert scroll.value()==position
        assert window.sim_progress.maximum()==32995 and 'Hoppino' in window.sim_summary.text() and f'{32995/1000:.2f}' in window.sim_summary.text()
        assert controller.state.status['arming_state']=='armed'
        assert not window.check_pyros_button.isEnabled() and not window.prepare_servos_button.isEnabled()
        assert window.arming_state.text()=='ARMED' and window.arming_state.property('state')=='armed'
        controller.command('sim stop'); wait(lambda:not controller.state.foreground)
        assert not controller.state.sim['active'] and controller.state.status['arming_state']=='disarmed'
        window.resize(previous_size)
        controller.command('preflight simulation'); wait(lambda:not controller.state.foreground)
        assert not controller.state.preflight['ready'] and not window.arm_button.isEnabled()
        window.servo_selection[0].setChecked(True)
        assert window.prepare_servos_button.isEnabled()
        window.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        assert 'servo arm 1' in backend.commands and window.servo_states[0].text()=='preparato'
        assert not window.prepare_servos_button.isEnabled() and window.prepare_servos_button.objectName()=='primary'
        assert window.release_servos_button.isEnabled() and window.release_servos_button.objectName()=='releaseServo'
        before=len(backend.commands); window.prepare_servos_button.click(); assert len(backend.commands)==before
        assert window.release_servos_button.isEnabled() and not window.arm_button.isEnabled() # IMU readiness still comes from the board.
        # Status can report held PWM before a fresh preflight mask is available.
        controller.state.preflight={}; window.render()
        assert controller.state.status['arming_state']=='disarmed' and controller.state.status['outputs_active']
        assert window.release_servos_button.isEnabled() and window.arm_button.text()=='ARM'
        window.release_servos_button.click(); wait(lambda:not controller.state.foreground)
        assert not window.release_servos_button.isEnabled()
        assert window.prepare_servos_button.isEnabled()
        assert not backend.prepared_mask and window.servo_states[0].text()=='non preparato' and window.servo_states[0].property('preparation')=='unprepared'
        window.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        # Held servo PWM is not an armed run: all idle settings stay available.
        assert window.factory_reset_button.isEnabled() and window.format_memory_button.isEnabled()
        assert window.configuration_page.add_rule_button.isEnabled() and window.check_pyros_button.isEnabled()
        window.orientation_panel.axis.setCurrentIndex(window.orientation_panel.axis.findData('-X')); window.render()
        assert window.orientation_panel.axis.isEnabled() and window.orientation_panel.save.isEnabled()
        assert backend.prepared_mask
        window.orientation_panel.axis.setCurrentIndex(window.orientation_panel.axis.findData('+X'))
        # A rejected run is not a configuration draft and must not latch Start disabled.
        controller._complete(controller._epoch,997,'Avvio simulazione',None,CommandError('sim begin',dict(code='CONFIG_INVALID',ok=False)))
        assert not controller.state.staged
        for physical in (True,False):
            window.sim_physical.setChecked(physical); window.render()
            assert backend.prepared_mask and controller.state.status['outputs_active']
            assert not controller.simulation_blocker() and window.sim_start.isEnabled()
        controller.state.staged=True; window.render()
        assert not window.sim_start.isEnabled() and 'bozza' in window.sim_blocker.text()
        controller.state.staged=False; window.render()
        # Prepared outputs no longer block virtual simulation; acknowledge the firmware release in Dashboard.
        assert backend.prepared_mask and window.sim_start.isEnabled()
        window.sim_physical.setChecked(False)
        original_request=backend.request
        def unsupported_release(command,timeout=5):
            if command=='servo release': raise CommandError(command,dict(code='INVALID_ARGUMENT',ok=False))
            return original_request(command,timeout)
        marker=len(backend.commands)
        with patch.object(backend,'request',side_effect=unsupported_release):
            window.sim_start.click(); wait(lambda:not controller.state.foreground)
        assert 'Aggiorna il firmware' in controller.state.error and backend.prepared_mask
        assert not any(command.startswith('sim begin') for command in backend.commands[marker:])
        marker=len(backend.commands)
        window.sim_record.setChecked(True)
        def recovered_storage(command,timeout=5):
            if command=='flights recover': backend.commands.append(command); return dict(ready=True)
            return original_request(command,timeout)
        with patch.object(backend,'request',side_effect=recovered_storage):
            window.sim_start.click(); wait(lambda:not controller.state.foreground)
        sequence=backend.commands[marker:]
        assert sequence.index('servo release')<sequence.index('flights recover')<sequence.index('sim begin record')
        assert controller.state.sim['active'] and controller.state.sim['output_mode']=='VIRTUAL'
        assert not backend.prepared_mask and window.servo_states[0].text()=='non preparato'
        window.sim_stop.click(); wait(lambda:not controller.state.foreground)
        window.sim_record.setChecked(False)
        window.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        backend.ready_to_arm=True; window.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert window.arm_button.isEnabled() and not window.arm_button.property('blocked')
        physical_preflight=deepcopy(controller.state.preflight); requests=len(backend.commands)
        for checked in (True,False):
            window.sim_physical.setChecked(checked); window.render()
            assert window.arm_button.isEnabled() and controller.state.preflight==physical_preflight
        assert len(backend.commands)==requests # Simulation selection never changes real-flight checks or sends commands.
        window.arm_button.click(); wait(lambda:not controller.state.foreground)
        assert 'IMU' in controller.state.error and controller.state.status['arming_state']=='disarmed'
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE']
        assert not window.arm_button.isEnabled() and window.arm_button.property('blocked')
        backend.ready_to_arm=False
        # Physical selection only changes the request. Firmware refusal stays authoritative.
        backend.reject_physical=True; window.sim_physical.setChecked(True); window.sim_start.click()
        wait(lambda:not controller.state.foreground)
        assert 'sim begin physical' in backend.commands and not controller.state.sim['active']
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE'] and 'Continuità' in controller.state.error
        backend.reject_physical=False; gate=threading.Event(); backend.gates['sim begin physical']=gate
        window.sim_start.click(); wait(lambda:backend.commands.count('sim begin physical')==2)
        assert not window.sim_physical.isEnabled() and controller.state.status['arming_state']=='disarmed'
        gate.set(); wait(lambda:controller.state.sim.get('active') and not controller.state.foreground)
        assert controller.state.sim['output_mode']=='PHYSICAL' and window.arming_state.text()=='ARMED'
        assert window.sim_physical.isChecked() and not window.sim_physical.isEnabled()
        assert window.arm_button.text()=='DISARM' and window.arm_button.isEnabled()
        assert not window.release_servos_button.isEnabled()
        window.arm_button.click(); wait(lambda:not controller.state.foreground)
        assert 'disarm' in backend.commands and window.arm_button.text()=='ARM'
        assert not controller.state.sim['active'] and window.sim_physical.isEnabled()
        window.sim_physical.setChecked(False)
        # Local edit -> firmware RAM validation -> save/readback -> new applied baseline.
        local=controller.state.local.clone(); local.rules[0]['name']='GUI native roundtrip'
        local.rules[0]['outputs'].append(dict(kind='servo',channel=2,delay_ms=0))
        controller.edit_plan(local); assert controller.state.dirty and not window.configuration_page.apply_button.isEnabled()
        assert not window.configuration_page.servo_notice.isHidden()
        assert window.configuration_page.channel_buttons[1].text()=='Configura'
        controller.validate(); wait(lambda:not controller.state.foreground)
        assert controller.state.validated and controller.state.staged and window.configuration_page.apply_button.isEnabled()
        assert backend.request('config show')['draft_active']
        controller.validate(apply=True); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and not controller.state.dirty and not controller.state.staged
        assert controller.state.base.rules[0]['name']=='GUI native roundtrip'
        assert not controller.state.base.servos[1]['calibrated']
        # Real flash-format/download path inside the PC firmware fixture, not fabricated CSV.
        backend.request('flights init'); backend.request('hil begin 0 200 logged_body record 4096')
        for i in range(22): backend.request(f'hil sample {i*5} 0 0 9.80665 0 0 0 101325')
        backend.request('hil stop completed')
        controller.list_flights(); wait(lambda:not controller.state.foreground)
        assert len(controller.state.flights)==1
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'flight.csv'
            window.navigate(3)
            assert window.download_feedback.currentIndex()==0 and not window.download_progress.isVisible()
            with patch('perpetual_app.window.QFileDialog.getSaveFileName',return_value=('','CSV (*.csv)')):
                window.download_button.click()
            assert not controller.state.download and not window.download_progress.isVisible()
            gate=threading.Event(); backend.gates['flights read 1 0 0']=gate
            with patch('perpetual_app.window.QFileDialog.getSaveFileName',return_value=(str(target),'CSV (*.csv)')):
                window.download_button.click()
            wait(lambda:'flights read 1 0 0' in backend.commands and controller.state.download.get('total')==22)
            assert window.download_progress.isVisible() and window.download_result.isHidden()
            assert window.download_progress.maximum()==100 and window.download_progress.value()<100
            assert window.download_progress.text()=='0% · Calcolo tempo…'
            controller._on_progress(controller._epoch-1,11,22,10) # Results from a previous connection cannot change ETA.
            assert controller.state.download['eta_seconds'] is None
            controller._on_progress(controller._epoch,11,22,.5)
            assert controller.state.download['eta_seconds'] is None
            controller._on_progress(controller._epoch,11,22,10)
            assert controller.state.download['eta_seconds']==10 and window.download_progress.text()=='50% · ~10 s rimanenti'
            controller._on_progress(controller._epoch,22,22,20)
            assert window.download_progress.text()=='99% · Finalizzazione…'
            gate.set(); wait(lambda:not controller.state.foreground); backend.gates.pop('flights read 1 0 0')
            assert not controller.state.error and controller.state.flight['samples']==22
            assert target.with_suffix('.json').exists() and len(window.flight_plot.points)==22
            assert controller.state.download['state']=='completed' and controller.state.download['done']==22
            assert not window.download_progress.isVisible() and window.download_result.isVisible()
            assert window.download_result.text()=='Downloaded at '+str(target.resolve())
            # Cancel during a real pending reply, after a chunk has been saved.
            cancelled=Path(folder)/'cancelled.csv'; marker=len(backend.commands)
            gate=threading.Event(); backend.gates['flights read 1 0 2']=gate
            controller.download('1',cancelled)
            wait(lambda:'flights read 1 0 2' in backend.commands[marker:] and controller.state.download.get('done')==2)
            assert cancelled.exists() and window.download_button.text()=='Annulla' and window.download_button.isEnabled()
            assert window.download_button.objectName()=='downloadCancel'
            window.download_button.click()
            assert controller.state.download['state']=='cancelling' and controller.state.foreground
            assert window.download_button.text()=='Annullamento…' and not window.download_button.isEnabled()
            gate.set(); wait(lambda:not controller.state.foreground); backend.gates.pop('flights read 1 0 2')
            assert controller.state.download['state']=='cancelled' and not controller.state.error and controller.state.connected
            assert window.download_result.text()=='Download annullato' and not window.download_progress.isVisible()
            assert window.download_button.text()=='Scarica selezionato' and window.download_button.isEnabled()
            assert not cancelled.exists() and not cancelled.with_suffix('.json').exists()
            assert [command for command in backend.commands[marker:] if command.startswith('flights read ')]==['flights read 1 0 0','flights read 1 0 2']
            assert backend.request('flights info 1')['samples']==22 # Reply drained; remote recording and connection are intact.
            controller.download('1',cancelled); wait(lambda:not controller.state.foreground)
            assert controller.state.download['state']=='completed' and cancelled.with_suffix('.json').exists()
            for page in range(6):
                window.navigate(page); app.processEvents(); assert window.grab().width()>=1120
            check_flight_plots(window,controller,app)
            controller.download('1',target); wait(lambda:not controller.state.foreground) # Existing files are rejected.
            assert controller.state.download['state']=='failed' and window.download_result.text()=='Download non completato'
            assert not window.download_progress.isVisible()
            with patch('download_flight.download',side_effect=CommandError('flights info 1',dict(ok=False,code='Corrupt'))):
                controller.download('1',Path(folder)/'corrupt.csv'); wait(lambda:not controller.state.foreground)
            assert window.download_result.text()=='Volo corrotto' and controller.state.error=='Volo corrotto'
            assert not window.download_progress.isVisible()
            backend.request('hil begin 0 200 logged_body record 4096'); backend.request('hil stop completed')
            empty=Path(folder)/'empty_flight.csv'; controller.download('2',empty); wait(lambda:not controller.state.foreground)
            assert empty.exists() and empty.with_suffix('.json').exists()
            assert controller.state.download['state']=='completed' and window.download_result.text()=='Downloaded at '+str(empty.resolve())
            assert 'anteprima non disponibile' in controller.state.error # An empty flight still downloads successfully.
            controller.list_flights(); wait(lambda:not controller.state.foreground)
            assert [e['sequence'] for e in controller.state.flights]==['2','1']
            assert window.flights_table.item(0,0).text()=='2'
            assert window.flights_table.currentRow()==1 # The selected flight 1 moved down; preserve its identity.
            with patch.object(controller,'download') as download, patch('perpetual_app.window.QFileDialog.getSaveFileName',return_value=(str(target),'CSV (*.csv)')):
                window.download_selected()
                download.assert_called_once_with('1',str(target))
        # Maintenance uses real firmware storage and preserves the opposite partition.
        assert window.nav[4].text()=='Impostazioni' and window.nav[5].text()=='Console'
        panel=window.orientation_panel
        panel.axis.setCurrentIndex(panel.axis.findData('-X')); app.processEvents()
        assert panel.caption.text()=='USB in alto' and panel.save.isEnabled()
        panel.save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.orientation['up_axis']=='-X'
        assert not panel.save.isEnabled() and '-X' in panel.saved.text()
        panel.axis.setCurrentIndex(panel.axis.findData('+Y')); app.processEvents()
        assert panel.caption.text()=='USB a sinistra' and panel.image.pixmap().width()>panel.image.pixmap().height()
        left=panel.image.pixmap().toImage()
        panel.axis.setCurrentIndex(panel.axis.findData('-Y')); app.processEvents()
        assert panel.caption.text()=='USB a destra' and panel.image.pixmap().width()>panel.image.pixmap().height()
        assert panel.image.pixmap().toImage()!=left
        panel.axis.setCurrentIndex(panel.axis.findData('+Z')); app.processEvents()
        assert panel.caption.text()=='Immagine di riferimento +X'
        saved=controller.state.base.clone()
        with patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.No):
            before=len(backend.commands); window.confirm_maintenance(False)
            assert len(backend.commands)==before
        controller.save_machine(True,50); wait(lambda:not controller.state.foreground)
        assert controller.state.machine['bistage']
        controller.maintenance(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.base is None and not controller.state.local.rules
        assert controller.state.orientation['up_axis'] is None and panel.axis.currentData() is None
        assert panel.axis.currentText()==chr(8212) and not panel.save.isEnabled()
        assert not controller.state.machine['bistage'] and not window.bistage_enabled.isChecked()
        assert not backend.request('config show')['available']
        assert backend.request('flights info 1')['samples']==22
        controller.edit_plan(saved); controller.validate(apply=True); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.base==saved
        assert 'ORIENTATION_NOT_READY' in controller.state.preflight['reasons'] and not window.arm_button.isEnabled()
        assert any('Orientamento' in window.readiness_table.item(i,0).text() for i in range(window.readiness_table.rowCount()))
        panel.axis.setCurrentIndex(panel.axis.findData('+X')); assert panel.save.isEnabled()
        panel.save.click(); wait(lambda:not controller.state.foreground)
        assert controller.state.orientation['orientation_ready'] and controller.state.orientation['up_axis']=='+X'
        assert 'ORIENTATION_NOT_READY' not in controller.state.preflight['reasons']
        backend.request('hil begin 0 200 logged_body')
        try:
            for command in ('config reset','flights format','config orientation -Z'):
                try: backend.request(command)
                except CommandError as error: assert error.code=='BUSY'
                else: raise AssertionError('Maintenance accepted during active run')
        finally: backend.request('hil stop completed')
        controller.maintenance(True); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.flights==[] and controller.state.flight=={}
        assert not backend.request('flights list 0')['available']
        assert controller.state.base==saved and read_plan(backend)[0]==saved
        backend.request('config begin')
        backend.request('config discard')
        # Source loss invalidates controls while retaining a local unsaved draft.
        local=controller.state.local.clone(); local.rules[0]['name']='Unsaved local'
        controller.edit_plan(local)
        foreign=controller.state.base.clone(); foreign.rules[0]['name']='Foreign draft'
        controller._complete(controller._epoch,998,'poll',dict(configuration=dict(plan=foreign,meta=dict(available=True,draft_active=True),view='draft'),control=dict(owner='bluetooth',transport='usb')),None)
        assert controller.state.dirty and controller.state.local.rules[0]['name']=='Unsaved local'
        assert not controller.state.editable
        controller.disconnect(); wait(lambda:controller.connection is None)
        assert window.arming_state.text()=='—'
        assert window.imu_label.text().endswith('—') and window.baro_label.text().endswith('—')
        assert controller.state.dirty and not window.sim_start.isEnabled() and not window.command_send.isEnabled()
        controller.discard(); assert not controller.state.dirty
        # A result from an old connection cannot update a new UI session.
        before=deepcopy(controller.state.status)
        controller._complete(controller._epoch-1,999,'poll',dict(status=dict(armed=True)),None)
        assert controller.state.status==before
    finally:
        if controller.connection: controller.disconnect(); wait(lambda:controller.can_close())
        window._allow_close=True; window.close(); app.processEvents()
        for backend in backends: backend.close()
    # Dispose native widgets before QApplication teardown, while Python owners still exist.
    from PySide6.QtCore import QCoreApplication,QEvent
    for widget in app.topLevelWidgets(): widget.deleteLater()
    QCoreApplication.sendPostedEvents(None,QEvent.DeferredDelete)
    print('Desktop passed: native config transactions/conflicts, internal stable IDs and named references, queued Qt UI, acknowledged state, virtual controls, native recording/download/plot and stale-session isolation.')


if __name__=='__main__': run()
