"""Shared desktop test support: PC firmware fixture, Qt helpers."""
from contextlib import contextmanager
from types import SimpleNamespace
from concurrent.futures import Future
import os
import re
from pathlib import Path
import subprocess
import sys
import threading
import time

os.environ['QT_QPA_PLATFORM']='offscreen'
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT/'tools/replay')]
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFontDatabase
from device_client import Client,CommandError,ConnectionLost
from perpetual_app.controller import Controller
from perpetual_app.window import MainWindow
from perpetual_app.i18n import set_language,use_decimal_point


class Native:
    """PC firmware stdin endpoint. Qt UI tests supplement hardware-only status APIs."""
    refused_connections=0 # Next connections that fail, as while a radio module restarts.
    bluetooth_default='Perpetual' # Name of the simulated module when a fixture starts.
    bluetooth_starting=False # The module is still starting when a fixture connects.
    def __init__(self):
        self.process=subprocess.Popen([str(ROOT/'.pio/tests/bin/command_device.exe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.client=Client(self); self.state='disconnected'; self.active=False; self.commands=[]; self.threads=[]; self.gates={}
        self.output_mode='VIRTUAL'; self.reject_physical=False; self.ready_to_arm=False; self.prepared_mask=0
        self.sensor_time=0; self.transport='usb'; self.failed_pyros=0; self.power_mode='active'
        self.bluetooth_name=Native.bluetooth_default # The PC firmware has no radio: the module name is simulated here.
        self.bluetooth_ready=not Native.bluetooth_starting; self.bluetooth_rename='None'
        self.client.request('config orientation +X')
    def write(self,data): self.process.stdin.write(data); self.process.stdin.flush(); return len(data)
    def read(self,count): return self.process.stdout.read(count)
    def connect(self):
        f=Future()
        if Native.refused_connections:
            Native.refused_connections-=1; f.set_exception(TimeoutError('Radio restarting')); return f
        self.state='connected'; f.set_result(dict(device='Perpetual',protocol_version=1)); return f
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
                raise CommandError(command,dict(code='CONTINUITY_FAULT',reasons=['CONTINUITY_FAULT'],failed_pyro_mask=1,ready=False,ok=False))
            self.active=True; self.prepared_mask=0; self.output_mode='PHYSICAL' if 'physical' in command else 'VIRTUAL'
        if command=='sim stop': self.active=False
        if command.startswith('sim begin') or command in ('sim stop','sim status'):
            return dict(active=self.active,arming_state='armed' if self.active else 'disarmed',output_mode=self.output_mode,
                        source='hoppino',duration_ms=32995,total_samples=6600,
                        samples=1 if self.active else 0,t_ms=0,phase=0,z_est=0,starts=0,reason='running' if self.active else 'stopped')
        if command=='sim events': return dict(starts=[],phases=[],output_mode=self.output_mode)
        if command=='imu status': # The PC device has no IMU: a verified calibration, idle.
            return dict(imu_calibration_state='verified',imu_calibration_reason='None',active=False,collecting=False,verification=False,
                        face=0,face_mask=0,samples=0,target=240,last_error='None',window_error='None',revision=1,bias_mps2=[0,0,0],scale=[1,1,1])
        if command=='bluetooth status':
            ready=self.bluetooth_ready
            return dict(ready=ready,connected=ready and self.transport=='bluetooth',error='None' if ready else 'STARTING',baud=230400,uart_rx_overflows=0,
                        name=self.bluetooth_name if ready else '',rename=self.bluetooth_rename,configured=False)
        if command.startswith('bluetooth name '):
            suffix=command.split()[-1]
            if suffix!='default' and not re.fullmatch('[A-Za-z0-9-]{1,4}',suffix): raise CommandError(command,dict(code='INVALID_NAME',ok=False))
            self.bluetooth_name='Perpetual' if suffix=='default' else 'Perpetual-'+suffix; self.bluetooth_rename='OK'
            return dict(name=self.bluetooth_name,ble_name=self.bluetooth_name+'_BLE',restarting=True)
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


def start_app(language='it'):
    # The behaviour areas assert the Italian texts, which also checks that the catalog reproduces them;
    # desktop_i18n checks the English default and the catalog itself.
    set_language(language); use_decimal_point()
    app=QApplication.instance() or QApplication([])
    for name in ('segoeui.ttf','segoeuib.ttf','seguisym.ttf'):
        path=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'/name
        if path.exists(): QFontDatabase.addApplicationFont(str(path))
    return app


def make_wait(app):
    def wait(test):
        until=time.monotonic()+5
        while not test():
            assert time.monotonic()<until,'Qt operation did not finish'
            app.processEvents(); time.sleep(.003)
        app.processEvents()
    return wait


def raises(kind,operation):
    try: operation()
    except kind as error: return error
    raise AssertionError(f'Expected {kind.__name__}')


@contextmanager
def session(app,wait,connect=True):
    """Main window driven by a fresh PC firmware fixture; always closed afterwards."""
    backends=[]
    def factory(_): backend=Native(); backends.append(backend); return backend
    controller=Controller(factory); window=MainWindow(controller); window.show()
    s=SimpleNamespace(controller=controller,window=window,backends=backends,backend=None)
    try:
        if connect:
            controller.connect_port('PC_FIXTURE')
            wait(lambda:controller.state.local is not None and not controller.state.foreground)
            s.backend=backends[-1]
        yield s
    finally:
        if controller.connection: controller.disconnect(); wait(lambda:controller.can_close())
        window._allow_close=True; window.close(); app.processEvents()
        for backend in backends: backend.close()


def dispose(app):
    """Dispose native widgets before QApplication teardown, while Python owners still exist."""
    from PySide6.QtCore import QCoreApplication,QEvent
    for widget in app.topLevelWidgets(): widget.deleteLater()
    QCoreApplication.sendPostedEvents(None,QEvent.DeferredDelete)
