"""Qt application state and asynchronous operations; widgets never own serial IO."""
from concurrent.futures import ThreadPoolExecutor,CancelledError
from dataclasses import dataclass, field
from pathlib import Path
import json
import math
import time
from threading import Event

from PySide6.QtCore import QObject, QTimer, Signal, Slot, Qt
from device_client import CommandError, DeviceConnection
from .configuration import Plan, read_plan, stage_plan, rule_label
from .ports import Port, enumerate_ports
from .connection import ConnectionSession
from .flight_data import load_flight
from .i18n import t

# Job kinds drive behaviour; job names are user-facing labels only.
OPERATION, POLL, PREVIEW, DOWNLOAD, SIMULATION_START = 'operation', 'poll', 'servo_preview', 'download', 'simulation_start'

# State parts read with one plain command; the others use a _read_<name> method (optional on older firmware).
READS = dict(status='status',preflight='preflight',sensors='sensors',sim='sim status',events='sim events',
             control='control status',capabilities='capabilities',bluetooth='bluetooth status')

OLD_FIRMWARE = ('UNKNOWN_COMMAND','INVALID_ARGUMENT','NO_DRAFT') # Error codes meaning "this firmware lacks the command".

REASONS = { # English; error messages use them as they are, the Dashboard checklist translates them with t().
    'IMU_CALIBRATION_MISSING':'IMU calibration missing',
    'IMU_VERIFICATION_REQUIRED':'IMU calibration needs verification',
    'IMU_CALIBRATION_IO':'IMU calibration could not be saved',
    'IMU_UNAVAILABLE':'IMU not available', 'BAROMETER_UNAVAILABLE':'Barometer not available',
    'STORAGE_NOT_READY':'Recording memory to prepare',
    'CONTINUITY_FAULT':'Pyro channel continuity to check', 'SERVOS_NOT_PREPARED':'Servos to prepare',
    'SERVOS_NOT_CONFIGURED':'Servo initial and final positions to configure',
    'SERVO_PREPARATION_FAILED':'Servo initial position preparation failed',
    'OUTPUT_ERROR':'Physical output command failed',
    'SERVO_NOT_ASSIGNED':'Servo not assigned to any rule',
    'PHYSICAL_SERVOS_PREPARED':'Servos still prepared: use Release servos and update the board firmware',
    'ORIENTATION_NOT_READY':'Board orientation to save in Settings',
    'CONFIG_INVALID':'Invalid configuration',
    'AIRBRAKES_SETTING_INVALID':'Airbrake enable to save again in Settings',
    'STATE_MACHINE_INVALID':'State machine to save again in Settings',
    'DRAFT_ACTIVE':'Board draft still active', 'BUSY':'Board busy',
    'OUTPUT_ALREADY_ASSIGNED':'Channel already assigned: each channel can belong to only one rule',
    'CONTROL_BUSY':'Control reserved by another connection',
    'SENSOR_SLEEP_FAILED':'Sensor sleep failed',
    'FLASH_SLEEP_FAILED':'Memory sleep failed', 'FLASH_WAKE_FAILED':'Memory wake-up failed',
    'DISARM_PROHIBITED_IN_FLIGHT':'Disarm refused during flight', 'NO_DRAFT':'No draft on the board',
    'BLUETOOTH_NOT_READY':'Bluetooth module not ready', 'INVALID_NAME':'Invalid name: up to 4 letters, digits or -',
    'NAME_NOT_VERIFIED':'The Bluetooth module did not confirm the name',
}


def error_text(error,plan=None):
    if isinstance(error, CommandError):
        details = error.response
        text = REASONS.get(error.code,error.code)
        if details.get('validation'):
            validation=details['validation']; index=details.get('index',0)
            if validation=='InvalidPyroSettings':
                text += f' · PYRO {index+1}: pulse length missing or out of range'
            elif validation=='InvalidServoSettings':
                text += f' · SERVO {index+1}: settings incomplete or out of range. Check Channels → SERVO {index+1}'
            else:
                text += f' · {validation}'
                if details.get('rule_id'): text += ' · '+rule_label(plan.rules if plan else [],details['rule_id'])
                if details.get('field') in (2,3): text += f' · {"condition" if details["field"]==2 else "output"} {index+1}'
        if details.get('reasons'): text += ': '+', '.join(REASONS.get(r,r) for r in details['reasons'])
        return text
    return str(error)


@dataclass
class AppState:
    connection: dict = field(default_factory=lambda: dict(state='disconnected', transport=None, port='', reply_age_s=None))
    status: dict = field(default_factory=dict)
    sensors: dict = field(default_factory=dict)
    sensor_availability: dict = field(default_factory=dict) # Last awake observation; sleep is not a sensor fault.
    preflight: dict = field(default_factory=dict)
    imu_calibration: dict = field(default_factory=dict)
    bluetooth: dict = field(default_factory=dict) # Reply of `bluetooth status`, including the module name.
    bluetooth_restart: str = '' # Name being applied while the module restarts after a rename over Bluetooth.
    pyro_continuity: dict = field(default_factory=dict)
    orientation: dict = field(default_factory=dict)
    airbrakes: dict = field(default_factory=dict)
    airbrake_status: dict = field(default_factory=dict)
    machine: dict = field(default_factory=dict)
    setting_drafts: dict = field(default_factory=dict) # Local form values; operational state above is acknowledged by the board.
    capabilities: dict = field(default_factory=dict)
    control: dict = field(default_factory=dict)
    config_meta: dict = field(default_factory=dict)
    base: object = None
    local: object = None
    config_view: str = 'applied'
    validated: bool = False
    staged: bool = False
    sim: dict = field(default_factory=dict)
    sim_preflight: dict = field(default_factory=dict) # Last rejected physical simulation; separate from real-flight readiness.
    events: dict = field(default_factory=dict)
    flights: list = field(default_factory=list)
    flights_loaded: bool = False
    flight: dict = field(default_factory=dict)
    download: dict = field(default_factory=dict)
    foreground: str = ''
    foreground_kind: str = ''
    message: str = ''
    error: str = ''

    @property
    def connected(self): return self.connection.get('state') in ('connected','waiting')

    @property
    def dirty(self): return self.local is not None and self.local != self.base and self.config_view=='applied'

    def saved_setting(self,group):
        if group=='orientation': return self.orientation.get('up_axis')
        if group=='machine': return self.machine.get('bistage',False),self.machine.get('minimum_coast_velocity',0)
        if group=='airbrakes': return self.airbrakes.get('enabled',False)
        raise ValueError('Unknown setting: '+group)

    def local_setting(self,group): return self.setting_drafts.get(group,self.saved_setting(group))

    def setting_changed(self,group):
        local,saved=self.local_setting(group),self.saved_setting(group)
        if group=='machine':
            # A dormant threshold stays in the form but is not applied in normal mode.
            local=(local[0],local[1] if local[0] else 0); saved=(saved[0],saved[1] if saved[0] else 0)
        return local!=saved

    @property
    def has_unsaved_changes(self): return self.dirty or any(self.setting_changed(group) for group in self.setting_drafts)

    @property
    def editable(self):
        return self.connected and self.owns_control and not self.foreground and not self.imu_calibration.get('active') and self.config_view=='applied' and self.status.get('arming_state')=='disarmed' and not self.status.get('estimate_replay_active')

    @property
    def owns_control(self): return self.control.get('owner','none') in ('none',self.connection.get('transport'))


class Controller(QObject):
    changed = Signal()
    connection_changed = Signal(object)
    ports_changed = Signal(object)
    port_choice = Signal(object)
    logged = Signal(str,str)
    sensors_received = Signal(object)
    servo_previewed = Signal(object)
    _finished = Signal(int,int,str,object,object,str)
    _progress = Signal(int,int,int,float)

    def __init__(self, connection_factory=DeviceConnection, port_scanner=enumerate_ports):
        super().__init__()
        self.state = AppState()
        self._executor = ThreadPoolExecutor(max_workers=1,thread_name_prefix='Perpetual operations')
        self._epoch, self._serial = 0, 0
        self._jobs = {}
        self._foreground_job = None
        self._last_status = self._last_sim = self._last_sensors = self._last_airbrakes = self._last_imu = 0
        self._want_events = False
        self.page = 0
        self._sensor_stream = False
        self._download_cancel = None
        self._bluetooth_restart = None # Reconnection after a rename over Bluetooth: port, expected name, timing.
        self.bluetooth_restart_delay = 6.0 # Seconds for the module to restart before the first attempt.
        self.bluetooth_restart_window = 30.0 # Then retry for this long; afterwards report it.
        self.bluetooth_retry_interval = 3.0
        self._finished.connect(self._complete, Qt.ConnectionType.QueuedConnection)
        self._progress.connect(self._on_progress, Qt.ConnectionType.QueuedConnection)
        self.session=ConnectionSession(self._executor,connection_factory,port_scanner,self)
        self.session.changed.connect(self._connection_update)
        self.session.reset.connect(self._reset_session)
        self.session.ready.connect(self.refresh)
        self.session.ports_changed.connect(self.ports_changed.emit)
        self.session.port_choice.connect(self.port_choice.emit)
        self.session.logged.connect(self.logged.emit)
        self.timer = QTimer(self); self.timer.setInterval(250); self.timer.timeout.connect(self.tick); self.timer.start()

    def _job(self, name, function, foreground=True, kind=OPERATION):
        if foreground and self.state.foreground: return False
        self._serial += 1
        job, epoch = self._serial,self._epoch
        self._jobs[job]=(name,epoch)
        if foreground:
            self._foreground_job=job; self.state.foreground_kind=kind
            self.state.foreground=name; self.state.error=''; self.state.message=name+'…'; self.changed.emit()
        def execute():
            try: result,error=function(),None
            except Exception as e: result,error=None,e
            self._finished.emit(epoch,job,name,result,error,kind)
        self._executor.submit(execute)
        return True

    def request(self, client, command, timeout=5):
        # Emitting a Qt signal is safe here; widgets receive it on their own thread.
        self.logged.emit('tx',command)
        try: response=client.request(command,timeout=timeout)
        except CommandError as error:
            self.logged.emit('error',json.dumps(error.response,ensure_ascii=False)); raise
        self.logged.emit('rx',json.dumps(response,ensure_ascii=False))
        return response

    @property
    def connection(self): return self.session.client

    @property
    def connection_busy(self): return self.session.busy

    def scan_ports(self):
        if not self.state.foreground: self.session.scan_ports()

    def connect_auto(self, preferred=None, quiet=False):
        if not self.state.foreground: self.session.start(preferred=preferred,quiet=quiet)

    def connect_port(self, port):
        if not self.state.foreground: self.session.start(port=port)

    def choose_port(self, port): self.session.choose_port(port)

    def disconnect(self): self.session.disconnect()

    def edit_setting(self,group,value):
        self.state.saved_setting(group) # Reject an unknown group before modifying the draft.
        if group not in self.state.setting_drafts or self.state.setting_drafts[group]!=value:
            self.state.setting_drafts[group]=value; self.changed.emit()

    def _reset_session(self, starting):
        self.state.setting_drafts={group:value for group,value in self.state.setting_drafts.items() if self.state.setting_changed(group)}
        self._epoch+=1; self.state.foreground=''; self.state.foreground_kind=''; self._foreground_job=None; self.state.validated=False; self.state.staged=False
        self.state.flights=[]; self.state.flights_loaded=False # Board inventory belongs to this session; local CSV data is independent.
        self.state.sim_preflight={}
        self.state.imu_calibration={}
        if self._download_cancel: self._download_cancel.set(); self._download_cancel=None
        if self.state.download.get('state') in ('downloading','cancelling'): self.state.download['state']='interrupted'
        if starting:
            for key in ('status','preflight','pyro_continuity','sim','events','sensors','sensor_availability','orientation','machine','airbrakes','airbrake_status','imu_calibration','bluetooth'): setattr(self.state,key,{})

    def _connection_update(self, snapshot, message, error):
        if self._bluetooth_restart and snapshot.get('state') in ('error','unresponsive','disconnected','disconnecting'):
            message,error=t('The Bluetooth module is restarting to apply the name. Reconnecting…'),'' # Expected, not a failure.
        previous=self.state.connection.get('state')
        self.state.connection=snapshot
        if message: self.state.message=message
        if error is not None: self.state.error=error
        self.connection_changed.emit(snapshot)
        if previous!=snapshot.get('state') or message or error is not None: self.changed.emit()

    def _read(self, client, *names):
        """Read the named parts of the device state, in this order."""
        return {name:self.request(client,READS[name]) if name in READS else getattr(self,'_read_'+name)(client) for name in names}

    def _read_configuration(self, client):
        plan,meta,view=read_plan(client)
        return dict(plan=plan,meta=meta,view=view)

    def _optional(self,client,command,old_firmware=OLD_FIRMWARE,flag=True):
        """Read a feature newer firmware provides; on older firmware return supported=False ({} without flag)."""
        try: response=self.request(client,command)
        except CommandError as error:
            if error.code not in old_firmware: raise
            return dict(supported=False) if flag else {}
        return dict(response,supported=True) if flag else response

    def _read_orientation(self,client): return self._optional(client,'config orientation')
    def _read_imu_calibration(self,client): return self._optional(client,'imu status',('UNKNOWN_COMMAND',))
    def _read_machine(self,client): return self._optional(client,'config machine')
    def _read_airbrakes(self,client): return self._optional(client,'config airbrakes')
    def _read_airbrake_status(self,client): return self._optional(client,'airbrake status',flag=False)

    def save_airbrakes(self,enabled):
        s=self.state
        if not s.editable or s.dirty or s.staged or s.sim.get('active') or s.status.get('sensor_hil_active') or s.airbrakes.get('supported') is not True: return
        client=self.connection
        def save():
            reply=self.request(client,'config airbrakes '+('on' if enabled else 'off'),60)
            if not all(reply.get(k) is True for k in ('persisted','verified')): raise RuntimeError('Airbrake save not confirmed')
            setting=self._read_airbrakes(client)
            if setting.get('enabled')!=enabled or not setting.get('ready'): raise RuntimeError('Airbrake setting verification failed')
            return dict(saved_setting='airbrakes',airbrakes=setting,**self._read(client,'airbrake_status','status','preflight'))
        self._job(t('Saving airbrakes'),save)

    def save_machine(self,bistage,minimum):
        s=self.state
        if not s.editable or s.dirty or s.staged or s.machine.get('supported') is False: return
        if s.sim.get('active') or s.status.get('sensor_hil_active'): return
        if type(bistage) is not bool or not math.isfinite(minimum) or not 0<=minimum<=100000: return
        client=self.connection
        def save():
            command='config machine '+('bistage '+format(minimum,'.9g') if bistage else 'normal')
            reply=self.request(client,command,60)
            if not all(reply.get(key) is True for key in ('persisted','verified')): raise RuntimeError('State machine save not confirmed')
            machine=self._read_machine(client)
            if machine.get('bistage')!=bistage or machine.get('minimum_coast_velocity')!=(minimum if bistage else 0):
                raise RuntimeError('State machine verification failed')
            return dict(saved_setting='machine',machine=machine,**self._read(client,'status','preflight'))
        self._job(t('Saving state machine'),save)

    def refresh(self):
        if not self.state.connected or self.state.foreground: return
        client=self.connection
        def read():
            return self._read(client,'status','capabilities','imu_calibration','control','preflight','sensors',
                              'configuration','sim','orientation','machine','airbrakes','airbrake_status','bluetooth')
        self._job(t('Synchronizing board'),read)

    def reload_config(self):
        if self.state.connected:
            client=self.connection
            self._job(t('Reading configuration'),lambda:self._read(client,'configuration'))

    def edit_plan(self, plan):
        self.state.local=plan; self.state.validated=False; self.state.error=''; self.changed.emit()

    def validate(self, apply=False):
        if not self.state.editable or not self.state.dirty: return
        client,base,local=self.connection,self.state.base,self.state.local.clone()
        def write():
            stage_plan(client,base,local)
            if not apply: return dict(validated=True,**self._read(client,'control'))
            saved=self.request(client,'config apply',60)
            if not all(saved.get(k) is True for k in ('applied','persisted','verified')):
                raise RuntimeError('The board did not fully confirm the save')
            return dict(applied=True,**self._read(client,'configuration','status','preflight'))
        self._job(t('Saving configuration') if apply else t('Validating draft'),write)

    def discard(self):
        if self.state.foreground: return
        if not self.state.connected:
            self.state.local=self.state.base.clone() if self.state.base else None
            self.state.validated=False; self.changed.emit(); return
        client=self.connection
        def discard_remote():
            meta=self.request(client,'config draft'); control=self.request(client,'control status')
            if meta['available'] and control['owner']==control['transport']: self.request(client,'config discard')
            return dict(discarded=True,**self._read(client,'status','preflight'))
        self._job(t('Discarding draft'),discard_remote)

    @staticmethod
    def _command_effects(command, response):
        """Reply fields to keep and state to re-read after a command, in firmware request order."""
        if command=='status': return dict(status=response),()
        if command.startswith('preflight'): return (dict(preflight=response) if response.get('requested_mode')!='simulation' else {}),()
        if command=='sensors': return dict(sensors=response),()
        if command=='airbrake status': return dict(airbrake_status=response),()
        if command=='pyro continuity': return dict(pyro_continuity=response),()
        if command.startswith('imu '): return dict(imu_calibration=dict(response,supported=True)),('preflight','status')
        if command.startswith('config airbrakes'): return {},('airbrakes','airbrake_status')
        if command.startswith('sim '): return {},('sim','status')
        if command in ('arm','disarm','ds','sleep','wake') or command.startswith('servo arm') or command=='servo release':
            return {},('status',)+(() if command=='sleep' else ('preflight',))+(('sensors',) if command=='wake' else ())+(('sim',) if command in ('disarm','ds') else ())
        return {},()

    def command(self, command, name=None):
        if not self.state.connected: return
        client=self.connection; was_sleeping=self.state.status.get('power_mode')=='sleeping'
        def send():
            response=self.request(client,command,timeout=60 if command.startswith('flights ') else 5)
            stored,rereads=self._command_effects(command,response)
            data=dict(console=response,**stored); data.update(self._read(client,*rereads))
            if was_sleeping and command!='sleep': # Any command wakes the board: refresh what sleep invalidated.
                data.update(self._read(client,'status'))
                if data['status'].get('power_mode')=='active':
                    data.update(self._read(client,*(name for name in ('preflight','sensors') if name not in data)))
            return data
        self._job(name or t('Command: {command}',command=command),send)

    def save_orientation(self,axis):
        s=self.state
        if s.orientation.get('supported') is False: return
        if axis not in ('+X','-X','+Y','-Y','+Z','-Z'): return
        if not s.editable or s.dirty or s.staged or s.sim.get('active') or s.status.get('sensor_hil_active'): return
        client=self.connection
        def save():
            response=self.request(client,'config orientation '+axis,60)
            if not all(response.get(key) is True for key in ('persisted','verified','orientation_ready')) or response.get('up_axis')!=axis:
                raise RuntimeError('Orientation not confirmed by the board')
            orientation=self.request(client,'config orientation')
            if orientation.get('up_axis')!=axis or not orientation.get('orientation_ready'): raise RuntimeError('Orientation verification failed')
            return dict(saved_setting='orientation',orientation=orientation,**self._read(client,'status','preflight'))
        self._job(t('Saving orientation'),save)

    def maintenance(self,format_memory=False):
        s=self.state
        if not s.editable or s.sim.get('active') or s.status.get('sensor_hil_active'): return
        if format_memory and (s.dirty or s.staged): return
        client=self.connection
        def perform():
            response=self.request(client,'flights format' if format_memory else 'config reset',60)
            required=('formatted','ready') if format_memory else ('reset','persisted','verified')
            if not all(response.get(key) is True for key in required): raise RuntimeError('Operation not confirmed by the board')
            result=self._read(client,'status','preflight')
            if format_memory: result.update(flights=[],flight={})
            else: result.update(reset_settings=True,applied=True,**self._read(client,'configuration','control'),
                               orientation=self.request(client,'config orientation'),**self._read(client,'machine','airbrakes','airbrake_status'))
            return result
        self._job(t('Formatting memory') if format_memory else t('Restoring factory settings'),perform)

    def rename_bluetooth(self, suffix):
        """Rename the radio ('default' restores "Perpetual"). The module restarts to apply it."""
        s=self.state
        if not s.editable or s.sim.get('active') or s.status.get('sensor_hil_active') or 'name' not in s.bluetooth: return
        client,connection=self.connection,dict(s.connection)
        def rename():
            reply=self.request(client,'bluetooth name '+suffix)
            if connection.get('transport')=='bluetooth': # This link drops now: reconnect once, then verify.
                return dict(bluetooth_restart=dict(port=connection.get('port'),identity=connection.get('identity',''),name=reply['name']))
            deadline=time.monotonic()+15
            while True: # Over USB, wait for the module restart and read the result back.
                time.sleep(.5)
                status=self.request(client,'bluetooth status')
                if status.get('ready') and status.get('rename') not in ('PENDING','ACTIVE'): break
                if time.monotonic()>deadline: raise RuntimeError('The Bluetooth module did not restart in time')
            if status.get('rename')!='OK' or status.get('name')!=reply['name']:
                raise RuntimeError('Bluetooth name not confirmed: '+REASONS.get(status.get('rename'),str(status.get('rename'))))
            return dict(bluetooth=status,saved_bluetooth_name=reply['name'])
        self._job(t('Renaming Bluetooth'),rename)

    def prepare_servos(self, channels):
        if not self.state.connected: return
        client=self.connection
        def prepare():
            for command in ([f'servo arm {i}' for i in channels] if channels else ['servo arm']): self.request(client,command)
            return self._read(client,'status','preflight')
        self._job(t('Preparing servos'),prepare)

    def preview_servo(self, channel, pwm):
        if not self.state.editable or self._jobs: return False
        client=self.connection
        return self._job('Anteprima servo',lambda:self.request(client,f'servo preview {channel} {pwm}'),False,PREVIEW)

    def release_servo(self, channel):
        if not self.state.connected: return
        client=self.connection
        def release():
            self.request(client,f'servo release {channel}')
            return self._read(client,'status','preflight')
        self._job(t('Releasing servos'),release,False)

    def simulation_blocker(self):
        s=self.state
        if not s.connected: return t('Connect the board to start the simulation.')
        if s.foreground: return t('Operation in progress: {operation}',operation=s.foreground)
        if not s.owns_control: return t('Board controlled by another connection.')
        if s.sim.get('active') or s.status.get('simulation_active'): return t('Simulation running.')
        if s.status.get('arming_state')!='disarmed': return t('The board must be disarmed to start the simulation.')
        if s.status.get('sensor_hil_active') or s.status.get('estimate_replay_active'): return t('Stop the running test before starting the simulation.')
        if not s.status.get('config_valid'): return t('Apply a valid configuration before starting the simulation.')
        if s.dirty: return t('Apply or discard the configuration changes.')
        if s.staged: return t('Apply or discard the draft validated on the board.')
        return ''

    def simulate(self, record=False, physical=False):
        if self.simulation_blocker(): return
        self.state.sim_preflight={}
        client=self.connection
        def begin():
            if not physical:
                status=self.request(client,'status')
                if status.get('arming_state')!='disarmed' or any(status.get(key) for key in ('simulation_active','sensor_hil_active','estimate_replay_active')):
                    raise RuntimeError('Stop the current activity before starting the virtual simulation.')
                if status.get('outputs_active'):
                    try: released=self.request(client,'servo release')
                    except CommandError as error:
                        if error.code in OLD_FIRMWARE:
                            raise RuntimeError('Update the board firmware: automatic servo release is not supported.') from error
                        raise
                    if released.get('released') is not True or released.get('prepared_servo_mask')!=0:
                        raise RuntimeError('Servo release not confirmed by the board. Update the firmware before retrying.')
                    if self.request(client,'status').get('outputs_active'):
                        raise RuntimeError('Servo PWM is still active: simulation not started.')
            if record: self.request(client,'flights recover',600)
            command='sim begin'+(' physical' if physical else '')+(' record' if record else '')
            return dict(sim=self.request(client,command,600 if record else 5),
                        **self._read(client,'status','events','preflight'))
        self._job(t('Preparing simulation') if record else t('Starting simulation'),begin,kind=SIMULATION_START)

    def clear_simulation_preflight(self):
        if self.state.sim_preflight:
            self.state.sim_preflight={}; self.changed.emit()

    def list_flights(self):
        if not self.state.connected or self.state.foreground: return
        self.state.flights_loaded=False
        client=self.connection
        def read():
            self.request(client,'flights recover',600)
            entries=[]
            for i in range(48):
                entry=self.request(client,f'flights list {i}')
                if not entry['available']: break
                entries.append(entry)
            return dict(flights=entries)
        self._job(t('Reading recorded flights'),read)

    def download(self, sequence, path):
        if not self.state.connected or self.state.foreground: return
        client,epoch=self.connection,self._epoch
        destination=Path(path).expanduser().resolve()
        cancel=Event(); self._download_cancel=cancel
        self.state.download=dict(state='downloading',path=str(destination),done=0,total=None,eta_seconds=None)
        def read():
            from download_flight import download
            last_percent=-1; started=None; last_update=0
            def progress(done,total):
                nonlocal last_percent,started,last_update
                now=time.monotonic()
                if started is None: started=now
                percent=done*100//max(1,total)
                if percent!=last_percent or now-last_update>=.5:
                    last_percent=percent; last_update=now; self._progress.emit(epoch,done,total,now-started)
            download(client,sequence,destination,progress=progress,cancelled=cancel.is_set,overwrite=True)
            result=dict(downloaded_path=str(destination))
            try: result['flight']=load_flight(destination)
            except ValueError as error: result['preview_error']='Download completed; preview not available: '+str(error)
            return result
        self._job(t('Downloading flight {sequence}',sequence=sequence),read,kind=DOWNLOAD)

    def cancel_download(self):
        if self._download_cancel and self.state.download.get('state')=='downloading':
            self._download_cancel.set(); self.state.download['state']='cancelling'
            self.state.message=t('Cancelling download…'); self.changed.emit()

    def open_flight(self, path): self._job(t('Opening local flight'),lambda:dict(flight=load_flight(path)))

    def stream_sensors(self,enabled):
        self._sensor_stream=bool(enabled)
        if enabled: self._last_sensors=0

    @Slot(int,int,int,float)
    def _on_progress(self, epoch, done, total, elapsed):
        if epoch==self._epoch and self.state.download.get('state')=='downloading':
            eta=elapsed*(total-done)/done if 0<done<total and elapsed>=1 else None
            self.state.download.update(done=done,total=total,eta_seconds=eta); self.changed.emit()

    def _poll(self, function):
        """Background read: never blocks the UI or replaces the current feedback."""
        return self._job(t('Background update'),function,False,POLL)

    @Slot(int,int,str,object,object,str)
    def _complete(self, epoch, job, name, result, error, kind=OPERATION):
        self._jobs.pop(job,None)
        if epoch != self._epoch: return # A result from an old connection never reaches the new session.
        if kind==PREVIEW:
            self.servo_previewed.emit(dict(error=error_text(error)) if error else result)
            return
        if self._foreground_job==job:
            self._foreground_job=None; self.state.foreground=''; self.state.foreground_kind=''
        if kind==DOWNLOAD:
            self._download_cancel=None
            if isinstance(error,CancelledError):
                self.state.download['state']='cancelled'; self.state.error=''; self.state.message=t('Download cancelled')
                self.changed.emit(); return
        if error: self._apply_error(error,kind)
        else: self._apply_result(result or {},kind)
        self.changed.emit()

    def _apply_error(self, error, kind):
        s=self.state; command=error if isinstance(error,CommandError) else None
        if command and command.command=='pyro continuity': s.pyro_continuity={}
        s.error=error_text(error,s.local); s.message=t('Operation not completed')
        if kind==DOWNLOAD and s.download.get('state') in ('downloading','cancelling'):
            code=command.code if command else ''
            s.download.update(state='failed',error_code=code)
            if code=='Corrupt': s.error='Corrupted flight'
        if command:
            # Firmware refusals carry the reasons: show them where the user acts on them.
            if (command.command=='arm' or command.command.startswith('servo arm')) and command.response.get('reasons'): s.preflight=command.response
            if command.command.startswith('sim begin physical') and command.response.get('reasons'): s.sim_preflight=command.response
            if command.code=='CONFIG_INVALID' and command.command in ('config validate','config apply'): s.staged=True
            if command.response.get('store_error'): s.status['config_valid']=False; s.config_meta['available']=False
        self.logged.emit('error',s.error)

    def _apply_result(self, result, kind):
        self._apply_device_state(result)
        self._apply_configuration(result)
        self._apply_settings(result)
        self._apply_simulation(result)
        if kind!=POLL: # Background reads never replace the feedback of the user's last operation.
            self.state.error=result.get('preview_error',''); self.state.message=t('Configuration saved and verified') if result.get('applied') else t('Valid draft · not saved yet') if result.get('validated') else ''
            if result.get('bluetooth_restart'): self.state.message=t('The Bluetooth module is restarting to apply the name. Reconnecting…')
        s=self.state
        restart=self._bluetooth_restart
        if 'bluetooth' in result and restart and restart['dropped']: # Reconnected after a rename: verify the name.
            self._bluetooth_restart=None
            if result['bluetooth'].get('name')!=restart['name']:
                s.error='Bluetooth name not confirmed after the restart'; self.logged.emit('error',s.error)
            s.bluetooth_restart=''

    def _apply_device_state(self, result):
        s=self.state
        if 'status' in result and result['status'].get('power_mode')!=s.status.get('power_mode'):
            s.preflight={}; s.pyro_continuity={} # Power transitions invalidate earlier checks.
        if result.get('downloaded_path'): s.download.update(state='completed',path=result['downloaded_path'])
        for key in ('status','preflight','pyro_continuity','sensors','capabilities','control','events','flights','flight','orientation','machine','imu_calibration','bluetooth'):
            if key in result: setattr(s,key,result[key])
        if result.get('bluetooth_restart'):
            restart=result['bluetooth_restart']; s.bluetooth_restart=restart['name']
            first=time.monotonic()+self.bluetooth_restart_delay
            self._bluetooth_restart=dict(restart,next=first,deadline=first+self.bluetooth_restart_window,dropped=False)
        if 'flights' in result: s.flights_loaded=True
        if result.get('preflight',{}).get('ready'): s.sim_preflight={}
        if s.status.get('power_mode')!='sleeping': # Sleep is not a sensor fault: keep the last awake observation.
            for source in (result.get('sensors',{}),result.get('status',{})):
                for key in ('imu_ready','baro_ready'):
                    if key in source: s.sensor_availability[key]=source[key]
        if 'sensors' in result: self.sensors_received.emit(result['sensors'])

    def _apply_configuration(self, result):
        s=self.state
        if 'configuration' in result:
            s.pyro_continuity={}
            config=result['configuration']; s.config_meta=config['meta']
            if not (s.dirty and not result.get('applied')): # Keep the user's unsaved local edits.
                s.config_view=config['view']
                s.base=config['plan'] if config['view']=='applied' else None
                s.local=config['plan'].clone() if config['plan'] else Plan()
            if result.get('applied'): s.validated=False; s.staged=False
        if result.get('validated'): s.validated=True; s.staged=True
        if result.get('discarded'):
            s.local=s.base.clone() if s.base else Plan(); s.validated=False; s.staged=False

    def _apply_settings(self, result):
        s=self.state
        for key in ('airbrakes','airbrake_status'):
            if key in result: setattr(s,key,result[key])
        if result.get('reset_settings'): s.setting_drafts.clear()
        elif result.get('saved_setting'): s.setting_drafts.pop(result['saved_setting'],None)

    def _apply_simulation(self, result):
        s=self.state
        if 'sim' not in result: return
        old=s.sim; s.sim=result['sim']
        if s.sim.get('active'): s.sim_preflight={}
        if s.sim.get('error') not in (None,'None',''): s.error=REASONS.get(s.sim['error'],s.sim['error'])
        if 'events' not in result and any(old.get(k)!=s.sim.get(k) for k in ('starts','active','phase')): self._want_events=True

    @Slot()
    def tick(self):
        self.session.poll()
        restart=self._bluetooth_restart
        if restart:
            now=time.monotonic()
            if self.connection is None: restart['dropped']=True
            if self.connection is None and not self.connection_busy and now>=restart['next']:
                if now<=restart['deadline']: # Expected failures while the module restarts stay silent.
                    restart['next']=now+self.bluetooth_retry_interval; self.connect_port(Port(restart['port'],'bluetooth',restart['identity']))
                else:
                    self._bluetooth_restart=None; self.state.bluetooth_restart=''
                    self.state.error='Bluetooth did not come back after the rename: press Connect'
                    self.logged.emit('error',self.state.error); self.changed.emit()
            elif not restart['dropped'] and self.connection is not None and now>=restart['deadline']: # The link never dropped.
                self._bluetooth_restart=None; self.state.bluetooth_restart=''; self.refresh()
        if self._bluetooth_restart: return # The link is about to drop: no background reads that would fail.
        if self.connection_busy: return
        if not self.state.connected or self._jobs or self.state.foreground: return
        client,now=self.connection,time.monotonic()
        sensor_interval=1 if self.state.connection.get('transport')=='bluetooth' else .25
        if self._want_events:
            self._want_events=False
            self._poll(lambda:dict(events=client.request('sim events')))
        elif (self.state.sim.get('active') or self.state.status.get('simulation_active') or self.page==2) and now-self._last_sim>=1.2:
            previous=self.state.sim.get('active',False)
            def poll_sim():
                sim=client.request('sim status'); result=dict(sim=sim)
                if previous and not sim['active']:
                    result['status']=client.request('status'); result['events']=client.request('sim events')
                return result
            self._last_sim=now; self._poll(poll_sim)
        elif self.page==0 and self.state.airbrakes.get('enabled') and self.state.base and self.state.base.airbrakes['channel_mask'] and now-self._last_airbrakes>=(1.5 if self.state.connection.get('transport')=='bluetooth' else .8):
            self._last_airbrakes=now
            self._poll(lambda:self._read(client,'airbrake_status'))
        elif now-self._last_status>=4:
            previous=self.state.status.get('power_mode')
            previous_arming=self.state.status.get('arming_state')
            radio=self.state.bluetooth # Read once at connection; the module may still be starting then.
            bluetooth_pending=bool(radio) and (not radio.get('ready') or ('name' in radio and not radio['name']))
            def poll_status():
                status=client.request('status'); result=dict(status=status,control=client.request('control status'))
                if previous=='sleeping' and status.get('power_mode')=='active':
                    result['preflight']=client.request('preflight'); result['sensors']=client.request('sensors')
                elif previous_arming in ('armed','arming','preparing','closing') and status.get('arming_state')=='disarmed':
                    result['preflight']=client.request('preflight')
                if bluetooth_pending: result['bluetooth']=client.request('bluetooth status') # Until it is ready.
                return result
            self._last_status=now; self._poll(poll_status)
        elif self.state.imu_calibration.get('supported') and (self.page==1 or self.state.imu_calibration.get('active')) and now-self._last_imu>=.75:
            was_active=self.state.imu_calibration.get('active')
            def poll_imu():
                imu=self._read_imu_calibration(client); result=dict(imu_calibration=imu)
                if was_active and not imu.get('active'): result['preflight']=client.request('preflight')
                return result
            self._last_imu=now; self._poll(poll_imu)
        elif self._sensor_stream and now-self._last_sensors>=sensor_interval:
            self._last_sensors=now; self._poll(lambda:dict(sensors=client.request('sensors')))

    def can_close(self): return self.session.idle and not self._jobs

    def shutdown(self):
        self.timer.stop()
        self._executor.shutdown(wait=False,cancel_futures=True)
