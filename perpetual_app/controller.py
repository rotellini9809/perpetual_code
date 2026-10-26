"""Qt application state and asynchronous operations; widgets never own serial IO."""
from concurrent.futures import ThreadPoolExecutor,CancelledError
from dataclasses import dataclass, field
from pathlib import Path
import csv
import json
import math
import time
from threading import Event

from PySide6.QtCore import QObject, QTimer, Signal, Slot, Qt
from device_client import CommandError, DeviceConnection
from .configuration import Plan, read_plan, stage_plan, rule_label
from .ports import enumerate_ports
from .connection import ConnectionSession

PHASES = ['PAD', 'BOOST', 'COAST', 'DESCENT', 'LANDED', 'BOOST 2', 'COAST 2', 'ABORT']
REASONS = {
    'IMU_UNAVAILABLE':'IMU non disponibile', 'BAROMETER_UNAVAILABLE':'Barometro non disponibile',
    'STORAGE_NOT_READY':'Memoria di registrazione da preparare',
    'CONTINUITY_FAULT':'Continuità dei canali pyro da verificare', 'SERVOS_NOT_PREPARED':'Servo da preparare',
    'SERVOS_NOT_CONFIGURED':'Posizioni iniziale e finale dei servo da configurare',
    'SERVO_PREPARATION_FAILED':'Preparazione della posizione iniziale del servo non riuscita',
    'OUTPUT_ERROR':'Comando a un’uscita fisica non riuscito',
    'SERVO_NOT_ASSIGNED':'Servo non assegnato ad alcuna regola',
    'PHYSICAL_SERVOS_PREPARED':'Servo ancora preparati: usa Rilascia servo e aggiorna il firmware della scheda',
    'ORIENTATION_NOT_READY':'Orientamento della scheda da salvare in Impostazioni',
    'CONFIG_INVALID':'Configurazione non valida',
    'DRAFT_ACTIVE':'Bozza sulla scheda ancora attiva', 'BUSY':'Scheda occupata',
    'OUTPUT_ALREADY_ASSIGNED':'Canale già assegnato: ogni canale può appartenere a una sola regola',
    'CONTROL_BUSY':'Controllo riservato a un altro collegamento',
    'SENSOR_SLEEP_FAILED':'Sospensione dei sensori non riuscita',
    'FLASH_SLEEP_FAILED':'Sospensione della memoria non riuscita', 'FLASH_WAKE_FAILED':'Riattivazione della memoria non riuscita',
    'DISARM_PROHIBITED_IN_FLIGHT':'Disarmo rifiutato durante il volo', 'NO_DRAFT':'Nessuna bozza sulla scheda',
}


def error_text(error,plan=None):
    if isinstance(error, CommandError):
        details = error.response
        text = REASONS.get(error.code,error.code)
        if details.get('validation'):
            validation=details['validation']; index=details.get('index',0)
            if validation=='InvalidPyroSettings':
                text += f' · PYRO {index+1}: durata impulso mancante o fuori dai limiti'
            elif validation=='InvalidServoSettings':
                text += f' · SERVO {index+1}: impostazioni incomplete o fuori dai limiti. Controlla Canali → SERVO {index+1}'
            else:
                text += f' · {validation}'
                if details.get('rule_id'): text += ' · '+rule_label(plan.rules if plan else [],details['rule_id'])
                if details.get('field') in (2,3): text += f' · {"condizione" if details["field"]==2 else "uscita"} {index+1}'
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
    pyro_continuity: dict = field(default_factory=dict)
    orientation: dict = field(default_factory=dict)
    machine: dict = field(default_factory=dict)
    capabilities: dict = field(default_factory=dict)
    control: dict = field(default_factory=dict)
    config_meta: dict = field(default_factory=dict)
    base: object = None
    local: object = None
    config_view: str = 'applied'
    validated: bool = False
    staged: bool = False
    sim: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)
    flights: list = field(default_factory=list)
    flight: dict = field(default_factory=dict)
    download: dict = field(default_factory=dict)
    foreground: str = ''
    message: str = ''
    error: str = ''

    @property
    def connected(self): return self.connection.get('state') in ('connected','waiting')

    @property
    def dirty(self): return self.local is not None and self.local != self.base and self.config_view=='applied'

    @property
    def editable(self):
        return self.connected and self.owns_control and not self.foreground and self.config_view=='applied' and self.status.get('arming_state')=='disarmed' and not self.status.get('estimate_replay_active')

    @property
    def owns_control(self): return self.control.get('owner','none') in ('none',self.connection.get('transport'))


def load_flight(path):
    path = Path(path)
    rows = []; phases=[]; total=0; first=None; last=None
    fields=('z_est','vz_est','az_est','roll','pitch','yaw','gyroX','gyroY','gyroZ','battery_v')
    def number(row,key):
        try:
            value=float(row.get(key,''))
            if key.startswith('gyro'): value=math.degrees(value)
        except (TypeError,ValueError): return None
        return value if math.isfinite(value) else None
    with path.open(newline='',encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        if not {'t_ms','z_est','vz_est'} <= set(reader.fieldnames or []): raise ValueError('CSV non compatibile: servono t_ms, z_est e vz_est')
        previous = -1
        for row in reader:
            stamp = int(row['t_ms'])
            if stamp <= previous: raise ValueError('Tempi del CSV non crescenti')
            previous = stamp; total+=1
            if first is None: first=stamp
            last=stamp
            rows.append((stamp/1000,*(number(row,key) for key in fields)))
            phase=row.get('state') or ''
            name=PHASES[int(phase)] if phase.isdigit() and 0<=int(phase)<len(PHASES) else None
            if not phases or phases[-1][1]!=name: phases.append((stamp/1000,name))
    valid=[r for r in rows if r[1] is not None and r[2] is not None]
    if not valid: raise ValueError('Nessun campione valido nel CSV')
    metadata = {}
    if path.with_suffix('.json').exists(): metadata=json.loads(path.with_suffix('.json').read_text(encoding='utf-8'))
    if (metadata.get('calibration') or {}).get('bistage'):
        phases=[(stamp,{'BOOST':'BOOST 1','COAST':'COAST 1'}.get(name,name)) for stamp,name in phases]
    step = max(1,math.ceil(len(rows)/1200))
    plot=[]
    for i in range(0,len(rows),step):
        # Preserve gaps even when invalid readings fall between displayed points.
        bucket=rows[max(0,i-step+1):i+1]
        plot.append((rows[i][0],*(rows[i][col] if all(r[col] is not None for r in bucket) else None for col in range(1,len(fields)+1))))
    if (len(rows)-1)%step: plot.append(rows[-1])
    return dict(path=str(path),rows=plot,phases=phases,samples=total,valid_samples=len(valid),duration=(last-first)/1000,
                max_altitude=max(r[1] for r in valid),max_speed=max(abs(r[2]) for r in valid),metadata=metadata)


class Controller(QObject):
    changed = Signal()
    connection_changed = Signal(object)
    ports_changed = Signal(object)
    port_choice = Signal(object)
    logged = Signal(str,str)
    sensors_received = Signal(object)
    servo_previewed = Signal(object)
    _finished = Signal(int,int,str,object,object)
    _progress = Signal(int,int,int,float)

    def __init__(self, connection_factory=DeviceConnection, port_scanner=enumerate_ports):
        super().__init__()
        self.state = AppState()
        self._executor = ThreadPoolExecutor(max_workers=1,thread_name_prefix='Perpetual operations')
        self._epoch, self._serial = 0, 0
        self._jobs = {}
        self._last_status = self._last_sim = self._last_sensors = 0
        self._want_events = False
        self.page = 0
        self._sensor_stream = False
        self._download_cancel = None
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

    def _job(self, name, function, foreground=True):
        if foreground and self.state.foreground: return False
        self._serial += 1
        job, epoch = self._serial,self._epoch
        self._jobs[job]=(name,epoch)
        if foreground:
            self.state.foreground=name; self.state.error=''; self.state.message=name+'…'; self.changed.emit()
        def execute():
            try: result,error=function(),None
            except Exception as e: result,error=None,e
            self._finished.emit(epoch,job,name,result,error)
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

    def _reset_session(self, starting):
        self._epoch+=1; self.state.foreground=''; self.state.validated=False; self.state.staged=False
        if self._download_cancel: self._download_cancel.set(); self._download_cancel=None
        if self.state.download.get('state') in ('downloading','cancelling'): self.state.download['state']='interrupted'
        if starting:
            for key in ('status','preflight','pyro_continuity','sim','events','sensors','sensor_availability','orientation','machine'): setattr(self.state,key,{})

    def _connection_update(self, snapshot, message, error):
        previous=self.state.connection.get('state')
        self.state.connection=snapshot
        if message: self.state.message=message
        if error is not None: self.state.error=error
        self.connection_changed.emit(snapshot)
        if previous!=snapshot.get('state') or message or error is not None: self.changed.emit()

    def _read_config(self, client):
        plan,meta,view=read_plan(client)
        return dict(plan=plan,meta=meta,view=view)

    def _read_orientation(self,client):
        try: return dict(self.request(client,'config orientation'),supported=True)
        except CommandError as error:
            if error.code not in ('UNKNOWN_COMMAND','INVALID_ARGUMENT','NO_DRAFT'): raise
            return dict(supported=False)

    def _read_machine(self,client):
        try: return dict(self.request(client,'config machine'),supported=True)
        except CommandError as error:
            if error.code not in ('UNKNOWN_COMMAND','INVALID_ARGUMENT','NO_DRAFT'): raise
            return dict(supported=False)

    def save_machine(self,bistage,minimum):
        s=self.state
        if not s.editable or s.dirty or s.staged or s.machine.get('supported') is False: return
        if s.sim.get('active') or s.status.get('sensor_hil_active'): return
        if type(bistage) is not bool or not math.isfinite(minimum) or not 0<=minimum<=100000: return
        client=self.connection
        def save():
            command='config machine '+('bistage '+format(minimum,'.9g') if bistage else 'normal')
            reply=self.request(client,command,60)
            if not all(reply.get(key) is True for key in ('persisted','verified')): raise RuntimeError('Salvataggio macchina a stati non confermato')
            machine=self._read_machine(client)
            if machine.get('bistage')!=bistage or machine.get('minimum_coast_velocity')!=(minimum if bistage else 0):
                raise RuntimeError('Verifica macchina a stati non riuscita')
            return dict(machine=machine,status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Salvataggio macchina a stati',save)

    def refresh(self):
        if not self.state.connected or self.state.foreground: return
        client=self.connection
        def read():
            return dict(status=self.request(client,'status'),capabilities=self.request(client,'capabilities'),
                        control=self.request(client,'control status'),preflight=self.request(client,'preflight'),
                        sensors=self.request(client,'sensors'),configuration=self._read_config(client),sim=self.request(client,'sim status'),orientation=self._read_orientation(client),machine=self._read_machine(client))
        self._job('Sincronizzazione scheda',read)

    def reload_config(self):
        if self.state.connected:
            client=self.connection
            self._job('Lettura configurazione',lambda:dict(configuration=self._read_config(client)))

    def edit_plan(self, plan):
        self.state.local=plan; self.state.validated=False; self.state.error=''; self.changed.emit()

    def validate(self, apply=False):
        if not self.state.editable or not self.state.dirty: return
        client,base,local=self.connection,self.state.base,self.state.local.clone()
        def write():
            stage_plan(client,base,local)
            if not apply: return dict(validated=True,control=self.request(client,'control status'))
            saved=self.request(client,'config apply',60)
            if not all(saved.get(k) is True for k in ('applied','persisted','verified')):
                raise RuntimeError('Il salvataggio non è stato confermato completamente dalla scheda')
            return dict(applied=True,configuration=self._read_config(client),status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Salvataggio configurazione' if apply else 'Validazione della bozza',write)

    def discard(self):
        if self.state.foreground: return
        if not self.state.connected:
            self.state.local=self.state.base.clone() if self.state.base else None
            self.state.validated=False; self.changed.emit(); return
        client=self.connection
        def discard_remote():
            meta=self.request(client,'config draft'); control=self.request(client,'control status')
            if meta['available'] and control['owner']==control['transport']: self.request(client,'config discard')
            return dict(discarded=True,status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Scarto della bozza',discard_remote)

    def command(self, command, name=None):
        if not self.state.connected: return
        client=self.connection; was_sleeping=self.state.status.get('power_mode')=='sleeping'
        def send():
            response=self.request(client,command,timeout=60 if command.startswith('flights ') else 5)
            data=dict(console=response)
            if command=='status': data['status']=response
            elif command.startswith('preflight') and response.get('requested_mode')!='simulation': data['preflight']=response
            elif command=='sensors': data['sensors']=response
            elif command=='pyro continuity': data['pyro_continuity']=response
            elif command.startswith('sim '): data.update(sim=self.request(client,'sim status'),status=self.request(client,'status'))
            elif command in ('arm','disarm','ds','sleep','wake') or command.startswith('servo arm') or command=='servo release':
                data['status']=self.request(client,'status')
                if command!='sleep': data['preflight']=self.request(client,'preflight')
                if command=='wake': data['sensors']=self.request(client,'sensors')
                if command in ('disarm','ds'): data['sim']=self.request(client,'sim status')
            if was_sleeping and command!='sleep':
                data['status']=self.request(client,'status')
                if data['status'].get('power_mode')=='active':
                    if 'preflight' not in data: data['preflight']=self.request(client,'preflight')
                    if 'sensors' not in data: data['sensors']=self.request(client,'sensors')
            return data
        self._job(name or 'Comando: '+command,send)

    def save_orientation(self,axis):
        s=self.state
        if s.orientation.get('supported') is False: return
        if axis not in ('+X','-X','+Y','-Y','+Z','-Z'): return
        if not s.editable or s.dirty or s.staged or s.sim.get('active') or s.status.get('sensor_hil_active'): return
        client=self.connection
        def save():
            response=self.request(client,'config orientation '+axis,60)
            if not all(response.get(key) is True for key in ('persisted','verified','orientation_ready')) or response.get('up_axis')!=axis:
                raise RuntimeError('Orientamento non confermato dalla scheda')
            orientation=self.request(client,'config orientation')
            if orientation.get('up_axis')!=axis or not orientation.get('orientation_ready'): raise RuntimeError('Verifica orientamento non riuscita')
            return dict(orientation=orientation,status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Salvataggio orientamento',save)

    def maintenance(self,format_memory=False):
        s=self.state
        if not s.editable or s.sim.get('active') or s.status.get('sensor_hil_active'): return
        if format_memory and (s.dirty or s.staged): return
        client=self.connection
        def perform():
            response=self.request(client,'flights format' if format_memory else 'config reset',60)
            required=('formatted','ready') if format_memory else ('reset','persisted','verified')
            if not all(response.get(key) is True for key in required): raise RuntimeError('Operazione non confermata dalla scheda')
            result=dict(status=self.request(client,'status'),preflight=self.request(client,'preflight'))
            if format_memory: result.update(flights=[],flight={})
            else: result.update(applied=True,configuration=self._read_config(client),control=self.request(client,'control status'),orientation=self.request(client,'config orientation'),machine=self._read_machine(client))
            return result
        self._job('Formattazione memoria' if format_memory else 'Ripristino dati di fabbrica',perform)

    def prepare_servos(self, channels):
        if not self.state.connected: return
        client=self.connection
        def prepare():
            for command in ([f'servo arm {i}' for i in channels] if channels else ['servo arm']): self.request(client,command)
            return dict(status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Preparazione servo',prepare)

    def preview_servo(self, channel, pwm):
        if not self.state.editable or self._jobs: return False
        client=self.connection
        return self._job('servo preview',lambda:self.request(client,f'servo preview {channel} {pwm}'),False)

    def release_servo(self, channel):
        if not self.state.connected: return
        client=self.connection
        def release():
            self.request(client,f'servo release {channel}')
            return dict(status=self.request(client,'status'),preflight=self.request(client,'preflight'))
        self._job('Rilascio servo',release,False)

    def simulation_blocker(self):
        s=self.state
        if not s.connected: return 'Collega la scheda per avviare la simulazione.'
        if s.foreground: return 'Operazione in corso: '+s.foreground
        if not s.owns_control: return 'Scheda controllata da un altro collegamento.'
        if s.sim.get('active') or s.status.get('simulation_active'): return 'Simulazione in corso.'
        if s.status.get('arming_state')!='disarmed': return 'La scheda deve essere disarmata per avviare la simulazione.'
        if s.status.get('sensor_hil_active') or s.status.get('estimate_replay_active'): return 'Arresta il test in corso prima di avviare la simulazione.'
        if not s.status.get('config_valid'): return 'Applica una configurazione valida prima di avviare la simulazione.'
        if s.dirty: return 'Applica o scarta le modifiche alla configurazione.'
        if s.staged: return 'Applica o scarta la bozza validata sulla scheda.'
        return ''

    def simulate(self, record=False, physical=False):
        if self.simulation_blocker(): return
        client=self.connection
        def begin():
            if not physical:
                status=self.request(client,'status')
                if status.get('arming_state')!='disarmed' or any(status.get(key) for key in ('simulation_active','sensor_hil_active','estimate_replay_active')):
                    raise RuntimeError('Arresta l'+chr(8217)+'attivit'+chr(224)+' in corso prima di avviare la simulazione virtuale.')
                if status.get('outputs_active'):
                    try: released=self.request(client,'servo release')
                    except CommandError as error:
                        if error.code in ('UNKNOWN_COMMAND','INVALID_ARGUMENT','NO_DRAFT'):
                            raise RuntimeError('Aggiorna il firmware della scheda: il rilascio automatico dei servo non '+chr(232)+' supportato.') from error
                        raise
                    if released.get('released') is not True or released.get('prepared_servo_mask')!=0:
                        raise RuntimeError('Rilascio servo non confermato dalla scheda. Aggiorna il firmware prima di riprovare.')
                    if self.request(client,'status').get('outputs_active'):
                        raise RuntimeError('Il PWM dei servo risulta ancora attivo: simulazione non avviata.')
            if record: self.request(client,'flights recover',600)
            command='sim begin'+(' physical' if physical else '')+(' record' if record else '')
            return dict(sim=self.request(client,command,600 if record else 5),
                        status=self.request(client,'status'),events=self.request(client,'sim events'),preflight=self.request(client,'preflight'))
        self._job('Preparazione della simulazione' if record else 'Avvio simulazione',begin)

    def list_flights(self):
        if not self.state.connected: return
        client=self.connection
        def read():
            self.request(client,'flights recover',600)
            entries=[]
            for i in range(48):
                entry=self.request(client,f'flights list {i}')
                if not entry['available']: break
                entries.append(entry)
            return dict(flights=entries)
        self._job('Lettura dei voli registrati',read)

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
            download(client,sequence,destination,progress=progress,cancelled=cancel.is_set)
            result=dict(downloaded_path=str(destination))
            try: result['flight']=load_flight(destination)
            except ValueError as error: result['preview_error']='Download completato; anteprima non disponibile: '+str(error)
            return result
        self._job(f'Download volo {sequence}',read)

    def cancel_download(self):
        if self._download_cancel and self.state.download.get('state')=='downloading':
            self._download_cancel.set(); self.state.download['state']='cancelling'
            self.state.message='Annullamento download…'; self.changed.emit()

    def open_flight(self, path): self._job('Apertura volo locale',lambda:dict(flight=load_flight(path)))

    def stream_sensors(self,enabled):
        self._sensor_stream=bool(enabled)
        if enabled: self._last_sensors=0

    @Slot(int,int,int,float)
    def _on_progress(self, epoch, done, total, elapsed):
        if epoch==self._epoch and self.state.download.get('state')=='downloading':
            eta=elapsed*(total-done)/done if 0<done<total and elapsed>=1 else None
            self.state.download.update(done=done,total=total,eta_seconds=eta); self.changed.emit()

    @Slot(int,int,str,object,object)
    def _complete(self, epoch, job, name, result, error):
        self._jobs.pop(job,None)
        if epoch != self._epoch: return
        if name=='servo preview':
            self.servo_previewed.emit(dict(error=error_text(error)) if error else result)
            return
        if self.state.foreground==name: self.state.foreground=''
        if name.startswith('Download volo '):
            self._download_cancel=None
            if isinstance(error,CancelledError):
                self.state.download['state']='cancelled'; self.state.error=''; self.state.message='Download annullato'
                self.changed.emit(); return
        if error:
            if isinstance(error,CommandError) and error.command=='pyro continuity': self.state.pyro_continuity={}
            self.state.error=error_text(error,self.state.local); self.state.message='Operazione non completata'
            if name.startswith('Download volo ') and self.state.download.get('state') in ('downloading','cancelling'):
                code=error.code if isinstance(error,CommandError) else ''
                self.state.download.update(state='failed',error_code=code)
                if code=='Corrupt': self.state.error='Volo corrotto'
            if isinstance(error,CommandError) and (error.command=='arm' or error.command.startswith('servo arm')) and error.response.get('reasons'): self.state.preflight=error.response
            if isinstance(error,CommandError) and error.code=='CONFIG_INVALID' and error.command in ('config validate','config apply'): self.state.staged=True
            if isinstance(error,CommandError) and error.response.get('store_error'):
                self.state.status['config_valid']=False; self.state.config_meta['available']=False
            self.logged.emit('error',self.state.error)
        else:
            result=result or {}
            if 'status' in result and result['status'].get('power_mode')!=self.state.status.get('power_mode'):
                self.state.preflight={}; self.state.pyro_continuity={}
            if result.get('downloaded_path'):
                self.state.download.update(state='completed',path=result['downloaded_path'])
            for key in ('status','preflight','pyro_continuity','sensors','capabilities','control','events','flights','flight','orientation','machine'):
                if key in result: setattr(self.state,key,result[key])
            if self.state.status.get('power_mode')!='sleeping':
                for source in (result.get('sensors',{}),result.get('status',{})):
                    for key in ('imu_ready','baro_ready'):
                        if key in source: self.state.sensor_availability[key]=source[key]
            if 'sensors' in result: self.sensors_received.emit(result['sensors'])
            if 'configuration' in result:
                self.state.pyro_continuity={}
                config=result['configuration']; self.state.config_meta=config['meta']
                preserve=self.state.dirty and not result.get('applied')
                if not preserve:
                    self.state.config_view=config['view']
                    self.state.base=config['plan'] if config['view']=='applied' else None
                    self.state.local=config['plan'].clone() if config['plan'] else Plan()
                if result.get('applied'): self.state.validated=False; self.state.staged=False
            if 'sim' in result:
                old=self.state.sim
                self.state.sim=result['sim']
                if self.state.sim.get('error') not in (None,'None',''):
                    self.state.error=REASONS.get(self.state.sim['error'],self.state.sim['error'])
                if 'events' not in result and any(old.get(k)!=self.state.sim.get(k) for k in ('starts','active','phase')): self._want_events=True
            if result.get('validated'): self.state.validated=True; self.state.staged=True
            if result.get('discarded'):
                self.state.local=self.state.base.clone() if self.state.base else Plan(); self.state.validated=False; self.state.staged=False
            if name!='poll':
                self.state.error=result.get('preview_error',''); self.state.message='Configurazione salvata e verificata' if result.get('applied') else 'Bozza valida · non ancora salvata' if result.get('validated') else ''
        self.changed.emit()

    @Slot()
    def tick(self):
        self.session.poll()
        if self.connection_busy: return
        if not self.state.connected or self._jobs or self.state.foreground: return
        client,now=self.connection,time.monotonic()
        sensor_interval=1 if self.state.connection.get('transport')=='bluetooth' else .25
        if self._want_events:
            self._want_events=False
            self._job('poll',lambda:dict(events=client.request('sim events')),False)
        elif (self.state.sim.get('active') or self.state.status.get('simulation_active') or self.page==2) and now-self._last_sim>=1.2:
            previous=self.state.sim.get('active',False)
            def poll_sim():
                sim=client.request('sim status'); result=dict(sim=sim)
                if previous and not sim['active']:
                    result['status']=client.request('status'); result['events']=client.request('sim events')
                return result
            self._last_sim=now; self._job('poll',poll_sim,False)
        elif now-self._last_status>=4:
            previous=self.state.status.get('power_mode')
            def poll_status():
                status=client.request('status'); result=dict(status=status,control=client.request('control status'))
                if previous=='sleeping' and status.get('power_mode')=='active':
                    result['preflight']=client.request('preflight'); result['sensors']=client.request('sensors')
                return result
            self._last_status=now; self._job('poll',poll_status,False)
        elif self._sensor_stream and now-self._last_sensors>=sensor_interval:
            self._last_sensors=now; self._job('poll',lambda:dict(sensors=client.request('sensors')),False)

    def can_close(self): return self.session.idle and not self._jobs

    def shutdown(self):
        self.timer.stop()
        self._executor.shutdown(wait=False,cancel_futures=True)
