"""Connection lifecycle above DeviceConnection, using the app's existing executor."""
from PySide6.QtCore import QObject, Signal, Slot, Qt
from device_client import DeviceConnection
from .ports import Port, enumerate_ports, candidates
from .i18n import t


class ConnectionSession(QObject):
    changed = Signal(object, str, object)
    reset = Signal(bool)
    ready = Signal()
    ports_changed = Signal(object)
    port_choice = Signal(object)
    logged = Signal(str, str)
    _finished = Signal(int, str, object, object)

    def __init__(self, executor, factory=DeviceConnection, port_scanner=enumerate_ports, parent=None):
        super().__init__(parent)
        self._executor=executor; self._factory=factory; self.port_scanner=port_scanner
        self.client=None; self.phase='idle'; self.snapshot=self._empty('disconnected')
        # _generation changes on every new attempt or disconnect: callbacks of an older one are ignored.
        self._generation=0; self._pending_scans=0; self._candidates=[]; self._candidate=None
        self._preferred={}; self._failures=[]; self._close_reason=''; self._quiet=False
        self._finished.connect(self._complete, Qt.ConnectionType.QueuedConnection)

    @staticmethod
    def _empty(state): return dict(state=state,transport=None,port='',reply_age_s=None)

    @property
    def busy(self): return self.phase in ('searching','choosing','connecting','closing')

    @property
    def idle(self): return self.client is None and not self.busy and not self._pending_scans

    def _publish(self, snapshot, message='', error=None):
        self.snapshot=snapshot
        self.changed.emit(snapshot,message,error)

    def _watch(self, name, future):
        generation=self._generation
        def done(completed):
            try: result,error=completed.result(),None
            except Exception as e: result,error=None,e
            self._finished.emit(generation,name,result,error)
        future.add_done_callback(done)

    def _scan(self, name):
        self._pending_scans+=1
        self._watch(name,self._executor.submit(self.port_scanner))

    def scan_ports(self):
        if not self.busy and not self._pending_scans: self._scan('ports')

    def start(self, port=None, preferred=None, quiet=False):
        if self.client is not None or self.busy: return
        self._generation+=1; self.phase='searching'; self._failures=[]; self._candidate=None
        self._preferred=preferred or {}; self._candidates=[]; self._quiet=quiet
        self.reset.emit(True)
        self._publish(self._empty('connecting'),t('Connecting…'),'')
        if port is None: self._scan('discover')
        else:
            self._candidates=[port if isinstance(port,Port) else Port(port)]
            self._try_next()

    def _try_next(self):
        if not self._candidates:
            self.phase='idle'
            if self._quiet:
                self._quiet=False
                self._publish(self._empty('disconnected'),t('Disconnected'),'')
                return
            error='; '.join(self._failures) or 'No compatible port found. Connect the board over USB or pair Perpetual via Bluetooth in the Windows settings.'
            self._publish(dict(self._empty('error'),error=error),t('Connection failed'),error)
            return
        group=[p for p in self._candidates if p.transport==self._candidates[0].transport]
        if len(group)>1 and group[0].identity!=self._preferred.get(group[0].transport):
            self.phase='choosing'; self.port_choice.emit(group); return
        self._candidate=self._candidates.pop(0); self.phase='connecting'
        self._publish(dict(state='connecting',transport=self._candidate.transport,port=self._candidate.device,reply_age_s=None),
                      t('Connecting… {port}',port=self._candidate.label))
        try:
            self.client=self._factory(self._candidate.device)
            self._watch('connect',self.client.connect())
        except Exception as error: self._connect_failed(error)

    def choose_port(self, port):
        if self.phase!='choosing': return
        if port is None: self.disconnect(); return
        if port not in self._candidates: return
        transport=self._candidates[0].transport
        self._candidates=[port]+[p for p in self._candidates if p.transport!=transport]
        self._try_next()

    def _connect_failed(self, error):
        raw=str(error); text=raw
        if isinstance(error,PermissionError) or any(marker in raw.casefold() for marker in ('access is denied','accesso negato','permissionerror')):
            text='Port busy or access denied: close Serial Monitor and the other apps using this port'
        elif isinstance(error,TimeoutError):
            text='No response from the board: check the power supply and the MAIN firmware'
        self._failures.append(self._candidate.label+': '+text)
        if not self._quiet: self.logged.emit('error',self._candidate.label+': '+raw)
        if self.client is None: self._try_next()
        else: self._close('next')

    def _close(self, reason):
        self.phase='closing'; self._close_reason=reason
        self._watch('closed',self.client.disconnect(wait=False))
        message=t('Disconnecting…') if reason=='disconnected' else ''
        self._publish(dict(self.snapshot,state='disconnecting'),message)

    def disconnect(self):
        self._candidates=[]
        # Preserve the in-progress close callback, but prevent fallback to another port.
        if self.phase=='closing':
            self._close_reason='disconnected'; return
        self._generation+=1; self.reset.emit(False)
        if self.client is not None: self._close('disconnected')
        else:
            self.phase='idle'
            self._publish(self._empty('disconnected'),t('Disconnected'))

    def poll(self):
        if self.phase!='connected': return
        snapshot=self.client.snapshot()
        if self._candidate: snapshot['identity']=self._candidate.identity
        if snapshot['state'] in ('error','unresponsive','disconnected'):
            error=snapshot.get('error') or 'Connection lost'
            self.logged.emit('error',error)
            self._generation+=1; self.reset.emit(False)
            self._publish(snapshot,error=error)
            self._close('failed')
        else: self._publish(snapshot)

    @Slot(int,str,object,object)
    def _complete(self, generation, name, result, error):
        if name in ('ports','discover'): self._pending_scans-=1
        if generation!=self._generation: return
        if name=='ports':
            if not error: self.ports_changed.emit(result)
        elif name=='discover':
            if error: self._failures.append('Port search: '+str(error))
            else:
                self.ports_changed.emit(result); self._candidates=candidates(result,self._preferred)
            self._try_next()
        elif name=='connect':
            if error: self._connect_failed(error); return
            self._quiet=False; self.phase='connected'; self._candidates=[]
            self.poll()
            if self.phase=='connected':
                self._publish(self.snapshot,t('Connection established'))
                self.ready.emit()
        elif name=='closed':
            reason=self._close_reason; self.client=None; self.phase='idle'
            if reason=='next': self._try_next(); return
            message=t('Connection lost · press Connect to retry') if reason=='failed' else t('Disconnected · the last data are no longer live')
            self._publish(self._empty('error' if reason=='failed' else 'disconnected'),message,str(error) if error else None)
