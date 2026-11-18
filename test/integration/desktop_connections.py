"""USB/Bluetooth discovery, choice and connect/close lifecycle."""
from concurrent.futures import Future
import tempfile
import threading
import time
from pathlib import Path
from perpetual_app.controller import Controller,POLL
from perpetual_app.flight_data import load_flight
from perpetual_app.window import MainWindow
from perpetual_app.ports import Port,describe_port,candidates


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
            # Board inventory is session-scoped; the opened CSV remains independent.
            local_csv=Path(folder)/'kept.csv'; local_csv.write_text('t_us,z_est,vz_est\n0,0,1\n5000,10,1\n')
            local_flight=load_flight(local_csv); controller.state.flight=local_flight
            old_inventory=[dict(sequence='1',status=2,committed_bytes=1024,allocated_bytes=4096)]
            controller._complete(controller._epoch,0,'poll',dict(flights=old_inventory,status=dict(arming_state='disarmed')),None,POLL)
            page=window.recorded_flights_page; old_epoch=controller._epoch
            assert page.flights_table.rowCount()==1 and page.download_button.isEnabled()
            # Lost sessions close themselves. No manual Disconnect and no replay/reconnect.
            links[1].state='unresponsive'; controller.tick()
            assert not window.connect_button.isEnabled() and window.connect_button.text()=='Connetti'
            assert controller.state.flights==[] and page.flights_table.rowCount()==0
            assert not controller.state.flights_loaded and page.flights_empty.isHidden()
            assert page.flights_table.currentRow()==-1 and not page.download_button.isEnabled()
            assert controller.state.flight is local_flight and page.flight_plot.rows is local_flight['rows']
            links[1].finish_close(); wait(lambda:controller.connection is None)
            assert window.connect_button.isEnabled() and len(links)==2
            window.connect_button.click(); wait(lambda:len(links)==3)
            assert links[2].port==usb.device # USB wins even after Bluetooth was last used.
            links[2].succeed(); wait(lambda:controller.state.connected)
            assert window.transport.text()=='USB · COM43'
            # Even overlapping sequence IDs from another device require a fresh list.
            assert controller.state.flights==[] and page.flights_table.rowCount()==0
            controller._complete(old_epoch,999,'poll',dict(flights=old_inventory),None,POLL)
            assert controller.state.flights==[] and not page.download_button.isEnabled()
            new_inventory=[dict(sequence='1',status=2,committed_bytes=2048,allocated_bytes=4096)]
            controller._complete(controller._epoch,0,'poll',dict(flights=new_inventory,status=dict(arming_state='disarmed')),None,POLL)
            assert page.flights_table.item(0,2).text()=='2.0 KiB' and page.download_button.isEnabled()
            assert controller.state.flight is local_flight

            window.connect_button.click(); links[2].finish_close(); wait(lambda:controller.can_close())
            assert controller.state.flights==[] and page.flights_table.rowCount()==0
            # Both candidates fail; the next click starts a fresh search directly.
            window.connect_button.click(); wait(lambda:len(links)==4)
            links[3].ready.set_exception(PermissionError('busy')); wait(lambda:links[3].close_called)
            links[3].finish_close(); wait(lambda:len(links)==5)
            links[4].ready.set_exception(TimeoutError('no reply')); wait(lambda:links[4].close_called)
            links[4].finish_close(); wait(lambda:not controller.connection_busy)
            assert controller.connection is None and window.connect_button.isEnabled()
            assert 'Port busy' in controller.state.error and 'No response' in controller.state.error
            # A pending open/handshake can still be stopped by closing the app.
            window.connect_button.click(); wait(lambda:len(links)==6)
            controller.disconnect(); links[5].finish_close(); links[5].succeed()
            wait(lambda:controller.can_close()); assert not controller.state.connected and len(links)==6
            controller.session.port_scanner=lambda:[]
            window.connect_button.click(); wait(lambda:not controller.connection_busy)
            assert 'pair Perpetual' in controller.state.error and window.connect_button.isEnabled()
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
