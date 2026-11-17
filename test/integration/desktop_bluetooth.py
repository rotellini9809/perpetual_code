"""Bluetooth name from Settings: verified over USB; over Bluetooth a silent reconnection after the module restart."""
from desktop_support import Native,session
from perpetual_app.ports import Port


def check_bluetooth_name(app,wait):
    assert Port('COM6','bluetooth','bluetooth:x','',True,'Perpetual-A1').label=='Bluetooth · Perpetual-A1 · COM6' # Several boards.
    assert Port('COM3','usb').label=='USB · COM3'
    # Connected while the module is still starting: the status poll reads it again until it is ready.
    Native.bluetooth_starting=True
    try:
        with session(app,wait) as s:
            page=s.window.settings_page; controller=s.controller; controller.timer.stop()
            def status_poll(): controller._want_events=False; controller._last_status=0; controller.tick(); wait(lambda:not controller._jobs)
            assert page.bluetooth_current.text()=='Modulo Bluetooth non pronto.' and not page.bluetooth_suffix.isEnabled()
            status_poll()
            assert s.backend.commands.count('bluetooth status')==2 # Still starting: asked again.
            s.backend.bluetooth_ready=True
            status_poll()
            assert 'Perpetual' in page.bluetooth_current.text() and page.bluetooth_suffix.isEnabled()
            status_poll()
            assert s.backend.commands.count('bluetooth status')==3 # Ready: no further reads.
    finally: Native.bluetooth_starting=False
    with session(app,wait) as s:
        controller,window,backends=s.controller,s.window,s.backends
        page=window.settings_page; state=controller.state
        assert state.bluetooth['name']=='Perpetual' and 'Perpetual' in page.bluetooth_current.text()
        assert not page.bluetooth_default.isEnabled() and not page.bluetooth_save.isEnabled()
        page.bluetooth_suffix.setText('A_1'); assert not page.bluetooth_save.isEnabled() # Up to 4 letters, digits or -.
        page.bluetooth_suffix.setText('A1'); assert page.bluetooth_save.isEnabled()
        # Over USB the result is read back after the restart.
        page.bluetooth_save.click(); wait(lambda:not state.foreground)
        assert 'bluetooth name A1' in s.backend.commands and state.bluetooth['name']=='Perpetual-A1' and not state.error
        assert 'Perpetual-A1_BLE' in page.bluetooth_current.text() and page.bluetooth_default.isEnabled() and not page.bluetooth_save.isEnabled()
        # Over Bluetooth the link drops while the module restarts: no error, one reconnection, then verification.
        controller.bluetooth_restart_delay=controller.bluetooth_retry_interval=.2
        def restart_over_bluetooth(click):
            backend=backends[-1]; backend.transport='bluetooth'
            controller.state.connection['transport']='bluetooth'
            click(); wait(lambda:not state.foreground)
            assert state.bluetooth_restart and 'Riavvio del modulo Bluetooth' in page.bluetooth_current.text()
            assert not page.bluetooth_suffix.isEnabled()
            backend.state='error' # The radio link is gone.
            count=len(backends)
            wait(lambda:controller.connection is None or len(backends)>count)
            assert not state.error and 'Riconnessione' in state.message # Expected, not a failure.
            wait(lambda:len(backends)>count and state.connected and not state.foreground and not state.bluetooth_restart)
        restart_over_bluetooth(page.bluetooth_default.click)
        assert not state.error and state.bluetooth['name']=='Perpetual' and 'bluetooth name default' in backends[-2].commands
        # A module that comes back with another name is reported (the new fixture keeps "Perpetual").
        page.bluetooth_suffix.setText('B2')
        restart_over_bluetooth(page.bluetooth_save.click)
        assert state.error=='Bluetooth name not confirmed after the restart' # Errors stay English.
        # Windows may refuse the port while the module restarts: retries stay silent until it is back.
        Native.refused_connections=2; Native.bluetooth_default='Perpetual-D4' # The module comes back renamed.
        page.bluetooth_suffix.setText('D4'); restart_over_bluetooth(page.bluetooth_save.click)
        assert not state.error and state.bluetooth['name']=='Perpetual-D4' and Native.refused_connections==0
        # A board that never comes back within the window is reported once.
        controller.bluetooth_restart_window=.5; Native.refused_connections=1000
        backend=backends[-1]; backend.transport='bluetooth'; state.connection['transport']='bluetooth'
        page.bluetooth_suffix.setText('C3'); page.bluetooth_save.click(); wait(lambda:not state.foreground)
        backend.state='error'
        wait(lambda:state.error.startswith('Bluetooth did not come back') and not state.bluetooth_restart)
        Native.refused_connections=0; Native.bluetooth_default='Perpetual'
