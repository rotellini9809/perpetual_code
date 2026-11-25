"""Orientation, maintenance and stale-session isolation against a fresh PC firmware fixture."""
from desktop_support import session
from copy import deepcopy
from unittest.mock import patch
from PySide6.QtWidgets import QMessageBox
from PySide6.QtGui import QTransform
from device_client import CommandError
from perpetual_app.configuration import read_plan
from perpetual_app.controller import POLL


def check_maintenance(app,wait):
    """Maintenance uses real firmware storage and preserves the opposite partition."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
        backend.request('flights init') # One recorded flight must survive configuration reset.
        backend.request('hil begin 0 200 logged_body record 4096')
        for i in range(22): backend.request(f'hil sample {i*5} 0 0 9.80665 0 0 0 101325')
        backend.request('hil stop completed')
        # Maintenance uses real firmware storage and preserves the opposite partition.
        assert window.nav[4].text()=='Impostazioni' and window.nav[5].text()=='Console'
        panel=window.settings_page.orientation_panel
        panel.axis.setCurrentIndex(panel.axis.findData('-X')); app.processEvents()
        assert panel.image.accessibleName()=='Anteprima orientamento -X' and panel.save.isEnabled()
        panel.save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.orientation['up_axis']=='-X'
        assert not panel.save.isEnabled() and '-X' in panel.saved.text()
        panel.axis.setCurrentIndex(panel.axis.findData('+Y')); app.processEvents()
        assert panel.image.accessibleName()=='Anteprima orientamento +Y' and panel.image.pixmap().width()>panel.image.pixmap().height()
        left=panel.image.pixmap().toImage()
        panel.axis.setCurrentIndex(panel.axis.findData('-Y')); app.processEvents()
        assert panel.image.accessibleName()=='Anteprima orientamento -Y' and panel.image.pixmap().width()>panel.image.pixmap().height()
        assert panel.image.pixmap().toImage()!=left
        panel.axis.setCurrentIndex(panel.axis.findData('+Z')); app.processEvents()
        assert panel.image.accessibleName()=='Anteprima orientamento +Z' and not panel.image.pixmap().isNull()
        positive_z=panel.image.pixmap().toImage()
        panel.axis.setCurrentIndex(panel.axis.findData('-Z')); app.processEvents()
        assert panel.image.accessibleName()=='Anteprima orientamento -Z' and panel.image.pixmap().toImage()==positive_z.transformed(QTransform().rotate(180))
        saved=controller.state.base.clone()
        with patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.No):
            before=len(backend.commands); window.settings_page.confirm_maintenance(False)
            assert len(backend.commands)==before
        controller.save_machine(True,50); wait(lambda:not controller.state.foreground)
        assert controller.state.machine['bistage']
        controller.maintenance(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.base is None and not controller.state.local.rules
        assert controller.state.orientation['up_axis'] is None and panel.axis.currentData() is None
        assert panel.axis.currentText()==chr(8212) and not panel.save.isEnabled()
        assert panel.image_opacity.opacity()==.25
        assert not controller.state.machine['bistage'] and not window.settings_page.bistage_enabled.isChecked()
        assert not backend.request('config show')['available']
        assert backend.request('flights info 1')['samples']==22
        controller.edit_plan(saved); controller.validate(apply=True); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.base==saved
        assert 'ORIENTATION_NOT_READY' in controller.state.preflight['reasons'] and not window.dashboard_page.arm_button.isEnabled()
        assert any('Orientamento' in window.dashboard_page.readiness_table.item(i,0).text() for i in range(window.dashboard_page.readiness_table.rowCount()))
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


def check_source_loss(app,wait):
    """Source loss keeps a local draft; results from an old connection are ignored."""
    with session(app,wait) as s:
        controller,window=s.controller,s.window
        # Source loss invalidates controls while retaining a local unsaved draft.
        local=controller.state.local.clone(); local.rules[0]['name']='Unsaved local'
        controller.edit_plan(local)
        foreign=controller.state.base.clone(); foreign.rules[0]['name']='Foreign draft'
        controller._complete(controller._epoch,998,'poll',dict(configuration=dict(plan=foreign,meta=dict(available=True,draft_active=True),view='draft'),control=dict(owner='bluetooth',transport='usb')),None,POLL)
        assert controller.state.dirty and controller.state.local.rules[0]['name']=='Unsaved local'
        assert not controller.state.editable
        controller.disconnect(); wait(lambda:controller.connection is None)
        assert window.arming_state.text()=='—'
        assert window.dashboard_page.imu_label.text().endswith('—') and window.dashboard_page.baro_label.text().endswith('—')
        assert controller.state.dirty and not window.sim_start.isEnabled() and not window.command_send.isEnabled()
        controller.discard(); assert not controller.state.dirty
        # A result from an old connection cannot update a new UI session.
        before=deepcopy(controller.state.status)
        controller._complete(controller._epoch-1,999,'poll',dict(status=dict(armed=True)),None,POLL)
        assert controller.state.status==before
