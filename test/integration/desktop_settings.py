"""Setting drafts stay local until their individual save is verified by firmware."""
from unittest.mock import patch
from PySide6.QtWidgets import QMessageBox
from device_client import CommandError
from perpetual_app.controller import Controller
from perpetual_app.window import MainWindow


def check_setting_drafts(app,wait,native):
    backend=native(); controller=Controller(lambda _:backend,lambda:[]); controller.timer.stop()
    window=MainWindow(controller); window.show(); app.processEvents()
    try:
        controller.connect_port('SETTINGS_FIXTURE'); wait(lambda:controller.state.base is not None and not controller.state.foreground)
        s=controller.state; page=window.settings_page; orientation=page.orientation_panel
        assert not s.has_unsaved_changes
        assert not window.bistage_badge.property('active') and not window.airbrake_badge.property('active')
        assert window.config_value.text()==f'{len(s.base.rules)} regole applicate'
        assert all(rule['name'] in window.config_detail.text() for rule in s.base.rules)
        applied_names=window.config_detail.text(); local=s.base.clone(); local.rules[0]['name']='Nome ancora in bozza'
        controller.edit_plan(local)
        assert window.config_detail.text()==applied_names # Dashboard names belong to applied rules, not the editable draft.
        controller.edit_plan(s.base.clone())

        status,preflight=s.status.copy(),s.preflight.copy()
        for code,detail,target in (('Missing','Nessuna configurazione salvata',1),('Corrupt','danneggiati',1),
                                   ('STATE_MACHINE_INVALID','Macchina a stati',4),('AIRBRAKES_SETTING_INVALID','Aerofreni',4)):
            s.status=dict(status,config_valid=False,config_error=code)
            s.preflight=dict(preflight,ready=False,reasons=['CONFIG_INVALID']); window.render()
            text=window.readiness_table.item(0,0).text()
            assert text.startswith('Configurazione non valida') and detail.lower() in text.lower()
            assert not window.config_issue_button.isHidden() and not window.arm_button.isEnabled()
            window.config_issue_button.click(); assert window.pages.currentIndex()==target
        s.status=status; s.preflight=preflight; window.navigate(0); window.render()
        assert window.config_issue_button.isHidden()
        marker=len(backend.commands)
        page.bistage_enabled.setChecked(True); page.bistage_min_speed.setValue(42.5)
        page.airbrakes_enabled.setChecked(True)
        orientation.axis.setCurrentIndex(orientation.axis.findData('-X'))
        assert s.has_unsaved_changes and not s.dirty
        assert not s.machine['bistage'] and not s.airbrakes['enabled'] and s.orientation['up_axis']=='+X'
        assert not any(command.startswith(('config machine ','config orientation ','config airbrakes ')) for command in backend.commands[marker:])
        assert window.configuration_page.airbrake_button.isHidden() # Previewing enable must not activate the feature.
        assert not window.bistage_badge.property('active') and not window.airbrake_badge.property('active')
        window.navigate(2); assert window.sim_start.isEnabled() # A run continues to use acknowledged settings.
        controller.refresh(); wait(lambda:not s.foreground)
        for index in (0,1,2,3,4): window.navigate(index); app.processEvents()
        assert page.bistage_enabled.isChecked() and page.bistage_min_speed.value()==42.5
        assert page.airbrakes_enabled.isChecked() and orientation.axis.currentData()=='-X'

        request=backend.request
        def failed_save(command,timeout=5):
            if command.startswith('config machine bistage '): raise CommandError(command,dict(ok=False,code='STORAGE_ERROR'))
            return request(command,timeout)
        with patch.object(backend,'request',side_effect=failed_save):
            page.machine_save.click(); wait(lambda:not s.foreground)
        assert s.error and not s.machine['bistage'] and s.has_unsaved_changes and page.machine_save.isEnabled()
        assert not window.bistage_badge.property('active')
        assert page.bistage_enabled.isChecked() and page.bistage_min_speed.value()==42.5

        page.machine_save.click(); wait(lambda:not s.foreground)
        assert not s.error and backend.request('config machine')['bistage']
        assert backend.request('config machine')['minimum_coast_velocity']==42.5
        assert not s.setting_changed('machine') and s.setting_changed('orientation') and s.setting_changed('airbrakes')
        assert window.bistage_badge.property('active') and not window.airbrake_badge.property('active')
        assert page.machine_save.text()=='Salvato' and not page.machine_save.isEnabled()
        assert orientation.axis.currentData()=='-X' and page.airbrakes_enabled.isChecked()
        assert not backend.request('config airbrakes')['enabled'] and backend.request('config orientation')['up_axis']=='+X'
        orientation.save.click(); wait(lambda:not s.foreground)
        assert not s.setting_changed('orientation') and s.setting_changed('airbrakes')
        assert backend.request('config orientation')['up_axis']=='-X'
        page.airbrakes_save.click(); wait(lambda:not s.foreground)
        assert backend.request('config airbrakes')['enabled'] and not s.has_unsaved_changes
        assert not window.configuration_page.airbrake_button.isHidden()
        assert window.bistage_badge.property('active') and window.airbrake_badge.property('active')

        # Closing also commits numeric text still being edited without Enter/focus loss.
        page.bistage_min_speed.lineEdit().setText('45.5')
        assert page.bistage_min_speed.value()==42.5
        with patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.No) as question:
            window.close(); question.assert_called_once()
        assert s.has_unsaved_changes and s.machine['minimum_coast_velocity']==42.5
        page.bistage_min_speed.setValue(42.5); assert not s.has_unsaved_changes

        # Reuse the same generic close confirmation for every group, even when controls are busy.
        edits=((lambda:page.bistage_enabled.setChecked(False),lambda:page.bistage_enabled.setChecked(True)),
               (lambda:page.airbrakes_enabled.setChecked(False),lambda:page.airbrakes_enabled.setChecked(True)),
               (lambda:orientation.axis.setCurrentIndex(orientation.axis.findData('+X')),lambda:orientation.axis.setCurrentIndex(orientation.axis.findData('-X'))))
        for edit,undo in edits:
            edit(); assert s.has_unsaved_changes
            assert window.bistage_badge.property('active') and window.airbrake_badge.property('active') # Unsaved disable previews leave saved badges active.
            s.foreground='Operazione in corso'; window.render()
            with patch('perpetual_app.window.QMessageBox.question',return_value=QMessageBox.No) as question,patch.object(controller,'disconnect') as disconnect:
                window.close()
                question.assert_called_once(); disconnect.assert_not_called()
                message=question.call_args.args[2].lower()
                assert 'modifiche' in message and all(word not in message for word in ('orientamento','bistadio','aerofreni'))
            assert window.isVisible() and not window._closing
            s.foreground=''; undo(); window.render(); assert not s.has_unsaved_changes

        # A link loss retains local edits in the app; it never writes them to the board.
        page.bistage_enabled.setChecked(False); assert s.has_unsaved_changes
        controller.disconnect(); wait(controller.can_close)
        assert s.has_unsaved_changes and not page.bistage_enabled.isChecked()
        assert not window.bistage_badge.property('active') and not window.airbrake_badge.property('active')
    finally:
        if controller.connection: controller.disconnect(); wait(controller.can_close)
        window._allow_close=True; window.close(); app.processEvents(); controller.shutdown()
