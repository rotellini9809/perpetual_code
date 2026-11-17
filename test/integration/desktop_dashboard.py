"""Dashboard, Settings and Configuration views against a fresh PC firmware fixture."""
from desktop_support import session
from copy import deepcopy
import threading
import time
from unittest.mock import patch
from device_client import CommandError
from perpetual_app.controller import POLL
from PySide6.QtTest import QTest


def check_bistage_settings(app,wait):
    """Bistage machine settings are saved individually and acknowledged by firmware."""
    with session(app,wait,connect=False) as s:
        controller,window=s.controller,s.window
        assert not window.dashboard_page.arm_button.isEnabled() and not window.sim_start.isEnabled()
        assert not window.settings_page.bistage_enabled.isChecked() and not window.settings_page.bistage_min_speed.isEnabled()
        assert window.settings_page.flight_state_sequence.text()==(' '+chr(8594)+' ').join(('PAD','BOOST','COAST','DESCENT','LANDED'))

        controller.connect_port('PC_FIXTURE')
        wait(lambda:controller.state.local is not None and not controller.state.foreground)
        assert window.settings_page.bistage_enabled.parentWidget() is window.settings_page.bistage_parameters
        assert window.settings_page.bistage_enabled.isEnabled() and window.settings_page.machine_saved.isHidden()
        window.settings_page.bistage_enabled.setChecked(True); assert window.settings_page.bistage_min_speed.isEnabled()
        assert 'BOOST 2' in window.settings_page.flight_state_sequence.text() and 'COAST 2' in window.settings_page.flight_state_sequence.text()
        assert 'ABORT' in window.settings_page.flight_state_sequence.text() and chr(8595) in window.settings_page.flight_state_sequence.text()
        window.settings_page.bistage_min_speed.setValue(42.5)
        assert window.settings_page.bistage_min_speed.value()==42.5 and not window.settings_page.bistage_min_speed.suffix()
        window.settings_page.bistage_enabled.setChecked(False); assert not window.settings_page.bistage_min_speed.isEnabled()
        assert window.settings_page.bistage_min_speed.value()==42.5
        window.settings_page.bistage_enabled.setChecked(True); assert window.settings_page.machine_save.isEnabled()
        assert window.settings_page.machine_save.text()=='Salva' and window.settings_page.machine_saved.isHidden()
        window.settings_page.machine_save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.error and controller.state.machine['bistage']
        assert controller.state.machine['minimum_coast_velocity']==42.5
        assert not window.settings_page.machine_save.isEnabled()
        assert window.settings_page.machine_saved.isHidden()
        window.settings_page.bistage_enabled.setChecked(False); window.settings_page.machine_save.click(); wait(lambda:not controller.state.foreground)
        assert not controller.state.machine['bistage']


def check_rule_selection(app,wait):
    """Rules tree selection targets whole rules and survives local edits."""
    with session(app,wait) as s:
        controller,window=s.controller,s.window
        assert window.configuration_page.channels_table.rowCount()==8 and window.configuration_page.rules_tree.topLevelItemCount()==2
        for i,r in enumerate(controller.state.local.rules):
            assert window.configuration_page.rules_tree.topLevelItem(i).text(0)==r['name']
        # A click on a detail selects one whole rule, including its sibling rows.
        from PySide6.QtCore import Qt
        tree=window.configuration_page.rules_tree; window.navigate(1); app.processEvents()
        first,second=(tree.topLevelItem(i) for i in range(2))
        QTest.mouseClick(tree.viewport(),Qt.LeftButton,pos=tree.visualItemRect(first.child(0)).center())
        assert tree.currentItem() is first and first.isSelected()
        assert all(first.child(i).isSelected() for i in range(first.childCount()))
        assert not second.isSelected() and not any(second.child(i).isSelected() for i in range(second.childCount()))
        QTest.keyClick(tree,Qt.Key_Down)
        assert tree.currentItem() is second and second.isSelected() and not first.isSelected()
        QTest.mouseClick(tree.viewport(),Qt.LeftButton,Qt.ControlModifier,pos=tree.visualItemRect(first.child(1)).center())
        assert tree.currentItem() is first and all(first.child(i).isSelected() for i in range(first.childCount()))
        assert not second.isSelected()
        with patch('perpetual_app.configuration_page.RuleDialog') as dialog:
            dialog.return_value.exec.return_value=0
            QTest.mouseClick(tree.viewport(),Qt.LeftButton,pos=tree.visualItemRect(first.child(0)).center())
            QTest.mouseDClick(tree.viewport(),Qt.LeftButton,pos=tree.visualItemRect(first.child(0)).center())
            assert dialog.call_args.args[1]['id']==controller.state.local.rules[0]['id']
        selected=window.configuration_page.selected_rule()
        updated=controller.state.local.clone(); updated.rules[0]['name']='Local selection check'
        controller.edit_plan(updated)
        assert window.configuration_page.selected_rule()==selected
        controller.edit_plan(controller.state.base.clone()); window.navigate(0); app.processEvents()
        assert not window.dashboard_page.arm_button.isEnabled() and window.dashboard_page.arm_button.property('blocked') and window.sim_start.isEnabled()
        assert window.dashboard_page.arm_button.text()=='ARM' and window.dashboard_page.arm_button.objectName()=='armFlight'
        assert not hasattr(window,'phase_metric') and not hasattr(window,'sensor_metric')
        assert [not check.isHidden() for check in window.dashboard_page.servo_selection]==[True,False,False,False]
        assert window.dashboard_page.check_pyros_button.isEnabled() and 'da verificare' in window.dashboard_page.pyro_states[0].text()


def check_orientation_and_pyros(app,wait):
    """Older firmware without orientation still loads; pyro continuity is separate from preflight."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
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
            assert not window.dashboard_page.servo_selection[0].isHidden() and window.dashboard_page.check_pyros_button.isEnabled()
            assert 'Aggiorna il firmware' in window.settings_page.orientation_panel.saved.text() and not window.settings_page.orientation_panel.save.isEnabled()
        controller.refresh(); wait(lambda:not controller.state.foreground)
        assert controller.state.orientation['supported'] and controller.state.orientation['up_axis']=='+X'
        backend.failed_pyros=1
        preflight=deepcopy(controller.state.preflight); before=backend.commands.count('preflight')
        window.dashboard_page.check_pyros_button.click(); wait(lambda:not controller.state.foreground)
        assert backend.commands.count('preflight')==before and 'pyro continuity' in backend.commands
        assert window.dashboard_page.pyro_states[0].property('preparation')=='unprepared' and controller.state.preflight==preflight
        window.dashboard_page.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert 'CONTINUITY_FAULT' in controller.state.preflight['reasons'] # Preflight still checks independently.
        backend.failed_pyros=0; window.dashboard_page.check_pyros_button.click(); wait(lambda:not controller.state.foreground)
        assert all(state.property('preparation')=='prepared' for state in window.dashboard_page.pyro_states[:2]) and window.dashboard_page.pyro_rows[2].isHidden()
        assert 'CONTINUITY_FAULT' in controller.state.preflight['reasons'] # A manual pass does not approve arming.
        window.dashboard_page.preflight_button.click(); wait(lambda:not controller.state.foreground)


def check_servo_preparation(app,wait):
    """Servo preparation uses applied channels and guides unconfigured servos to the editor."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
        empty_servo=dict(calibrated=False,initial_us=0,final_us=0)
        applied=controller.state.base
        assert [b.text() for b in window.configuration_page.channel_buttons]==['Modifica']
        assert all(window.configuration_page.channels_table.cellWidget(i,3) is None for i in (5,6,7))
        # Missing endpoints lead to the local editor without sending a preparation command.
        unconfigured=applied.clone(); unconfigured.servos[0]=deepcopy(empty_servo)
        controller.state.base=unconfigured; controller.state.local=unconfigured.clone(); window.render()
        window.dashboard_page.servo_selection[0].setChecked(True)
        assert window.dashboard_page.prepare_servos_button.text()=='Vai alla configurazione' and window.dashboard_page.prepare_servos_button.objectName()=='configureServo'
        before=len(backend.commands)
        with patch('perpetual_app.configuration_page.ChannelDialog') as editor:
            editor.return_value.exec.return_value=0
            window.dashboard_page.prepare_servos_button.click()
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
            window.dashboard_page.prepare_servos_button.click(); editor.assert_not_called() # Already edited: guide to Apply instead.
        controller.state.base=applied; controller.state.local=applied.clone(); window.dashboard_page.servo_selection[0].setChecked(False); window.render(); window.navigate(0)
        # Preparation uses applied channels; an unsaved rule edit cannot expose a new servo.
        draft=applied.clone(); draft.rules[0]['outputs'].append(dict(kind='servo',channel=4,delay_ms=0))
        controller.edit_plan(draft)
        assert window.dashboard_page.servo_selection[3].isHidden() and not window.dashboard_page.check_pyros_button.isEnabled()
        assert window.configuration_page.channels_table.cellWidget(7,3) is not None
        controller.edit_plan(applied.clone())
        assert window.configuration_page.channels_table.cellWidget(7,3) is None
        window.dashboard_page.servo_selection[0].setChecked(True)
        without_servo=applied.clone()
        for r in without_servo.rules: r['outputs']=[a for a in r['outputs'] if a['kind']!='servo']
        controller.state.base=without_servo; controller.state.local=without_servo.clone(); window.render()
        assert all(check.isHidden() and not check.isChecked() for check in window.dashboard_page.servo_selection)
        assert not window.dashboard_page.prepare_servos_button.isEnabled() and not window.dashboard_page.servo_empty.isHidden()
        controller.state.base=applied; controller.state.local=applied.clone(); window.render()


def check_sensors_and_power(app,wait):
    """Live sensor details, polling rates and power-save display."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
        main_thread=threading.get_ident()
        assert not window.dashboard_page.imu_label.isHidden() and not window.dashboard_page.baro_label.isHidden()
        assert window.dashboard_page.imu_label.palette().color(window.dashboard_page.imu_label.foregroundRole()).name()=='#c43d3d'
        assert not window.dashboard_page.sensor_refresh.isEnabled()
        sensor_state=controller.state.sensors
        sensor_status={key:controller.state.status.get(key) for key in ('imu_ready','baro_ready')}
        controller.state.status.update(imu_ready=True,baro_ready=True)
        controller.state.sensors=dict(imu_ready=True,imu_sample_valid=True,accel_mps2=[1.25,-2,9.8],gyro_radps=[.1,.2,.3],baro_ready=True,baro_sample_valid=True,pressure_pa=101325,temperature_c=24.5)
        window.render()
        assert window.dashboard_page.imu_label.text().endswith('Disponibile') and window.dashboard_page.baro_label.text().endswith('Disponibile')
        assert window.dashboard_page.imu_label.palette().color(window.dashboard_page.imu_label.foregroundRole()).name()=='#187f73'
        assert window.dashboard_page.sensor_refresh.isEnabled()
        controller.state.sensors['imu_ready']=False; controller.state.status['imu_ready']=False; window.render()
        assert window.dashboard_page.sensor_refresh.isEnabled() # Barometer alone is enough to open details.
        window.dashboard_page.sensor_refresh.click(); details=window._sensor_details
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
        controller._complete(controller._epoch,0,'poll',dict(status=awake_status),None,POLL)
        backend.power_mode='sleeping'; controller._last_status=0; controller.tick(); wait(lambda:not controller._jobs)
        assert controller.state.status['power_mode']=='sleeping' and not controller.state.preflight
        assert not window.dashboard_page.power_badge.isHidden() and window.dashboard_page.power_badge.text()=='POWER SAVE ACTIVE'
        assert window.dashboard_page.readiness_title.text()=='Risparmio energetico' and not window.dashboard_page.arm_button.isEnabled()
        assert window.dashboard_page.imu_label.text()=='Non disponibile' and window.dashboard_page.baro_label.text()=='Disponibile'
        assert controller.state.status['baro_ready'] is False and window.dashboard_page.sensor_refresh.isEnabled()
        assert backend.commands.count('sensors')==before and backend.power_mode=='sleeping'
        window.dashboard_page.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert backend.power_mode=='active' and controller.state.status['power_mode']=='active'
        assert window.dashboard_page.power_badge.isHidden()
        assert window.dashboard_page.baro_label.text()=='Non disponibile' # Wake uses the new probe, including actual failures.
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE']
        controller.timer.start()
        assert window.arming_state.text()=='DISARMED' and window.arming_state.property('state')=='disarmed'
        assert not window.sim_physical.isChecked()
        assert backend.threads and all(t!=main_thread for t in backend.threads)
