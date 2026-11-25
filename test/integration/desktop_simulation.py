"""Simulation, arming and configuration round trip against a fresh PC firmware fixture."""
from desktop_support import session
from copy import deepcopy
import threading
import time
from unittest.mock import patch
from device_client import CommandError


def check_simulation_and_arming(app,wait):
    """Pending/acknowledged simulation and arming state, physical outputs and firmware refusals."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
        # No UI state is changed to Armed while a request is merely pending.
        gate=threading.Event(); backend.gates['sim begin']=gate
        from PySide6.QtTest import QTest
        from PySide6.QtCore import Qt
        previous_size=window.size(); window.resize(1120,730); window.navigate(2); app.processEvents()
        assert window.sim_record.x()<window.sim_physical.x() and window.sim_preflight_notice.isHidden()
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
        assert not window.dashboard_page.check_pyros_button.isEnabled() and not window.dashboard_page.prepare_servos_button.isEnabled()
        assert window.arming_state.text()=='ARMED' and window.arming_state.property('state')=='armed'
        controller.command('sim stop'); wait(lambda:not controller.state.foreground)
        assert not controller.state.sim['active'] and controller.state.status['arming_state']=='disarmed'
        window.resize(previous_size)
        controller.command('preflight simulation'); wait(lambda:not controller.state.foreground)
        assert not controller.state.preflight['ready'] and not window.dashboard_page.arm_button.isEnabled()
        window.dashboard_page.servo_selection[0].setChecked(True)
        assert window.dashboard_page.prepare_servos_button.isEnabled()
        window.dashboard_page.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        assert 'servo arm 1' in backend.commands and window.dashboard_page.servo_states[0].text()=='preparato'
        assert not window.dashboard_page.prepare_servos_button.isEnabled() and window.dashboard_page.prepare_servos_button.objectName()=='primary'
        assert window.dashboard_page.release_servos_button.isEnabled() and window.dashboard_page.release_servos_button.objectName()=='releaseServo'
        before=len(backend.commands); window.dashboard_page.prepare_servos_button.click(); assert len(backend.commands)==before
        assert window.dashboard_page.release_servos_button.isEnabled() and not window.dashboard_page.arm_button.isEnabled() # IMU readiness still comes from the board.
        # Status can report held PWM before a fresh preflight mask is available.
        controller.state.preflight={}; window.render()
        assert controller.state.status['arming_state']=='disarmed' and controller.state.status['outputs_active']
        assert window.dashboard_page.release_servos_button.isEnabled() and window.dashboard_page.arm_button.text()=='ARM'
        window.dashboard_page.release_servos_button.click(); wait(lambda:not controller.state.foreground)
        assert not window.dashboard_page.release_servos_button.isEnabled()
        assert window.dashboard_page.prepare_servos_button.isEnabled()
        assert not backend.prepared_mask and window.dashboard_page.servo_states[0].text()=='non preparato' and window.dashboard_page.servo_states[0].property('preparation')=='unprepared'
        window.dashboard_page.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        # Held servo PWM is not an armed run: all idle settings stay available.
        assert window.settings_page.factory_reset_button.isEnabled() and window.settings_page.format_memory_button.isEnabled()
        assert window.configuration_page.add_rule_button.isEnabled() and window.dashboard_page.check_pyros_button.isEnabled()
        window.settings_page.orientation_panel.axis.setCurrentIndex(window.settings_page.orientation_panel.axis.findData('-X')); window.render()
        assert window.settings_page.orientation_panel.axis.isEnabled() and window.settings_page.orientation_panel.save.isEnabled()
        assert backend.prepared_mask
        window.settings_page.orientation_panel.axis.setCurrentIndex(window.settings_page.orientation_panel.axis.findData('+X'))
        # A rejected run is not a configuration draft and must not latch Start disabled.
        controller._complete(controller._epoch,997,'Avvio simulazione',None,CommandError('sim begin',dict(code='CONFIG_INVALID',ok=False)))
        assert not controller.state.staged
        for physical in (True,False):
            window.sim_physical.setChecked(physical); window.render()
            assert backend.prepared_mask and controller.state.status['outputs_active']
            assert not controller.simulation_blocker() and window.sim_start.isEnabled()
            assert window.sim_preflight_notice.isHidden() # Selecting physical outputs alone never shows the recovery action.
        controller.state.staged=True; window.render()
        assert not window.sim_start.isEnabled() and 'bozza' in window.sim_blocker.text()
        controller.state.staged=False; window.render()
        # Prepared outputs no longer block virtual simulation; acknowledge the firmware release in Dashboard.
        assert backend.prepared_mask and window.sim_start.isEnabled()
        window.sim_physical.setChecked(False)
        original_request=backend.request
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
        assert not backend.prepared_mask and window.dashboard_page.servo_states[0].text()=='non preparato'
        window.sim_stop.click(); wait(lambda:not controller.state.foreground)
        window.sim_record.setChecked(False)
        window.dashboard_page.prepare_servos_button.click(); wait(lambda:not controller.state.foreground)
        backend.ready_to_arm=True; window.dashboard_page.preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert window.dashboard_page.arm_button.isEnabled() and not window.dashboard_page.arm_button.property('blocked')
        physical_preflight=deepcopy(controller.state.preflight); requests=len(backend.commands)
        for checked in (True,False):
            window.sim_physical.setChecked(checked); window.render()
            assert window.dashboard_page.arm_button.isEnabled() and controller.state.preflight==physical_preflight
        assert len(backend.commands)==requests # Simulation selection never changes real-flight checks or sends commands.
        previous_page=window.pages.currentIndex(); window.navigate(0); window.activateWindow(); app.processEvents()
        QTest.mousePress(window.dashboard_page.arm_button,Qt.LeftButton); QTest.qWait(1580); QTest.mouseRelease(window.dashboard_page.arm_button,Qt.LeftButton)
        wait(lambda:not controller.state.foreground); window.navigate(previous_page)
        assert 'IMU' in controller.state.error and controller.state.status['arming_state']=='disarmed'
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE']
        assert not window.dashboard_page.arm_button.isEnabled() and window.dashboard_page.arm_button.property('blocked')
        backend.ready_to_arm=False
        # Physical selection only changes the request. Firmware refusal stays authoritative.
        backend.reject_physical=True; window.sim_physical.setChecked(True)
        assert window.sim_preflight_notice.isHidden()
        window.sim_start.click()
        wait(lambda:not controller.state.foreground)
        assert 'sim begin physical' in backend.commands and not controller.state.sim['active']
        assert controller.state.preflight['reasons']==['IMU_UNAVAILABLE'] and 'continuity' in controller.state.error
        assert controller.state.sim_preflight['reasons']==['CONTINUITY_FAULT'] and not window.sim_preflight_notice.isHidden()
        before=backend.commands.count('preflight'); starts=backend.commands.count('sim begin physical')
        window.sim_preflight_button.click(); wait(lambda:not controller.state.foreground)
        assert window.pages.currentIndex()==0 and controller.page==0 and backend.commands.count('preflight')==before+1
        assert backend.commands.count('sim begin physical')==starts # Reviewing checks never starts or arms anything.
        window.navigate(2); window.sim_physical.setChecked(False)
        assert window.sim_preflight_notice.isHidden() and not controller.state.sim_preflight
        window.sim_physical.setChecked(True); assert window.sim_preflight_notice.isHidden()
        window.sim_start.click(); wait(lambda:not controller.state.foreground)
        assert not window.sim_preflight_notice.isHidden()
        backend.reject_physical=False; gate=threading.Event(); backend.gates['sim begin physical']=gate
        starts=backend.commands.count('sim begin physical')
        window.sim_start.click(); wait(lambda:backend.commands.count('sim begin physical')==starts+1)
        assert window.sim_preflight_notice.isHidden() and not controller.state.sim_preflight
        assert not window.sim_physical.isEnabled() and controller.state.status['arming_state']=='disarmed'
        gate.set(); wait(lambda:controller.state.sim.get('active') and not controller.state.foreground)
        assert controller.state.sim['output_mode']=='PHYSICAL' and window.arming_state.text()=='ARMED'
        assert window.sim_physical.isChecked() and not window.sim_physical.isEnabled()
        assert window.dashboard_page.arm_button.text()=='DISARM' and window.dashboard_page.arm_button.isEnabled()
        assert not window.dashboard_page.release_servos_button.isEnabled()
        window.navigate(0); app.processEvents()
        window.dashboard_page.arm_button.click(); assert 'disarm' not in backend.commands[-1:]
        QTest.mousePress(window.dashboard_page.arm_button,Qt.LeftButton); QTest.qWait(1580); QTest.mouseRelease(window.dashboard_page.arm_button,Qt.LeftButton)
        wait(lambda:not controller.state.foreground)
        assert 'disarm' in backend.commands and window.dashboard_page.arm_button.text()=='ARM'
        assert not controller.state.sim['active'] and window.sim_physical.isEnabled()
        window.sim_physical.setChecked(False)


def check_config_roundtrip(app,wait):
    """Local edit, firmware RAM validation, save/readback and new applied baseline."""
    with session(app,wait) as s:
        controller,window,backend=s.controller,s.window,s.backend
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
