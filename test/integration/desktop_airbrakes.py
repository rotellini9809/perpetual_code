"""Airbrake cases in the existing desktop suite, using its native firmware fixture."""
import csv
import json
from copy import deepcopy
from pathlib import Path
import tempfile
from PySide6.QtWidgets import QLabel,QProgressBar
from perpetual_app.airbrake_dialog import AirbrakeDialog
from perpetual_app.airbrakes import control_description
from perpetual_app.controller import Controller
from perpetual_app.dialogs import ChannelDialog
from perpetual_app.window import MainWindow


def check_airbrakes(app,wait,native):
    backend=native(); controller=Controller(lambda _:backend,lambda:[]); controller.timer.stop()
    window=MainWindow(controller); window.show(); app.processEvents()
    try:
        controller.connect_port('AIRBRAKES_FIXTURE'); wait(lambda:controller.state.base is not None and not controller.state.foreground)
        s=controller.state; page=window.settings_page; config=window.configuration_page
        base=s.base.clone()
        assert not page.airbrakes_enabled.isChecked() and page.airbrakes_save.text()=='Salvato'
        assert config.airbrake_button.isHidden()
        warning=page.airbrakes_enabled.parentWidget().findChild(QLabel,'controlWarning').text()
        assert 'implementare' in warning and '0%' in warning
        page.airbrakes_enabled.setChecked(True); assert page.airbrakes_save.isEnabled()
        assert config.airbrake_button.isHidden() # Pending edits do not enable the device setting.
        page.airbrakes_save.click(); wait(lambda:not s.foreground)
        assert s.airbrakes['enabled'] and backend.request('config airbrakes')['enabled']
        assert not config.airbrake_button.isHidden()
        assert page.airbrakes_save.text()=='Salvato' and not page.airbrakes_save.isEnabled()
        blocked=AirbrakeDialog(window,base)
        assert not blocked.channels[0].isEnabled() and 'assegnato' in blocked.channels[0].text()
        blocked.reject()
        plan=base.clone()
        for r in plan.rules: r['outputs']=[a for a in r['outputs'] if not (a['kind']=='servo' and a['channel']==1)]
        plan.airbrakes=dict(channel_mask=1,conditions=[dict(type='phase',phase='68')])
        controller.edit_plan(plan)
        assert not page.airbrakes_enabled.isEnabled() # Device settings wait for the plan transaction.
        assert config.channels_table.item(4,2).text()=='Aerofreni'
        assert 'Chiuso 0%' in config.channels_table.item(4,1).text()
        item=config.rules_tree.topLevelItem(0); assert item.text(0)=='Aerofreni' and 'continuo' in item.text(1)
        editor=AirbrakeDialog(window,plan)
        assert editor.channels[0].isChecked() and editor.channels[0].isEnabled() and not hasattr(editor,'delay')
        editor.reject()
        calibration=ChannelDialog(window,'servo',1,plan.servos[0],s.capabilities,airbrake=True)
        captions=[w.text() for w in calibration.findChildren(QLabel)]
        assert any('Chiuso' in text and '0%' in text for text in captions)
        assert any('Aperto' in text and '100%' in text for text in captions)
        calibration.reject()
        controller.validate(apply=True); wait(lambda:not s.foreground)
        assert not s.error and not s.dirty and s.base.airbrakes['channel_mask']==1
        config.rules_tree.setCurrentItem(config.rules_tree.topLevelItem(0)); config.remove_rule()
        assert s.local.airbrakes['channel_mask']==0 and s.dirty
        controller.edit_plan(s.base.clone())
        assert window.dashboard_page.servo_selection[0].isVisible() and 'Aerofreni' in window.dashboard_page.servo_selection[0].text()
        assert window.dashboard_page.airbrake_summary.isHidden() and not window.dashboard_page.airbrake_summary.text()
        # No live control state on the Dashboard, even during an armed run.
        s.airbrake_status=dict(active=True,channel_mask=1,flags=3,failed_conditions=0,opening_pct=[37.5,None,None,None])
        s.status['arming_state']='armed'; window.render()
        assert window.dashboard_page.airbrake_summary.isHidden() and not window.dashboard_page.airbrake_summary.text()
        assert not window.dashboard_page.prepare_servos_button.parentWidget().findChildren(QProgressBar)
        s.status['arming_state']='disarmed'; s.airbrake_status['active']=False; window.render()
        controller.save_airbrakes(False); wait(lambda:not s.foreground)
        assert not s.error and not s.airbrakes['enabled'] and s.base.airbrakes['channel_mask']==1
        assert config.airbrake_button.isHidden()
        assert not window.dashboard_page.servo_selection[0].isEnabled() and window.dashboard_page.servo_states[0].text()=='disabilitato'
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'airbrakes.csv'
            with path.open('w',newline='',encoding='utf-8') as stream:
                writer=csv.writer(stream)
                writer.writerow(['t_us','z_est','vz_est','state',*[f'airbrake_{i}_pct' for i in range(1,5)],'airbrake_flags','airbrake_failed_conditions','airbrake_channel_mask'])
                for time,opening,flags,failed,phase in ((0,0,5,1,0),(6000,0,261,1,2),(6035,0,5,1,2),(8000,25,3,0,6),(10000,70,3,0,6),(15000,0,5,1,3),(20000,0,17,0,7)):
                    writer.writerow([time*1000,30,10,phase,opening,'','','',flags,failed,1])
            path.with_suffix('.json').write_text(json.dumps(dict(sensor_mode=1,configuration=dict(rules=[],airbrakes=dict(channel_mask=1,conditions=[dict(kind=0,phase_mask=64)])),calibration=dict(bistage=True,airbrakes_enabled=True,airbrake_law_version=1))),encoding='utf-8')
            controller.open_flight(path); wait(lambda:not s.foreground)
            assert not s.error
            flights=window.recorded_flights_page; window.navigate(3); app.processEvents()
            assert not flights.airbrake_card.isHidden() and flights.airbrake_conditions.rowCount()==1
            assert flights.layout().indexOf(flights.airbrake_card)==flights.layout().indexOf(flights.flight_motion.parentWidget())+1
            assert flights.layout().indexOf(flights.airbrake_card)<flights.layout().indexOf(flights.flight_rates.parentWidget())
            assert flights.flight_airbrakes.fixed_range==(0,100) and len(flights.flight_airbrakes.curves)==1
            assert flights.flight_airbrakes.rows[4][1]==70
            assert flights.airbrake_conditions.columnCount()==1 and flights.airbrake_conditions.item(0,0).text()=='Fase: COAST 2'
            assert len(s.flight['airbrakes']['transitions'])==6 # Raw history still drives the plot's control intervals.
            assert control_description(9)[0]=='Chiusura di sicurezza' and 'Kalman' in control_description(9)[1]
            assert control_description(257)[0]=='Inibito' # Unknown context must not be described as the bistage velocity check.
            recorded=s.flight
            disabled=deepcopy(recorded); disabled['metadata']['calibration']['airbrakes_enabled']=False
            controller._complete(controller._epoch,0,'Apertura volo locale',dict(flight=disabled),None)
            assert disabled['airbrakes']['channel_mask']==1 and flights.airbrake_card.isHidden()
            unspecified=deepcopy(recorded); del unspecified['metadata']['calibration']['airbrakes_enabled']
            controller._complete(controller._epoch,0,'Apertura volo locale',dict(flight=unspecified),None)
            assert flights.airbrake_card.isHidden()
            controller._complete(controller._epoch,0,'Apertura volo locale',dict(flight=recorded),None)
            assert not flights.airbrake_card.isHidden() and not s.airbrakes['enabled'] # Use this flight's saved setting, not the connected board's current setting.
            out=Path('.pio/ui-airbrakes'); out.mkdir(parents=True,exist_ok=True)
            flights.airbrake_card.grab().save(str(out/'recorded.png'))
        window.navigate(4); app.processEvents(); page.airbrakes_enabled.parentWidget().grab().save(str(out/'settings.png'))
        window.navigate(1); app.processEvents(); config.grab().save(str(out/'configuration.png'))
        window.navigate(0); app.processEvents(); window.grab().save(str(out/'dashboard.png'))
        assert backend.request('config airbrakes')['enabled'] is False
        backend.request('config reset'); assert backend.request('config airbrakes')['enabled'] is False
    finally:
        controller.disconnect(); wait(controller.can_close)
        window._allow_close=True; window.close(); app.processEvents(); controller.shutdown()
