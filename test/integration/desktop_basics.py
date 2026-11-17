"""Desktop checks without a connected main window: SIGINT, sensor history, dialogs, config transactions."""
from desktop_support import Native,ROOT,raises
from copy import deepcopy
from unittest.mock import patch
from device_client import CommandError
from perpetual_app.configuration import Plan,read_plan,stage_plan,removal_order,next_rule_id,condition_text,rule_label
from perpetual_app.controller import Controller,error_text
from perpetual_app.dialogs import RuleDialog,ConditionDialog,OutputDialog,ChannelDialog
from perpetual_app.window import MainWindow
from perpetual_app.sensor_details import SensorHistory


def check_sigint(app,wait):
    """A real SIGINT goes through Qt close/cleanup, including repeated interrupts."""
    import signal
    from PySide6.QtTest import QTest
    from perpetual_app.main import terminal_interrupt
    interrupt_controller=Controller(port_scanner=lambda:[]); interrupt_window=MainWindow(interrupt_controller); interrupt_window.show()
    interrupt_controller.state.local=Plan() # Unsaved rules and settings must not open a modal on SIGINT.
    interrupt_controller.edit_setting('machine',(True,42.5))
    assert interrupt_controller.state.has_unsaved_changes
    previous=signal.getsignal(signal.SIGINT)
    try:
        with patch.object(interrupt_controller,'disconnect') as disconnect, patch.object(interrupt_controller,'can_close',return_value=False) as can_close, patch('perpetual_app.window.QMessageBox.question') as question:
            with terminal_interrupt(interrupt_window):
                signal.raise_signal(signal.SIGINT)
                wait(lambda:interrupt_window._closing)
                assert interrupt_window.isVisible() and disconnect.call_count==1
                signal.raise_signal(signal.SIGINT); QTest.qWait(150)
                assert interrupt_window.isVisible() and disconnect.call_count==1
                can_close.return_value=True
                wait(lambda:not interrupt_window.isVisible())
                question.assert_not_called()
        assert signal.getsignal(signal.SIGINT)==previous
    finally:
        interrupt_window._allow_close=True; interrupt_window.close(); app.processEvents()


def check_sensor_history():
    reading=dict(imu_ready=True,imu_sample_valid=True,accel_mps2=[1,2,3],gyro_radps=[.1,.2,.3],baro_ready=True,baro_sample_valid=True,pressure_pa=101325,temperature_c=24.5)
    history=SensorHistory()
    for stamp in range(0,60250,250): assert history.append(dict(reading,uptime_ms=stamp))
    assert history.extent==(0,60) and len(history.samples)==241
    assert history.append(dict(reading,uptime_ms=60250))
    assert history.extent==(.25,60.25) and history.samples[0][0]==250 and len(history.samples)==241
    assert not history.append(dict(reading,uptime_ms=60250)) # No duplicate points for the same response timestamp.
    assert history.append(dict(reading,uptime_ms=60500,imu_sample_valid=False,baro_sample_valid=False))
    assert history.series('pressure_pa',1)[0][-1][1] is None
    assert all(series[-1][1] is None for series in history.series('accel_mps2',3)) # Break curves across invalid readings.
    assert history.append(dict(reading,uptime_ms=130000)) and len(history.samples)==1
    assert not history.append(reading) # Old firmware must not get fabricated local timestamps.
    assert history.append(dict(reading,uptime_ms=100)) and history.extent==(0,0) and len(history.samples)==1 # Reboot.
    history.clear(); history.append(dict(reading,uptime_ms=0xFFFFFFF0)); history.append(dict(reading,uptime_ms=0x20))
    assert history.elapsed_ms==48 and len(history.samples)==2 # uint32 rollover is continuous.
    history.clear()
    for stamp,value in ((0,20),(1000,30),(6000,40)):
        history.append(dict(reading,uptime_ms=stamp,temperature_c=value))
    assert history.temperature_average()==35 # The 20-degree sample is older than five seconds.
    assert history.samples[-1][1]['temperature_c']==(40.0,) # Display smoothing does not change raw data.
    history.append(dict(reading,uptime_ms=6500,temperature_c=float('nan')))
    assert history.temperature_average() is None # Do not reuse an average for an invalid current reading.
    history.append(dict(reading,uptime_ms=7000,temperature_c=50))
    assert history.temperature_average()==45
    history.append(dict(reading,uptime_ms=10,temperature_c=10))
    assert history.temperature_average()==10 # No average survives a board reboot.


def check_dialogs(app,wait):
    """Rule/condition/output/servo dialogs keep IDs internal and live previews bounded."""
    output=OutputDialog(value=dict(kind='pyro',channel=2))
    assert output.result_value()==dict(kind='pyro',channel=2,delay_ms=0)
    output.delay.setValue(150)
    assert output.result_value()['delay_ms']==150
    output.close()
    empty_servo=dict(calibrated=False,initial_us=0,final_us=0)
    servo=ChannelDialog(None,'servo',1,empty_servo,dict(servo_min_us=1000,servo_max_us=2000))
    servo.setStyleSheet((ROOT/'perpetual_app/assets/style.qss').read_text(encoding='utf-8'))
    servo.show(); app.processEvents()
    assert set(servo.pwm_labels)=={'initial_us','final_us'}
    for display in servo.pwm_labels.values():
        assert display.isVisible() and '1500</b>' in display.text()
        assert abs(display.geometry().center().x()-display.parentWidget().rect().center().x())<=1
    servo.sliders['final_us'].setValue(1800); assert '1800</b>' in servo.pwm_labels['final_us'].text()
    servo.sliders['initial_us'].setValue(1100)
    from PySide6.QtWidgets import QPushButton
    decrement=next(b for b in servo.findChildren(QPushButton) if b.text()=='−10')
    decrement.click(); assert '1090</b>' in servo.pwm_labels['initial_us'].text()
    servo.sliders['initial_us'].setValue(1000); decrement.click(); assert servo.sliders['initial_us'].value()==1000
    assert servo.result_value()==dict(calibrated=True,initial_us=1000,final_us=1800)
    assert empty_servo==dict(calibrated=False,initial_us=0,final_us=0)
    servo.reject()
    saved_servo=ChannelDialog(None,'servo',1,dict(calibrated=True,initial_us=1200,final_us=1800),{})
    assert saved_servo.result_value()==dict(calibrated=True,initial_us=1200,final_us=1800)
    saved_servo.reject()
    # One live request in flight; intermediate slider positions are replaced, never queued.
    from types import SimpleNamespace
    from PySide6.QtCore import QObject,Signal
    class Preview(QObject):
        servo_previewed=Signal(object)
        changed=Signal()
        def __init__(self):
            super().__init__(); self.state=SimpleNamespace(editable=True,connected=True); self.sent=[]; self.released=[]
        def preview_servo(self,channel,pwm): self.sent.append((channel,pwm)); return True
        def release_servo(self,channel): self.released.append(channel)
    preview=Preview()
    servo=ChannelDialog(None,'servo',2,empty_servo,{},preview)
    servo.apply_buttons[0].click(); wait(lambda:preview.sent==[(2,1500)])
    servo.live_buttons['final_us'].click()
    assert servo.live_buttons['final_us'].isChecked() and not servo.live_buttons['initial_us'].isEnabled()
    assert servo.sliders['final_us'].isEnabled() and not servo.sliders['initial_us'].isEnabled()
    assert not any(b.isEnabled() for b in servo.apply_buttons+sum(servo.step_buttons.values(),[]))
    for pwm in (1600,1700,1800): servo.sliders['final_us'].setValue(pwm)
    servo._send_position(); assert preview.sent==[(2,1500)] # Still waiting for the board.
    preview.servo_previewed.emit(dict(channel=2,pwm=1500))
    wait(lambda:preview.sent==[(2,1500),(2,1800)])
    preview.servo_previewed.emit(dict(channel=2,pwm=1800))
    assert not servo.feedback.text()
    servo.sliders['final_us'].setValue(1900); servo.live_buttons['final_us'].click()
    servo._send_position(); assert len(preview.sent)==2
    assert all(b.isEnabled() for b in servo.apply_buttons+list(servo.live_buttons.values()))
    servo.apply_buttons[1].click(); wait(lambda:len(preview.sent)==3)
    preview.servo_previewed.emit(dict(error='BUSY'))
    assert servo._live_key is None and servo.feedback.text()=='BUSY'
    servo.live_buttons['initial_us'].click()
    assert servo._live_key=='initial_us' and not servo.live_buttons['final_us'].isEnabled()
    assert servo.pwm_labels['initial_us'].isEnabled() and not servo.pwm_labels['final_us'].isEnabled()
    preview.state.editable=False; preview.state.connected=False; preview.changed.emit()
    assert not any(b.isEnabled() for b in list(servo.live_buttons.values())+servo.apply_buttons)
    servo.reject(); assert preview.released==[2] and not servo.preview_timer.isActive()
    rule=dict(id=0xFFFFFFFF,name='Stable ID',delay_ms=0,conditions=[],outputs=[])
    other=dict(id=7,name='Second rule',delay_ms=0,conditions=[],outputs=[])
    dialog=RuleDialog(None,rule,[rule,other])
    assert not hasattr(dialog,'identity')
    assert hasattr(dialog,'delay') and not hasattr(dialog,'confirmation')
    dialog.delay.setValue(250)
    dialog.name.setText('Renamed'); dialog.finish()
    assert dialog.rule['id']==0xFFFFFFFF and dialog.rule['name']=='Renamed' and dialog.rule['delay_ms']==250; dialog.close()
    for bistage,expected in ((False,{'2','4','6','8','14'}),(True,{'2','4','6','8','14','32','64','34','68','110'})):
        c=ConditionDialog(bistage=bistage)
        assert {c.phase.itemData(i) for i in range(c.phase.count())}==expected
        c.close()
    # Opening an old condition must never silently change its stored phase mask.
    c=ConditionDialog(value=dict(type='phase',phase='110'),bistage=False)
    assert c.result_value()['phase']=='110'; c.close()
    reference=dict(type='since_rule_ms',reference=0xFFFFFFFF,elapsed_ms=1)
    c=ConditionDialog(value=reference,rules=[rule,other],exclude_id=other['id'])
    assert not c.reference.isEditable() and c.reference.findData(other['id'])<0
    assert c.reference.currentText()=='Stable ID' and c.result_value()['reference']==0xFFFFFFFF
    assert '4294967295' not in condition_text(reference,[rule,other]); c.close()
    c=ConditionDialog(value=reference,rules=[other])
    assert c.reference.currentData() is None
    raises(ValueError,c.result_value); c.close()
    assert next_rule_id(Plan(rules=[rule,other]))==1
    # Removed saved IDs and dangling references cannot be silently rebound to a new rule.
    local=Plan(rules=[dict(other,conditions=[dict(reference,reference=8)])])
    assert next_rule_id(local,Plan(rules=[dict(other,id=9)]))==10
    duplicates=[dict(rule,name='Same'),dict(other,name='Same')]
    assert rule_label(duplicates,rule['id'])!=rule_label(duplicates,other['id'])
    error=CommandError('config validate',dict(code='CONFIG_INVALID',validation='InvalidPyroSettings',rule_id=0,index=1,field=4))
    assert 'PYRO 2' in error_text(error) and 'rule 0' not in error_text(error)
    error=CommandError('config validate',dict(code='CONFIG_INVALID',validation='InvalidCondition',rule_id=rule['id'],index=0,field=2))
    assert 'Stable ID' in error_text(error,Plan(rules=[rule])) and '4294967295' not in error_text(error,Plan(rules=[rule]))


def check_config_transactions(app,wait):
    """Native firmware configuration transactions, conflicts and invisible airbrake settings."""
    backend=Native(); backend.state='connected'
    try:
        base,meta,view=read_plan(backend)
        assert view=='applied' and meta['available'] and len(base.rules)==2
        local=base.clone(); local.servos[0]['final_us']=1500
        local.rules[0]['outputs'].append(dict(kind='pyro',channel=3,delay_ms=125)) # No manual pyro settings required.
        local.rules[0]['outputs'].append(dict(kind='servo',channel=2,delay_ms=0)) # Rules can persist before servo setup.
        local.rules[0]['name']='New "label" \\ x'
        local.rules[0]['conditions'].append(dict(type='since_rule_ms',reference=20,elapsed_ms=50))
        stage_plan(backend,base,local)
        staged,_,view=read_plan(backend); assert staged==local and view=='draft'
        saved=backend.request('config apply'); assert all(saved[k] for k in ('applied','persisted','verified'))
        current,_,view=read_plan(backend); assert current==local and view=='applied'
        assert removal_order(current).index(10)<removal_order(current).index(20)
        error=raises(RuntimeError,lambda:stage_plan(backend,base,base.clone()))
        assert 'changed' in str(error) and not backend.request('config draft')['available']
        bad=current.clone(); bad.rules[0]['name']='bad\ncommand'
        start=len(backend.commands); raises(ValueError,lambda:stage_plan(backend,current,bad))
        assert len(backend.commands)==start # Local invalid input never creates a remote draft.
        duplicate=current.clone()
        duplicate.rules[1]['outputs'].append(deepcopy(duplicate.rules[0]['outputs'][0]))
        start=len(backend.commands); error=raises(ValueError,lambda:stage_plan(backend,current,duplicate))
        assert 'PYRO 1' in str(error) and current.rules[0]['name'] in str(error)
        assert len(backend.commands)==start # Duplicate ownership is explained before any board mutation.
        empty=current.clone(); empty.rules=[]
        error=raises(CommandError,lambda:stage_plan(backend,current,empty))
        assert error.code=='CONFIG_INVALID' and backend.request('config show')['available']
        stage_plan(backend,current,current.clone()) # Repairs an own partial/invalid draft safely.
        backend.request('config discard')
        backend.request('config begin')
        class Foreign:
            def request(self,command,**kwargs):
                if command=='control status': return dict(owner='bluetooth',transport='usb')
                return backend.request(command,**kwargs)
        start=len(backend.commands)
        raises(RuntimeError,lambda:stage_plan(Foreign(),current,current.clone()))
        assert 'config discard' not in backend.commands[start:] and backend.request('config draft')['available']
        backend.request('config discard')
        # Invisible airbrake settings survive an ordinary app configuration edit.
        air=current.clone(); air.servos[2]=dict(calibrated=True,initial_us=1100,final_us=1900)
        air.airbrakes=dict(channel_mask=4,conditions=[dict(type='phase',phase='68'),dict(type='since_rule_ms',reference=20,elapsed_ms=0)])
        stage_plan(backend,current,air); backend.request('config apply')
        frozen,_,_=read_plan(backend); assert frozen==air
        edited=frozen.clone(); edited.rules[0]['name']='Another edit'
        stage_plan(backend,frozen,edited); backend.request('config apply')
        assert read_plan(backend)[0].airbrakes==air.airbrakes
    finally: backend.close()
