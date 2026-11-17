"""Draft editors; servo positioning uses explicit, separate hardware commands."""
from copy import deepcopy
from PySide6.QtCore import Qt,QTimer
from PySide6.QtWidgets import (QDialog,QVBoxLayout,QGridLayout,QFormLayout,QDialogButtonBox,QComboBox,QDoubleSpinBox,
    QSpinBox,QLineEdit,QMessageBox,QTableWidgetItem,QSlider,QWidget)
from .configuration import CONDITION_TYPES, condition_text, rule_label
from .widgets import button,row,table,label,card
from .i18n import t


def spin(value=0,maximum=3600000,minimum=0,suffix=''):
    milliseconds=suffix.strip()=='ms'
    widget=QSpinBox(); widget.setRange(minimum,maximum); widget.setValue(value); widget.setSuffix('' if milliseconds else suffix)
    widget.setProperty('milliseconds',milliseconds)
    if milliseconds: widget.setFixedWidth(160)
    return widget


def unit_field(widget,unit):
    container=QWidget(); layout=row(widget,unit,None); layout.setContentsMargins(0,0,0,0)
    container.setLayout(layout); return container


def time_field(widget): return unit_field(widget,label('ms','hint'))


def buttons(dialog,layout,accept=None):
    box=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
    box.button(QDialogButtonBox.StandardButton.Ok).setText(t('Confirm'))
    box.button(QDialogButtonBox.StandardButton.Cancel).setText(t('Cancel'))
    box.accepted.connect(accept or dialog.accept); box.rejected.connect(dialog.reject); layout.addWidget(box)


class ConditionDialog(QDialog):
    def __init__(self,parent=None,value=None,*,rules=(),exclude_id=None,bistage=False):
        super().__init__(parent); self.setWindowTitle(t('Rule condition')); self.setMinimumWidth(430)
        layout=QVBoxLayout(self); layout.setSpacing(15); form=QFormLayout(); form.setSpacing(12)
        self.kind=QComboBox()
        names=[t('Flight phase'),t('Apogee detected'),t('Altitude'),t('Altitude range'),t('Absolute vertical speed'),t('Speed range'),
               t('Time since launch'),t('Time since apogee'),t('Time since a rule')]
        for name,kind in zip(names,CONDITION_TYPES): self.kind.addItem(name,kind)
        self.phase=QComboBox()
        phases=[('BOOST / COAST','6'),('DESCENT','8'),(t('All in-flight phases'),'14'),('BOOST','2'),('COAST','4')]
        if bistage:
            phases=[(t('BOOST 1 / COAST 1 (or normal)'),'6'),('DESCENT','8'),(t('All in-flight phases'),'110'),(t('BOOST 1 (or normal)'),'2'),(t('COAST 1 (or normal)'),'4'),('BOOST 2','32'),('COAST 2','64'),(t('BOOST: both stages'),'34'),(t('COAST: both stages'),'68'),(t('First stage and descent'),'14')]
        for name,mask in phases: self.phase.addItem(name,mask)
        self.compare=QComboBox(); self.compare.addItems(['<','>'])
        self.value=QDoubleSpinBox(); self.upper=QDoubleSpinBox()
        for widget in (self.value,self.upper): widget.setRange(-1000000,1000000); widget.setDecimals(3)
        self.value_unit=label('m','hint'); self.upper_unit=label('m','hint')
        self.value_row=unit_field(self.value,self.value_unit); self.upper_row=unit_field(self.upper,self.upper_unit)
        self.elapsed=spin(suffix=' ms'); self.reference=QComboBox()
        self.elapsed_row=time_field(self.elapsed)
        self.reference.addItem(t('Select a rule'),None)
        for rule in rules:
            if rule['id']!=exclude_id: self.reference.addItem(rule_label(rules,rule['id']),rule['id'])
        form.addRow(t('Type'),self.kind); form.addRow(t('Phase'),self.phase); form.addRow(t('Comparison'),self.compare)
        form.addRow(t('Value / minimum'),self.value_row); form.addRow(t('Maximum'),self.upper_row); form.addRow(t('Minimum time'),self.elapsed_row); form.addRow(t('Reference rule'),self.reference)
        self.form=form; layout.addLayout(form)
        hint=label(t('All conditions of the same rule must be true.'),'hint'); hint.setWordWrap(True); layout.addWidget(hint)
        self.kind.currentIndexChanged.connect(self.update_fields)
        if value:
            self.kind.setCurrentIndex(CONDITION_TYPES.index(value['type']))
        self.update_fields()
        if value:
            self.value.setValue(value.get('value',0)); self.upper.setValue(value.get('upper',0)); self.elapsed.setValue(value.get('elapsed_ms',0))
            if value['type']=='since_rule_ms':
                index=self.reference.findData(value.get('reference'))
                if index<0: self.reference.setItemText(0,t('Rule not available: select another one'))
                self.reference.setCurrentIndex(max(index,0))
            index=self.phase.findData(value.get('phase','8'))
            if index<0:
                self.phase.addItem(t('Mask {mask}',mask=value['phase']),value['phase']); index=self.phase.count()-1
            self.phase.setCurrentIndex(index)
            self.compare.setCurrentText(value.get('comparison','<'))
        buttons(self,layout,self.finish)

    def finish(self):
        if self.kind.currentData()=='since_rule_ms' and self.reference.currentData() is None:
            QMessageBox.information(self,t('Reference rule'),t('Select another existing rule.')); return
        self.accept()

    def update_fields(self):
        kind=self.kind.currentData()
        self.compare.clear(); self.compare.addItems(['<=','>='] if kind=='speed' else ['<','>'])
        for widget,visible in ((self.phase,kind=='phase'),(self.compare,kind in ('speed','altitude')),
                (self.value_row,kind in ('altitude','speed','altitude_range','speed_range')),
                (self.upper_row,kind in ('altitude_range','speed_range')),(self.elapsed_row,kind.startswith('since_')),(self.reference,kind=='since_rule_ms')):
            widget.setVisible(visible); self.form.labelForField(widget).setVisible(visible)
        for unit in (self.value_unit,self.upper_unit): unit.setText('m/s' if 'speed' in kind else 'm')

    def result_value(self):
        kind=self.kind.currentData(); c=dict(type=kind)
        if kind=='phase': c['phase']=self.phase.currentData()
        elif kind in ('altitude','speed'): c.update(comparison=self.compare.currentText(),value=self.value.value())
        elif kind in ('altitude_range','speed_range'): c.update(value=self.value.value(),upper=self.upper.value())
        elif kind.startswith('since_'):
            c['elapsed_ms']=self.elapsed.value()
            if kind=='since_rule_ms':
                if self.reference.currentData() is None: raise ValueError('Select an existing reference rule.')
                c['reference']=self.reference.currentData()
        return c


class OutputDialog(QDialog):
    def __init__(self,parent=None,value=None):
        super().__init__(parent); self.setWindowTitle(t('Rule output')); self.setMinimumWidth(350)
        layout=QVBoxLayout(self); form=QFormLayout()
        self.kind=QComboBox(); self.kind.addItem('Pyro','pyro'); self.kind.addItem('Servo','servo')
        self.channel=spin(1,4,1); self.delay=spin(suffix=' ms')
        self.delay.setToolTip(t('Extra wait after the rule delay. It does not change how long the output is driven.'))
        form.addRow(t('Type'),self.kind); form.addRow(t('Channel'),self.channel); form.addRow(t('Channel delay'),time_field(self.delay))
        layout.addLayout(form)
        if value:
            self.kind.setCurrentIndex(self.kind.findData(value['kind'])); self.channel.setValue(value['channel'])
            self.delay.setValue(value.get('delay_ms',0))
        buttons(self,layout)

    def result_value(self): return dict(kind=self.kind.currentData(),channel=self.channel.value(),delay_ms=self.delay.value())


class RuleDialog(QDialog):
    def __init__(self,parent,rule,rules,*,bistage=False):
        super().__init__(parent); self.setWindowTitle(t('Deployment rule')); self.resize(690,650)
        self.rule=deepcopy(rule); self._identity=rule['id']; self._rules=deepcopy(rules); self.bistage=bistage
        layout=QVBoxLayout(self); layout.setSpacing(14)
        form=QFormLayout()
        self.name=QLineEdit(rule['name']); self.name.setMaxLength(47); self.delay=spin(rule['delay_ms'],suffix=' ms')
        self.delay.setToolTip(t('Wait after all conditions are satisfied. Each output then adds its own delay.'))
        form.addRow(t('Name'),self.name)
        layout.addLayout(form)
        layout.addLayout(row(label(t('CONDITIONS · AND'),'eyebrow'),None,button(t('+ Condition'),self.add_condition)))
        self.conditions=table([t('Condition'),t('Parameter')]); self.conditions.setMinimumHeight(140); self.conditions.itemDoubleClicked.connect(lambda _:self.edit_condition())
        layout.addWidget(self.conditions); layout.addLayout(row(button(t('Edit'),self.edit_condition),button(t('Remove'),self.remove_condition),None))
        delay_form=QFormLayout(); delay_form.addRow(t('Rule delay'),time_field(self.delay)); layout.addLayout(delay_form)
        layout.addLayout(row(label(t('OUTPUTS'),'eyebrow'),None,button(t('+ Output'),self.add_output)))
        self.outputs=table([t('Channel'),t('Channel delay')]); self.outputs.setMinimumHeight(125); self.outputs.itemDoubleClicked.connect(lambda _:self.edit_output())
        layout.addWidget(self.outputs); layout.addLayout(row(button(t('Edit'),self.edit_output),button(t('Remove'),self.remove_output),None))
        buttons(self,layout,self.finish); self.render()

    def render(self):
        self.conditions.setRowCount(len(self.rule['conditions']))
        for i,c in enumerate(self.rule['conditions']):
            self.conditions.setItem(i,0,QTableWidgetItem(f'{i+1:02d} · {c["type"]}')); self.conditions.setItem(i,1,QTableWidgetItem(condition_text(c,self._rules)))
        self.outputs.setRowCount(len(self.rule['outputs']))
        for i,a in enumerate(self.rule['outputs']):
            self.outputs.setItem(i,0,QTableWidgetItem(f'{a["kind"].upper()} {a["channel"]}'))
            self.outputs.setItem(i,1,QTableWidgetItem(f'{a.get("delay_ms",0)} ms'))

    def add_condition(self):
        if len(self.rule['conditions'])>=8: return
        dialog=ConditionDialog(self,rules=self._rules,exclude_id=self._identity,bistage=self.bistage)
        if dialog.exec(): self.rule['conditions'].append(dialog.result_value()); self.render()

    def edit_condition(self):
        i=self.conditions.currentRow()
        if i<0: return
        dialog=ConditionDialog(self,self.rule['conditions'][i],rules=self._rules,exclude_id=self._identity,bistage=self.bistage)
        if dialog.exec(): self.rule['conditions'][i]=dialog.result_value(); self.render()

    def remove_condition(self):
        i=self.conditions.currentRow()
        if i>=0: del self.rule['conditions'][i]; self.render()

    def add_output(self):
        if len(self.rule['outputs'])>=8: return
        dialog=OutputDialog(self)
        if dialog.exec(): self.rule['outputs'].append(dialog.result_value()); self.render()

    def edit_output(self):
        i=self.outputs.currentRow()
        if i<0: return
        dialog=OutputDialog(self,self.rule['outputs'][i])
        if dialog.exec(): self.rule['outputs'][i]=dialog.result_value(); self.render()

    def remove_output(self):
        i=self.outputs.currentRow()
        if i>=0: del self.rule['outputs'][i]; self.render()

    def finish(self):
        if not self.name.text().strip() or not self.name.text().isascii():
            QMessageBox.information(self,t('Rule name'),t('Enter a non-empty ASCII name.')); return
        self.rule.update(id=self._identity,name=self.name.text(),delay_ms=self.delay.value()); self.accept()


class ChannelDialog(QDialog):
    def __init__(self,parent,kind,channel,value,limits,controller=None,*,airbrake=False):
        super().__init__(parent); self.setWindowTitle(f'{kind.upper()} {channel} · '+t('Configuration')); self.setMinimumWidth(560)
        self.kind=kind; layout=QVBoxLayout(self); layout.setSpacing(16); self.pwm_labels={}; self.sliders={}
        self.controller=controller; self.channel=channel; self._pending=None; self._sending=False; self._tested=False
        self.live_buttons={}; self.step_buttons={}; self._live_key=None
        self.feedback=label('','hint'); self.feedback.hide(); self.apply_buttons=[]
        self.preview_timer=QTimer(self); self.preview_timer.setInterval(100); self.preview_timer.timeout.connect(self._send_position)
        minimum=limits.get('servo_min_us',1000); maximum=limits.get('servo_max_us',2000)
        layout.addWidget(label(t('SERVO {channel} · PWM POSITIONS',channel=channel),'eyebrow'))
        for key,title in [('initial_us',t('Closed · 0%') if airbrake else t('Initial position')),('final_us',t('Open · 100%') if airbrake else t('Final position'))]:
            section=QVBoxLayout(); section.setSpacing(7); section.addWidget(label(title,'cardTitle'))
            frame,body=card(); section.addWidget(frame); layout.addLayout(section)
            display=label('','servoPwm'); display.setAlignment(Qt.AlignCenter); self.pwm_labels[key]=display
            live=button('Live',kind='servoLive'); live.setCheckable(True); self.live_buttons[key]=live
            live.toggled.connect(lambda enabled,key=key:self._live_changed(key,enabled))
            apply=button(t('Apply'),lambda key=key:self._queue_position(key))
            self.apply_buttons.append(apply)
            position=QGridLayout(); position.setSpacing(12); position.setColumnStretch(1,1)
            edge=max(live.sizeHint().width(),apply.sizeHint().width())
            for column in (0,2): position.setColumnMinimumWidth(column,edge)
            position.addWidget(live,0,0,Qt.AlignLeft); position.addWidget(display,0,1); position.addWidget(apply,0,2,Qt.AlignRight)
            body.addLayout(position)
            slider=QSlider(Qt.Horizontal); slider.setRange(minimum,maximum); slider.setValue(value[key] or (minimum+maximum)//2)
            slider.setSingleStep(1); slider.setPageStep(10); self.sliders[key]=slider
            slider.valueChanged.connect(lambda pwm,key=key:self._position_changed(key,pwm)); self._position_changed(key,slider.value())
            steps=[button(text,lambda delta=delta,slider=slider:slider.setValue(slider.value()+delta)) for text,delta in [('−10',-10),('−1',-1),('+1',1),('+10',10)]]
            self.step_buttons[key]=steps
            body.addLayout(row(steps[0],steps[1],slider,steps[2],steps[3]))
        layout.addWidget(self.feedback)
        hint=label(t('Live follows the slider of the selected position. Confirm only updates the local draft. Closing releases the PWM.'),'hint')
        hint.setWordWrap(True); layout.addWidget(hint); buttons(self,layout)
        if controller:
            controller.servo_previewed.connect(self._position_result); controller.changed.connect(self._availability)
        self._availability()
        self.preview_timer.start()

    def _live_changed(self, key, enabled):
        self._live_key=key if enabled else None
        self._pending=None
        if enabled: self._queue_position(key)
        self._availability()

    def _queue_position(self, key):
        self._pending=self.sliders[key].value(); self.feedback.clear(); self.feedback.hide()

    def _position_changed(self, key, pwm):
        self.pwm_labels[key].setText(f'PWM &nbsp; <b style="font-size:22px;">{pwm}</b>')
        if self._live_key==key: self._queue_position(key)

    def _send_position(self):
        if self._pending is None or self._sending or not self.controller: return
        pwm=self._pending
        if self.controller.preview_servo(self.channel,pwm):
            self._pending=None; self._sending=True; self._tested=True

    def _position_result(self, response):
        if not self._sending: return
        self._sending=False
        if response.get('error'):
            if self._live_key: self.live_buttons[self._live_key].setChecked(False)
            self._pending=None; self.feedback.setText(response['error']); self.feedback.show()
        else: self.feedback.clear(); self.feedback.hide()

    def _availability(self):
        editable=self.controller is None or self.controller.state.editable
        enabled=self.controller is not None and editable
        if not enabled and self._live_key: self.live_buttons[self._live_key].setChecked(False)
        for i,(key,live) in enumerate(self.live_buttons.items()):
            active=self._live_key in (None,key)
            live.setEnabled(enabled and active)
            self.pwm_labels[key].setEnabled(editable and active); self.sliders[key].setEnabled(editable and active)
            for step in self.step_buttons[key]: step.setEnabled(editable and self._live_key is None)
            self.apply_buttons[i].setEnabled(enabled and self._live_key is None)
        if self.controller and not enabled:
            self._pending=None
            if not self.controller.state.connected: self._sending=False
            self.feedback.setText(t('Movement available with the board disarmed and free')); self.feedback.show()

    def done(self, result):
        self.preview_timer.stop(); self._pending=None
        if self.controller:
            self.controller.servo_previewed.disconnect(self._position_result); self.controller.changed.disconnect(self._availability)
            if self._tested: self.controller.release_servo(self.channel)
            self.controller=None
        super().done(result)

    def result_value(self):
        values={key:widget.value() for key,widget in self.sliders.items()}
        values['calibrated']=True
        return values
