"""Configuration page: rule/channel editing and local draft presentation."""
from copy import deepcopy
from PySide6.QtCore import Qt,QItemSelection,QItemSelectionModel
from PySide6.QtWidgets import QWidget,QFrame,QVBoxLayout,QHBoxLayout,QTabWidget,QTreeWidget,QTreeWidgetItem,QHeaderView,QMessageBox
from .configuration import condition_text,rule_label,next_rule_id
from .dialogs import RuleDialog,ChannelDialog
from .airbrake_dialog import AirbrakeDialog
from .imu_calibration_page import ImuCalibrationPage
from .widgets import label,button,card,row,table,fill_table
from .i18n import t


class RuleTree(QTreeWidget):
    """One rule is the selection unit, including all its condition/output rows."""
    def __init__(self):
        super().__init__(); self._selecting=False
        self.setSelectionMode(self.SelectionMode.SingleSelection)
        self.setSelectionBehavior(self.SelectionBehavior.SelectRows)
        self.currentItemChanged.connect(lambda current,previous:self.select_rule(current))
        self.itemSelectionChanged.connect(self.select_rule)

    @staticmethod
    def rule_root(item):
        while item and item.parent(): item=item.parent()
        return item

    def select_rule(self,item=None):
        if self._selecting: return
        if item is None:
            selected=self.selectedItems()
            if not selected: return
            current=self.currentItem(); item=current if current in selected else selected[0]
        root=self.rule_root(item); selection=QItemSelection(); pending=[root]
        while pending:
            item=pending.pop()
            selection.select(self.indexFromItem(item,0),self.indexFromItem(item,self.columnCount()-1))
            pending.extend(item.child(i) for i in range(item.childCount()))
        self._selecting=True
        try:
            self.setCurrentItem(root)
            self.selectionModel().select(selection,QItemSelectionModel.ClearAndSelect|QItemSelectionModel.Rows)
        finally: self._selecting=False

    def keyPressEvent(self,event):
        key=event.key()
        if self.topLevelItemCount() and key in (Qt.Key_Up,Qt.Key_Down,Qt.Key_Home,Qt.Key_End):
            index=self.indexOfTopLevelItem(self.rule_root(self.currentItem()))
            if key==Qt.Key_Home: index=0
            elif key==Qt.Key_End: index=self.topLevelItemCount()-1
            else: index+=1 if key==Qt.Key_Down else -1
            item=self.topLevelItem(max(0,min(index,self.topLevelItemCount()-1)))
            self.setCurrentItem(item); self.select_rule(item); self.scrollToItem(item)
            event.accept(); return
        super().keyPressEvent(event)


class ConfigurationPage(QWidget):
    def __init__(self,controller):
        super().__init__(); self.controller=controller; self._rendered_plan=None; self._rendered_pulse=None; self.channel_buttons=[]
        self.setObjectName('canvas'); box=QVBoxLayout(self); box.setContentsMargins(0,0,0,0); box.setSpacing(18)
        title,subtitle=t('Configuration'),t('Deployment rules and channels. Changes stay in a local draft until Apply.')
        self.reload_config=button(t('Reload from board'),self.reload_configuration)
        box.addWidget(label(title,'title'))
        sub=label(subtitle,'subtitle'); sub.setWordWrap(True); box.addWidget(sub)
        self.servo_notice=label('','hint'); self.servo_notice.setWordWrap(True); box.addWidget(self.servo_notice)
        tabs=QTabWidget(); self.tabs=tabs; box.addWidget(tabs,1)
        tabs.setCornerWidget(self.reload_config,Qt.TopRightCorner)
        channels=QWidget(); ch=QVBoxLayout(channels); ch.setContentsMargins(0,14,0,0)
        frame,table_box=card(); ch.addWidget(frame,1)
        self.channels_table=table([t('CHANNEL'),t('SETTINGS'),t('ASSIGNED RULE'),t('STATUS')]); self.channels_table.itemDoubleClicked.connect(lambda _:self.edit_channel())
        table_box.addWidget(self.channels_table)
        rules=QWidget(); rl=QVBoxLayout(rules); rl.setContentsMargins(0,14,0,0)
        frame,rule_box=card(); rl.addWidget(frame,1)
        self.rules_tree=RuleTree(); self.rules_tree.setHeaderLabels([t('RULE / CONDITION'),t('DETAILS')]); self.rules_tree.setRootIsDecorated(True)
        self.rules_tree.setObjectName('rulesTree')
        self.rules_tree.header().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); self.rules_tree.itemDoubleClicked.connect(lambda *_:self.edit_rule())
        rule_box.addWidget(self.rules_tree)
        self.add_rule_button=button(t('+ New rule'),self.add_rule,'primary'); self.edit_rule_button=button(t('Edit'),self.edit_rule); self.remove_rule_button=button(t('Remove'),self.remove_rule)
        self.airbrake_button=button(t('Configure airbrakes'),self.edit_airbrakes); self.airbrake_button.hide()
        rule_box.addLayout(row(self.add_rule_button,self.airbrake_button,self.edit_rule_button,self.remove_rule_button,None))
        tabs.addTab(rules,t('Rules')); tabs.addTab(channels,t('Channels')); tabs.setCurrentIndex(0)
        self.imu_page=ImuCalibrationPage(controller); tabs.addTab(self.imu_page,t('IMU calibration'))
        self.attach_footer(box)

    def attach_footer(self,layout):
        self.draft_bar=QFrame(); self.draft_bar.setObjectName('draftBar'); box=QHBoxLayout(self.draft_bar); box.setContentsMargins(16,12,16,12)
        self.draft_text=label(t('Unapplied changes · local draft'))
        self.validate_button=button(t('Validate'),self.controller.validate)
        self.apply_button=button(t('Apply'),self.apply_configuration,'saveAction')
        self.discard_button=button(t('Discard'),self.controller.discard)
        box.addWidget(self.draft_text,1); box.addWidget(self.validate_button); box.addWidget(self.apply_button); box.addWidget(self.discard_button); layout.addWidget(self.draft_bar)

    def render(self,s):
        self.imu_page.render(s)
        self.draft_bar.setVisible(s.dirty or s.staged)
        self.draft_text.setText(t('Draft validated by the board · not applied yet') if s.validated else t('Unapplied changes · local draft'))
        self.validate_button.setEnabled(s.editable and s.dirty); self.apply_button.setEnabled(s.editable and s.dirty and s.validated)
        self.discard_button.setEnabled(not s.foreground)
        self.reload_config.setEnabled(s.connected and not s.foreground)
        editable=s.editable
        for b in (*self.channel_buttons,self.add_rule_button,self.edit_rule_button,self.remove_rule_button,self.airbrake_button): b.setEnabled(editable)
        self.airbrake_button.setVisible(s.connected and s.airbrakes.get('enabled',False))
        plan=s.local
        missing=[f'SERVO {i+1}' for i,v in enumerate(plan.servos) if (plan.assigned('servo',i+1) or plan.airbrake_channel(i+1)) and not v['calibrated']] if plan else []
        self.servo_notice.setText(t('{servos}: positions to configure in Channels. Rules can be saved; real arming requires the servos to be configured.',servos=', '.join(missing)))
        self.servo_notice.setVisible(bool(missing))
        pulse=s.capabilities.get('pyro_pulse_ms',1000)
        if plan==self._rendered_plan and pulse==self._rendered_pulse: return
        self._rendered_plan=deepcopy(plan); self._rendered_pulse=pulse
        if plan is None: fill_table(self.channels_table,[]); self.channel_buttons=[]; self.rules_tree.clear(); return
        rows=[]
        for kind,values in [('pyro',[pulse]*4),('servo',plan.servos)]:
            for i,value in enumerate(values):
                r=plan.assigned(kind,i+1)
                if kind=='pyro': text=t('Fixed pulse {pulse} ms · firmware',pulse=value)
                elif value['calibrated']: text=f'{value["initial_us"]} → {value["final_us"]} PWM'
                else: text=t('Initial and final positions to define')
                air=kind=='servo' and plan.airbrake_channel(i+1)
                if air and value['calibrated']: text=t('Closed 0%: {initial} / Open 100%: {final}',initial=value['initial_us'],final=value['final_us'])
                state=t('To configure') if kind=='servo' and (r or air) and not value['calibrated'] else t('Assigned') if r or air else t('Free')
                rows.append((f'{kind.upper()} {i+1}',text,t('Airbrakes') if air else rule_label(plan.rules,r['id']) if r else '—',state))
        fill_table(self.channels_table,rows)
        self.channel_buttons=[]
        for i,value in enumerate(plan.servos):
            self.channels_table.removeCellWidget(i+4,3); self.channels_table.setRowHeight(i+4,43)
            if not (plan.assigned('servo',i+1) or plan.airbrake_channel(i+1)): continue
            self.channels_table.item(i+4,3).setText(''); self.channels_table.setRowHeight(i+4,60)
            action=button(t('Edit') if value['calibrated'] else t('Configure'),lambda channel=i+1:self.edit_channel(channel))
            action.setEnabled(editable); action.setMinimumWidth(100); self.channel_buttons.append(action)
            cell=QWidget(); cell_box=QHBoxLayout(cell); cell_box.setContentsMargins(6,3,6,3)
            cell_box.addWidget(action,0,Qt.AlignCenter); self.channels_table.setCellWidget(i+4,3,cell)
        expanded={self.rules_tree.topLevelItem(i).data(0,Qt.ItemDataRole.UserRole) for i in range(self.rules_tree.topLevelItemCount()) if self.rules_tree.topLevelItem(i).isExpanded()}
        selected=self.selected_rule()
        self.rules_tree.clear()
        if plan.airbrakes['channel_mask'] or plan.airbrakes['conditions']:
            names=', '.join(f'SERVO {i}' for i in range(1,5) if plan.airbrake_channel(i)) or t('No channel')
            item=QTreeWidgetItem([t('Airbrakes'), t('Continuous AND control · {channels}',channels=names)]); item.setData(0,Qt.ItemDataRole.UserRole,'airbrakes')
            self.rules_tree.addTopLevelItem(item)
            for c in plan.airbrakes['conditions']: QTreeWidgetItem(item,[t('Condition · AND'),condition_text(c,plan.rules)])
            item.setExpanded(True)
        for r in plan.rules:
            item=QTreeWidgetItem([rule_label(plan.rules,r['id']),t('Delay {delay} ms · {count} outputs',delay=r['delay_ms'],count=len(r['outputs']))])
            item.setData(0,Qt.ItemDataRole.UserRole,r['id']); self.rules_tree.addTopLevelItem(item)
            for c in r['conditions']: QTreeWidgetItem(item,[t('Condition · AND'),condition_text(c,plan.rules)])
            for a in r['outputs']: QTreeWidgetItem(item,[f'{a["kind"].upper()} {a["channel"]}',t('Channel delay {delay} ms',delay=a.get('delay_ms',0))])
            item.setExpanded(r['id'] in expanded or len(plan.rules)<=2)
        for i in range(self.rules_tree.topLevelItemCount()):
            item=self.rules_tree.topLevelItem(i)
            if item.data(0,Qt.ItemDataRole.UserRole)==selected: self.rules_tree.setCurrentItem(item); break

    def edit_airbrakes(self):
        s=self.controller.state
        if not s.editable or s.local is None: return
        dialog=AirbrakeDialog(self,s.local,bistage=s.machine.get('bistage',False))
        if dialog.exec():
            plan=s.local.clone(); plan.airbrakes=dialog.rule; self.controller.edit_plan(plan)

    def show_servo(self,channel,edit=False):
        self.tabs.setCurrentIndex(1); self.channels_table.selectRow(channel+3)
        self.channels_table.scrollToItem(self.channels_table.item(channel+3,0))
        if edit: self.edit_channel(channel)

    def edit_channel(self,channel=None):
        s=self.controller.state; i=channel+3 if channel is not None else self.channels_table.currentRow()
        if not s.editable or s.local is None or i<0: return
        kind='pyro' if i<4 else 'servo'; index=i%4
        if kind=='pyro':
            QMessageBox.information(self,t('Firmware pyro profile'),
                t('Fixed {pulse} ms pulse, not repeated. No per-channel settings are needed.',pulse=s.capabilities.get('pyro_pulse_ms',1000)))
            return
        value=s.local.servos[index]
        dialog=ChannelDialog(self,kind,index+1,value,s.capabilities,self.controller,airbrake=s.local.airbrake_channel(index+1))
        if dialog.exec(): self.controller.save_servo(index+1,dialog.result_value()) # Saved to the board at once.

    def selected_rule(self):
        if not self.rules_tree.selectedItems(): return None
        item=self.rules_tree.rule_root(self.rules_tree.currentItem())
        return item.data(0,Qt.ItemDataRole.UserRole) if item else None

    def add_rule(self):
        s=self.controller.state
        if not s.editable or s.local is None or len(s.local.rules)>=8: return
        rule=dict(id=next_rule_id(s.local,s.base),name='',delay_ms=0,conditions=[],outputs=[])
        dialog=RuleDialog(self,rule,s.local.rules,bistage=s.machine.get('bistage',False))
        if dialog.exec(): plan=s.local.clone(); plan.rules.append(dialog.rule); self.controller.edit_plan(plan)

    def edit_rule(self):
        s=self.controller.state; identity=self.selected_rule()
        if not s.editable or not identity: return
        if identity=='airbrakes': self.edit_airbrakes(); return
        rule=next(r for r in s.local.rules if r['id']==identity)
        dialog=RuleDialog(self,rule,s.local.rules,bistage=s.machine.get('bistage',False))
        if dialog.exec():
            plan=s.local.clone(); plan.rules=[dialog.rule if r['id']==identity else r for r in plan.rules]; self.controller.edit_plan(plan)

    def remove_rule(self):
        s=self.controller.state; identity=self.selected_rule()
        if not s.editable or not identity: return
        plan=s.local.clone()
        if identity=='airbrakes': plan.airbrakes=dict(channel_mask=0,conditions=[])
        else: plan.rules=[r for r in plan.rules if r['id']!=identity]
        self.controller.edit_plan(plan)

    def reload_configuration(self):
        s=self.controller.state
        if s.dirty:
            if QMessageBox.question(self,t('Reload configuration'),t('Discard the local changes and read the board again?'))!=QMessageBox.StandardButton.Yes: return
            s.local=s.base.clone() if s.base else None
        self.controller.reload_config()

    def apply_configuration(self):
        self.controller.validate(apply=True)
