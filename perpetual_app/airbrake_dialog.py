"""Editor for the single continuous AND gate; no physical commands."""
from copy import deepcopy
from PySide6.QtWidgets import QDialog,QVBoxLayout,QCheckBox,QMessageBox
from .configuration import condition_text,rule_label
from .dialogs import ConditionDialog,buttons
from .widgets import label,row,button,table,fill_table
from .i18n import t


class AirbrakeDialog(QDialog):
    def __init__(self,parent,plan,*,bistage=False):
        super().__init__(parent); self.setWindowTitle(t('Airbrakes · continuous control')); self.resize(680,590)
        self.rule=deepcopy(plan.airbrakes); self.rules=plan.rules; self.bistage=bistage
        layout=QVBoxLayout(self); layout.setSpacing(14)
        layout.addWidget(label(t('DEDICATED SERVOS'),'eyebrow')); self.channels=[]
        for i in range(4):
            assigned=plan.assigned('servo',i+1)
            text=f'SERVO {i+1}'
            if assigned: text+=t(' · assigned to {rule}',rule=rule_label(plan.rules,assigned['id']))
            check=QCheckBox(text); check.setChecked(bool(self.rule['channel_mask']&(1<<i)))
            check.setEnabled(assigned is None); check.setToolTip(t('Remove this servo from the ordinary rule first.') if assigned else t('Initial position = closed 0%; final = open 100%.'))
            self.channels.append(check); layout.addWidget(check)
        layout.addLayout(row(label(t('CONDITIONS · AND'),'eyebrow'),None,button(t('+ Condition'),self.add_condition)))
        self.conditions=table([t('Condition')]); self.conditions.itemDoubleClicked.connect(lambda _:self.edit_condition())
        layout.addWidget(self.conditions,1)
        layout.addLayout(row(button(t('Edit'),self.edit_condition),button(t('Remove'),self.remove_condition),None))
        hint=label(t('Control is allowed while all conditions are true. When one is not satisfied, in ABORT or with an invalid estimate, the airbrakes return to 0%. No delays apply.'),'hint')
        hint.setWordWrap(True); layout.addWidget(hint)
        hint=label(t('The general enable is in Settings. Selecting no servo removes the airbrake assignment.'),'hint'); hint.setWordWrap(True); layout.addWidget(hint)
        buttons(self,layout,self.finish); self.render()

    def render(self): fill_table(self.conditions,[(condition_text(c,self.rules),) for c in self.rule['conditions']])

    def add_condition(self):
        if len(self.rule['conditions'])>=8: return
        dialog=ConditionDialog(self,rules=self.rules,bistage=self.bistage)
        if dialog.exec(): self.rule['conditions'].append(dialog.result_value()); self.render()

    def edit_condition(self):
        i=self.conditions.currentRow()
        if i<0: return
        dialog=ConditionDialog(self,self.rule['conditions'][i],rules=self.rules,bistage=self.bistage)
        if dialog.exec(): self.rule['conditions'][i]=dialog.result_value(); self.render()

    def remove_condition(self):
        i=self.conditions.currentRow()
        if i>=0: del self.rule['conditions'][i]; self.render()

    def finish(self):
        mask=sum(1<<i for i,c in enumerate(self.channels) if c.isChecked() and c.isEnabled())
        if mask and not self.rule['conditions']:
            QMessageBox.information(self,t('Airbrake conditions'),t('Add at least one condition.')); return
        self.rule['channel_mask']=mask; self.accept()
