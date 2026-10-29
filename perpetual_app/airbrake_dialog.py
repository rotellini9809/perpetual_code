"""Editor for the single continuous AND gate; no physical commands."""
from copy import deepcopy
from PySide6.QtWidgets import QDialog,QVBoxLayout,QCheckBox,QMessageBox
from .configuration import condition_text,rule_label
from .dialogs import ConditionDialog,buttons
from .widgets import label,row,button,table,fill_table


class AirbrakeDialog(QDialog):
    def __init__(self,parent,plan):
        super().__init__(parent); self.setWindowTitle('Aerofreni · controllo continuo'); self.resize(680,590)
        self.rule=deepcopy(plan.airbrakes); self.rules=plan.rules
        layout=QVBoxLayout(self); layout.setSpacing(14)
        layout.addWidget(label('SERVO DEDICATI','eyebrow')); self.channels=[]
        for i in range(4):
            assigned=plan.assigned('servo',i+1)
            text=f'SERVO {i+1}'
            if assigned: text+=' · assegnato a '+rule_label(plan.rules,assigned['id'])
            check=QCheckBox(text); check.setChecked(bool(self.rule['channel_mask']&(1<<i)))
            check.setEnabled(assigned is None); check.setToolTip('Rimuovi prima questo servo dalla regola ordinaria.' if assigned else 'Posizione iniziale = chiuso 0%; finale = aperto 100%.')
            self.channels.append(check); layout.addWidget(check)
        layout.addLayout(row(label('CONDIZIONI · AND','eyebrow'),None,button('+ Condizione',self.add_condition)))
        self.conditions=table(['Condizione']); self.conditions.itemDoubleClicked.connect(lambda _:self.edit_condition())
        layout.addWidget(self.conditions,1)
        layout.addLayout(row(button('Modifica',self.edit_condition),button('Rimuovi',self.remove_condition),None))
        hint=label('Il controllo è consentito finché tutte le condizioni sono vere. Quando una non è soddisfatta, in ABORT o con stima non valida, gli aerofreni tornano allo 0%. Non si applicano ritardi.','hint')
        hint.setWordWrap(True); layout.addWidget(hint)
        hint=label('L’abilitazione generale si trova in Impostazioni. Nessun servo selezionato rimuove l’assegnazione agli aerofreni.','hint'); hint.setWordWrap(True); layout.addWidget(hint)
        buttons(self,layout,self.finish); self.render()

    def render(self): fill_table(self.conditions,[(condition_text(c,self.rules),) for c in self.rule['conditions']])

    def add_condition(self):
        if len(self.rule['conditions'])>=8: return
        dialog=ConditionDialog(self,rules=self.rules)
        if dialog.exec(): self.rule['conditions'].append(dialog.result_value()); self.render()

    def edit_condition(self):
        i=self.conditions.currentRow()
        if i<0: return
        dialog=ConditionDialog(self,self.rule['conditions'][i],rules=self.rules)
        if dialog.exec(): self.rule['conditions'][i]=dialog.result_value(); self.render()

    def remove_condition(self):
        i=self.conditions.currentRow()
        if i>=0: del self.rule['conditions'][i]; self.render()

    def finish(self):
        mask=sum(1<<i for i,c in enumerate(self.channels) if c.isChecked() and c.isEnabled())
        if mask and not self.rule['conditions']:
            QMessageBox.information(self,'Condizioni aerofreni','Aggiungi almeno una condizione.'); return
        self.rule['channel_mask']=mask; self.accept()
