"""Deliberate ARM gesture and animation; board readiness remains owned by the controller."""
from PySide6.QtCore import Qt,QEvent,QTimer,QElapsedTimer,QVariantAnimation,QEasingCurve,QRectF,Signal
from PySide6.QtGui import QColor,QFont,QLinearGradient,QPainter,QPainterPath
from PySide6.QtWidgets import QPushButton,QStyle,QStyleOptionButton,QStylePainter,QDialog,QVBoxLayout


class ArmButton(QPushButton):
    confirmed=Signal()
    previewed=Signal()

    def __init__(self,callback=None,*,hold_ms=1500,preview=False):
        super().__init__('ARM'); self.setObjectName('armFlight'); self.setMinimumSize(180,50)
        self.setAutoDefault(False); self.setAutoRepeat(False); self.setCursor(Qt.PointingHandCursor)
        self.setAccessibleName('ARM'); self.setAccessibleDescription('Tieni premuto il pulsante o il tasto Spazio per 1,5 secondi; rilascia per annullare.')
        self._hold_ms=hold_ms; self._preview=preview; self._disarming=False
        self._source=None; self._key=None; self._press_mode=None; self._holding=False; self._submitted=False; self._progress=0.0
        self._elapsed=QElapsedTimer(); self._last_tick=0
        self._timer=QTimer(self); self._timer.setTimerType(Qt.PreciseTimer); self._timer.setInterval(16); self._timer.timeout.connect(self._advance)
        self._return=QVariantAnimation(self); self._return.setDuration(220); self._return.setEasingCurve(QEasingCurve.OutCubic)
        self._return.valueChanged.connect(self._set_progress)
        self._preview_timer=QTimer(self); self._preview_timer.setSingleShot(True); self._preview_timer.timeout.connect(self._finish_preview)
        self.clicked.connect(self._clicked)
        if callback: self.confirmed.connect(callback)

    def set_disarming(self,value):
        if self._disarming==value: return
        self.cancel_hold(immediate=True); self._disarming=value
        self.setText('DISARM' if value else 'ARM'); self.setAccessibleName(self.text()); self.update()

    def _set_progress(self,value):
        self._progress=float(value); self.update()

    def cancel_hold(self,*,immediate=False):
        self._timer.stop(); self._holding=False
        self._source=None; self._key=None; self._press_mode=None; self.setDown(False)
        if self._submitted and not immediate: return # An emitted request is awaiting its acknowledged result.
        self._preview_timer.stop(); self._submitted=False; self._return.stop()
        if immediate: self._set_progress(0)
        elif self._progress:
            self._return.setStartValue(self._progress); self._return.setEndValue(0.0); self._return.start()

    def _begin_hold(self,source,key=None):
        if self._submitted or self._source is not None: return
        self._return.stop(); self._source=source; self._key=key; self._press_mode='arm'; self._holding=True
        self.setDown(True); self._set_progress(0); self._last_tick=0; self._elapsed.start(); self._timer.start()

    def _advance(self):
        if not self._holding or not self.isEnabled() or not self.isVisible() or self.visibleRegion().isEmpty() or self._disarming:
            self.cancel_hold(immediate=True); return
        elapsed=self._elapsed.elapsed()
        if elapsed-self._last_tick>250: # A stalled UI must not complete a gesture after queued input has gone stale.
            self.cancel_hold(); return
        self._last_tick=elapsed; self._set_progress(min(1.0,elapsed/self._hold_ms))
        if elapsed>=self._hold_ms:
            self._timer.stop(); self._holding=False; self._submitted=True; self.setDown(False)
            if self._preview:
                self.previewed.emit(); self._preview_timer.start(900)
            else: self.confirmed.emit()

    def _finish_preview(self):
        self._submitted=False; self.cancel_hold()

    def set_request_pending(self,pending):
        if not pending and self._submitted and not self._preview:
            self._submitted=False; self.cancel_hold(immediate=not self.isEnabled())

    def _clicked(self):
        # Programmatic click/accessibility activation cannot bypass the ARM hold.
        if self._disarming and self.isEnabled(): self.confirmed.emit()

    def mousePressEvent(self,event):
        if event.button()!=Qt.LeftButton or not self.isEnabled(): event.ignore(); return
        self.setFocus(Qt.MouseFocusReason)
        if self._disarming:
            self._source='mouse'; self._press_mode='disarm'; super().mousePressEvent(event)
        else: self._begin_hold('mouse'); event.accept()

    def mouseReleaseEvent(self,event):
        if event.button()==Qt.LeftButton and self._source=='mouse' and self._press_mode=='disarm' and self._disarming:
            self._source=None; self._press_mode=None; super().mouseReleaseEvent(event)
        else:
            self.cancel_hold(); event.accept() # Never turn the release of an ARM hold into a DISARM click.

    def mouseMoveEvent(self,event):
        if self._holding and self._source=='mouse' and not self.rect().contains(event.position().toPoint()): self.cancel_hold()
        super().mouseMoveEvent(event)

    def mouseDoubleClickEvent(self,event): event.accept()

    def leaveEvent(self,event):
        if self._holding and self._source=='mouse': self.cancel_hold()
        super().leaveEvent(event)

    def keyPressEvent(self,event):
        if event.key() in (Qt.Key_Space,Qt.Key_Return,Qt.Key_Enter):
            if not event.isAutoRepeat() and self.isEnabled() and self._source is None:
                if self._disarming:
                    self._source='keyboard'; self._key=event.key(); self._press_mode='disarm'; self.setDown(True)
                else: self._begin_hold('keyboard',event.key())
            event.accept(); return
        if event.key()==Qt.Key_Escape and self._holding: self.cancel_hold(); event.accept(); return
        super().keyPressEvent(event)

    def keyReleaseEvent(self,event):
        if event.key() in (Qt.Key_Space,Qt.Key_Return,Qt.Key_Enter):
            if not event.isAutoRepeat() and self._source=='keyboard' and event.key()==self._key:
                disarm=self._press_mode=='disarm' and self._disarming
                self.cancel_hold()
                if disarm: self.click()
            event.accept(); return
        super().keyReleaseEvent(event)

    def focusOutEvent(self,event):
        self.cancel_hold(); super().focusOutEvent(event)

    def hideEvent(self,event):
        self.cancel_hold(immediate=True); super().hideEvent(event)

    def changeEvent(self,event):
        if event.type()==QEvent.EnabledChange:
            if not self.isEnabled(): self.cancel_hold(immediate=not self._submitted)
            elif self._submitted: self._submitted=False; self.cancel_hold()
        super().changeEvent(event)

    def paintEvent(self,event):
        option=QStyleOptionButton(); self.initStyleOption(option); option.text=''
        p=QStylePainter(self); p.setRenderHint(QPainter.Antialiasing); p.drawControl(QStyle.CE_PushButton,option)
        box=QRectF(self.rect()).adjusted(1,1,-1,-1)
        if self._progress:
            p.save(); clip=QPainterPath(); clip.addRoundedRect(box,7,7); p.setClipPath(clip)
            edge=box.left()+box.width()*self._progress
            fill=QLinearGradient(box.left(),0,edge,0); fill.setColorAt(0,QColor('#982b30')); fill.setColorAt(1,QColor('#ef7568'))
            p.fillRect(QRectF(box.left(),box.top(),edge-box.left(),box.height()),fill)
            shine=QLinearGradient(edge-30,0,edge+8,0)
            shine.setColorAt(0,QColor(255,255,255,0)); shine.setColorAt(.75,QColor(255,255,255,65)); shine.setColorAt(1,QColor(255,255,255,0))
            p.fillRect(QRectF(edge-30,box.top(),38,box.height()),shine)
            p.fillRect(QRectF(box.left(),box.bottom()-3,edge-box.left(),3),QColor(255,255,255,180)); p.restore()
        title=self.text(); subtitle=''
        if self._holding:
            title='MANTIENI PREMUTO'; subtitle=f'{self._progress*self._hold_ms/1000:.1f} / {self._hold_ms/1000:.1f} s'
        elif self._submitted and not self._disarming:
            title='TEST RIUSCITO' if self._preview else 'INVIO RICHIESTA…'
            subtitle='Nessun comando inviato' if self._preview else 'In attesa della scheda'
        elif self.isEnabled() and not self._disarming: subtitle='Tieni premuto · 1,5 s'
        p.setPen(self.palette().buttonText().color()); font=QFont(self.font())
        if subtitle:
            if self._holding or self._submitted: font.setPixelSize(12)
            p.setFont(font); p.drawText(QRectF(4,4,self.width()-8,24),Qt.AlignCenter,title)
            font.setPixelSize(10); font.setWeight(QFont.Normal); font.setLetterSpacing(QFont.AbsoluteSpacing,.2)
            p.setFont(font); p.drawText(QRectF(4,29,self.width()-8,15),Qt.AlignCenter,subtitle)
        else:
            p.setFont(font); p.drawText(self.rect(),Qt.AlignCenter,title)


class ArmButtonPreview(QDialog):
    """Isolated, temporary animation test. Creates no controller or device connection."""
    def __init__(self):
        from .widgets import ASSETS,button,card,label,row
        super().__init__(); self.setWindowTitle('Perpetual · Anteprima ARM'); self.setObjectName('canvas')
        self.setStyleSheet((ASSETS/'style.qss').read_text(encoding='utf-8')); self.resize(520,300)
        box=QVBoxLayout(self); box.setContentsMargins(20,20,20,20)
        frame,body=card('Anteprima del pulsante ARM'); box.addWidget(frame)
        hint=label('Prova l’animazione tenendo premuto per 1,5 secondi. Questa anteprima non comanda la scheda.','hint')
        hint.setWordWrap(True); body.addWidget(hint)
        self.arm_button=ArmButton(preview=True); body.addLayout(row(None,self.arm_button,None))
        hint=label('Rilascia prima del termine per annullare. Puoi usare anche il tasto Spazio.','hint'); hint.setWordWrap(True); body.addWidget(hint)
        box.addLayout(row(None,button('Chiudi anteprima',self.close)))

    def close_from_terminal(self): self.close()
