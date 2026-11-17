"""Deliberate hold gestures, cancellation, release suppression and isolated preview."""
import time
from PySide6.QtCore import Qt,QEvent
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QWidget,QVBoxLayout,QLineEdit
from PySide6.QtTest import QTest
from perpetual_app.arm_button import ArmButton,ArmButtonPreview


def check_arm_button(app):
    window=QWidget(); layout=QVBoxLayout(window); button=ArmButton(hold_ms=120); other=QLineEdit()
    layout.addWidget(button); layout.addWidget(other); window.resize(280,150); window.show(); window.activateWindow(); app.processEvents()
    confirmed=[]; button.confirmed.connect(lambda:confirmed.append(button.text()))
    try:
        button.click(); QTest.mouseClick(button,Qt.LeftButton); QTest.qWait(160)
        assert not confirmed # A click cannot bypass the hold.
        for cancel in (lambda:QTest.mouseRelease(button,Qt.LeftButton),lambda:button.setEnabled(False),
                       lambda:button.hide(),lambda:other.setFocus(),lambda:QTest.keyClick(button,Qt.Key_Escape),
                       lambda:app.sendEvent(button,QEvent(QEvent.Leave))):
            button.setEnabled(True); button.show(); window.activateWindow(); app.processEvents()
            QTest.mousePress(button,Qt.LeftButton); QTest.qWait(35); cancel(); QTest.qWait(150)
            QTest.mouseRelease(button,Qt.LeftButton); assert not confirmed
            button.cancel_hold(immediate=True)

        # The hold must survive only responsive UI time, not a blocked event loop.
        button.setEnabled(True); button.show(); window.activateWindow(); app.processEvents()
        QTest.mousePress(button,Qt.LeftButton); time.sleep(.3); app.processEvents()
        QTest.mouseRelease(button,Qt.LeftButton); assert not confirmed

        # A synchronous board acknowledgement changing ARM to DISARM must consume the original release.
        def acknowledge(): button.set_disarming(True)
        button.confirmed.connect(acknowledge)
        QTest.mousePress(button,Qt.LeftButton); QTest.qWait(160)
        assert confirmed==['ARM'] and button.text()=='DISARM'
        QTest.mouseRelease(button,Qt.LeftButton); assert confirmed==['ARM']
        assert button.property('disarming') is True
        button.click(); QTest.mouseClick(button,Qt.LeftButton); QTest.qWait(160)
        assert confirmed==['ARM'] # DISARM also requires a full hold.
        button.confirmed.disconnect(acknowledge)
        for cancel in (lambda:QTest.mouseRelease(button,Qt.LeftButton),lambda:button.setEnabled(False),
                       lambda:button.hide(),lambda:other.setFocus(),lambda:QTest.keyClick(button,Qt.Key_Escape),
                       lambda:app.sendEvent(button,QEvent(QEvent.Leave))):
            button.setEnabled(True); button.show(); window.activateWindow(); app.processEvents()
            QTest.mousePress(button,Qt.LeftButton); QTest.qWait(35); cancel(); QTest.qWait(150)
            QTest.mouseRelease(button,Qt.LeftButton); assert confirmed==['ARM']
            button.cancel_hold(immediate=True)
        button.setEnabled(True); button.show(); window.activateWindow(); app.processEvents()
        def acknowledge_disarm(): button.set_disarming(False)
        button.confirmed.connect(acknowledge_disarm)
        QTest.mousePress(button,Qt.LeftButton); QTest.qWait(160)
        assert confirmed==['ARM','DISARM'] and button.text()=='ARM' and button.property('disarming') is False
        QTest.mouseRelease(button,Qt.LeftButton); assert confirmed==['ARM','DISARM']
        button.confirmed.disconnect(acknowledge_disarm)

        button.set_disarming(False); confirmed.clear(); button.setFocus(); app.processEvents()
        QTest.keyPress(button,Qt.Key_Space); QTest.qWait(35)
        repeat=QKeyEvent(QEvent.KeyPress,Qt.Key_Space,Qt.NoModifier,' ',True,1); app.sendEvent(button,repeat)
        QTest.qWait(125); QTest.keyRelease(button,Qt.Key_Space)
        assert confirmed==['ARM']
        button.set_request_pending(False); assert not button._submitted
        button.set_disarming(True); confirmed.clear()
        for key in (Qt.Key_Space,Qt.Key_Return,Qt.Key_Enter):
            QTest.keyClick(button,key); QTest.qWait(160); assert not confirmed
            QTest.keyPress(button,key); QTest.qWait(160); QTest.keyRelease(button,key)
            assert confirmed==['DISARM']; button.set_request_pending(False); confirmed.clear()
        button.set_disarming(False)

        # Rejection can leave the button disabled, but must clear the pending visual state.
        button.cancel_hold(immediate=True); confirmed.clear(); button.setEnabled(True)
        QTest.mousePress(button,Qt.LeftButton); QTest.qWait(160)
        button.setEnabled(False); QTest.mouseRelease(button,Qt.LeftButton)
        button.set_request_pending(False)
        assert confirmed==['ARM'] and not button._submitted
    finally: window.close(); app.processEvents()

    preview=ArmButtonPreview(); preview.show(); preview.activateWindow(); app.processEvents()
    sent=[]; completed=[]; preview.arm_button.confirmed.connect(lambda:sent.append(1)); preview.arm_button.previewed.connect(lambda:completed.append(1))
    try:
        preview.arm_button._hold_ms=120
        QTest.mousePress(preview.arm_button,Qt.LeftButton); QTest.qWait(160); QTest.mouseRelease(preview.arm_button,Qt.LeftButton)
        assert completed==[1] and not sent and preview.arm_button.text()=='DISARM' and preview.arm_button.property('disarming') is True
        assert preview.arm_button._completion_flash>0 # Completion survives the state change and original release.
        QTest.qWait(400); assert preview.arm_button._completion_flash==0 and completed==[1]
        QTest.mouseClick(preview.arm_button,Qt.LeftButton); QTest.qWait(160)
        assert completed==[1] and preview.arm_button.text()=='DISARM'
        QTest.mousePress(preview.arm_button,Qt.LeftButton); QTest.qWait(160); QTest.mouseRelease(preview.arm_button,Qt.LeftButton)
        assert completed==[1,1] and not sent and preview.arm_button.text()=='ARM' and preview.arm_button.property('disarming') is False
        assert preview.arm_button._completion_flash>0
        assert not preview.arm_button._submitted and preview.arm_button._progress==0
    finally: preview.close(); app.processEvents()
