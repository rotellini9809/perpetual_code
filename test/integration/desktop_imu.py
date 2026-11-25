"""UI follows firmware IMU status and sends only explicit acquisition commands."""
from copy import deepcopy
import time
from pathlib import Path
from unittest.mock import patch
from PySide6.QtCore import Qt
from perpetual_app.controller import Controller
from perpetual_app.window import MainWindow


def check_imu(app,wait,native):
    backend=native(); controller=Controller(lambda _:backend,lambda:[]); controller.timer.stop()
    window=MainWindow(controller); window.show(); app.processEvents()
    imu=dict(imu_calibration_state='absent',imu_calibration_reason='MISSING',active=False,
             collecting=False,verification=False,face_mask=0,face=0,samples=0,target=240,last_error='None',window_error='None')
    original=backend.request
    def request(command,timeout=5):
        if command.startswith('imu '):
            backend.commands.append(command)
            if command in ('imu begin','imu verify'):
                imu.update(active=True,collecting=False,face_mask=0,verification=command=='imu verify')
            elif command.startswith('imu capture '):
                imu.update(collecting=True,face=('+X','-X','+Y','-Y','+Z','-Z').index(command.split()[-1]),samples=0,window_error='None')
            elif command=='imu cancel': imu.update(active=False,collecting=False)
            return deepcopy(imu)
        response=original(command,timeout)
        if command=='preflight':
            state=imu['imu_calibration_state']
            reason={'absent':'IMU_CALIBRATION_MISSING','required':'IMU_VERIFICATION_REQUIRED'}.get(state)
            response.update(imu_calibration_state=state,imu_calibration_reason=imu['imu_calibration_reason'])
            if reason: response['reasons'].append(reason); response['ready']=False
        return response
    try:
        with patch.object(backend,'request',side_effect=request):
            controller.connect_port('IMU_FIXTURE'); wait(lambda:controller.state.base is not None and not controller.state.foreground)
            s=controller.state; page=window.configuration_page.imu_page
            index=next(i for i in range(window.dashboard_page.readiness_table.rowCount()) if window.dashboard_page.readiness_table.item(i,0).data(Qt.ItemDataRole.UserRole)=='imu')
            assert window.dashboard_page.readiness_table.item(index,0).text()=='Calibrazione IMU assente · Nessuna calibrazione salvata. · Apri calibrazione'
            assert not window.dashboard_page.arm_button.isEnabled()
            window.dashboard_page.open_preflight_item(index,0)
            assert window.pages.currentIndex()==1 and window.configuration_page.tabs.currentWidget() is page
            assert page.begin_button.isEnabled() and not page.verify_button.isEnabled()
            page.begin_button.click(); wait(lambda:not s.foreground)
            assert s.imu_calibration['active'] and not s.editable and not page.begin_button.isEnabled()
            assert page.capture_button.text()=='Acquisisci +X' and page.session.isVisibleTo(page) and page.board_caption.text()=='+X IN ALTO'
            assert [chip.property('step') for chip in page.chips]==['current']+['pending']*5 and not any(chip.isHidden() for chip in page.chips)
            page.capture_button.click(); wait(lambda:not s.foreground)
            assert backend.commands[-3]=='imu capture +X' and not page.capture_button.isEnabled()
            imu.update(collecting=False,face_mask=1,samples=240)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.capture_button.text()=='Acquisisci -X' and page.capture_button.isEnabled()
            assert page.progress.property('phase')=='done' and page.progress.value()==240
            page.reset_timer.timeout.emit(); assert page.progress.property('phase')=='idle' and page.progress.value()==0
            assert [chip.property('step') for chip in page.chips[:2]]==['done','current'] and page.board_caption.text()=='-X IN ALTO'
            page.cancel_button.click(); wait(lambda:not s.foreground)
            assert s.editable and not s.imu_calibration['active']
            imu.update(imu_calibration_state='required',imu_calibration_reason='AFTER_BOOST',revision=2,bias_mps2=[0.02,-0.4,0.1],scale=[1.001,0.98,1.0])
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.values['bias',1].text()=='-0.4000' and page.values['scale',1].text()=='0.98000'
            assert page.drift_caption.isHidden() # No recalibration in this session.
            assert page.residual_caption.isHidden() and page.meter_hint.isHidden() # Not verified yet: no empty row.
            imu.update(verification_residual_mps2=[0.0175,-0.4,None]) # A failed verification: +Z not reached.
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert [w.text() for w in page.residuals]==['+0.0175','-0.4000','—'] and [m.value() for m in page.residual_meters]==[50,1000,0]
            assert page.coefficients.isVisibleTo(page) and not page.session.isVisibleTo(page) and page.detail.isHidden()
            assert page.verify_button.isEnabled() and 'dopo il volo' in page.cause_label.text() and page.verify_button.property('attention')
            page.verify_button.click(); wait(lambda:not s.foreground)
            imu.update(face_mask=1)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.capture_button.text()=='Acquisisci +Y' and [chip.isHidden() for chip in page.chips]==[False,True]*3
            imu.update(collecting=True,face=2,samples=150,window_error='None')
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.progress.property('phase')=='loading' and page.progress.value()==150 and page.detail.isHidden()
            imu.update(collecting=False,samples=0,window_error='POSITION_OR_MOTION') # The firmware stops the capture.
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.progress.property('phase')=='stopped' and page.progress.value()==150 and page.progress_caption.isHidden()
            assert page.detail.isVisibleTo(page) and page.detail.property('tone')=='warn'
            assert page.capture_button.isEnabled() and page.capture_button.text()=='Acquisisci +Y'
            page.capture_button.click(); wait(lambda:not s.foreground) # Capturing again clears the warning in the firmware.
            assert backend.commands[-3]=='imu capture +Y' and page.progress.property('phase')=='loading' and page.progress.value()==0 and page.detail.isHidden()
            imu.update(collecting=False,samples=0,window_error='None')
            imu.update(face_mask=5)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.capture_button.text()=='Acquisisci +Z'
            window.grab().save(str(Path(__file__).resolve().parents[2]/'.pio/tests/imu-calibration.png'))
            imu.update(active=False,collecting=False,face_mask=21,imu_calibration_state='verified',imu_calibration_reason='None')
            controller._last_imu=0; controller._last_status=time.monotonic(); controller._want_events=False; controller.tick()
            wait(lambda:not controller._jobs)
            assert s.preflight['imu_calibration_state']=='verified' and 'Verificata' in page.state_label.text()
            table=window.dashboard_page.readiness_table
            assert not any('IMU' in table.item(i,0).text() and 'Calibrazione' in table.item(i,0).text() for i in range(table.rowCount())) # Verified: not a warning.
            assert page.capture_button.isHidden()
            # A recalibration shows how much the bias moved: new bias minus the one read before it.
            page.begin_button.click(); wait(lambda:not s.foreground)
            imu.update(active=False,revision=3,bias_mps2=[0.03,-0.35,0.1],imu_calibration_state='required',imu_calibration_reason='NOT_VERIFIED',verification_residual_mps2=None)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert [w.text() for w in page.drift]==['+0.0100','+0.0500','+0.0000'] and page.drift_caption.isVisibleTo(page)
            assert page.residual_caption.isHidden()
            imu.update(imu_calibration_state='invalid',imu_calibration_reason='VERIFICATION_FAILED',last_error='IMU_VERIFICATION_FAILED',identity_allowed=False)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert page.begin_button.isEnabled() and not page.verify_button.isEnabled()
            assert 'Da ricalibrare' in page.state_label.text() and 'disabilitata definitivamente' in page.cause_label.text()
            imu.update(imu_calibration_state='identity',imu_calibration_reason='IDENTITY',last_error='None',revision=0,identity_allowed=True)
            controller.command('imu status'); wait(lambda:not s.foreground)
            assert 'Identità (predefinita)' in page.state_label.text() and page.begin_button.isEnabled()
            # An old connection's approval must not survive disconnect/reconnect.
            controller.disconnect(); wait(lambda:controller.connection is None)
            assert not s.imu_calibration
    finally:
        if controller.connection: controller.disconnect(); wait(lambda:controller.can_close())
        window._allow_close=True; window.close(); app.processEvents(); backend.close()
