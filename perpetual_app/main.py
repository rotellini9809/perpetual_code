"""Perpetual Windows desktop entry point (source or PyInstaller bundle)."""
from pathlib import Path
from contextlib import contextmanager
import signal
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT/'tools')); sys.path.insert(0,str(ROOT/'tools/replay'))

from PySide6.QtWidgets import QApplication,QMessageBox
from PySide6.QtCore import QTimer
from perpetual_app.window import MainWindow,ASSETS


@contextmanager
def terminal_interrupt(window):
    """Let Qt perform its normal asynchronous close instead of raising KeyboardInterrupt."""
    pending=False
    def interrupt(signum,frame):
        nonlocal pending
        pending=True
    def poll():
        nonlocal pending
        if pending:
            pending=False; window.close()
    previous=signal.signal(signal.SIGINT,interrupt)
    timer=QTimer(window); timer.timeout.connect(poll); timer.start(100)
    try: yield
    finally:
        timer.stop(); timer.deleteLater(); signal.signal(signal.SIGINT,previous)


def main():
    import argparse
    parser=argparse.ArgumentParser(description='Perpetual · Flight Console')
    parser.add_argument('--port',help='Connect explicitly to this USB/SPP port on launch')
    parser.add_argument('--smoke-test',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.smoke_test:
        import os
        os.environ['QT_QPA_PLATFORM']='offscreen'
    app=QApplication(sys.argv); app.setApplicationName('Perpetual'); app.setOrganizationName('Perpetual'); app.setStyle('Fusion')
    from PySide6.QtGui import QIcon
    app.setWindowIcon(QIcon(str(ASSETS/'perpetual.ico')))
    if app.platformName()=='offscreen':
        import os
        from PySide6.QtGui import QFontDatabase
        for name in ('segoeui.ttf','segoeuib.ttf','seguisym.ttf'):
            path=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'/name
            if path.exists(): QFontDatabase.addApplicationFont(str(path))
    def exception(kind,value,traceback):
        import traceback as tb
        text=''.join(tb.format_exception(kind,value,traceback))
        log=Path.home()/'Perpetual-desktop-error.log'
        try: log.write_text(text,encoding='utf-8')
        except OSError: pass
        if args.smoke_test: app.exit(1)
        else: QMessageBox.critical(None,'Perpetual · Errore',str(value)+'\n\nDettagli: '+str(log))
    sys.excepthook=exception
    window=MainWindow(); window.show()
    if args.port: QTimer.singleShot(150,lambda:window.controller.connect_port(args.port))
    elif not args.smoke_test: QTimer.singleShot(0,lambda:window.connect_automatically(quiet=True))
    if args.smoke_test:
        import time
        smoke_deadline=time.monotonic()+25
        def smoke():
            import json
            if args.port and (window.controller.state.foreground or window.controller.state.local is None):
                if window.controller.state.error: raise RuntimeError(window.controller.state.error)
                if time.monotonic()>smoke_deadline: raise TimeoutError('Desktop handshake/sync did not finish')
                QTimer.singleShot(100,smoke); return
            args.smoke_test.mkdir(parents=True,exist_ok=True)
            for i,name in enumerate(('dashboard','configuration','simulation','flights','console')):
                window.navigate(i); app.processEvents()
                if not window.grab().save(str(args.smoke_test/(name+'.png'))): raise OSError('Screenshot non salvato')
            (args.smoke_test/'result.json').write_text(json.dumps(dict(ok=True,pages=5,frozen=bool(getattr(sys,'frozen',False)),
                connected=window.controller.state.connected,transport=window.controller.state.connection.get('transport'))),encoding='utf-8')
            window._closing=True; window.controller.disconnect(); window.close_timer.start()
        QTimer.singleShot(700,smoke)
    with terminal_interrupt(window): return app.exec()


if __name__=='__main__': raise SystemExit(main())
