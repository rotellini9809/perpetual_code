"""Perpetual Windows desktop entry point (source or PyInstaller bundle)."""
from pathlib import Path
from contextlib import contextmanager
import signal
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT/'tools')); sys.path.insert(0,str(ROOT/'tools/replay'))

from PySide6.QtWidgets import QApplication,QMessageBox
from PySide6.QtCore import QTimer,QSettings,QProcess
from perpetual_app.window import MainWindow,ASSETS
from perpetual_app.i18n import load_language,use_decimal_point


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
            pending=False; window.close_from_terminal()
    previous=signal.signal(signal.SIGINT,interrupt)
    timer=QTimer(window); timer.timeout.connect(poll); timer.start(100)
    try: yield
    finally:
        timer.stop(); timer.deleteLater(); signal.signal(signal.SIGINT,previous)


def relaunch():
    """Start a new instance with the same arguments; it reads the saved language at startup."""
    if getattr(sys,'frozen',False): QProcess.startDetached(sys.executable,sys.argv[1:])
    else: QProcess.startDetached(sys.executable,[str(Path(__file__).resolve()),*sys.argv[1:]])


def main():
    import argparse
    parser=argparse.ArgumentParser(description='Perpetual · Flight Console')
    parser.add_argument('--port',help='Connect explicitly to this USB/SPP port on launch')
    parser.add_argument('--arm-preview',action='store_true',help='Test the ARM hold animation in an isolated preview without connecting to the board')
    parser.add_argument('--smoke-test',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.arm_preview and (args.port or args.smoke_test): parser.error('--arm-preview runs standalone; omit --port and --smoke-test')
    if args.smoke_test:
        import os
        os.environ['QT_QPA_PLATFORM']='offscreen'
    app=QApplication(sys.argv); app.setApplicationName('Perpetual'); app.setOrganizationName('Perpetual'); app.setStyle('Fusion')
    from PySide6.QtGui import QIcon,QPalette,QColor
    palette=QPalette()
    for role,color in ((QPalette.Window,'#f4f6f8'),(QPalette.WindowText,'#213044'),
            (QPalette.Base,'#ffffff'),(QPalette.AlternateBase,'#f4f6f8'),(QPalette.Text,'#213044'),
            (QPalette.Button,'#ffffff'),(QPalette.ButtonText,'#213044'),(QPalette.Highlight,'#167c76'),
            (QPalette.HighlightedText,'#ffffff'),(QPalette.ToolTipBase,'#ffffff'),(QPalette.ToolTipText,'#213044'),
            (QPalette.Light,'#ffffff'),(QPalette.Midlight,'#e7ecf0'),(QPalette.Mid,'#b7c6d4'),
            (QPalette.Dark,'#758699'),(QPalette.Shadow,'#213044'),(QPalette.PlaceholderText,'#7b8998')):
        palette.setColor(role,QColor(color))
    for role in (QPalette.WindowText,QPalette.Text,QPalette.ButtonText):
        palette.setColor(QPalette.Disabled,role,QColor('#a3aeb9'))
    app.setPalette(palette)
    app.setWindowIcon(QIcon(str(ASSETS/'perpetual.ico')))
    load_language(QSettings('Perpetual','Flight Console')); use_decimal_point() # Before any window builds its texts.
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
        else: QMessageBox.critical(None,'Perpetual · Error',str(value)+'\n\nDetails: '+str(log))
    sys.excepthook=exception
    if args.arm_preview:
        from perpetual_app.arm_button import ArmButtonPreview
        window=ArmButtonPreview(); window.show()
        with terminal_interrupt(window): return app.exec()
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
            for i,name in enumerate(('dashboard','configuration','simulation','flights','additional-settings','console')):
                window.navigate(i); app.processEvents()
                if not window.grab().save(str(args.smoke_test/(name+'.png'))): raise OSError('Screenshot not saved')
            (args.smoke_test/'result.json').write_text(json.dumps(dict(ok=True,pages=6,frozen=bool(getattr(sys,'frozen',False)),
                connected=window.controller.state.connected,transport=window.controller.state.connection.get('transport'))),encoding='utf-8')
            window._closing=True; window.controller.disconnect(); window.close_timer.start()
        QTimer.singleShot(700,smoke)
    with terminal_interrupt(window): code=app.exec()
    if window.restart_requested: relaunch()
    return code


if __name__=='__main__': raise SystemExit(main())
