# Build on Windows: python -m PyInstaller perpetual_app/Perpetual.spec
from pathlib import Path

root=Path(SPECPATH).parent
a=Analysis([str(root/'perpetual_app/main.py')],
    pathex=[str(root),str(root/'tools'),str(root/'tools/replay')],
    binaries=[],datas=[(str(root/'perpetual_app/assets'),'perpetual_app/assets')],hiddenimports=['serial.tools.list_ports_windows','download_flight','flight_log'],
    hookspath=[],hooksconfig={},runtime_hooks=[],
    excludes=['PySide6.QtWebEngineCore','PySide6.QtWebEngineWidgets','PySide6.QtQml','PySide6.QtQuick','PySide6.QtCharts'],
    noarchive=False)
pyz=PYZ(a.pure)
exe=EXE(pyz,a.scripts,[],exclude_binaries=True,name='Perpetual',debug=False,
    bootloader_ignore_signals=False,strip=False,upx=False,console=False,icon=str(root/'perpetual_app/assets/perpetual.ico'))
coll=COLLECT(exe,a.binaries,a.datas,strip=False,upx=False,name='Perpetual')
