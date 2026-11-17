"""Assemble the portable ZIP with app source, dependency notices and build versions."""
from importlib import metadata
from pathlib import Path
import json
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
destination=ROOT/'.pio/desktop_dist/Perpetual'
if not (destination/'Perpetual.exe').exists(): raise SystemExit('Build Perpetual.exe first')
shutil.copy2(ROOT/'perpetual_app/PORTABLE.txt',destination/'README.txt')
for folder in ('perpetual_app','tools','tools/replay'):
    (destination/'source'/folder).mkdir(parents=True,exist_ok=True)
for file in (ROOT/'perpetual_app').glob('*.py'): shutil.copy2(file,destination/'source/perpetual_app'/file.name)
for name in ('requirements.txt','Perpetual.spec','build.ps1','PORTABLE.txt'):
    shutil.copy2(ROOT/'perpetual_app'/name,destination/'source/perpetual_app'/name)
shutil.copytree(ROOT/'perpetual_app/assets',destination/'source/perpetual_app/assets',dirs_exist_ok=True)
for file in ('tools/device_client.py','tools/replay/download_flight.py'):
    shutil.copy2(ROOT/file,destination/'source'/file)
notices=destination/'notices'; notices.mkdir(exist_ok=True)
for file in (ROOT/'perpetual_app/notices').glob('*'):
    if file.is_file():
        shutil.copy2(file,notices/file.name)
        (destination/'source/perpetual_app/notices').mkdir(parents=True,exist_ok=True)
        shutil.copy2(file,destination/'source/perpetual_app/notices'/file.name)
versions={}
for name in ('PySide6-Essentials','shiboken6','pyserial','pyinstaller'):
    dist=metadata.distribution(name); versions[name]=dist.version
    (notices/(name+'.txt')).write_text(dist.read_text('METADATA') or name+' '+dist.version,encoding='utf-8')
    for file in dist.files or []:
        if any(key in file.name.lower() for key in ('license','copying','notice')):
            path=Path(dist.locate_file(file))
            if path.is_file(): shutil.copy2(path,notices/(name+'-'+file.name))
(notices/'Python-LICENSE.txt').write_text((Path(sys.base_prefix)/'LICENSE.txt').read_text(encoding='utf-8'),encoding='utf-8')
(notices/'SOURCES.txt').write_text('Qt/PySide6 and Shiboken: https://code.qt.io/cgit/pyside/pyside-setup.git/\n'
    'Qt: https://code.qt.io/cgit/qt/qtbase.git/\n'
    'Qt licenses: https://doc.qt.io/qt-6/licenses-used-in-qt.html\n'
    'Qt for Python licenses: https://doc.qt.io/qtforpython-6/licenses.html\n'
    'pySerial: https://github.com/pyserial/pyserial/tree/v3.5\n'
    'PyInstaller bootloader: https://pyinstaller.org/en/stable/license.html\n'
    'The Qt DLLs are distributed separately in _internal and are not modified.\n',encoding='utf-8')
(destination/'build.json').write_text(json.dumps(dict(prototype=1,python=sys.version,dependencies=versions),indent=2),encoding='utf-8')
archive=shutil.make_archive(str(destination.parent/'Perpetual-Windows'),'zip',destination.parent,'Perpetual')
print(archive)
