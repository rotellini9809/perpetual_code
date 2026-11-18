"""Install the pinned host C++ compiler (WinLibs GCC 14.2, same GCC as the firmware).

Per user, no administrator rights, no PATH changes:
    %LOCALAPPDATA%\\Programs\\WinLibs\\<release>\\mingw64\\bin\\g++.exe
The archive is checked against the pinned SHA-256 before extraction. Host tests and
replay tools pick this compiler automatically once it exists (see host_build.py).
Run again to verify an existing installation; --force reinstalls.
"""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
from host_build import WINLIBS_ASSET, WINLIBS_RELEASE, WINLIBS_SHA256, pinned_root

URL = f'https://github.com/brechtsanders/winlibs_mingw/releases/download/{WINLIBS_RELEASE}/{WINLIBS_ASSET}'


def download(target):
    digest = hashlib.sha256()
    with urllib.request.urlopen(URL, timeout=60) as response, target.open('wb') as output:
        total = int(response.headers.get('Content-Length') or 0); done = 0
        while chunk := response.read(1 << 20):
            output.write(chunk); digest.update(chunk); done += len(chunk)
            if total:
                print(f'\rDownloading {done >> 20}/{total >> 20} MiB', end='', flush=True)
    print()
    if digest.hexdigest() != WINLIBS_SHA256:
        raise RuntimeError(f'SHA-256 mismatch for {WINLIBS_ASSET}: {digest.hexdigest()}')


def extract(archive, destination):
    root = destination.resolve()
    with zipfile.ZipFile(archive) as bundle:
        for member in bundle.infolist():
            path = (root / member.filename).resolve()
            if path != root and root not in path.parents:
                raise RuntimeError(f'Unsafe path in archive: {member.filename}')
        bundle.extractall(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--force', action='store_true', help='Reinstall even if present')
    args = parser.parse_args()
    if os.name != 'nt':
        parser.error('WinLibs is for Windows; elsewhere install GCC 14 and pass --compiler to the tools')
    destination = pinned_root(); compiler = destination / 'mingw64' / 'bin' / 'g++.exe'
    marker = destination / 'INSTALLED.sha256'
    if compiler.is_file() and marker.is_file() and marker.read_text().strip() == WINLIBS_SHA256 and not args.force:
        print(f'Already installed: {compiler}')
    else:
        if destination.exists():
            shutil.rmtree(destination)
        staging = destination.with_name(destination.name + '.partial')
        shutil.rmtree(staging, ignore_errors=True)
        with tempfile.TemporaryDirectory(prefix='winlibs-') as temporary:
            archive = Path(temporary) / WINLIBS_ASSET
            print(f'Fetching {URL}')
            download(archive)
            print(f'SHA-256 verified; extracting to {destination}')
            staging.mkdir(parents=True)
            extract(archive, staging)
        (staging / 'INSTALLED.sha256').write_text(WINLIBS_SHA256 + '\n')
        staging.rename(destination)
    banner = subprocess.check_output([str(compiler), '--version'], text=True).splitlines()[0]
    print(f'Host compiler ready: {banner}')


if __name__ == '__main__':
    main()
