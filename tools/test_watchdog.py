"""Run a Python test script with a deadline; on timeout print every thread's stack and exit.

Usage: python -u tools/test_watchdog.py SECONDS SCRIPT [ARGS...]
The script runs as __main__ with its own folder first on sys.path, as when started directly.
"""
import faulthandler
from pathlib import Path
import runpy
import sys


def main():
    seconds, script = float(sys.argv[1]), Path(sys.argv[2]).resolve()
    sys.argv = [str(script), *sys.argv[3:]]
    sys.path[0] = str(script.parent)
    # Runs on a separate C thread, so it also fires while Python waits in a blocking call or modal dialog.
    faulthandler.dump_traceback_later(seconds, exit=True)
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
