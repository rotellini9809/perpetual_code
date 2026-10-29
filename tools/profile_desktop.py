"""Measure desktop navigation/rendering offline; no device IO or automatic connection."""
import argparse
import cProfile
import json
from math import ceil
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT/'tools/replay')]
_imports_started=time.perf_counter()
from PySide6.QtWidgets import QApplication
_qt_imported=time.perf_counter()
from perpetual_app.controller import Controller
from perpetual_app.flight_data import load_flight
from perpetual_app.window import MainWindow
_app_imported=time.perf_counter()


def summary(values):
    ordered=sorted(values)
    return dict(median_ms=statistics.median(values),p95_ms=ordered[ceil(len(ordered)*.95)-1],max_ms=max(values))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flight',type=Path,help='Optional CSV to include actual recorded plots')
    parser.add_argument('--cycles',type=int,default=20)
    parser.add_argument('--output',type=Path,default=ROOT/'.pio/ui-profile.json')
    parser.add_argument('--cprofile',type=Path,help='Optional Python profile for pstats')
    args=parser.parse_args()
    if args.cycles<2: parser.error('--cycles must be at least 2')
    startup_start=time.perf_counter()
    app=QApplication([]);app.setStyle('Fusion')
    app_created=time.perf_counter()
    controller=Controller(port_scanner=lambda:[]);controller.timer.stop()
    window=MainWindow(controller);window_created=time.perf_counter()
    window.show();app.processEvents();first_events=time.perf_counter()
    startup=dict(qt_import_ms=(_qt_imported-_imports_started)*1000,
                 app_import_ms=(_app_imported-_qt_imported)*1000,
                 application_ms=(app_created-startup_start)*1000,
                 window_and_controller_ms=(window_created-app_created)*1000,
                 show_and_events_ms=(first_events-window_created)*1000)
    csv_ms=None
    try:
        if args.flight:
            start=time.perf_counter();controller.state.flight=load_flight(args.flight)
            csv_ms=(time.perf_counter()-start)*1000;window.render();app.processEvents()
        first=[]
        for index in range(6):
            start=time.perf_counter();window.navigate(index);app.processEvents()
            first.append((time.perf_counter()-start)*1000)
        profiler=cProfile.Profile();profiler.enable();pages=[[] for _ in range(6)]
        for _ in range(args.cycles):
            for index in range(6):
                start=time.perf_counter();window.navigate(index);app.processEvents()
                pages[index].append((time.perf_counter()-start)*1000)
        rendering=[]
        for _ in range(args.cycles):
            start=time.perf_counter();window.render();app.processEvents()
            rendering.append((time.perf_counter()-start)*1000)
        profiler.disable()
        report=dict(mode='Offline, Fusion style, no live serial traffic; processEvents includes painting',
                    startup=startup,csv_load_ms=csv_ms,flight=str(args.flight) if args.flight else None,cycles=args.cycles,
                    first_visit_ms=first,pages={button.text():summary(values) for button,values in zip(window.nav,pages)},
                    state_refresh=summary(rendering))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        if args.cprofile:
            args.cprofile.parent.mkdir(parents=True,exist_ok=True);profiler.dump_stats(str(args.cprofile))
        print(json.dumps(report,indent=2))
    finally:
        window._allow_close=True;window.close()


if __name__=='__main__': main()
