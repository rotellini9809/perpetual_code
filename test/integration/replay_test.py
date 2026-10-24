"""Replay regression scenarios; run through tools/run_tests.py."""
import csv
import pathlib
import subprocess
import tempfile

def check_hoppino_replay():
    ROOT = pathlib.Path(__file__).resolve().parents[2]
    EXE = ROOT / '.pio/tests/bin/hoppino_replay.exe'

    with tempfile.TemporaryDirectory() as directory:
        folder = pathlib.Path(directory)
        source = folder / 'source.csv'
        output = folder / 'events.csv'

        def run(text, success=True):
            source.write_text(text, encoding='utf-8')
            output.unlink(missing_ok=True)
            result = subprocess.run([str(EXE), str(source), str(output)], capture_output=True, text=True)
            assert (result.returncode == 0) == success, result.stdout + result.stderr
            if not success:
                assert not output.exists(), 'Malformed inputs must not create exports'
            return result

        header = 't_ms,z_est,vz_est,state\n'
        # Reordered columns, CRLF and missing estimate values are accepted.
        run('state,vz_est,t_ms,z_est\r\n0,0,100,0\r\n3,-10,200,50\r\n4,-10,450,nan\r\n5,0,500,0\r\n')
        events = list(csv.DictReader(output.open()))
        starts = [e for e in events if e['event'] == 'output_start']
        assert [(e['output_kind'], e['t_ms']) for e in starts] == [('pyro', '100'), ('servo', '100')]
        assert all(e['mode'] == 'VIRTUAL' for e in events)
        assert [e['phase'] for e in events if e['event'] == 'phase'] == ['PAD', 'DESCENT', 'LANDED']
        original = source.read_bytes()
        result = subprocess.run([str(EXE), str(source), str(source)], capture_output=True)
        assert result.returncode != 0 and source.read_bytes() == original
        existing = output.read_bytes()
        result = subprocess.run([str(EXE), str(source), str(output)], capture_output=True)
        assert result.returncode != 0 and output.read_bytes() == existing
        for text in [
            '', header, 't_ms,z_est,state\n0,0,0\n',
            header + '0,0,0,6\n', header + '0,0,0,0\n0,0,0,1\n',
            header + '10,0,0,0\n9,0,0,1\n', header + '-1,0,0,0\n',
            header + '0,bad,0,0\n', header + '0,0,0\n',
            header + '0,0,0,0extra\n',
        ]:
            run(text, success=False)
        # Outputs start together on the final sample; nothing is deferred beyond EOF.
        result = run(header + '0,0,0,0\n10,50,-10,3\n')
        assert '2 virtual starts' in result.stdout and '0 pending actions cancelled at EOF' in result.stdout

    print('Hoppino replay integration tests passed.')

if __name__ == "__main__":
    check_hoppino_replay()
