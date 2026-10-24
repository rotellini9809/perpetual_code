"""Small raw-UART JDY-34 diagnostic client. Renaming requires explicit --name."""
import argparse
from pathlib import Path
import time
import uuid

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--scan", action="store_true", help="read-only AT scan: six rates, both PWRC levels, four line endings")
    parser.add_argument("--name", help="explicitly set both module names and read them back (1..18 printable ASCII bytes)")
    parser.add_argument("--reboot", action="store_true", help="explicit AT+RESET after probe/rename, then read names again; drops radio connections")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.name is not None and (not 1 <= len(args.name) <= 18 or args.name.strip(" ") != args.name or
                                 any(not 32 <= ord(c) <= 126 for c in args.name)):
        parser.error("name must contain 1..18 printable ASCII bytes without edge spaces")
    path = args.output or Path(__file__).resolve().parents[1] / ".pio" / f"bluetooth_name_focus_{uuid.uuid4().hex}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as log, serial.Serial(args.port, 115200, timeout=0.1, write_timeout=2) as port:
        pending = bytearray()

        def record(text):
            print(text, flush=True)
            log.write(text + "\n")
            log.flush()

        def wait(prefix, timeout=8):
            lines = []
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                pending.extend(port.read(max(1, port.in_waiting)))
                if len(pending) > 16384:
                    raise RuntimeError("oversized device reply")
                while b"\n" in pending:
                    raw, _, tail = pending.partition(b"\n")
                    pending[:] = tail
                    line = raw.rstrip(b"\r").decode("ascii", errors="backslashreplace")
                    record(line)
                    lines.append(line)
                    if line.startswith(prefix):
                        return lines
            raise RuntimeError(f"no {prefix!r} reply; upload BLUETOOTH_NAME_TEST and close Serial Monitor")

        def command(text, prefix, timeout=8):
            record("> " + text)
            port.write((text + "\n").encode("ascii"))
            port.flush()
            return wait(prefix, timeout)

        def handshake():
            lines = command("at AT", "QUERY_END command=AT ")
            return any(" ack=1 " in line for line in lines if line.startswith("QUERY_END command=AT "))

        # Opening USB CDC can coincide with startup; identify the firmware before
        # sending anything that changes a module setting.
        time.sleep(1)
        command("status", "BT_NAME_TEST status ")
        command("gate low", "GATE ")
        found = False
        if args.scan:
            for eol in ("crlf", "cr", "lf", "none"):
                command("eol " + eol, "EOL ")
                for gate in ("low", "high"):
                    command("gate " + gate, "GATE ")
                    for baud in (9600, 115200, 19200, 38400, 57600, 4800):
                        command(f"baud {baud}", "LOCAL_BAUD ")
                        if handshake():
                            record(f"AT VERIFIED baud={baud} gate={gate} eol={eol}")
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
        else:
            found = handshake()
            if not found:
                record("Initial AT had no valid acknowledgment; retrying this read-only handshake once.")
                found = handshake()

        if found:
            command("probe", "PROBE_END", timeout=10)
            for query in ("AT+LADDR", "AT+STAT", "AT+BTMODE"):
                command("at " + query, "QUERY_END command=" + query + " ")
            if args.name is not None:
                # Quote/backslash escaping matches CommandProtocol's tokenizer.
                escaped = args.name.replace("\\", "\\\\").replace('"', '\\"')
                lines = command(f'name set "{escaped}"', "NAME_RESULT ", timeout=12)
                found = any(line.startswith("NAME_RESULT verified=1 ") for line in lines)
            if args.reboot and found:
                lines = command("module reboot", "MODULE_REBOOT ", timeout=10)
                if not any(line.startswith("MODULE_REBOOT requested=1 ack=1") for line in lines):
                    raise RuntimeError("module reboot was not acknowledged; inspect transcript before retrying")
                if not handshake():
                    raise RuntimeError("module did not answer AT after reboot")
                lines = command("probe", "PROBE_END", timeout=10)
                if args.name is not None:
                    # Require the requested names again after the module reboot.
                    # Raw bytes remain visible; mismatches/truncation never pass.
                    for key in ("NAME", "NAMB"):
                        expected = [(f"+{key}{sep}{args.name}\r\n").encode("ascii").hex(" ").upper()
                                    for sep in ("=", ":")]
                        if not any(line == "RX_HEX " + value for line in lines for value in expected):
                            raise RuntimeError(f"{key} did not read back the requested name after reboot")
        else:
            record("No valid AT acknowledgment. No name setter sent; UART/module path remains unverified.")
            command("eol crlf", "EOL ")
            command("gate low", "GATE ")
            command("baud 9600", "LOCAL_BAUD ")
            command("probe", "PROBE_END", timeout=10)
        command("status", "BT_NAME_TEST status ")
        record(f"Transcript: {path}")
        return 0 if found else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (serial.SerialException, RuntimeError) as error:
        raise SystemExit(str(error)) from error
