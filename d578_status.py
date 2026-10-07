"""
Read live status (zone, channel name, frequency) from an AnyTone AT-D578UV.

Uses the "+ADATA" protocol the BT-01 Bluetooth mic speaks, as captured in
BT-01-startup-squence.log and BT-01-PTT.log.

Frame format (both directions):
    b"+ADATA:00,NNN\r\n" + payload + b"\r\n"      NNN = payload length, decimal
Radio replies end in a checksum byte = sum(payload[:-1]) & 0xFF.

Usage:
    python d578_status.py --port COM4               # read once
    python d578_status.py --port COM4 --watch 2     # re-read every 2 s
    python d578_status.py --port COM4 --raw         # also dump every frame
    python d578_status.py --decode-log BT-01-PTT.log
    python d578_status.py --list-ports
"""
import argparse
import re
import sys
import threading
import time
from contextlib import contextmanager

HEADER = re.compile(rb"\+ADATA:(\d\d),(\d{3})\r\n")

# Query IDs (second byte of an 0x04 request), from the BT-01 capture
Q_ZONE_A, Q_ZONE_B = 0x29, 0x2A
Q_CHAN_A, Q_CHAN_B = 0x2C, 0x2D
Q_GPS = 0x52


# ---------------------------------------------------------------- framing

def frame(payload: bytes) -> bytes:
    return b"+ADATA:00,%03d\r\n" % len(payload) + payload + b"\r\n"


WAKE = frame(b"a")                          # also the keep-alive
COM_MODE = frame(b"\x01D578UV COM MODE")
COM_END = frame(b"dCOM CHECK END")
RELEASE = frame(b"")


def query(qid: int) -> bytes:
    return frame(bytes([0x04, qid, 0x07, 0x00, 0x00, 0x00]))


class FrameParser:
    """Incremental parser: feed() raw bytes, get back complete payloads."""

    def __init__(self):
        self.buf = b""

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            m = HEADER.search(self.buf)
            if not m:
                # keep a tail in case a header is split across reads
                self.buf = self.buf[-16:]
                return out
            n = int(m.group(2))
            end = m.end() + n
            if len(self.buf) < end + 2:
                self.buf = self.buf[m.start():]
                return out
            out.append(self.buf[m.end():end])
            self.buf = self.buf[end + 2:]


def checksum_ok(p: bytes) -> bool:
    return len(p) >= 2 and (sum(p[:-1]) & 0xFF) == p[-1]


# ---------------------------------------------------------------- decoding

def bcd_freq(b: bytes) -> float:
    """4 bytes BCD, units of 10 Hz -> MHz.  14 66 40 00 -> 146.64000"""
    return int(b.hex()) / 100000.0


def cstr(b: bytes) -> str:
    return b.split(b"\x00", 1)[0].decode("ascii", "replace")


def decode(p: bytes) -> dict | None:
    """Decode a radio reply payload into a dict, or None if not understood."""
    if len(p) < 2 or p[0] != 0x04:
        return None
    qid = p[1]
    if qid in (Q_ZONE_A, Q_ZONE_B) and len(p) >= 34:
        return {"type": "zone", "vfo": "A" if qid == Q_ZONE_A else "B",
                "name": cstr(p[2:34])}
    if qid in (Q_CHAN_A, Q_CHAN_B) and len(p) >= 53:
        return {"type": "channel", "vfo": "A" if qid == Q_CHAN_A else "B",
                "rx_mhz": bcd_freq(p[2:6]),
                # memory channel showed 00 06 00 00 (= 0.600 MHz repeater
                # offset); VFOs show other values. Meaning not yet confirmed.
                "field2_mhz": bcd_freq(p[6:10]),
                "name": cstr(p[37:53])}
    if qid == Q_GPS:
        return {"type": "gps", "text": " ".join(
            s.decode("ascii", "replace") for s in p[2:-1].split(b"\x00") if s)}
    return None


def fmt(d: dict) -> str:
    if d["type"] == "zone":
        return f"VFO {d['vfo']}  zone    : {d['name']}"
    if d["type"] == "channel":
        return (f"VFO {d['vfo']}  channel : {d['name']:<16}  "
                f"{d['rx_mhz']:.5f} MHz   (field2 {d['field2_mhz']:.5f})")
    if d["type"] == "gps":
        return f"GPS    : {d['text']}"
    return str(d)


# ---------------------------------------------------------------- radio I/O

class SharedSerial:
    """
    Lets the software mic GUI and the status poller share one serial port.

    Ordinary read()/write()/attribute access passes straight through, so the
    mic code behaves as before and its threads still run concurrently.  A
    status poll runs inside transaction(): it waits for in-flight mic calls
    to finish, then holds them off until the poll is done, so the mic's
    keep-alive reads can't swallow the radio's replies.
    """

    def __init__(self, ser):
        object.__setattr__(self, "raw", ser)
        object.__setattr__(self, "_cond", threading.Condition())
        object.__setattr__(self, "_busy", False)    # transaction active
        object.__setattr__(self, "_active", 0)      # mic calls in flight

    @contextmanager
    def transaction(self):
        with self._cond:
            while self._busy:
                self._cond.wait()
            object.__setattr__(self, "_busy", True)
            while self._active:
                self._cond.wait()
        try:
            yield self.raw
        finally:
            with self._cond:
                object.__setattr__(self, "_busy", False)
                self._cond.notify_all()

    @contextmanager
    def _op(self):
        with self._cond:
            while self._busy:
                self._cond.wait()
            object.__setattr__(self, "_active", self._active + 1)
        try:
            yield
        finally:
            with self._cond:
                object.__setattr__(self, "_active", self._active - 1)
                self._cond.notify_all()

    def write(self, data):
        with self._op():
            return self.raw.write(data)

    def read(self, size=1):
        with self._op():
            return self.raw.read(size)

    def __getattr__(self, name):
        return getattr(self.raw, name)

    def __setattr__(self, name, value):       # e.g. the mic's ser.timeout = 0.1
        with self._op():
            setattr(self.raw, name, value)


class Radio:
    def __init__(self, port: str | None = None, baud: int = 115200,
                 raw: bool = False, link: SharedSerial | None = None):
        if link is None:
            import serial  # pip install pyserial
            link = SharedSerial(serial.Serial(port, baud, timeout=0.1))
        self.link = link
        self.raw = raw

    def close(self):
        self.link.close()

    def _send(self, ser, data: bytes):
        if self.raw:
            print(f"  >> {data.hex(' ')}")
        ser.write(data)

    def _recv(self, ser, parser, want, wait: float) -> bytes | None:
        """Read until a payload satisfying want(payload) arrives, or timeout."""
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            chunk = ser.read(max(1, ser.in_waiting))
            for p in parser.feed(chunk):
                if self.raw:
                    ok = "ok" if checksum_ok(p) else "BAD SUM"
                    print(f"  << [{len(p):3d}] {p.hex(' ')}  ({ok})")
                if want(p):
                    return p
        return None

    def read_status(self, queries=(Q_ZONE_A, Q_ZONE_B, Q_CHAN_A, Q_CHAN_B)) -> list[dict]:
        # Same sequence the BT-01 uses: wake x3, enter COM MODE, query, end.
        with self.link.transaction() as ser:
            saved_timeout, ser.timeout = ser.timeout, 0.02
            try:
                ser.reset_input_buffer()
                parser = FrameParser()
                for _ in range(3):
                    self._send(ser, WAKE)
                    self._recv(ser, parser, lambda p: True, 0.1)
                ack = lambda p: p[:2] == b"\x03\x01"
                self._send(ser, COM_MODE)
                if not self._recv(ser, parser, ack, 0.5):
                    self._send(ser, COM_MODE)       # the BT-01 also sends it twice
                    if not self._recv(ser, parser, ack, 0.5):
                        raise TimeoutError("no reply to COM MODE request")

                results = []
                for q in queries:
                    self._send(ser, query(q))
                    p = self._recv(ser, parser, lambda p: p[:2] == bytes([0x04, q]), 0.6)
                    d = decode(p) if p else None
                    if d:
                        results.append(d)
                self._send(ser, COM_END)
                self._recv(ser, parser, lambda p: p[:2] == b"\x03\x64", 0.3)
                self._send(ser, RELEASE)
                return results
            finally:
                ser.timeout = saved_timeout


# ---------------------------------------------------------------- log replay

def decode_log(path: str):
    """Decode every radio response in one of the captured .log files."""
    hexbytes = b""
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.search(r"esponse - hex:\s*([0-9a-fA-F ]+)", line)
        if m:
            hexbytes += bytes.fromhex(m.group(1).replace(" ", ""))
    for p in FrameParser().feed(hexbytes):
        d = decode(p)
        tag = "ok " if checksum_ok(p) else "BAD"
        print(f"[{tag}] {fmt(d) if d else '(unknown) ' + p[:12].hex(' ')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--watch", type=float, metavar="SECS",
                    help="poll repeatedly with this interval")
    ap.add_argument("--gps", action="store_true", help="also query GPS")
    ap.add_argument("--raw", action="store_true", help="print raw frames")
    ap.add_argument("--decode-log", metavar="FILE")
    ap.add_argument("--list-ports", action="store_true")
    a = ap.parse_args()

    if a.decode_log:
        return decode_log(a.decode_log)
    if a.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device:8} {p.description}")
        return
    if not a.port:
        ap.error("--port is required (try --list-ports)")

    queries = [Q_ZONE_A, Q_ZONE_B, Q_CHAN_A, Q_CHAN_B] + ([Q_GPS] if a.gps else [])
    radio = Radio(a.port, a.baud, a.raw)
    try:
        while True:
            try:
                results = radio.read_status(queries)
                print(time.strftime("%H:%M:%S"))
                for d in results or [{"type": "none"}]:
                    print("  " + (fmt(d) if d["type"] != "none" else "no decodable replies"))
            except TimeoutError as e:
                print(f"{time.strftime('%H:%M:%S')}  {e}", file=sys.stderr)
            if not a.watch:
                break
            time.sleep(a.watch)
    except KeyboardInterrupt:
        pass
    finally:
        radio.close()


if __name__ == "__main__":
    main()
