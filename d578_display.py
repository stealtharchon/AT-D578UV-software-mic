"""
BT-01 style remote display for the AnyTone AT-D578UV.

Polls the radio over the "+ADATA" protocol (see d578_status.py) and shows
zone, channel name and frequency for both VFOs on an LCD-style panel.

Standalone:
    python d578_display.py --port COM5 [--interval 2] [--gps]
    python d578_display.py --demo          # replay captured radio replies

Inside the software mic GUI (d578uv-gui.py) the display shares the mic's
serial connection; see DisplayWindow.
"""
import argparse
import os
import queue
import threading
import time
import tkinter as tk

from d578_status import (FrameParser, Q_CHAN_A, Q_CHAN_B, Q_GPS, Q_ZONE_A,
                         Q_ZONE_B, Radio, SharedSerial, frame)

# LCD palette
BG = "#0b1622"
PANEL = "#12212f"
TEXT = "#e6f0fa"
DIM = "#6f8499"
FREQ = "#7fd8ff"
STALE = "#3d4f61"
CHIP_A = "#2f7de1"
CHIP_B = "#d9822b"
TX_ON = "#e0283a"

MONO = ("Consolas", 28, "bold")
NAME = ("Segoe UI", 13, "bold")
SMALL = ("Segoe UI", 9)
CHIP = ("Segoe UI", 11, "bold")


# ---------------------------------------------------------------- polling

class Poller(threading.Thread):
    """
    Background thread that reads radio status every `interval` seconds and
    posts ("status", {...}) or ("error", msg) onto `self.events`.

    get_link() returns the current SharedSerial (or None if the port is
    closed); it is called on every poll because the mic GUI reopens its
    port when settings change.  paused() returning True skips the poll
    (used while the software mic is keyed).
    """

    def __init__(self, get_link, interval=2.0, gps=False, paused=lambda: False):
        super().__init__(daemon=True)
        self.get_link = get_link
        self.interval = interval
        self.paused = paused
        self.queries = [Q_ZONE_A, Q_ZONE_B, Q_CHAN_A, Q_CHAN_B] + ([Q_GPS] if gps else [])
        self.events = queue.Queue()
        self.enabled = threading.Event()
        self.enabled.set()
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()

    def run(self):
        while not self._halt.is_set():
            if self.enabled.is_set():
                self.events.put(self._poll_once())
            self._halt.wait(self.interval)

    def _poll_once(self):
        if self.paused():
            return ("paused", None)
        link = self.get_link()
        if link is None or not link.is_open:
            return ("error", "PORT CLOSED")
        try:
            results = Radio(link=link).read_status(self.queries)
        except TimeoutError:
            return ("error", "NO RESPONSE")
        except Exception as e:                 # port yanked, closed mid-poll...
            return ("error", type(e).__name__.upper())
        status = {}
        for d in results:
            key = d["type"] if d["type"] == "gps" else f"{d['type']}_{d['vfo']}"
            status[key] = d
        if not status:
            return ("error", "NO DATA")
        return ("status", status)


# ---------------------------------------------------------------- widgets

class VfoRow(tk.Frame):
    """One VFO: chip + zone on top, channel name, big frequency."""

    def __init__(self, master, vfo, chip_color):
        super().__init__(master, bg=PANEL, padx=10, pady=8)
        top = tk.Frame(self, bg=PANEL)
        top.pack(fill="x")
        tk.Label(top, text=f" {vfo} ", font=CHIP, bg=chip_color, fg="white").pack(side="left")
        self.mode = tk.Label(top, text="", font=SMALL, bg=PANEL, fg=DIM)
        self.mode.pack(side="right")
        self.zone = tk.Label(top, text="--", font=SMALL, bg=PANEL, fg=DIM, anchor="w")
        self.zone.pack(side="left", padx=8, fill="x", expand=True)
        self.name = tk.Label(self, text="--", font=NAME, bg=PANEL, fg=TEXT, anchor="w")
        self.name.pack(fill="x", pady=(4, 0))
        self.freq = tk.Label(self, text="---.-----", font=MONO, bg=PANEL, fg=FREQ, anchor="e")
        self.freq.pack(fill="x")

    def update_from(self, zone: dict | None, chan: dict | None):
        if zone:
            self.zone.config(text=zone["name"] or "--")
        if chan:
            name = chan["name"]
            is_vfo = name.startswith("Channel VFO")
            self.mode.config(text="VFO" if is_vfo else "MEM")
            self.name.config(text="Frequency mode" if is_vfo else (name or "--"))
            self.freq.config(text=f"{chan['rx_mhz']:.5f}")
        self.set_stale(False)

    def set_stale(self, stale: bool):
        self.name.config(fg=STALE if stale else TEXT)
        self.freq.config(fg=STALE if stale else FREQ)


class LcdPanel(tk.Frame):
    def __init__(self, master, title="AT-D578UV"):
        super().__init__(master, bg=BG, padx=10, pady=10)
        tk.Frame(self, bg=BG, width=320, height=0).pack()     # fixes the width
        head = tk.Frame(self, bg=BG)
        head.pack(fill="x", pady=(0, 8))
        tk.Label(head, text=title, font=SMALL, bg=BG, fg=DIM).pack(side="left")
        self.tx = tk.Label(head, text=" TX ", font=CHIP, bg=BG, fg=BG)
        self.tx.pack(side="right")
        self.rows = {"A": VfoRow(self, "A", CHIP_A), "B": VfoRow(self, "B", CHIP_B)}
        self.rows["A"].pack(fill="x")
        self.rows["B"].pack(fill="x", pady=(8, 0))
        self.gps = tk.Label(self, text="", font=SMALL, bg=BG, fg=DIM, anchor="w",
                            justify="left", wraplength=320)
        self.gps.pack(fill="x", pady=(8, 0))
        self.status = tk.Label(self, text="connecting...", font=SMALL, bg=BG, fg=DIM, anchor="w")
        self.status.pack(fill="x")

    def show_status(self, status: dict):
        for vfo, row in self.rows.items():
            row.update_from(status.get(f"zone_{vfo}"), status.get(f"channel_{vfo}"))
        if "gps" in status:
            self.gps.config(text=f"GPS  {status['gps']['text']}")
        self.status.config(text=f"updated {time.strftime('%H:%M:%S')}", fg=DIM)

    def show_error(self, msg: str):
        for row in self.rows.values():
            row.set_stale(True)
        self.status.config(text=f"{msg}  ({time.strftime('%H:%M:%S')})", fg=TX_ON)

    def set_tx(self, on: bool):
        self.tx.config(bg=TX_ON if on else BG, fg="white" if on else BG)


class DisplayWindow(tk.Toplevel):
    """
    The display as its own window.  Closing it hides it and pauses polling;
    call show() to bring it back.

    Embedding in the software mic GUI:
        lcd = DisplayWindow(window, get_link=lambda: ser, ptt=ptt_active.is_set)
    """

    def __init__(self, master, get_link, interval=2.0, gps=False,
                 ptt=lambda: False, title="AT-D578UV"):
        super().__init__(master, bg=BG)
        self.title("BT-01 Display - AT-D578UV")
        self.resizable(False, False)
        icon = os.path.join(os.path.dirname(os.path.abspath(__file__)), "favicon.ico")
        if os.path.exists(icon):
            try:
                self.iconbitmap(icon)
            except tk.TclError:
                pass
        self.panel = LcdPanel(self, title)
        self.panel.pack(fill="both", expand=True)
        self.ptt = ptt
        self.poller = Poller(get_link, interval, gps, paused=ptt)
        self.poller.start()
        self.protocol("WM_DELETE_WINDOW", self.hide)
        self._pump()

    def _pump(self):
        try:
            while True:
                kind, payload = self.poller.events.get_nowait()
                if kind == "status":
                    self.panel.show_status(payload)
                elif kind == "error":
                    self.panel.show_error(payload)
                elif kind == "paused":
                    self.panel.status.config(text="transmitting - polling paused", fg=DIM)
        except queue.Empty:
            pass
        self.panel.set_tx(self.ptt())
        self.after(100, self._pump)

    def show(self, beside=None):
        if beside is not None:
            beside.update_idletasks()
            x = beside.winfo_rootx() + beside.winfo_width() + 8
            self.geometry(f"+{x}+{beside.winfo_rooty()}")
        self.deiconify()
        self.lift()
        self.poller.enabled.set()

    def hide(self):
        self.poller.enabled.clear()
        self.withdraw()

    def toggle(self, beside=None):
        if self.state() == "withdrawn":
            self.show(beside)
        else:
            self.hide()

    def destroy(self):
        self.poller.stop()
        super().destroy()


# ---------------------------------------------------------------- demo

class DemoSerial:
    """Fake radio that answers with the replies captured in the BT-01 logs."""

    def __init__(self):
        here = os.path.dirname(os.path.abspath(__file__))
        self.replies = {}
        import re
        for log in ("BT-01-startup-squence.log", "BT-01-PTT.log"):
            data = b""
            for line in open(os.path.join(here, log), encoding="utf-8", errors="replace"):
                m = re.search(r"esponse - hex:\s*([0-9a-fA-F ]+)", line)
                if m:
                    data += bytes.fromhex(m.group(1).replace(" ", ""))
            for p in FrameParser().feed(data):
                self.replies.setdefault(p[:2], []).append(p)
        self.parser = FrameParser()
        self.out = b""
        self.timeout = 0.1
        self.is_open = True
        self.n = 0

    @property
    def in_waiting(self):
        return len(self.out)

    def write(self, data):
        for p in self.parser.feed(data):
            if p.startswith(b"\x01D578UV COM MODE"):
                self.out += frame(b"\x03\x01\x00\x00\x04")
            elif p.startswith(b"dCOM CHECK END"):
                self.out += frame(b"\x03\x64\x00\x00\x67")
            elif p[:1] == b"\x04" and p[:2] in self.replies:
                choices = self.replies[p[:2]]
                self.out += frame(choices[self.n % len(choices)])
                if p[1] == Q_CHAN_B:
                    self.n += 1          # alternate between the two captures
        return len(data)

    def read(self, size=1):
        if not self.out:
            time.sleep(self.timeout)
        chunk, self.out = self.out[:size], self.out[size:]
        return chunk

    def reset_input_buffer(self):
        self.out = b""

    def close(self):
        self.is_open = False


def main():
    ap = argparse.ArgumentParser(description="BT-01 style display for the AT-D578UV")
    ap.add_argument("--port", help="serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interval", type=float, default=2.0, help="poll interval, seconds")
    ap.add_argument("--gps", action="store_true", help="also show GPS position")
    ap.add_argument("--demo", action="store_true", help="no radio: replay captured replies")
    a = ap.parse_args()
    if not (a.port or a.demo):
        ap.error("--port or --demo is required")

    if a.demo:
        link = SharedSerial(DemoSerial())
    else:
        import serial
        link = SharedSerial(serial.Serial(a.port, a.baud, timeout=0.1))

    root = tk.Tk()
    root.withdraw()
    win = DisplayWindow(root, get_link=lambda: link, interval=a.interval, gps=a.gps,
                        title="DEMO" if a.demo else a.port)
    win.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()
    link.close()


if __name__ == "__main__":
    main()
