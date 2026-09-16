#!/usr/bin/env python3
# Part of the ODKI VBT Firmware
# Copyright (C) 2026 Lodovico Cortelazzo
#
# Licensed under the GNU General Public License, Version 3 (this
# specific version only, not "or any later version"), modified by the
# Commons Clause License Condition v1.0 -- see LICENSE-FIRMWARE in the
# repository root for the full text of both. In short: you may use,
# study, modify, and share this file (including a modified version) for
# non-commercial purposes; you may not sell it, or a product/service
# substantially derived from it, without a separate agreement with the
# copyright holder.
#
# This program is distributed WITHOUT ANY WARRANTY, without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
# PURPOSE -- see LICENSE-FIRMWARE for details.

"""
VBT ODKI - live serial monitor
===============================

A small desktop GUI that opens the device's USB-serial port and plots the
debug log (the "S,../R,../B,../P,.." lines - see logSampleCsv()/
reportPhaseScore()/closeBracketFn()/closePhaseFn() in MotionTracker.cpp) as
it streams in, in real time: raw and/or corrected velocity (toggle between
the two, or both at once), acceleration, the current phase (idle/
eccentric/concentric) and rep number, and every completed rep as it's
scored. The colors and dark theme deliberately mirror the phone app's own
live chart (see stream_line_chart.dart / app_colors.dart in the ODKI-VBT
app) - same palette, same phase colors, same raw-vs-corrected convention
(one hue, corrected drawn solid, raw drawn faint).

Not a purely passive reader: the Calibrate/Start/Stop buttons send the
plain-text `CALIBRATE`/`START`/`STOP` serial commands added in firmware
v3.11.21/v3.11.22 (`pollSerialCommands()` in the .ino), so the whole
tracking lifecycle can be driven from here without the phone app. As of
firmware v3.11.24, opening this tool's serial connection is ALSO what
turns the raw "S,../R,.." log on in the first place - it used to be a
separate BLE-only `debugLogEnabled` setting the app had to flip first
(removed; see the version note in MotionTracker.cpp), but the sensor now
prints it automatically to whatever has its serial port open, the same
signal the status LED uses to know a serial client is present, and stops
the moment nothing does - no app step needed at all now for this tool by
itself. CALIBRATE still needs the device physically held still in the
right pose for about 2 seconds, exactly as when triggered from the app;
sending it from here doesn't change that requirement, only where the
button lives. The Delay dropdown next to START (off/5/10/.../30s) is a
self-timer purely local to this tool - it just holds off sending the
`START` command for that long (counting down in the big phase readout;
the button reads "Cancel" and doubles as one while it counts down), so
whoever's about to lift has time to get under the bar; the firmware
itself knows nothing about it and receives a plain immediate `START`
once the count reaches zero, same as with no delay at all. The runtime
Config (rep direction, phase thresholds, …) remains BLE-only - it
already has a typed BLE packet, see BleServer.h, a text command would
just duplicate. Every other line the firmware prints
on serial (battery status, BLE messages, calibration warnings,
RESETREAS...) shows as-is in the console panel, and the tool
resynchronizes automatically on "REC_START"/"REC_STOP".

Usage:
    pip install pyserial numpy matplotlib
    python vbt_live_monitor.py
    python vbt_live_monitor.py --port /dev/cu.usbmodem14201

(Tkinter ships with the standard python.org macOS/Windows installers; on
Debian/Ubuntu install it separately with `sudo apt install python3-tk`.
The Barlow/Barlow Condensed fonts used by the app are not bundled here -
if they happen to be installed on the system Tk picks them up, otherwise
it silently falls back to the platform default.)

Optionally records every raw line to a .log file as it arrives (the
"Record to file" button) - the exact same format `vbt_log_viewer.py`
replays offline, so a live session can be re-analyzed later.

Performance (v3.11.25): the matplotlib redraw - not the 100Hz serial
read/parse, which stays cheap - is what actually costs CPU, so it no
longer runs on a fixed timer. It's skipped entirely whenever nothing new
has arrived (idle/disconnected/paused now draw ZERO frames instead of a
constant ~20fps), and throttled to ~8fps while actively streaming (see
REDRAW_MIN_INTERVAL_S). This is also why the window used to occasionally
stall when resized to fullscreen or when a dropdown was opened: Tkinter
is single-threaded, so a background timer doing expensive Agg rendering
20 times a second was competing with the OS's own resize/popup redraws
on the same thread. SampleBuffer is also capped to the largest
selectable window (60s) rather than 10 minutes of history nothing ever
reads, which kept the per-frame array-conversion cost constant instead
of slowly growing over a long session.
"""

from __future__ import annotations

import argparse
import collections
import queue
import re
import sys
import threading
import time
from datetime import datetime
from typing import TextIO

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Requires pyserial: pip install pyserial")

try:
    import numpy as np
except ImportError:
    sys.exit("Requires numpy: pip install numpy")

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError:
    sys.exit("Requires matplotlib: pip install matplotlib")

try:
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
except ImportError:
    sys.exit("Requires Tkinter (ships with python.org installers; on "
              "Debian/Ubuntu: sudo apt install python3-tk)")


# ----------------------------------------------------------------------
# Serial / protocol constants (firmware v3.11.21 - see the version note
# at the top of MotionTracker.cpp, and pollSerialCommands() in
# VBT_Quaternions.ino for START/STOP). Keep in sync with
# vbt_log_viewer.py and with logSampleCsv()/reportPhaseScore()/
# closeBracketFn()/closePhaseFn() in MotionTracker.cpp if the printed
# format ever changes.
# ----------------------------------------------------------------------

BAUD_RATE = 115200

S_COLUMNS = [
    "t_ms", "dt_ms", "state", "rep",
    "linAccX", "linAccY", "linAccZ",
    "worldAccX", "worldAccY", "worldAccZ",
    "velZ", "refVelZ", "velZLive",
    "repCalibRate", "repCalibIntercept", "repCalibCount",
    "worldVelX", "worldVelY", "worldPosZ", "worldPosX", "worldPosY",
    "quatW", "quatX", "quatY", "quatZ",
    "gyroMag", "accZBias",
    "gyroBiasX", "gyroBiasY", "gyroBiasZ",
    "gyroFilteredX", "gyroFilteredY", "gyroFilteredZ",
    "velClamped", "bracketClosed", "confirmedStillNow", "flatGuardOverrideFired",
]

R_COLUMNS = [
    "rep", "peakVelocity", "meanVelocity", "peakAcceleration", "meanAcceleration",
    "displacementM", "eccPeakVelocity", "eccMeanVelocity", "quality1", "correctionStatus",
]

B_COLUMNS = ["bracketId", "t0", "t1", "baseline", "drift", "rate", "emaRateAfter", "basisTotal"]

P_COLUMNS = ["phaseId", "repNumber", "phaseType", "closeReason", "durationS",
             "sampleCount", "discarded", "reported"]

# RuntimeConfig defaults (MotionTracker.h) - shown as a reference line only:
# the actual live-configured value isn't readable over serial (config is
# BLE-only, see BleServer.cpp), so this is a guide, not a live readout.
DEFAULT_FLAT_GUARD_CEILING = 0.20

BATTERY_RE = re.compile(r"Battery:\s*([\d.]+)\s*V\s*\((\d+)%\)(.*)")


# ----------------------------------------------------------------------
# Palette - lifted from the phone app (lib/ui/theme/app_colors.dart /
# app_theme.dart in the ODKI-VBT-app repo) so this tool reads as the same
# instrument, not a separate one: same dark "arena at night" surfaces,
# same phase colors (concentric=electricBlue, eccentric=emberOrange,
# idle=disconnectedGrey), same raw-vs-corrected convention (ONE hue for
# velocity, corrected drawn solid, raw drawn faint - the app distinguishes
# them by opacity on a single trace, not by a second color; this tool
# keeps that convention even though it can show both traces at once,
# which the app's single-metric view never needs to).
# ----------------------------------------------------------------------

BG_VOID = "#0A0A0E"
BG_SURFACE = "#16161C"
BG_SURFACE_RAISED = "#1E1E27"
BORDER_SUBTLE = "#2C2C37"
TEXT_PRIMARY = "#F5F5F7"
TEXT_SECONDARY = "#9A9AA5"

ELECTRIC_BLUE = "#2FE0FF"   # velocity metric / concentric phase
EMBER_ORANGE = "#FF6A00"    # acceleration metric / eccentric phase
VOLT_LIME = "#CCFF00"       # brand accent - selected/highlighted points, primary actions
DISCONNECTED_GREY = "#6E6E78"  # idle phase
ALERT_RED = "#FF3B5C"       # not an app color (the app has no concept of a discarded phase) -
                             # picked to sit clearly apart from the three colors above

STATE_COLOR = {"idle": DISCONNECTED_GREY, "eccentric": EMBER_ORANGE, "concentric": ELECTRIC_BLUE}
STATE_TEXT_COLOR = {"idle": TEXT_PRIMARY, "eccentric": TEXT_PRIMARY, "concentric": BG_VOID}
STATE_LABEL = {"idle": "IDLE", "eccentric": "ECCENTRIC", "concentric": "CONCENTRIC"}
STATUS_LABEL = {0: "provisional", 1: "calibrated", 2: "corrected"}

FONT_DISPLAY = "Barlow Condensed"  # falls back to the platform default if not installed
FONT_BODY = "Barlow"
FONT_MONO = "Menlo"

RAW_ALPHA = 0.40
LIVE_ALPHA = 1.0

CHART_MODES = [("live", "velZLive"), ("raw", "velZ raw"), ("both", "Both")]

# Start-delay choices (seconds) - a self-timer so whoever's about to lift
# has time to get under the bar before START actually fires. "No delay"
# keeps the previous immediate behavior.
START_DELAY_CHOICES = [("No delay", 0), ("5 s", 5), ("10 s", 10), ("15 s", 15),
                        ("20 s", 20), ("25 s", 25), ("30 s", 30)]


# ----------------------------------------------------------------------
# Line parsing - one generic function for all four row types, since only
# their column list differs (all-numeric except S's "state" field).
# ----------------------------------------------------------------------

def parse_row(line: str, prefix: str, columns: list[str]) -> dict[str, float | str] | None:
    if not line.startswith(prefix + ","):
        return None
    parts = line.split(",")[1:]
    if len(parts) != len(columns):
        return None
    row: dict[str, float | str] = {}
    for name, raw in zip(columns, parts):
        if name == "state":
            row[name] = raw
            continue
        try:
            row[name] = float(raw)
        except ValueError:
            return None  # the one-time header line the firmware prints - not real data
    return row


# ----------------------------------------------------------------------
# Serial reader - runs on its own thread, only ever pushes onto a Queue.
# Never touches Tkinter widgets directly (not thread-safe). send_command()
# is called from the main (GUI) thread while this thread is blocked in
# readline() - writing and reading a pyserial handle from two threads
# like this is the standard, safe pattern (they don't share mutable state).
# ----------------------------------------------------------------------

class SerialReaderThread(threading.Thread):
    def __init__(self, port: str, baud: int, out_queue: "queue.Queue[tuple[str, object]]") -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self._stop = threading.Event()
        self.ser: serial.Serial | None = None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:
            self.out_queue.put(("error", str(exc)))
            return
        self.out_queue.put(("connected", self.port))
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.out_queue.put(("error", str(exc)))
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if line:
                self.out_queue.put(("line", line))
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.out_queue.put(("disconnected", None))

    def send_command(self, text: str) -> None:
        """START/STOP/CALIBRATE - see pollSerialCommands() in
        VBT_Quaternions.ino (v3.11.21/v3.11.22). Newline-terminated plain
        text, case-insensitive on the firmware side."""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write((text + "\n").encode("ascii"))
        except Exception as exc:
            self.out_queue.put(("error", f"failed to send command: {exc}"))

    def stop(self) -> None:
        self._stop.set()


# ----------------------------------------------------------------------
# Rolling, time-windowed sample buffer. Sized to the largest selectable
# window (see WINDOW_CHOICES) so switching the dropdown never needs a
# re-fetch - the draw step just slices the tail that falls inside the
# chosen window.
# ----------------------------------------------------------------------

class SampleBuffer:
    # v3.11.25: was 60_000 (10 min at 100Hz) - "generous", but nothing ever
    # reads more than the current window (60s at most, see WINDOW_CHOICES),
    # so the extra history just sat there making window_arrays() below
    # progressively more expensive as a session ran longer: np.fromiter()
    # over the WHOLE deque runs on every redraw, not just once, so a
    # 60,000-sample deque meant converting 60,000 floats (x4 arrays) tens
    # of times a second once a long session had filled it - real, escalating
    # CPU/battery cost for history the chart can never actually show.
    # Capped to exactly the largest window instead: this array conversion
    # now costs microseconds regardless of how long the tool has been open.
    MAXLEN = 6_000  # 60s at 100Hz - matches the largest WINDOW_CHOICES entry

    def __init__(self) -> None:
        self.t: collections.deque[float] = collections.deque(maxlen=self.MAXLEN)
        self.vel_raw: collections.deque[float] = collections.deque(maxlen=self.MAXLEN)
        self.vel_live: collections.deque[float] = collections.deque(maxlen=self.MAXLEN)
        self.acc: collections.deque[float] = collections.deque(maxlen=self.MAXLEN)
        self.state: collections.deque[str] = collections.deque(maxlen=self.MAXLEN)
        self.rep: collections.deque[int] = collections.deque(maxlen=self.MAXLEN)
        self.session_t0: float | None = None  # first sample's t_ms, to build a relative clock
        self._prev_still = 0
        self.still_events: list[float] = []               # rising edges of confirmedStillNow
        self.rep_events: list[tuple[float, int, float, int, int]] = []       # from "R," rows
        self.discard_events: list[tuple[float, int, float]] = []             # "P," rows, reported==0

    def clear(self) -> None:
        self.__init__()

    def add_sample(self, row: dict) -> float:
        if self.session_t0 is None:
            self.session_t0 = row["t_ms"]
        t = (row["t_ms"] - self.session_t0) / 1000.0
        self.t.append(t)
        self.vel_raw.append(row["velZ"])
        self.vel_live.append(row["velZLive"])
        self.acc.append(row["worldAccZ"])
        self.state.append(row["state"])
        self.rep.append(int(row["rep"]))
        still = int(row["confirmedStillNow"])
        if still == 1 and self._prev_still == 0:
            self.still_events.append(t)
        self._prev_still = still
        return t

    def add_rep(self, row: dict, t_now: float) -> None:
        self.rep_events.append((t_now, int(row["rep"]), row["peakVelocity"],
                                 int(row["quality1"]), int(row["correctionStatus"])))

    def add_discarded_phase(self, row: dict, t_now: float) -> None:
        if int(row["reported"]) == 0:
            self.discard_events.append((t_now, int(row["phaseType"]), row["durationS"]))

    def latest_t(self) -> float:
        return self.t[-1] if self.t else 0.0

    def window_arrays(self, window_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self.t:
            return (np.array([]),) * 4
        t_arr = np.fromiter(self.t, dtype=float)
        lo = t_arr[-1] - window_s
        idx = np.searchsorted(t_arr, lo)
        return (t_arr[idx:], np.fromiter(self.vel_raw, dtype=float)[idx:],
                np.fromiter(self.vel_live, dtype=float)[idx:],
                np.fromiter(self.acc, dtype=float)[idx:])


# ----------------------------------------------------------------------
# Main application
# ----------------------------------------------------------------------

class LiveMonitorApp(tk.Tk):
    POLL_MS = 50               # queue-drain tick - cheap (deque/array writes only), stays fast
    # v3.11.25: the actual matplotlib redraw is the expensive part (Agg
    # rendering of thousands of points across 2 subplots) - throttled
    # separately from POLL_MS, and skipped entirely when nothing changed
    # (see _dirty below). Redrawing on a fixed 20fps timer regardless of
    # whether there was anything new to show - including while idle,
    # disconnected, or paused - was burning CPU (and, per a user report,
    # visibly heating the laptop) for no visual benefit, and competed with
    # Tk's own event loop badly enough to cause real stalls when a combobox
    # popup opened or the window entered fullscreen (both drive Tk's own
    # animation/redraw on the same single thread). ~8fps is still smooth
    # for a scrolling strip-chart and roughly a third of the previous draw
    # load.
    REDRAW_MIN_INTERVAL_S = 0.12
    WINDOW_CHOICES = [("10 s", 10), ("20 s", 20), ("30 s", 30), ("60 s", 60)]

    def __init__(self, initial_port: str | None = None) -> None:
        super().__init__()
        self.title("ODKI VBT - Live Monitor")
        self.geometry("1320x820")
        self.minsize(1000, 660)
        self.configure(bg=BG_VOID)

        self.reader: SerialReaderThread | None = None
        self.line_queue: queue.Queue = queue.Queue()
        self.buf = SampleBuffer()
        self.window_s = tk.IntVar(value=20)
        self.chart_mode = tk.StringVar(value="both")
        self.paused = False
        self.record_file: TextIO | None = None
        self.recording = False
        self.sample_count = 0
        self.connected_port: str | None = None
        self._countdown_job = None      # Tk `after` id for the pending countdown tick, or None
        self._countdown_remaining = 0
        self._dirty = False          # set True whenever something arrives that the chart should reflect
        self._last_redraw_t = 0.0    # time.monotonic() of the last actual canvas.draw_idle() call

        self._build_style()
        self._build_ui()
        self._refresh_ports()
        if initial_port:
            self.port_var.set(initial_port)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.POLL_MS, self._poll_queue)

    # ---------------- style / theme ----------------

    def _build_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".", background=BG_VOID, foreground=TEXT_PRIMARY,
                         fieldbackground=BG_SURFACE, font=(FONT_BODY, 10))
        style.configure("TFrame", background=BG_VOID)
        style.configure("Surface.TFrame", background=BG_SURFACE)
        style.configure("TLabel", background=BG_VOID, foreground=TEXT_PRIMARY)
        style.configure("Muted.TLabel", background=BG_VOID, foreground=TEXT_SECONDARY, font=(FONT_BODY, 9))
        style.configure("Heading.TLabel", background=BG_VOID, foreground=TEXT_PRIMARY,
                         font=(FONT_DISPLAY, 13, "bold"))

        style.configure("TButton", background=BG_SURFACE_RAISED, foreground=TEXT_PRIMARY,
                         borderwidth=0, padding=6, font=(FONT_BODY, 10))
        style.map("TButton", background=[("active", BORDER_SUBTLE), ("disabled", BG_SURFACE)],
                  foreground=[("disabled", TEXT_SECONDARY)])
        style.configure("Calibrate.TButton", background=BG_SURFACE_RAISED, foreground=ELECTRIC_BLUE,
                         font=(FONT_BODY, 10, "bold"))
        style.map("Calibrate.TButton", background=[("active", BORDER_SUBTLE), ("disabled", BG_SURFACE)],
                  foreground=[("disabled", TEXT_SECONDARY)])
        style.configure("Start.TButton", background=VOLT_LIME, foreground=BG_VOID, font=(FONT_BODY, 10, "bold"))
        style.map("Start.TButton", background=[("active", "#b8e600"), ("disabled", BG_SURFACE_RAISED)],
                  foreground=[("disabled", TEXT_SECONDARY)])
        style.configure("Stop.TButton", background=ALERT_RED, foreground="#ffffff", font=(FONT_BODY, 10, "bold"))
        style.map("Stop.TButton", background=[("active", "#d92a49"), ("disabled", BG_SURFACE_RAISED)],
                  foreground=[("disabled", TEXT_SECONDARY)])

        style.configure("TCombobox", fieldbackground=BG_SURFACE, background=BG_SURFACE,
                         foreground=TEXT_PRIMARY, arrowcolor=TEXT_SECONDARY)
        style.map("TCombobox", fieldbackground=[("readonly", BG_SURFACE)])
        self.option_add("*TCombobox*Listbox.background", BG_SURFACE)
        self.option_add("*TCombobox*Listbox.foreground", TEXT_PRIMARY)
        self.option_add("*TCombobox*Listbox.selectBackground", BORDER_SUBTLE)

        style.configure("TRadiobutton", background=BG_VOID, foreground=TEXT_PRIMARY, font=(FONT_BODY, 10))
        style.map("TRadiobutton", background=[("active", BG_VOID)], foreground=[("selected", ELECTRIC_BLUE)])

        style.configure("TPanedwindow", background=BG_VOID)
        style.configure("TScrollbar", background=BG_SURFACE_RAISED, troughcolor=BG_VOID,
                         bordercolor=BG_VOID, arrowcolor=TEXT_SECONDARY)

        style.configure("Treeview", background=BG_SURFACE, fieldbackground=BG_SURFACE,
                         foreground=TEXT_PRIMARY, rowheight=22, borderwidth=0, font=(FONT_MONO, 10))
        style.configure("Treeview.Heading", background=BG_SURFACE_RAISED, foreground=TEXT_SECONDARY,
                         borderwidth=0, font=(FONT_BODY, 9, "bold"))
        style.map("Treeview", background=[("selected", BORDER_SUBTLE)], foreground=[("selected", TEXT_PRIMARY)])

    # ---------------- UI construction ----------------

    def _build_ui(self):
        top = tk.Frame(self, bg=BG_VOID, padx=10, pady=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Serial port:", style="Muted.TLabel").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=26, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(4, 4))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=(10, 4))

        self.status_dot = tk.Canvas(top, width=14, height=14, bg=BG_VOID, highlightthickness=0)
        self.status_dot.pack(side=tk.LEFT, padx=(8, 4))
        self._dot = self.status_dot.create_oval(2, 2, 12, 12, fill=DISCONNECTED_GREY, outline="")
        self.status_label = ttk.Label(top, text="disconnected", style="Muted.TLabel")
        self.status_label.pack(side=tk.LEFT)

        # --- device control: START/STOP over serial (firmware v3.11.21) ---
        self.calibrate_btn = ttk.Button(top, text="CALIBRATE", style="Calibrate.TButton",
                                         command=self._send_calibrate, state="disabled")
        self.calibrate_btn.pack(side=tk.LEFT, padx=(20, 3))
        self.start_btn = ttk.Button(top, text="▶ START", style="Start.TButton",
                                     command=self._send_start, state="disabled")
        self.start_btn.pack(side=tk.LEFT, padx=3)

        ttk.Label(top, text="Delay:", style="Muted.TLabel").pack(side=tk.LEFT, padx=(8, 2))
        self.start_delay_var = tk.StringVar(value=START_DELAY_CHOICES[0][0])
        self.start_delay_combo = ttk.Combobox(top, textvariable=self.start_delay_var, state="disabled",
                                               width=8, values=[label for label, _s in START_DELAY_CHOICES])
        self.start_delay_combo.pack(side=tk.LEFT, padx=(3, 3))

        self.stop_btn = ttk.Button(top, text="■ STOP", style="Stop.TButton",
                                    command=self._send_stop, state="disabled")
        self.stop_btn.pack(side=tk.LEFT, padx=3)

        self.battery_label = ttk.Label(top, text="", style="Muted.TLabel")
        self.battery_label.pack(side=tk.RIGHT, padx=4)

        # --- second toolbar row: chart controls ---
        toolbar2 = tk.Frame(self, bg=BG_VOID, padx=10, pady=3)
        toolbar2.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(toolbar2, text="Trace:", style="Muted.TLabel").pack(side=tk.LEFT)
        for value, label in CHART_MODES:
            ttk.Radiobutton(toolbar2, text=label, value=value, variable=self.chart_mode,
                             command=self._apply_chart_mode).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Label(toolbar2, text="Window:", style="Muted.TLabel").pack(side=tk.LEFT, padx=(24, 4))
        win_combo = ttk.Combobox(toolbar2, textvariable=self.window_s, state="readonly", width=6,
                                  values=[str(v) for _, v in self.WINDOW_CHOICES])
        win_combo.set(str(self.window_s.get()))
        win_combo.pack(side=tk.LEFT)
        win_combo.bind("<<ComboboxSelected>>", lambda e: self.window_s.set(int(win_combo.get())))

        self.pause_btn = ttk.Button(toolbar2, text="Pause", command=self._toggle_pause)
        self.pause_btn.pack(side=tk.LEFT, padx=(20, 4))
        ttk.Button(toolbar2, text="Clear", command=self._clear_all).pack(side=tk.LEFT, padx=4)
        self.record_btn = ttk.Button(toolbar2, text="Record to file", command=self._toggle_recording)
        self.record_btn.pack(side=tk.LEFT, padx=(12, 4))

        # ---- status header: big phase/rep readout + live numbers ----
        header = tk.Frame(self, bg=STATE_COLOR["idle"], height=68)
        header.pack(side=tk.TOP, fill=tk.X)
        header.pack_propagate(False)
        self.header_frame = header
        self.phase_label = tk.Label(header, text="WAITING FOR DATA", font=(FONT_DISPLAY, 26, "bold"),
                                     bg=STATE_COLOR["idle"], fg=STATE_TEXT_COLOR["idle"])
        self.phase_label.pack(side=tk.LEFT, padx=22)
        self.still_label = tk.Label(header, text="", font=(FONT_BODY, 12),
                                     bg=STATE_COLOR["idle"], fg=STATE_TEXT_COLOR["idle"])
        self.still_label.pack(side=tk.LEFT, padx=10)

        self.readout_frame = tk.Frame(header, bg=STATE_COLOR["idle"])
        self.readout_frame.place(relx=1.0, rely=0.5, anchor="e", x=-20)
        self.readout_vars = {}
        self.readout_widgets = []
        for i, key in enumerate(["velZ", "velZLive", "drift", "worldAccZ", "gyroMag", "repCalibCount"]):
            var = tk.StringVar(value="--")
            self.readout_vars[key] = var
            cell = tk.Frame(self.readout_frame, bg=STATE_COLOR["idle"])
            cell.grid(row=0, column=i, padx=10)
            lbl_key = tk.Label(cell, text=key, font=(FONT_MONO, 9), bg=STATE_COLOR["idle"],
                                fg=STATE_TEXT_COLOR["idle"])
            lbl_key.pack()
            lbl_val = tk.Label(cell, textvariable=var, font=(FONT_MONO, 14, "bold"),
                                bg=STATE_COLOR["idle"], fg=STATE_TEXT_COLOR["idle"])
            lbl_val.pack()
            self.readout_widgets.append((cell, lbl_key, lbl_val))

        # ---- main split: charts (left) | reps + console (right) ----
        paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        paned.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        charts_frame = ttk.Frame(paned)
        paned.add(charts_frame, weight=4)
        self._build_charts(charts_frame)

        side_frame = ttk.Frame(paned)
        paned.add(side_frame, weight=1)
        self._build_side_panel(side_frame)

        bottom = tk.Frame(self, bg=BG_VOID, padx=10, pady=4)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        self.sample_count_label = ttk.Label(bottom, text="0 samples", style="Muted.TLabel")
        self.sample_count_label.pack(side=tk.LEFT)
        self.record_status_label = tk.Label(bottom, text="", bg=BG_VOID, fg=ALERT_RED, font=(FONT_BODY, 9))
        self.record_status_label.pack(side=tk.RIGHT)

    def _build_charts(self, parent):
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor=BG_VOID)
        self.ax_vel = self.fig.add_subplot(2, 1, 1, facecolor=BG_SURFACE)
        self.ax_acc = self.fig.add_subplot(2, 1, 2, facecolor=BG_SURFACE, sharex=self.ax_vel)

        for ax in (self.ax_vel, self.ax_acc):
            ax.tick_params(colors=TEXT_SECONDARY, labelsize=9)
            for spine in ax.spines.values():
                spine.set_color(BORDER_SUBTLE)
            ax.grid(True, color=BORDER_SUBTLE, linewidth=0.7, alpha=0.7)
            ax.yaxis.label.set_color(TEXT_SECONDARY)
            ax.xaxis.label.set_color(TEXT_SECONDARY)

        self.ax_vel.set_ylabel("velocity (m/s)")
        self.ax_vel.axhline(0, color=TEXT_SECONDARY, lw=0.8, ls=(0, (4, 3)))
        self.ceiling_lines = [
            self.ax_vel.axhline(DEFAULT_FLAT_GUARD_CEILING, color=BORDER_SUBTLE, lw=1.0, ls="--"),
            self.ax_vel.axhline(-DEFAULT_FLAT_GUARD_CEILING, color=BORDER_SUBTLE, lw=1.0, ls="--"),
        ]
        # same hue for raw/corrected velocity as the app - distinguished by
        # opacity (raw faint, corrected solid), not by a second color
        self.line_raw, = self.ax_vel.plot([], [], color=ELECTRIC_BLUE, lw=1.5, alpha=RAW_ALPHA, label="velZ raw")
        self.line_live, = self.ax_vel.plot([], [], color=ELECTRIC_BLUE, lw=1.8, alpha=LIVE_ALPHA, label="velZLive")
        self.still_scatter = self.ax_vel.scatter([], [], color=VOLT_LIME, marker="o", s=20,
                                                  zorder=5, label="stillness confirmed")
        self.discard_scatter = self.ax_vel.scatter([], [], color=ALERT_RED, marker="|", s=150,
                                                     zorder=5, label="discarded phase")
        self._build_vel_legend()

        self.ax_acc.set_ylabel("accel Z (m/s²)")
        self.ax_acc.set_xlabel("time (s)")
        self.ax_acc.axhline(0, color=TEXT_SECONDARY, lw=0.8, ls=(0, (4, 3)))
        self.line_acc, = self.ax_acc.plot([], [], color=EMBER_ORANGE, lw=1.3, label="worldAccZ")
        leg2 = self.ax_acc.legend(loc="upper left", fontsize=8, facecolor=BG_SURFACE_RAISED,
                                   edgecolor=BORDER_SUBTLE, labelcolor=TEXT_SECONDARY)
        leg2.get_frame().set_alpha(0.9)

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().configure(bg=BG_VOID, highlightthickness=0)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._rep_markers = []       # [(vline, text), ...] currently drawn for visible rep boundaries
        self._rep_markers_key = None  # last-drawn (t, rep) tuple set - skip rebuild when unchanged

    def _build_vel_legend(self):
        handles = [h for h in (self.line_raw, self.line_live, self.still_scatter, self.discard_scatter)
                   if h.get_visible()]
        leg = self.ax_vel.legend(handles=handles, loc="upper left", fontsize=8, ncol=4,
                                  facecolor=BG_SURFACE_RAISED, edgecolor=BORDER_SUBTLE, labelcolor=TEXT_SECONDARY)
        leg.get_frame().set_alpha(0.9)

    def _build_side_panel(self, parent):
        ttk.Label(parent, text="Completed reps", style="Heading.TLabel").pack(
            anchor="w", padx=8, pady=(8, 2))
        cols = ("rep", "peak", "mean", "qual", "status")
        self.rep_tree = ttk.Treeview(parent, columns=cols, show="headings", height=12)
        widths = (40, 60, 60, 45, 80)
        for c, w in zip(cols, widths):
            self.rep_tree.heading(c, text=c)
            self.rep_tree.column(c, width=w, anchor="center")
        self.rep_tree.pack(fill=tk.BOTH, expand=False, padx=8)

        ttk.Label(parent, text="Device console", style="Heading.TLabel").pack(
            anchor="w", padx=8, pady=(12, 2))
        console_frame = tk.Frame(parent, bg=BG_VOID)
        console_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.console = tk.Text(console_frame, height=10, font=(FONT_MONO, 10), wrap="word",
                                state="disabled", bg=BG_SURFACE, fg=TEXT_SECONDARY,
                                insertbackground=TEXT_PRIMARY, borderwidth=0, highlightthickness=0)
        scroll = ttk.Scrollbar(console_frame, command=self.console.yview)
        self.console.configure(yscrollcommand=scroll.set)
        self.console.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

    # ---------------- serial connection ----------------

    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.reader is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Missing port", "Select a serial port before connecting.")
            return
        self._clear_all()
        self.reader = SerialReaderThread(port, BAUD_RATE, self.line_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect")
        self.calibrate_btn.config(state="normal")
        self.start_btn.config(state="normal")
        self.start_delay_combo.config(state="readonly")
        self.stop_btn.config(state="normal")
        self._set_status("connecting...", "#e0a020")

    def _disconnect(self):
        if self._countdown_job is not None:
            self._cancel_countdown("cancelled (disconnected)")
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.connected_port = None
        self.connect_btn.config(text="Connect")
        self.calibrate_btn.config(state="disabled")
        self.start_btn.config(state="disabled")
        self.start_delay_combo.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self._set_status("disconnected", TEXT_SECONDARY)
        self._stop_recording()

    def _set_status(self, text, color):
        self.status_label.config(text=text, foreground=color)
        self.status_dot.itemconfig(self._dot, fill=color)

    # ---------------- device control (serial CALIBRATE/START/STOP, firmware v3.11.21/v3.11.22) ----------------

    def _send_calibrate(self):
        if self.reader is not None:
            if not messagebox.askokcancel(
                    "Calibrate",
                    "Hold the device/barbell completely still in its resting orientation.\n\n"
                    "Calibration takes about 2 seconds and blocks the device - "
                    "nothing else will respond until it finishes."):
                return
            self.reader.send_command("CALIBRATE")
            self._log_console(">>> CALIBRATE sent (hold still ~2s)")

    def _send_start(self):
        if self.reader is None:
            return
        if self._countdown_job is not None:
            # already counting down - the button doubles as Cancel while it runs
            self._cancel_countdown("cancelled")
            return
        delay_label = self.start_delay_var.get()
        delay_s = dict(START_DELAY_CHOICES).get(delay_label, 0)
        if delay_s <= 0:
            self.reader.send_command("START")
            self._log_console(">>> START sent")
        else:
            self._begin_start_countdown(delay_s)

    def _begin_start_countdown(self, seconds):
        self._countdown_remaining = seconds
        self.start_delay_combo.config(state="disabled")
        self._log_console(f">>> START in {seconds}s (press START again to cancel)")
        self._tick_start_countdown()

    def _tick_start_countdown(self):
        if self._countdown_remaining <= 0:
            self._countdown_job = None
            self.start_btn.config(text="▶ START")
            self.start_delay_combo.config(state="readonly")
            self.phase_label.config(text="STARTING…")
            if self.reader is not None:
                self.reader.send_command("START")
                self._log_console(">>> START sent")
            return
        self.start_btn.config(text=f"Cancel ({self._countdown_remaining}s)")
        self.phase_label.config(text=f"STARTING IN {self._countdown_remaining}…", bg=DISCONNECTED_GREY,
                                 fg=STATE_TEXT_COLOR["idle"])
        self.header_frame.config(bg=DISCONNECTED_GREY)
        self._countdown_remaining -= 1
        self._countdown_job = self.after(1000, self._tick_start_countdown)

    def _cancel_countdown(self, reason):
        if self._countdown_job is not None:
            self.after_cancel(self._countdown_job)
            self._countdown_job = None
        self.start_btn.config(text="▶ START")
        self.start_delay_combo.config(state="readonly")
        self.phase_label.config(text="WAITING FOR DATA")
        self._log_console(f">>> START countdown {reason}")

    def _send_stop(self):
        if self._countdown_job is not None:
            self._cancel_countdown("cancelled (STOP pressed)")
        if self.reader is not None:
            self.reader.send_command("STOP")
            self._log_console(">>> STOP sent")

    # ---------------- recording to file ----------------

    def _toggle_recording(self):
        if self.recording:
            self._stop_recording()
        else:
            default_name = f"vbt_capture_{datetime.now():%Y%m%d_%H%M%S}.log"
            path = filedialog.asksaveasfilename(defaultextension=".log", initialfile=default_name,
                                                 filetypes=[("Log file", "*.log"), ("All files", "*.*")])
            if not path:
                return
            try:
                self.record_file = open(path, "w", buffering=1)
            except OSError as exc:
                messagebox.showerror("Error", f"Could not open the file:\n{exc}")
                return
            self.recording = True
            self.record_btn.config(text="Stop recording")
            self.record_status_label.config(text=f"● recording to {path}")

    def _stop_recording(self):
        self.recording = False
        self.record_btn.config(text="Record to file")
        self.record_status_label.config(text="")
        if self.record_file is not None:
            try:
                self.record_file.close()
            except OSError:
                pass
            self.record_file = None

    # ---------------- misc controls ----------------

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    def _apply_chart_mode(self):
        mode = self.chart_mode.get()
        self.line_raw.set_visible(mode in ("raw", "both"))
        self.line_live.set_visible(mode in ("live", "both"))
        self._build_vel_legend()
        self.canvas.draw_idle()

    def _clear_all(self):
        self.buf.clear()
        self.sample_count = 0
        for item in self.rep_tree.get_children():
            self.rep_tree.delete(item)
        for vline, txt in self._rep_markers:
            vline.remove()
            txt.remove()
        self._rep_markers = []
        self._rep_markers_key = None
        self.line_raw.set_data([], [])
        self.line_live.set_data([], [])
        self.line_acc.set_data([], [])
        self.still_scatter.set_offsets(np.empty((0, 2)))
        self.discard_scatter.set_offsets(np.empty((0, 2)))
        self.canvas.draw_idle()
        self._log_console("--- buffer cleared ---")

    def _log_console(self, text):
        self.console.configure(state="normal")
        self.console.insert("end", text + "\n")
        self.console.see("end")
        # cap console length so it never grows unbounded during a long session
        if int(self.console.index("end-1c").split(".")[0]) > 500:
            self.console.delete("1.0", "2.0")
        self.console.configure(state="disabled")

    def _on_close(self):
        self._disconnect()
        self.destroy()

    # ---------------- queue draining / line dispatch ----------------

    def _poll_queue(self):
        drained = 0
        try:
            while True:
                kind, payload = self.line_queue.get_nowait()
                drained += 1
                if kind == "connected":
                    self.connected_port = payload
                    self._set_status(f"connected ({payload}) - waiting for REC_START", ELECTRIC_BLUE)
                elif kind == "error":
                    self._set_status("connection error", ALERT_RED)
                    self._log_console(f"[serial error] {payload}")
                    self._disconnect()
                elif kind == "disconnected":
                    pass
                elif kind == "line":
                    self._handle_line(payload)
                if drained > 4000:  # don't let one GUI tick starve on a huge backlog
                    break
        except queue.Empty:
            pass

        if not self.paused and self._dirty:
            now = time.monotonic()
            if (now - self._last_redraw_t) >= self.REDRAW_MIN_INTERVAL_S:
                self._redraw()
                self._dirty = False
                self._last_redraw_t = now
        self.sample_count_label.config(text=f"{self.sample_count} samples")
        self.after(self.POLL_MS, self._poll_queue)

    def _handle_line(self, line):
        if self.recording and self.record_file is not None:
            self.record_file.write(line + "\n")

        if line == "REC_START":
            self._clear_all()
            self._set_status(f"connected ({self.connected_port}) - recording active", VOLT_LIME)
            self._log_console("=== REC_START ===")
            return
        if line == "REC_STOP":
            self._set_status(f"connected ({self.connected_port}) - session ended", "#e0a020")
            self._log_console("=== REC_STOP ===")
            return

        row = parse_row(line, "S", S_COLUMNS)
        if row is not None:
            self.sample_count += 1
            t_now = self.buf.add_sample(row)
            self._update_readouts(row, t_now)
            self._dirty = True
            return

        row = parse_row(line, "R", R_COLUMNS)
        if row is not None:
            t_now = self.buf.latest_t()
            self.buf.add_rep(row, t_now)
            self._insert_rep_row(row)
            self._dirty = True
            return

        row = parse_row(line, "P", P_COLUMNS)
        if row is not None:
            self.buf.add_discarded_phase(row, self.buf.latest_t())
            self._dirty = True
            return

        row = parse_row(line, "B", B_COLUMNS)
        if row is not None:
            return  # bracket closes aren't drawn live (retroactive correction, see vbt_log_viewer.py) - logged only

        m = BATTERY_RE.search(line)
        if m:
            volts, pct, extra = m.groups()
            self.battery_label.config(text=f"🔋 {pct}% ({volts} V){extra}")
            return

        # anything else (BLE/config messages, calibration prompts, warnings,
        # RESETREAS, header lines that failed the all-numeric parse above,
        # the "START (serial): ..."/"STOP (serial): ..." echoes from
        # pollSerialCommands()...)
        self._log_console(line)

    def _update_readouts(self, row, t_now):
        state = row["state"]
        rep = int(row["rep"])
        bg = STATE_COLOR.get(state, DISCONNECTED_GREY)
        fg = STATE_TEXT_COLOR.get(state, TEXT_PRIMARY)
        label = STATE_LABEL.get(state, state.upper())
        self.phase_label.config(text=f"REP {rep}  ·  {label}", bg=bg, fg=fg)
        self.header_frame.config(bg=bg)
        self.still_label.config(bg=bg, fg=fg)
        self.readout_frame.config(bg=bg)
        for cell, lbl_key, lbl_val in self.readout_widgets:
            cell.config(bg=bg)
            lbl_key.config(bg=bg, fg=fg)
            lbl_val.config(bg=bg, fg=fg)
        self.still_label.config(
            text=("● stillness confirmed" if row["confirmedStillNow"] else
                  ("⚠ flat-guard override" if row["flatGuardOverrideFired"] else "")))

        self.readout_vars["velZ"].set(f"{row['velZ']:+.3f}")
        self.readout_vars["velZLive"].set(f"{row['velZLive']:+.3f}")
        self.readout_vars["drift"].set(f"{row['velZ'] - row['velZLive']:+.3f}")
        self.readout_vars["worldAccZ"].set(f"{row['worldAccZ']:+.2f}")
        self.readout_vars["gyroMag"].set(f"{row['gyroMag']:.1f}")
        self.readout_vars["repCalibCount"].set(f"{int(row['repCalibCount'])}")

    def _insert_rep_row(self, row):
        status = STATUS_LABEL.get(int(row["correctionStatus"]), "?")
        values = (int(row["rep"]), f"{row['peakVelocity']:.2f}", f"{row['meanVelocity']:.2f}",
                  int(row["quality1"]), status)
        rep_num = int(row["rep"])
        existing = None
        for item in self.rep_tree.get_children():
            if int(self.rep_tree.item(item, "values")[0]) == rep_num:
                existing = item
                break
        if existing is not None:
            self.rep_tree.item(existing, values=values)  # re-print as correctionStatus improves
        else:
            self.rep_tree.insert("", "end", values=values)
        children = self.rep_tree.get_children()
        if children:
            self.rep_tree.see(children[-1])

    # ---------------- drawing ----------------

    def _redraw(self):
        window_s = self.window_s.get()
        t, vraw, vlive, acc = self.buf.window_arrays(window_s)
        if len(t) == 0:
            self.canvas.draw_idle()
            return

        self.line_raw.set_data(t, vraw)
        self.line_live.set_data(t, vlive)
        self.line_acc.set_data(t, acc)

        t_lo, t_hi = t[0], max(t[-1], t[0] + 1.0)
        self.ax_vel.set_xlim(t_lo, t_hi)

        mode = self.chart_mode.get()
        visible_series = []
        if mode in ("raw", "both"):
            visible_series.append(vraw)
        if mode in ("live", "both"):
            visible_series.append(vlive)
        finite = np.concatenate(visible_series) if visible_series else np.array([0.0])
        finite = finite[np.isfinite(finite)]
        if finite.size:
            pad = max(0.2, 0.1 * (finite.max() - finite.min() + 1e-6))
            self.ax_vel.set_ylim(finite.min() - pad, finite.max() + pad)
        if acc.size:
            pad_a = max(0.5, 0.1 * (acc.max() - acc.min() + 1e-6))
            self.ax_acc.set_ylim(acc.min() - pad_a, acc.max() + pad_a)

        still_t = [e for e in self.buf.still_events if t_lo <= e <= t_hi]
        if still_t:
            still_v = np.interp(still_t, t, vlive)
            self.still_scatter.set_offsets(np.c_[still_t, still_v])
        else:
            self.still_scatter.set_offsets(np.empty((0, 2)))

        discard_t = [(e_t, e_type) for e_t, e_type, _dur in self.buf.discard_events if t_lo <= e_t <= t_hi]
        if discard_t:
            dt_vals = [d[0] for d in discard_t]
            dv_vals = np.interp(dt_vals, t, vlive)
            self.discard_scatter.set_offsets(np.c_[dt_vals, dv_vals])
        else:
            self.discard_scatter.set_offsets(np.empty((0, 2)))

        # Rep-boundary markers: only rebuilt when the set of reps actually
        # visible in the window changes (once every rep or two, not every
        # tick) - creating/destroying matplotlib artists ~20x/second for a
        # window that hasn't changed is wasted work over a long session.
        # The y position still tracks the current axis limits every tick,
        # cheaply, via set_position rather than a full rebuild.
        ylim = self.ax_vel.get_ylim()
        visible_reps = [(e_t, rep_n) for e_t, rep_n, _p, _q, _s in self.buf.rep_events if t_lo <= e_t <= t_hi]
        key = tuple(visible_reps)
        if key != self._rep_markers_key:
            for vline, txt in self._rep_markers:
                vline.remove()
                txt.remove()
            self._rep_markers = []
            for e_t, rep_n in visible_reps:
                vline = self.ax_vel.axvline(e_t, color=BORDER_SUBTLE, lw=0.7, alpha=0.8)
                txt = self.ax_vel.text(e_t, ylim[1] * 0.95, f"#{rep_n}", fontsize=7,
                                        ha="right", color=TEXT_SECONDARY)
                self._rep_markers.append((vline, txt))
            self._rep_markers_key = key
        else:
            for _vline, txt in self._rep_markers:
                txt.set_position((txt.get_position()[0], ylim[1] * 0.95))

        self.canvas.draw_idle()


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="preselect this serial port at startup")
    args = ap.parse_args()

    app = LiveMonitorApp(initial_port=args.port)
    app.mainloop()


if __name__ == "__main__":
    main()
