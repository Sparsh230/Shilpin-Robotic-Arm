"""
serial_link.py -- threaded, line-oriented serial transport to the Arduino.

Free of `bpy`, so the same class backs the Blender panel today and the
standalone dashboard later.

Why a thread
------------
Blender's UI runs on one thread.  A blocking `readline()` on a port that has
gone quiet would freeze the whole application.  All port I/O therefore happens
on a worker thread; callers either fire-and-forget or wait on a future with
their own timeout.

Why the e-stop is special
-------------------------
`estop()` sets a flag the worker checks *before* it looks at the command
queue, and it discards everything still queued.  A stop can therefore never
sit behind a backlog of queued MOVEs.
"""

from __future__ import annotations

import threading
import time
from collections import deque

try:
    import queue
except ImportError:  # pragma: no cover
    import Queue as queue  # type: ignore


# --------------------------------------------------------------------------
# pyserial discovery
# --------------------------------------------------------------------------

class SerialUnavailable(RuntimeError):
    """pyserial is not importable from this interpreter."""


def import_serial():
    """Import pyserial, raising SerialUnavailable with a helpful message."""
    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
        return serial
    except Exception as exc:  # pragma: no cover
        raise SerialUnavailable(
            "pyserial is not installed in this Python. "
            "Use the 'Install pyserial' button in the arm panel. (%s)" % exc)


def have_serial() -> bool:
    try:
        import_serial()
        return True
    except SerialUnavailable:
        return False


def list_ports():
    """Return [(device, description)] or [] if pyserial is missing."""
    try:
        serial = import_serial()
        # Use the module import_serial() handed back rather than importing
        # `serial` again: a test or the emulator may have substituted it.
        lp = serial.tools.list_ports
    except Exception:
        return []
    out = []
    try:
        for p in lp.comports():
            out.append((p.device, p.description or p.device))
    except Exception:
        return []
    out.sort(key=lambda t: t[0])
    return out


# --------------------------------------------------------------------------
# link states
# --------------------------------------------------------------------------

DISCONNECTED = "DISCONNECTED"
CONNECTING = "CONNECTING"
CONNECTED = "CONNECTED"
ERROR = "ERROR"


class Reply(object):
    __slots__ = ("command", "ok", "text", "timed_out")

    def __init__(self, command, ok=False, text="", timed_out=False):
        self.command = command
        self.ok = ok
        self.text = text
        self.timed_out = timed_out

    def __repr__(self):
        if self.timed_out:
            return "<Reply %r TIMEOUT>" % self.command
        return "<Reply %r %s %r>" % (self.command, "OK" if self.ok else "ERR", self.text)


class _Pending(object):
    __slots__ = ("command", "event", "reply")

    def __init__(self, command):
        self.command = command
        self.event = threading.Event()
        self.reply = None


# --------------------------------------------------------------------------
# the link
# --------------------------------------------------------------------------

class SerialLink(object):
    """One connection to one Arduino."""

    def __init__(self, event_log_size=200):
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self._estop = threading.Event()
        self._q = queue.Queue()
        self._lock = threading.Lock()

        self._state = DISCONNECTED
        self._error = ""
        self._events = deque(maxlen=event_log_size)
        self._traffic = deque(maxlen=event_log_size)

        # connection parameters, captured at connect()
        self._port = ""
        self._baud = 115200
        self._read_timeout = 1.0
        self._boot_settle = 2.5
        self._init_commands = []

    # -- observable state ---------------------------------------------------

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def is_connected(self):
        return self.state == CONNECTED

    @property
    def is_busy(self):
        return self.state in (CONNECTING, CONNECTED)

    @property
    def error(self):
        with self._lock:
            return self._error

    @property
    def port(self):
        return self._port

    def _set_state(self, state, error=""):
        with self._lock:
            self._state = state
            if error:
                self._error = error
            elif state in (CONNECTING, CONNECTED):
                self._error = ""

    def _log(self, direction, text):
        self._traffic.append((time.time(), direction, text))

    def drain_events(self):
        """Pop and return unsolicited 'EV ...' lines seen since the last call."""
        out = []
        while self._events:
            out.append(self._events.popleft())
        return out

    def traffic_tail(self, n=20):
        return list(self._traffic)[-n:]

    # -- lifecycle ----------------------------------------------------------

    def connect(self, port, baud=115200, read_timeout=1.0, boot_settle=2.5,
                init_commands=None):
        """
        Open `port` and start the worker.  Returns immediately -- the port
        open, the Uno's auto-reset settle and the init handshake all happen on
        the worker thread.  Watch `state` for the outcome.
        """
        if self.is_busy:
            raise RuntimeError("already connected or connecting")
        import_serial()  # fail fast and clearly if pyserial is missing

        self._port = port
        self._baud = int(baud)
        self._read_timeout = float(read_timeout)
        self._boot_settle = float(boot_settle)
        self._init_commands = list(init_commands or [])

        self._stop.clear()
        self._estop.clear()
        while not self._q.empty():          # discard anything left from before
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

        self._set_state(CONNECTING)
        self._thread = threading.Thread(target=self._run, name="arm-serial", daemon=True)
        self._thread.start()

    def disconnect(self, timeout=2.0):
        """Stop the worker and close the port.  Safe to call when not connected."""
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout)
        self._thread = None
        self._close_port()
        self._fail_pending("link closed")
        self._set_state(DISCONNECTED)

    def _close_port(self):
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _fail_pending(self, reason):
        while True:
            try:
                p = self._q.get_nowait()
            except queue.Empty:
                return
            p.reply = Reply(p.command, ok=False, text=reason)
            p.event.set()

    # -- sending ------------------------------------------------------------

    def send_async(self, command):
        """Queue a command and return at once.  Dropped if not connected."""
        if not self.is_connected:
            return None
        p = _Pending(str(command).strip())
        self._q.put(p)
        return p

    def send(self, command, timeout=2.0):
        """Queue a command and wait for its reply."""
        p = self.send_async(command)
        if p is None:
            return Reply(str(command), ok=False, text="not connected")
        if not p.event.wait(timeout):
            return Reply(p.command, ok=False, text="no reply", timed_out=True)
        return p.reply

    def estop(self):
        """
        Priority stop.  Jumps every queued command.  Returns True if the
        request was handed to a live worker.
        """
        if not self.is_busy:
            return False
        self._fail_pending("pre-empted by E-STOP")
        self._estop.set()
        return True

    # -- worker -------------------------------------------------------------

    def _run(self):
        serial = import_serial()
        try:
            self._ser = serial.Serial(
                port=self._port, baudrate=self._baud,
                timeout=self._read_timeout, write_timeout=2.0)
        except Exception as exc:
            self._set_state(ERROR, "cannot open %s: %s" % (self._port, exc))
            return

        # Opening the port toggles DTR, which resets an Uno. Wait it out,
        # then throw away the bootloader noise and the READY banner.
        deadline = time.time() + self._boot_settle
        while time.time() < deadline and not self._stop.is_set():
            time.sleep(0.05)
        if self._stop.is_set():
            self._close_port()
            self._set_state(DISCONNECTED)
            return
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

        self._set_state(CONNECTED)

        # Push the opening handshake (identity check, calibration, limits...).
        for cmd in self._init_commands:
            r = self._exchange(cmd)
            if r is None:
                self._set_state(ERROR, "link lost during handshake")
                self._close_port()
                return
            if not r.ok:
                self._set_state(ERROR, "handshake rejected: %s -> %s" % (cmd, r.text))
                # keep the link open so the user can still hit STOP
                break

        # main service loop
        while not self._stop.is_set():
            if self._estop.is_set():
                self._estop.clear()
                self._fail_pending("pre-empted by E-STOP")
                self._exchange("STOP")
                continue
            try:
                p = self._q.get(timeout=0.02)
            except queue.Empty:
                self._read_unsolicited()
                continue
            r = self._exchange(p.command)
            p.reply = r if r is not None else Reply(p.command, ok=False,
                                                    text="link lost", timed_out=True)
            p.event.set()
            if r is None:
                break

        self._close_port()
        if self.state != ERROR:
            self._set_state(DISCONNECTED)

    # -- framing ------------------------------------------------------------

    def _write_line(self, text):
        data = (text + "\n").encode("ascii", "replace")
        self._ser.write(data)
        self._ser.flush()
        self._log(">", text)

    def _read_line(self):
        raw = self._ser.readline()
        if not raw:
            return None
        line = raw.decode("ascii", "replace").strip()
        if line:
            self._log("<", line)
        return line

    def _exchange(self, command, attempts=3):
        """
        Write one command, then read until a terminal OK/ERR line.
        'EV ...' lines are filed as events and do not terminate the wait.
        Returns None if the port died.
        """
        try:
            self._write_line(command)
        except Exception as exc:
            self._set_state(ERROR, "write failed: %s" % exc)
            return None

        for _ in range(attempts):
            try:
                line = self._read_line()
            except Exception as exc:
                self._set_state(ERROR, "read failed: %s" % exc)
                return None
            if line is None:
                continue                      # readline timeout, try again
            if line.startswith("EV "):
                self._events.append(line)
                continue
            if line.startswith("OK"):
                return Reply(command, ok=True, text=line[2:].strip())
            if line.startswith("ERR"):
                return Reply(command, ok=False, text=line[3:].strip())
            # anything else is chatter; ignore and keep waiting
        return Reply(command, ok=False, text="no reply", timed_out=True)

    def _read_unsolicited(self):
        """Collect EV lines that arrive when we did not ask anything."""
        try:
            if self._ser is None or self._ser.in_waiting <= 0:
                return
            line = self._read_line()
        except Exception as exc:
            self._set_state(ERROR, "read failed: %s" % exc)
            return
        if line and line.startswith("EV "):
            self._events.append(line)


# --------------------------------------------------------------------------
# reply parsing helpers (protocol-aware, transport-agnostic)
# --------------------------------------------------------------------------

def parse_pos(reply_text):
    """
    Parse the body of 'OK POS <a0> <a1> <a2> busy <b> estop <e> armed <a>'.

    `reply_text` is what Reply.text holds, i.e. 'POS 1.000 0.000 ...'.
    Returns dict(angles=[...], busy=bool, estop=bool, armed=bool) or None.
    """
    parts = reply_text.split()
    if not parts or parts[0] != "POS":
        return None
    angles, i = [], 1
    while i < len(parts):
        try:
            angles.append(float(parts[i]))
        except ValueError:
            break
        i += 1
    flags = {}
    while i + 1 < len(parts):
        flags[parts[i].lower()] = parts[i + 1]
        i += 2
    return {
        "angles": angles,
        "busy": flags.get("busy") == "1",
        "estop": flags.get("estop") == "1",
        "armed": flags.get("armed") == "1",
    }
