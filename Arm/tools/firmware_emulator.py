"""
firmware_emulator.py -- a software stand-in for arm_firmware.ino.

Implements the same ASCII protocol, the same soft limits, the same delta
guard, the same e-stop latch and the same time-based stepping, so the host
side can be exercised with no Arduino and no powered motors.

Two ways to use it:

    # 1. as a fake port inside SerialLink
    from tools.firmware_emulator import install_emulator
    install_emulator(serial_link)          # now connect() to any port name

    # 2. directly, as a protocol oracle
    fw = EmulatedFirmware()
    print(fw.handle("PING"))               # -> "OK PONG"

Keep this in step with arm_firmware.ino when the protocol changes.
"""

from __future__ import annotations

import math
import time
import types


class EmulatedJoint(object):
    def __init__(self):
        self.pos = 0            # steps
        self.target = 0         # steps
        self.steps_per_rev = 4076.0
        self.gear_ratio = 1.0
        self.dir = 1
        self.min_deg = -10.0
        self.max_deg = 10.0
        self.last_step = 0.0

    @property
    def steps_per_deg(self):
        return (self.steps_per_rev * self.gear_ratio) / 360.0

    @property
    def angle(self):
        return (self.pos * self.dir) / self.steps_per_deg

    def angle_to_steps(self, deg):
        return int(round(deg * self.steps_per_deg)) * self.dir


class EmulatedFirmware(object):
    """Mirror of the .ino state machine."""

    NJOINTS = 3
    NAME = "ARM3DOF"
    VERSION = "1.0"

    def __init__(self, time_fn=time.monotonic):
        self._now = time_fn
        self.J = [EmulatedJoint() for _ in range(self.NJOINTS)]
        self.step_interval_us = 8000
        self.min_interval_us = 1200
        self.max_delta_deg = 10.0
        self.estopped = False
        self.armed = False
        self.hold = False
        self.watchdog_ms = 4000
        self.last_cmd = self._now()
        self.events = ["EV READY %s %s" % (self.NAME, self.VERSION)]

    # -- motion -------------------------------------------------------------

    def service(self):
        """Advance every motor according to elapsed wall time."""
        if self.estopped:
            return
        now = self._now()
        interval = self.step_interval_us / 1e6
        for j in self.J:
            if j.pos == j.target:
                continue
            n = int((now - j.last_step) / interval) if j.last_step else 1
            if n <= 0:
                continue
            j.last_step = now
            remaining = j.target - j.pos
            move = min(abs(remaining), n)
            j.pos += move if remaining > 0 else -move
        if self.watchdog_ms and self.any_moving():
            if (now - self.last_cmd) * 1000.0 > self.watchdog_ms:
                self._estop("WATCHDOG")

    def any_moving(self):
        return any(j.pos != j.target for j in self.J)

    def _estop(self, why):
        for j in self.J:
            j.target = j.pos
        self.estopped = True
        self.armed = False
        self.events.append("EV ESTOP %s" % why)

    # -- protocol -----------------------------------------------------------

    def _pos_line(self):
        angles = " ".join("%.3f" % j.angle for j in self.J)
        return ("OK POS %s busy %d estop %d armed %d"
                % (angles, 1 if self.any_moving() else 0,
                   1 if self.estopped else 0, 1 if self.armed else 0))

    def handle(self, line):
        self.service()
        self.last_cmd = self._now()
        parts = line.strip().split()
        if not parts:
            return None
        cmd = parts[0].upper()
        a = parts[1:]

        if cmd == "PING":
            return "OK PONG"
        if cmd == "ID":
            return "OK %s %s JOINTS %d" % (self.NAME, self.VERSION, self.NJOINTS)
        if cmd == "STOP":
            self._estop("COMMANDED")
            return "OK STOP"
        if cmd == "GET":
            return self._pos_line()
        if cmd == "RESUME":
            for j in self.J:
                j.target = j.pos
            self.estopped = False
            self.armed = False
            return "OK RESUME"
        if cmd == "ARM":
            if not a:
                return "ERR ARGS"
            if self.estopped:
                return "ERR ESTOPPED"
            self.armed = a[0] != "0"
            return "OK ARM %d" % (1 if self.armed else 0)

        if cmd == "SPEED":
            if not a:
                return "ERR ARGS"
            self.step_interval_us = max(int(float(a[0])), self.min_interval_us)
            return "OK SPEED %d" % self.step_interval_us
        if cmd == "CAL":
            if len(a) < 4:
                return "ERR ARGS"
            i = int(a[0])
            if not 0 <= i < self.NJOINTS:
                return "ERR INDEX"
            spr, gr = float(a[1]), float(a[2])
            if spr < 1.0 or gr <= 0.0:
                return "ERR RANGE"
            j = self.J[i]
            j.steps_per_rev, j.gear_ratio = spr, gr
            j.dir = -1 if int(a[3]) < 0 else 1
            j.pos = j.target = 0
            return "OK CAL %d" % i
        if cmd == "LIM":
            if len(a) < 3:
                return "ERR ARGS"
            i = int(a[0])
            if not 0 <= i < self.NJOINTS:
                return "ERR INDEX"
            lo, hi = float(a[1]), float(a[2])
            if hi < lo:
                return "ERR RANGE"
            self.J[i].min_deg, self.J[i].max_deg = lo, hi
            return "OK LIM %d" % i
        if cmd == "DELTA":
            if not a:
                return "ERR ARGS"
            v = float(a[0])
            if v <= 0:
                return "ERR RANGE"
            self.max_delta_deg = v
            return "OK DELTA %.3f" % v
        if cmd == "HOLD":
            if not a:
                return "ERR ARGS"
            self.hold = a[0] != "0"
            return "OK HOLD %d" % (1 if self.hold else 0)
        if cmd == "WD":
            if not a:
                return "ERR ARGS"
            self.watchdog_ms = int(float(a[0]))
            return "OK WD %d" % self.watchdog_ms
        if cmd == "ZERO":
            if self.any_moving():
                return "ERR BUSY"
            for j in self.J:
                j.pos = j.target = 0
            return "OK ZERO"

        if cmd in ("MOVE", "MOVEJ"):
            if self.estopped:
                return "ERR ESTOPPED"
            if not self.armed:
                return "ERR DISARMED"
            if cmd == "MOVEJ":
                if len(a) < 2:
                    return "ERR ARGS"
                i = int(a[0])
                if not 0 <= i < self.NJOINTS:
                    return "ERR INDEX"
                j, deg = self.J[i], float(a[1])
                if deg < j.min_deg or deg > j.max_deg:
                    return "ERR LIMIT"
                if abs(deg - j.angle) > self.max_delta_deg + 1e-4:
                    return "ERR DELTA"
                j.target = j.angle_to_steps(deg)
                return self._pos_line()

            if len(a) < self.NJOINTS:
                return "ERR ARGS"
            want = [float(x) for x in a[:self.NJOINTS]]
            for i, (j, w) in enumerate(zip(self.J, want)):
                if w < j.min_deg or w > j.max_deg:
                    return "ERR LIMIT J%d" % i
                if abs(w - j.angle) > self.max_delta_deg + 1e-4:
                    return "ERR DELTA J%d" % i
            for j, w in zip(self.J, want):
                j.target = j.angle_to_steps(w)
            return self._pos_line()

        return "ERR UNKNOWN %s" % cmd


# --------------------------------------------------------------------------
# a pyserial-shaped wrapper
# --------------------------------------------------------------------------

class EmulatedSerial(object):
    """Enough of serial.Serial for SerialLink to drive."""

    def __init__(self, port="EMU", baudrate=115200, timeout=1.0,
                 write_timeout=2.0, **_kw):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.fw = EmulatedFirmware()
        self._rx = bytearray()          # host -> device
        self._tx = bytearray()          # device -> host
        self._flush_events()

    def _flush_events(self):
        while self.fw.events:
            self._tx += (self.fw.events.pop(0) + "\r\n").encode("ascii")

    # -- pyserial surface ---------------------------------------------------

    @property
    def in_waiting(self):
        self.fw.service()
        self._flush_events()
        return len(self._tx)

    def write(self, data):
        self._rx += data
        while b"\n" in self._rx:
            line, _, rest = self._rx.partition(b"\n")
            self._rx = bytearray(rest)
            reply = self.fw.handle(line.decode("ascii", "replace"))
            self._flush_events()
            if reply:
                self._tx += (reply + "\r\n").encode("ascii")
        return len(data)

    def readline(self):
        self.fw.service()
        self._flush_events()
        if b"\n" not in self._tx:
            time.sleep(min(self.timeout, 0.01))
            self.fw.service()
            self._flush_events()
            if b"\n" not in self._tx:
                return b""
        line, _, rest = self._tx.partition(b"\n")
        self._tx = bytearray(rest)
        return bytes(line) + b"\n"

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._tx = bytearray()

    def reset_output_buffer(self):
        self._rx = bytearray()

    def close(self):
        self.is_open = False


def make_fake_serial_module():
    """A stand-in for the `serial` package exposing only what SerialLink uses."""
    mod = types.ModuleType("serial")
    mod.Serial = EmulatedSerial
    mod.SerialException = OSError
    tools = types.ModuleType("serial.tools")
    lp = types.ModuleType("serial.tools.list_ports")

    class _Port(object):
        def __init__(self, device, description):
            self.device, self.description = device, description

    lp.comports = lambda: [_Port("EMU", "Emulated 3-DOF arm firmware")]
    tools.list_ports = lp
    mod.tools = tools
    return mod


def install_emulator(serial_link_module):
    """
    Redirect a SerialLink module at the emulator.

    Returns a callable that restores the real behaviour.
    """
    original = serial_link_module.import_serial
    fake = make_fake_serial_module()
    serial_link_module.import_serial = lambda: fake

    def restore():
        serial_link_module.import_serial = original
    return restore
