"""
controller.py -- the brain between the UI and the hardware.

Still no `bpy`: this is the piece the future standalone dashboard will reuse
verbatim.  It owns

  * the parsed config,
  * the kinematic model (for clamping and for posing the viewport),
  * the serial link,
  * the commanded pose vs the pose the firmware reports back.

Command coalescing
------------------
A slider drag emits far more updates than a 1.8 RPM stepper can absorb.  The
UI therefore only ever calls `request_pose()`, which records intent.  A timer
calls `service()`, which sends at most one MOVE per `send_interval`.  Because
commands are absolute, a dropped intermediate value is harmless -- the newest
target simply supersedes it.

Long moves
----------
`max_command_delta_deg` caps one command.  `service()` walks toward a distant
goal in bounded hops rather than rejecting it, so raising the travel limit
later does not break large moves.
"""

from __future__ import annotations

import time

from . import config as cfgmod
from . import kinematics as kin
from . import serial_link as sl


class ArmController(object):

    def __init__(self, config=None):
        self.cfg = config or cfgmod.load()
        self.joints = kin.joints_from_config(self.cfg)
        self.model = kin.ArmModel(self.joints)
        self.link = sl.SerialLink()

        n = len(self.joints)
        self.desired = [0.0] * n       # what the user asked for
        self.commanded = [0.0] * n     # what we last sent
        self.actual = [0.0] * n        # what the firmware reports
        self.pose_for_viewport = list(self.desired)

        self.estopped = False
        self.armed = False
        self.busy = False
        self.firmware_id = ""
        self.status = "Idle"
        self.last_error = ""

        self._dirty = False
        self._link_was_connecting = False
        self._last_send = 0.0
        self._last_poll = 0.0
        self.send_interval = 0.10
        self.poll_interval = float(self.cfg["serial"].get("poll_interval_s", 0.25))
        self.zero_on_connect = bool(self.cfg["serial"].get("zero_on_connect", True))

    # -- convenience --------------------------------------------------------

    @property
    def n_joints(self):
        return len(self.joints)

    @property
    def max_delta(self):
        return float(self.cfg["safety"]["max_command_delta_deg"])

    @property
    def is_connected(self):
        return self.link.is_connected

    @property
    def state(self):
        return self.link.state

    def joint(self, i):
        return self.joints[i]

    # -- connection ---------------------------------------------------------

    def connect(self, port=None, arm_on_connect=True):
        """Open the link and push the handshake.  Non-blocking."""
        ser = self.cfg["serial"]
        port = port or ser.get("port") or ""
        if not port:
            self.last_error = "no serial port selected"
            return False
        init = cfgmod.build_init_commands(
            self.cfg,
            zero_on_connect=self.zero_on_connect,
            arm_on_connect=arm_on_connect)
        try:
            self.link.connect(
                port=port,
                baud=int(ser.get("baud", 115200)),
                read_timeout=float(ser.get("read_timeout_s", 1.0)),
                boot_settle=float(ser.get("boot_settle_s", 2.5)),
                init_commands=init)
        except Exception as exc:
            self.last_error = str(exc)
            self.status = "Connect failed"
            return False

        # Connecting adopts the arm's present physical pose as zero, so the
        # twin must start from zero too or the first command would be a jump.
        if self.zero_on_connect:
            self.desired = [0.0] * self.n_joints
            self.commanded = [0.0] * self.n_joints
            self.actual = [0.0] * self.n_joints
            self.pose_for_viewport = [0.0] * self.n_joints
        self.estopped = False
        self._dirty = False
        self.status = "Connecting..."
        self._link_was_connecting = True
        self.last_error = ""
        return True

    def disconnect(self):
        """Stop motion, disarm, then close.  Best effort -- always closes."""
        if self.link.is_connected:
            try:
                self.link.send("ARM 0", timeout=0.5)
            except Exception:
                pass
        self.link.disconnect()
        self.armed = False
        self.busy = False
        self.status = "Disconnected"

    # -- safety -------------------------------------------------------------

    def emergency_stop(self):
        """Latch the stop. Always succeeds locally, even with no link."""
        self.estopped = True
        self.armed = False
        self._dirty = False
        self.status = "EMERGENCY STOP"
        ok = self.link.estop()
        # freeze the twin where the hardware actually is
        self.desired = list(self.actual)
        self.commanded = list(self.actual)
        self.pose_for_viewport = list(self.actual)
        return ok

    def resume(self, rearm=True):
        """Clear the latch.  Re-arming is a separate, deliberate step."""
        if not self.link.is_connected:
            self.estopped = False
            self.status = "Idle"
            return True
        r = self.link.send("RESUME", timeout=2.0)
        if not r.ok:
            self.last_error = "resume failed: %s" % r.text
            return False
        self.estopped = False
        if rearm:
            ra = self.link.send("ARM 1", timeout=2.0)
            self.armed = ra.ok
        self.sync_from_hardware()
        self.desired = list(self.actual)
        self.commanded = list(self.actual)
        self.pose_for_viewport = list(self.actual)
        self.status = "Resumed"
        return True

    def set_zero_here(self):
        """Declare the current physical pose to be home."""
        if not self.link.is_connected:
            return False
        r = self.link.send("ZERO", timeout=2.0)
        if not r.ok:
            self.last_error = "zero failed: %s" % r.text
            return False
        self.desired = [0.0] * self.n_joints
        self.commanded = [0.0] * self.n_joints
        self.actual = [0.0] * self.n_joints
        self.pose_for_viewport = [0.0] * self.n_joints
        self.status = "Home set"
        return True

    # -- motion -------------------------------------------------------------

    def request_pose(self, angles):
        """
        Record the pose the user wants.  Clamped to soft limits here so the
        viewport never shows something the hardware would refuse.
        """
        clamped = self.model.clamp_pose(list(angles)[:self.n_joints])
        if clamped != self.desired:
            self.desired = clamped
            self._dirty = True
        # The twin follows the request immediately; the hardware catches up.
        self.pose_for_viewport = list(clamped)
        return clamped

    def request_joint(self, index, angle):
        angles = list(self.desired)
        angles[index] = angle
        return self.request_pose(angles)

    def go_home(self):
        return self.request_pose([0.0] * self.n_joints)

    def service(self, now=None):
        """
        Pump the controller.  Call from a timer at ~20 Hz.

        Returns True if anything the UI shows has changed.
        """
        now = now or time.monotonic()
        changed = False

        for ev in self.link.drain_events():
            changed = True
            if ev.startswith("EV ESTOP"):
                self.estopped = True
                self.armed = False
                self._dirty = False
                self.status = ev[3:].strip()
            elif ev.startswith("EV READY"):
                self.status = "Firmware ready"

        if self.link.state == sl.ERROR:
            self.last_error = self.link.error
            self.status = "Link error"
            return True
        if not self.link.is_connected:
            return changed

        if self._link_was_connecting:
            self._link_was_connecting = False
            self.status = "Ready"
            changed = True

        if self.estopped:
            return changed

        if self._dirty and (now - self._last_send) >= self.send_interval:
            changed |= self._send_step(now)

        if (now - self._last_poll) >= self.poll_interval:
            self._last_poll = now
            changed |= self.sync_from_hardware(blocking=False)

        return changed

    def _send_step(self, now):
        """Send one bounded hop toward `desired`."""
        base = self.actual if self.actual else self.commanded
        target, limited = self.model.limit_delta(base, self.desired, self.max_delta)
        self._last_send = now

        if all(abs(t - c) < 1e-4 for t, c in zip(target, self.commanded)) and not limited:
            self._dirty = False
            return False

        cmd = "MOVE " + " ".join("%.3f" % a for a in target)
        r = self.link.send(cmd, timeout=2.0)
        if not r.ok:
            self.last_error = "%s -> %s" % (cmd, r.text)
            self.status = "Move refused: %s" % r.text
            self._dirty = False
            return True

        self.commanded = list(target)
        info = sl.parse_pos(r.text)
        if info:
            self._absorb(info)
        # still short of the goal? stay dirty so the next tick hops again
        self._dirty = any(abs(d - t) > 1e-3 for d, t in zip(self.desired, target))
        self.status = "Moving" if self.busy else "Ready"
        return True

    def sync_from_hardware(self, blocking=True):
        """Ask the firmware where it actually is."""
        if not self.link.is_connected:
            return False
        r = self.link.send("GET", timeout=2.0 if blocking else 0.6)
        if not r.ok:
            return False
        info = sl.parse_pos(r.text)
        if not info:
            return False
        return self._absorb(info)

    def _absorb(self, info):
        changed = False
        angles = info["angles"][:self.n_joints]
        if angles and angles != self.actual:
            self.actual = list(angles)
            changed = True
        for attr in ("busy", "estop", "armed"):
            key = "estopped" if attr == "estop" else attr
            if getattr(self, key) != info[attr]:
                setattr(self, key, info[attr])
                changed = True
        return changed

    # -- reporting ----------------------------------------------------------

    def tracking_error(self):
        """Per-joint difference between what we asked and where it is."""
        return [d - a for d, a in zip(self.desired, self.actual)]

    def tip_position(self):
        tip = self.cfg.get("end_effector_object", "end_effector")
        if not self.model.has_rest():
            return None
        return self.model.tip_position(self.pose_for_viewport, tip)

    def describe(self):
        return {
            "state": self.link.state,
            "port": self.link.port,
            "status": self.status,
            "estopped": self.estopped,
            "armed": self.armed,
            "busy": self.busy,
            "desired": list(self.desired),
            "commanded": list(self.commanded),
            "actual": list(self.actual),
            "error": self.last_error or self.link.error,
        }
