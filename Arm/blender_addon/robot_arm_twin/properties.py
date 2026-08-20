"""
properties.py -- the Blender property groups behind the control panel.

Angle sliders carry `update=` callbacks, so dragging one immediately poses the
viewport and (when Live Sync is on) commands the hardware.

Soft limits are enforced in three places on purpose: the slider's soft_min /
soft_max, the controller's clamp, and the firmware's LIM check.  A bug in any
one of them still leaves two guards standing.
"""

from __future__ import annotations

import bpy
from bpy.props import (BoolProperty, CollectionProperty, EnumProperty,
                       FloatProperty, IntProperty, StringProperty)
from bpy.types import PropertyGroup

from . import serial_link as sl
from . import state


# --------------------------------------------------------------------------
# update callbacks
# --------------------------------------------------------------------------

def _on_angle(self, context):
    state.push_pose(context)


def _on_travel_limit(self, context):
    """Retighten every joint's slider range when the master travel limit moves."""
    if state.updates_suspended():
        return
    st = state.settings(context)
    ctrl = state.get_controller(create=False)
    if st is None:
        return
    lim = st.max_travel_deg
    with state.suspend_updates():
        for i, jp in enumerate(st.joints):
            jp.min_deg = -lim
            jp.max_deg = lim
            if jp.angle > lim:
                jp.angle = lim
            elif jp.angle < -lim:
                jp.angle = -lim
            if ctrl is not None and i < len(ctrl.joints):
                ctrl.joints[i].min_deg = -lim
                ctrl.joints[i].max_deg = lim
    state.push_pose(context)


def _on_ik_target(self, context):
    """
    Any change to the target or the branch choice retires the last solve.

    Verification is what unlocks hardware motion, so it must never outlive
    the numbers it was computed from.
    """
    if state.updates_suspended():
        return
    st = state.settings(context)
    if st is None:
        return
    with state.suspend_updates():
        st.ik_verified = False
        st.ik_reachable = False
        st.ik_in_limits = False
        st.ik_status = "Target changed - solve again"
        st.ik_detail = ""
        st.ik_solved_for = ""
        # The model is no longer standing at this target, so the operator
        # must not treat it as checked.
        st.ik_previewed_for = ""


def _on_physical_sync(self, context):
    """
    Physical Sync must never latch on when the arm cannot safely take
    commands.  The modal loop re-checks this every tick as well; this is
    just the earliest place to say no.
    """
    if state.updates_suspended():
        return
    st = state.settings(context)
    ctrl = state.get_controller(create=False)
    if st is None or not st.mouse_physical_sync:
        return
    unsafe = (ctrl is None or st.estopped or not ctrl.is_connected
              or not ctrl.armed)
    if unsafe:
        with state.suspend_updates():
            st.mouse_physical_sync = False
            st.mouse_status = "Physical Sync needs a connected, armed, un-stopped arm"


NO_PORT = "NONE"

# Blender does not keep its own reference to the strings a dynamic EnumProperty
# returns, so they must stay alive here or the UI shows garbage.
_PORT_ITEMS_CACHE = []


def _port_items(self, context):
    """Enumerate serial ports live, so plugging the Uno in mid-session works."""
    global _PORT_ITEMS_CACHE
    items = []
    for device, description in sl.list_ports():
        items.append((device, "%s - %s" % (device, description), description))
    if not items:
        # An empty identifier is not a valid enum item, so use a sentinel that
        # connect() knows to treat as "nothing selected".
        items = [(NO_PORT, "<no serial ports found>", "No serial ports detected")]
    _PORT_ITEMS_CACHE = items
    return _PORT_ITEMS_CACHE


def selected_port(st):
    """The chosen port, or '' when the placeholder is showing."""
    port = st.port or ""
    return "" if port == NO_PORT else port


# --------------------------------------------------------------------------
# per-joint
# --------------------------------------------------------------------------

class ArmJointSettings(PropertyGroup):
    """One joint: its live angle, its calibration and its readback."""

    joint_name: StringProperty(name="Name", default="")
    label: StringProperty(name="Label", default="Joint")

    angle: FloatProperty(
        name="Angle",
        description="Commanded joint angle in degrees",
        default=0.0, min=-180.0, max=180.0,
        soft_min=-10.0, soft_max=10.0,
        step=10, precision=2,
        update=_on_angle,
    )

    # readback (display only)
    actual: FloatProperty(
        name="Actual", description="Angle reported by the firmware",
        default=0.0, precision=2)
    commanded: FloatProperty(
        name="Commanded", description="Angle last sent to the firmware",
        default=0.0, precision=2)

    # calibration
    steps_per_rev: FloatProperty(
        name="Steps / rev",
        description=("Steps for one full revolution of this joint. 4076 is the "
                     "exact 28BYJ-48 half-step figure. Tune per motor"),
        default=4076.0, min=1.0, max=200000.0, precision=2)
    gear_ratio: FloatProperty(
        name="Gear ratio",
        description="Extra reduction between motor shaft and joint. 1.0 = direct",
        default=1.0, min=0.001, max=1000.0, precision=4)
    direction: EnumProperty(
        name="Direction",
        description="Flip if the physical joint turns opposite to the model",
        items=[('1', "Normal", "Motor and model turn the same way"),
               ('-1', "Reversed", "Motor turns opposite to the model")],
        default='1')

    # soft limits
    min_deg: FloatProperty(name="Min", default=-10.0, min=-180.0, max=180.0, precision=2)
    max_deg: FloatProperty(name="Max", default=10.0, min=-180.0, max=180.0, precision=2)


# --------------------------------------------------------------------------
# scene-level
# --------------------------------------------------------------------------

class ArmTwinSettings(PropertyGroup):
    """Everything the panel shows, hung off the Scene."""

    joints: CollectionProperty(type=ArmJointSettings)
    initialised: BoolProperty(default=False)

    # -- connection --
    port: EnumProperty(
        name="Port", description="Serial port the Arduino is on",
        items=_port_items)
    baud: IntProperty(name="Baud", default=115200, min=1200, max=1000000)

    # -- safety --
    max_travel_deg: FloatProperty(
        name="Travel limit",
        description=("Soft limit applied to every joint, in degrees either side "
                     "of home. Start small and raise it once you trust the rig"),
        default=10.0, min=0.5, max=180.0, precision=1,
        update=_on_travel_limit)
    max_command_delta_deg: FloatProperty(
        name="Max step",
        description="Largest angle change allowed in a single command",
        default=10.0, min=0.1, max=180.0, precision=1)
    step_interval_us: IntProperty(
        name="Step interval",
        description=("Microseconds between motor steps. Higher is slower and "
                     "safer. 8000 us is about 1.8 RPM"),
        default=8000, min=1200, max=60000)
    watchdog_ms: IntProperty(
        name="Watchdog",
        description=("Firmware halts if it hears nothing for this long while "
                     "moving. 0 disables"),
        default=4000, min=0, max=60000)
    hold_torque: BoolProperty(
        name="Hold torque",
        description=("Keep coils energised at rest. Holds position but the "
                     "motors get warm"),
        default=False)

    # -- behaviour --
    live_sync: BoolProperty(
        name="Live sync",
        description="Send joint angles to the hardware as the sliders move",
        default=True)
    follow_viewport: BoolProperty(
        name="Follow viewport edits",
        description=("Also pick up joints rotated by hand in the 3D viewport, "
                     "not just the sliders"),
        default=True)
    arm_on_connect: BoolProperty(
        name="Arm on connect",
        description="Enable motion as soon as the handshake completes",
        default=True)
    zero_on_connect: BoolProperty(
        name="Zero on connect",
        description=("Treat the arm's physical pose at connect time as home. "
                     "Leave on until homing switches are fitted"),
        default=True)

    # -- runtime readout (written by the timer) --
    link_state: StringProperty(name="Link", default="DISCONNECTED")
    status_text: StringProperty(name="Status", default="Idle")
    error_text: StringProperty(name="Error", default="")
    estopped: BoolProperty(name="E-stopped", default=False)
    armed: BoolProperty(name="Armed", default=False)
    busy: BoolProperty(name="Busy", default=False)
    rest_text: StringProperty(name="Rest pose", default="not captured")

    # -- inverse kinematics --
    # Changing any of these invalidates the previous verification, so the
    # hardware can never be commanded from a stale solve.
    ik_target_x: FloatProperty(
        name="X", description="Target X for the end effector, Blender world space",
        default=0.0, precision=3, step=10, update=_on_ik_target)
    ik_target_y: FloatProperty(
        name="Y", description="Target Y for the end effector, Blender world space",
        default=0.0, precision=3, step=10, update=_on_ik_target)
    ik_target_z: FloatProperty(
        name="Z", description="Target Z for the end effector, Blender world space",
        default=0.0, precision=3, step=10, update=_on_ik_target)

    ik_elbow: EnumProperty(
        name="Branch",
        description=("Which of the two elbow solutions to prefer. A reproduces "
                     "the modelled rest pose"),
        items=[('AUTO', "Auto", "Pick whichever is legal and needs least motion"),
               ('A', "Branch A", "The branch matching the rest pose"),
               ('B', "Branch B", "The mirrored elbow solution")],
        default='AUTO', update=_on_ik_target)

    ik_preview_only: BoolProperty(
        name="Preview only (no motors)",
        description=("Solve and move the Blender model only. Leave this on "
                     "until you have checked the pose looks right"),
        default=True)
    ik_allow_hardware: BoolProperty(
        name="Allow IK to drive the motors",
        description=("Off until an IK solve has been verified against the "
                     "Blender model. Deliberately separate from Preview"),
        default=False)

    # results (written by the operators, displayed read-only)
    ik_verified: BoolProperty(name="Verified", default=False)
    ik_reachable: BoolProperty(name="Reachable", default=False)
    ik_in_limits: BoolProperty(name="Within limits", default=False)
    ik_base: FloatProperty(name="Base", default=0.0, precision=3)
    ik_middle: FloatProperty(name="Middle", default=0.0, precision=3)
    ik_upper: FloatProperty(name="Upper", default=0.0, precision=3)
    ik_residual: FloatProperty(name="Residual", default=0.0, precision=6)
    ik_status: StringProperty(name="IK status", default="No solve yet")
    ik_detail: StringProperty(name="IK detail", default="")
    ik_solved_for: StringProperty(name="Solved for", default="")
    # The target the Blender model was actually posed to and left standing at.
    # Hardware motion requires this to match the current target: that is what
    # "verified against the Blender model" means operationally.
    ik_previewed_for: StringProperty(name="Previewed for", default="")

    # -- mouse target mode --
    mouse_target_active: BoolProperty(
        name="Mouse Target",
        description="Read-only: whether the mouse-tracking modal loop is running",
        default=False)
    mouse_physical_sync: BoolProperty(
        name="Physical Sync",
        description=("Let mouse tracking drive the real motors. Off means "
                     "Blender only. Cleared by the emergency stop"),
        default=False, update=_on_physical_sync)
    mouse_update_hz: IntProperty(
        name="Update rate",
        description=("How many times a second the mouse position is turned "
                     "into an IK solve. 10-20 is plenty"),
        default=15, min=1, max=60)
    mouse_plane_half_size: FloatProperty(
        name="Plane size",
        description="Half-width of the target plane when it is created",
        default=2.0, min=0.1, max=100.0, precision=2)

    # live readout, written by the modal loop
    mouse_target_x: FloatProperty(name="X", default=0.0, precision=3)
    mouse_target_y: FloatProperty(name="Y", default=0.0, precision=3)
    mouse_target_z: FloatProperty(name="Z", default=0.0, precision=3)
    mouse_base: FloatProperty(name="Base", default=0.0, precision=3)
    mouse_middle: FloatProperty(name="Middle", default=0.0, precision=3)
    mouse_upper: FloatProperty(name="Upper", default=0.0, precision=3)
    mouse_have_hit: BoolProperty(name="On plane", default=False)
    mouse_ik_ok: BoolProperty(name="IK solved", default=False)
    mouse_ik_status: StringProperty(name="IK status", default="idle")
    mouse_status: StringProperty(name="Mouse status", default="Off")
    mouse_sent: BoolProperty(name="Last update sent", default=False)

    # -- panel folding --
    show_calibration: BoolProperty(name="Calibration", default=False)
    show_safety: BoolProperty(name="Safety", default=True)
    show_diagnostics: BoolProperty(name="Diagnostics", default=False)


# --------------------------------------------------------------------------
# population from config
# --------------------------------------------------------------------------

def populate_from_config(scene, ctrl):
    """Fill the property groups from the loaded arm_config.json."""
    st = scene.robot_arm_twin
    cfg = ctrl.cfg
    saf = cfg["safety"]

    with state.suspend_updates():
        st.joints.clear()
        for jc, spec in zip(cfg["joints"], ctrl.joints):
            jp = st.joints.add()
            jp.joint_name = spec.name
            jp.label = spec.label
            jp.angle = 0.0
            jp.actual = 0.0
            jp.commanded = 0.0
            m = jc["motor"]
            jp.steps_per_rev = float(m["steps_per_rev"])
            jp.gear_ratio = float(m["gear_ratio"])
            jp.direction = '-1' if int(m["direction"]) < 0 else '1'
            jp.min_deg = float(jc["limits"]["min_deg"])
            jp.max_deg = float(jc["limits"]["max_deg"])

        st.max_travel_deg = float(saf["max_travel_deg"])
        st.max_command_delta_deg = float(saf["max_command_delta_deg"])
        st.step_interval_us = int(saf["step_interval_us"])
        st.watchdog_ms = int(saf["watchdog_ms"])
        st.hold_torque = bool(saf["hold_torque"])
        st.baud = int(cfg["serial"].get("baud", 115200))

        wanted = cfg["serial"].get("port", "")
        if wanted:
            available = {d for d, _ in sl.list_ports()}
            if wanted in available:
                st.port = wanted

        st.initialised = True


def write_back_to_config(scene, ctrl):
    """Copy panel values into the in-memory config, ready to save."""
    st = scene.robot_arm_twin
    cfg = ctrl.cfg

    cfg["serial"]["port"] = selected_port(st) or cfg["serial"].get("port", "")
    cfg["serial"]["baud"] = int(st.baud)

    saf = cfg["safety"]
    saf["max_travel_deg"] = float(st.max_travel_deg)
    saf["max_command_delta_deg"] = float(st.max_command_delta_deg)
    saf["step_interval_us"] = int(st.step_interval_us)
    saf["watchdog_ms"] = int(st.watchdog_ms)
    saf["hold_torque"] = bool(st.hold_torque)

    for jc, jp, spec in zip(cfg["joints"], st.joints, ctrl.joints):
        jc["motor"]["steps_per_rev"] = float(jp.steps_per_rev)
        jc["motor"]["gear_ratio"] = float(jp.gear_ratio)
        jc["motor"]["direction"] = int(jp.direction)
        jc["limits"]["min_deg"] = float(jp.min_deg)
        jc["limits"]["max_deg"] = float(jp.max_deg)
        spec.min_deg = float(jp.min_deg)
        spec.max_deg = float(jp.max_deg)
    return cfg


CLASSES = (ArmJointSettings, ArmTwinSettings)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.Scene.robot_arm_twin = bpy.props.PointerProperty(type=ArmTwinSettings)


def unregister():
    if hasattr(bpy.types.Scene, "robot_arm_twin"):
        del bpy.types.Scene.robot_arm_twin
    for c in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
