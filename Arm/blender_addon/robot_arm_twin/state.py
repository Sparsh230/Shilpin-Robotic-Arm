"""
state.py -- the add-on's single live controller plus the timer that pumps it.

Kept apart from controller.py so that module stays free of `bpy`.

Re-entrancy
-----------
Writing to a Blender property fires its update callback.  The timer writes
status properties every tick, so without a guard the timer would trigger the
slider callbacks, which would command motion, which would update status...
`suspend_updates()` breaks that loop, and every write from the timer goes
inside it.
"""

from __future__ import annotations

import contextlib
import os
import traceback

import bpy

from . import config as cfgmod
from . import controller as ctrlmod
from . import inverse_kinematics as ikmod
from . import rig

TIMER_INTERVAL = 0.05

_controller = None
_config_error = ""
_suspend_depth = 0
_timer_running = False

# Set when the add-on is shutting down or a new file is loading, so a
# running mouse-target modal operator stops touching a dying scene.
_mouse_abort = False


# --------------------------------------------------------------------------
# re-entrancy guard
# --------------------------------------------------------------------------

@contextlib.contextmanager
def suspend_updates():
    global _suspend_depth
    _suspend_depth += 1
    try:
        yield
    finally:
        _suspend_depth -= 1


def updates_suspended():
    return _suspend_depth > 0


def request_mouse_abort():
    global _mouse_abort
    _mouse_abort = True


def clear_mouse_abort():
    global _mouse_abort
    _mouse_abort = False


def mouse_abort():
    return _mouse_abort


# --------------------------------------------------------------------------
# controller lifetime
# --------------------------------------------------------------------------

def _register_search_roots():
    """
    Tell the config loader where this project is.

    Installed from the ZIP the add-on lives under AppData, so walking up from
    config.py never reaches the project.  The open .blend does sit inside it
    (<project>/blender file/*.blend), so its folder and ancestors are the
    reliable anchor.
    """
    blend = getattr(bpy.data, "filepath", "")
    if blend:
        cfgmod.add_search_root(os.path.dirname(blend))


def get_controller(create=True):
    """The one ArmController, built lazily from config/arm_config.json."""
    global _controller, _config_error
    if _controller is None and create:
        _register_search_roots()
        try:
            _controller = ctrlmod.ArmController()
            _config_error = ""
        except cfgmod.ConfigError as exc:
            _config_error = str(exc)
            return None
        except Exception as exc:
            _config_error = "%s: %s" % (type(exc).__name__, exc)
            return None
    return _controller


def config_error():
    return _config_error


def reload_config():
    """Re-read the JSON.  Disconnects first -- calibration must not change mid-move."""
    global _controller, _config_error
    if _controller is not None:
        try:
            _controller.disconnect()
        except Exception:
            pass
    _controller = None
    _config_error = ""
    return get_controller()


def shutdown():
    global _controller
    request_mouse_abort()
    stop_timer()
    if _controller is not None:
        try:
            _controller.disconnect()
        except Exception:
            pass
    _controller = None


# --------------------------------------------------------------------------
# scene <-> controller
# --------------------------------------------------------------------------

def settings(context=None):
    context = context or bpy.context
    return getattr(context.scene, "robot_arm_twin", None)


def slider_angles(st):
    return [j.angle for j in st.joints]


def ensure_rest(context=None):
    """Make sure the model knows the zero pose.  Returns (ok, message)."""
    ctrl = get_controller()
    if ctrl is None:
        return False, config_error() or "no controller"
    context = context or bpy.context
    return rig.ensure_rest(ctrl.model, ctrl.cfg, context.scene)


def push_pose(context=None, send=True):
    """
    Slider values -> viewport, and (if live) -> hardware.

    Called from the slider update callbacks.
    """
    if updates_suspended():
        return
    ctrl = get_controller()
    st = settings(context)
    if ctrl is None or st is None:
        return
    ok, _msg = ensure_rest(context)
    if not ok:
        return

    angles = slider_angles(st)
    clamped = ctrl.request_pose(angles)

    # request_pose clamps to soft limits; reflect that back into the sliders
    if clamped != angles:
        with suspend_updates():
            for jp, value in zip(st.joints, clamped):
                jp.angle = value

    rig.apply_pose(ctrl.model, ctrl.pose_for_viewport,
                   (context or bpy.context).scene)

    if not send or not st.live_sync:
        ctrl._dirty = False


def get_geometry(context=None):
    """
    Build the IK geometry from the live model.

    Returns (geometry, error_message).  Rebuilt each call rather than cached:
    it is a handful of subtractions, and caching it would go stale the moment
    the rest pose is recaptured.
    """
    ctrl = get_controller()
    if ctrl is None:
        return None, config_error() or "no controller"
    ok, msg = ensure_rest(context)
    if not ok:
        return None, msg
    tip = ctrl.cfg.get("end_effector_object", "end_effector")
    try:
        return ikmod.ArmGeometry.from_model(ctrl.model, tip), ""
    except ikmod.GeometryError as exc:
        return None, str(exc)


def apply_ik_pose(context, angles, send):
    """
    Drive the twin to an IK solution, and optionally the hardware.

    Goes through ArmController.request_pose exactly like the sliders do, so
    soft-limit clamping, the delta guard and the firmware's own checks all
    still apply -- IK gets no privileged path to the motors.
    """
    ctrl = get_controller(create=False)
    st = settings(context)
    if ctrl is None or st is None:
        return None
    context = context or bpy.context

    clamped = ctrl.request_pose(list(angles))
    with suspend_updates():
        for jp, value in zip(st.joints, clamped):
            jp.angle = value
    rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)

    if send:
        # request_pose only raises the dirty flag when the pose actually
        # changed.  After a preview, `desired` already equals the target, so
        # a later real GO would look like a no-op and never reach the wire.
        # An explicit GO must always be transmitted.
        ctrl._dirty = True
    else:
        # Twin only.  Dropping the dirty flag is what stops service() from
        # ever putting this pose on the wire.
        ctrl._dirty = False

    rig.tag_redraw(context)
    return clamped


def pull_pose_to_sliders(context=None):
    """Hardware/controller pose -> sliders, without re-triggering callbacks."""
    ctrl = get_controller(create=False)
    st = settings(context)
    if ctrl is None or st is None:
        return
    with suspend_updates():
        for jp, value in zip(st.joints, ctrl.desired):
            jp.angle = value


# --------------------------------------------------------------------------
# timer
# --------------------------------------------------------------------------

def _display_signature(ctrl):
    """Everything the status readout shows, as one comparable value."""
    return (ctrl.state, ctrl.status, ctrl.estopped, ctrl.armed, ctrl.busy,
            ctrl.last_error or ctrl.link.error,
            tuple(round(v, 3) for v in ctrl.actual),
            tuple(round(v, 3) for v in ctrl.desired))


def _shown_signature(st):
    return (st.link_state, st.status_text, st.estopped, st.armed, st.busy,
            st.error_text,
            tuple(round(jp.actual, 3) for jp in st.joints),
            tuple(round(jp.commanded, 3) for jp in st.joints))


def _refresh_status(st, ctrl):
    """Copy controller state into the read-only display properties."""
    with suspend_updates():
        st.link_state = ctrl.state
        st.status_text = ctrl.status
        st.estopped = ctrl.estopped
        st.armed = ctrl.armed
        st.busy = ctrl.busy
        st.error_text = ctrl.last_error or ctrl.link.error
        for i, jp in enumerate(st.joints):
            if i < len(ctrl.actual):
                jp.actual = ctrl.actual[i]
            if i < len(ctrl.desired):
                jp.commanded = ctrl.desired[i]


def _tick():
    """Timer body.  Never raises -- a raising timer is silently unregistered."""
    try:
        ctrl = get_controller(create=False)
        st = settings()
        if ctrl is None or st is None:
            return TIMER_INTERVAL

        changed = ctrl.service()

        if st.follow_viewport and not ctrl.estopped:
            changed |= _absorb_viewport_edits(st, ctrl)

        # service() reports whether it *did* anything, which is not the same
        # as whether the readout is stale -- a connection completing on the
        # worker thread changes no controller field service() touches. Compare
        # what is shown against what is true and refresh on any difference.
        if changed or _shown_signature(st) != _display_signature(ctrl):
            _refresh_status(st, ctrl)
            rig.tag_redraw()
    except Exception:
        traceback.print_exc()
    return TIMER_INTERVAL


def _absorb_viewport_edits(st, ctrl):
    """Pick up a joint the user rotated by hand in the 3D viewport."""
    scene = bpy.context.scene
    if scene is None or not ctrl.model.has_rest():
        return False
    angles, changed, rejected = rig.read_manual_angles(
        ctrl.model, ctrl.pose_for_viewport, scene)

    if rejected:
        names = ", ".join(ctrl.joints[i].label for i in rejected)
        with suspend_updates():
            st.status_text = ("Ignored a viewport edit on %s: not a rotation "
                              "about that joint's axis" % names)
        # snap the offending objects back onto the mechanism
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, scene)

    if not changed:
        return bool(rejected)

    ctrl.request_pose(angles)
    with suspend_updates():
        for jp, value in zip(st.joints, ctrl.desired):
            jp.angle = value
    rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, scene)
    return True


def start_timer():
    global _timer_running
    if _timer_running:
        return
    if not bpy.app.timers.is_registered(_tick):
        bpy.app.timers.register(_tick, first_interval=TIMER_INTERVAL,
                                persistent=True)
    _timer_running = True


def stop_timer():
    global _timer_running
    if bpy.app.timers.is_registered(_tick):
        try:
            bpy.app.timers.unregister(_tick)
        except Exception:
            pass
    _timer_running = False


def timer_running():
    return _timer_running
