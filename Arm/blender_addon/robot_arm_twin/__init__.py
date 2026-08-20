"""
robot_arm_twin -- Blender digital twin for a 3-DOF stepper arm.

Install:  Edit > Preferences > Add-ons > Install from Disk...
          and pick robot_arm_twin.zip
Use:      3D viewport > press N > "Robot Arm" tab.

Module map
----------
    kinematics.py   pure maths, no bpy/mathutils. Forward kinematics, limits.
    config.py       arm_config.json load/save/validate, handshake building.
    serial_link.py  threaded serial transport and protocol framing.
    controller.py   config + model + link. The reusable core.
    ---- everything below is the only bpy-aware code ----
    rig.py          reads and writes the Blender scene.
    state.py        the live controller singleton and its timer.
    properties.py   panel properties.
    operators.py    buttons.
    panel.py        the sidebar UI.

Importable outside Blender
--------------------------
The first four modules import fine in plain CPython; the Blender half is only
loaded when `bpy` is present.  So

    from robot_arm_twin import controller

works in a normal Python process, which is what makes the planned standalone
dashboard, an IK solver and teach-and-repeat drop-in additions rather than a
rewrite.  `tools/selftest.py` relies on this.
"""

bl_info = {
    "name": "3-DOF Robot Arm Twin",
    "author": "built for the 3-DOF arm project",
    "version": (1, 0, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar (N) > Robot Arm",
    "description": ("Digital twin for a 3-DOF 28BYJ-48 arm: joint sliders, "
                    "absolute-angle serial control, calibration, E-STOP"),
    "warning": "Moves real hardware. Keep the emergency stop in reach.",
    "category": "Object",
}

import os
import sys

# pyserial installed by the panel's button lives here, beside the add-on.
_VENDOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_vendor")
if os.path.isdir(_VENDOR) and _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

try:
    import bpy
    from bpy.app.handlers import persistent
    HAVE_BPY = True
except ImportError:          # plain CPython: core only, no UI
    bpy = None
    HAVE_BPY = False

    def persistent(fn):      # no-op stand-in so the decorators below parse
        return fn

# --- always available, never touches bpy ---
from . import config          # noqa: E402
from . import controller      # noqa: E402
from . import inverse_kinematics  # noqa: E402
from . import kinematics      # noqa: E402
from . import serial_link     # noqa: E402

__all__ = ["config", "controller", "kinematics", "inverse_kinematics",
           "serial_link", "HAVE_BPY"]


if not HAVE_BPY:
    def register():
        raise RuntimeError("robot_arm_twin needs Blender: bpy is not available")

    def unregister():
        pass

else:
    # Reload support, so editing a module and re-enabling the add-on takes effect.
    if "rig" in locals():
        import importlib
        for _name in ("kinematics", "inverse_kinematics", "config",
                      "serial_link", "controller", "mouse_target",
                      "rig", "state", "properties", "operators", "panel"):
            if _name in locals():
                importlib.reload(locals()[_name])

    from . import mouse_target  # noqa: E402
    from . import operators   # noqa: E402
    from . import panel       # noqa: E402
    from . import properties  # noqa: E402
    from . import rig         # noqa: E402
    from . import state       # noqa: E402

    __all__ += ["rig", "state", "properties", "operators", "panel",
            "mouse_target"]

    # ----------------------------------------------------------------------
    # scene initialisation
    # ----------------------------------------------------------------------

    def _initialise_scene(scene):
        """Populate the panel from config and capture the rest pose, once."""
        st = getattr(scene, "robot_arm_twin", None)
        if st is None:
            return
        ctrl = state.get_controller()
        if ctrl is None:
            return
        # Always re-read the config file, even into an already-initialised
        # scene.  Blender persists the panel's calibration inside the .blend,
        # so skipping this left stale values (steps/rev, direction, limits)
        # sitting in the UI -- and because connect() lets panel values win
        # over disk, those stale values were pushed to the firmware, silently
        # undoing any edit made to arm_config.json between sessions.
        # arm_config.json is the source of truth at load time; the panel is a
        # view of it, and "Save Calibration" is how edits go the other way.
        properties.populate_from_config(scene, ctrl)
        ok, msg = rig.ensure_rest(ctrl.model, ctrl.cfg, scene)
        st.rest_text = msg
        if ok:
            rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, scene)

        # Blender persists the slider values inside the .blend, but a fresh
        # controller always starts at zero and save_pre leaves the model at
        # rest.  Reopening a file saved mid-pose would therefore show stale
        # angles over an unposed model -- and nudging any one slider would
        # push all three stale values at the hardware.  Reconcile to the
        # controller, which is the only one that knows what was commanded.
        with state.suspend_updates():
            for jp, value in zip(st.joints, ctrl.desired):
                jp.angle = value
            st.ik_verified = False
            st.ik_previewed_for = ""

    @persistent
    def _on_load_post(_dummy):
        """A new .blend means a new scene, a new rest pose and a dead link."""
        state.shutdown()
        scene = bpy.context.scene
        if scene is None:
            return
        _initialise_scene(scene)
        st = getattr(scene, "robot_arm_twin", None)
        if st is not None:
            with state.suspend_updates():
                st.mouse_target_active = False
                st.mouse_physical_sync = False
                st.link_state = serial_link.DISCONNECTED
                st.status_text = "Idle"
                st.error_text = ""
                st.estopped = False
                st.armed = False

    @persistent
    def _on_save_pre(_dummy):
        """
        Put the model back to rest before the file is written.

        Without this, saving mid-pose bakes the pose into the .blend.  The
        rest transforms are stored on the scene so the pose is recoverable
        either way, but the file should look the way it was modelled.
        """
        ctrl = state.get_controller(create=False)
        scene = bpy.context.scene
        if ctrl is None or scene is None or not ctrl.model.has_rest():
            return
        rig.restore_rest(ctrl.model, scene)

    @persistent
    def _on_save_post(_dummy):
        """Re-apply the working pose once the save completes."""
        ctrl = state.get_controller(create=False)
        scene = bpy.context.scene
        if ctrl is None or scene is None or not ctrl.model.has_rest():
            return
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, scene)

    def _deferred_init():
        """Run once after registration, when a scene definitely exists."""
        scene = getattr(bpy.context, "scene", None)
        if scene is not None:
            try:
                _initialise_scene(scene)
            except Exception:
                import traceback
                traceback.print_exc()
        return None      # returning None unregisters this timer

    # ----------------------------------------------------------------------
    # registration
    # ----------------------------------------------------------------------

    _HANDLERS = (
        ("load_post", _on_load_post),
        ("save_pre", _on_save_pre),
        ("save_post", _on_save_post),
    )

    def register():
        properties.register()
        operators.register()
        panel.register()

        for name, fn in _HANDLERS:
            handlers = getattr(bpy.app.handlers, name)
            if fn not in handlers:
                handlers.append(fn)

        bpy.app.timers.register(_deferred_init, first_interval=0.1)

    def unregister():
        # Safety: never leave motors energised or a worker thread running.
        try:
            state.shutdown()
        except Exception:
            pass

        for name, fn in _HANDLERS:
            handlers = getattr(bpy.app.handlers, name)
            if fn in handlers:
                handlers.remove(fn)

        panel.unregister()
        operators.unregister()
        properties.unregister()


if __name__ == "__main__":
    register()
