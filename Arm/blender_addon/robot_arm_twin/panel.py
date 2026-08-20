"""
panel.py -- the control panel, in the 3D viewport sidebar (press N -> Robot Arm).

Layout priorities:
  1. Emergency STOP is always visible, always enabled, and never scrolls away.
  2. The three joint sliders sit directly under it.
  3. Everything else folds away.
"""

from __future__ import annotations

import bpy
from bpy.types import Panel

from . import serial_link as sl
from . import state

CATEGORY = "Robot Arm"


class _ArmPanel:
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = CATEGORY


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

class ARM_PT_main(_ArmPanel, Panel):
    bl_idname = "ARM_PT_main"
    bl_label = "3-DOF Arm Twin"

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin
        ctrl = state.get_controller(create=False)

        if ctrl is None:
            box = layout.box()
            box.alert = True
            box.label(text="Configuration problem", icon='ERROR')
            for line in _wrap(state.config_error() or "controller unavailable", 42):
                box.label(text=line)
            box.operator("robot_arm.reload_config", icon='FILE_REFRESH')
            return

        if not sl.have_serial():
            box = layout.box()
            box.alert = True
            box.label(text="pyserial not installed", icon='ERROR')
            box.label(text="Blender's Python has no serial module.")
            box.operator("robot_arm.install_pyserial", icon='IMPORT')
            layout.separator()

        from . import config as cfgmod
        if cfgmod.is_bundled(ctrl.cfg):
            box = layout.box()
            box.alert = True
            box.label(text="Using the add-on's built-in config", icon='ERROR')
            box.label(text="Your project's config was not found, so")
            box.label(text="calibration will not save to your project.")
            box.label(text="Open the project .blend, then Reload Config.")

        # ---- EMERGENCY STOP -------------------------------------------
        stop_row = layout.row()
        stop_row.scale_y = 2.0
        stop_row.alert = True
        stop_row.operator("robot_arm.estop", text="EMERGENCY STOP", icon='CANCEL')

        if st.estopped:
            warn = layout.box()
            warn.alert = True
            warn.label(text="E-STOP LATCHED - motion blocked", icon='ERROR')
            warn.operator("robot_arm.resume", icon='PLAY')

        # ---- connection ------------------------------------------------
        box = layout.box()
        head = box.row(align=True)
        head.label(text="Connection", icon='PLUGIN')
        head.label(text=_state_label(st.link_state), icon=_state_icon(st.link_state))

        row = box.row(align=True)
        row.prop(st, "port", text="")
        row.operator("robot_arm.refresh_ports", text="", icon='FILE_REFRESH')

        row = box.row(align=True)
        connected = st.link_state == sl.CONNECTED
        connecting = st.link_state == sl.CONNECTING
        sub = row.row(align=True)
        sub.enabled = not (connected or connecting)
        sub.operator("robot_arm.connect", icon='LINKED')
        sub = row.row(align=True)
        sub.enabled = connected or connecting
        sub.operator("robot_arm.disconnect", icon='UNLINKED')

        info = box.row(align=True)
        info.label(text="Armed" if st.armed else "Disarmed",
                   icon='CHECKMARK' if st.armed else 'BLANK1')
        info.label(text="Moving" if st.busy else "Idle",
                   icon='SORTTIME' if st.busy else 'BLANK1')

        if st.status_text:
            for line in _wrap(st.status_text, 40):
                box.label(text=line, icon='INFO')
        if st.error_text:
            err = box.box()
            err.alert = True
            for line in _wrap(st.error_text, 40):
                err.label(text=line, icon='ERROR')


# --------------------------------------------------------------------------
# joints
# --------------------------------------------------------------------------

class ARM_PT_joints(_ArmPanel, Panel):
    bl_idname = "ARM_PT_joints"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Joints"

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            return

        if not st.joints:
            layout.label(text="Not initialised", icon='ERROR')
            layout.operator("robot_arm.reload_config", icon='FILE_REFRESH')
            return

        layout.enabled = not st.estopped

        for i, jp in enumerate(st.joints):
            box = layout.box()
            head = box.row(align=True)
            head.label(text=jp.label, icon='CON_ROTLIKE')
            head.label(text="%+.2f deg" % jp.angle)

            box.prop(jp, "angle", text="", slider=True)

            if st.link_state == sl.CONNECTED:
                err = jp.angle - jp.actual
                row = box.row(align=True)
                row.label(text="actual %+.2f" % jp.actual)
                sub = row.row()
                sub.alert = abs(err) > 0.75
                sub.label(text="err %+.2f" % err)

        col = layout.column(align=True)
        col.operator("robot_arm.home", icon='HOME')
        col.prop(st, "live_sync", toggle=True,
                 icon='RADIOBUT_ON' if st.live_sync else 'RADIOBUT_OFF')
        col.prop(st, "follow_viewport", toggle=True,
                 icon='RADIOBUT_ON' if st.follow_viewport else 'RADIOBUT_OFF')

        tip = ctrl.tip_position()
        if tip:
            layout.label(text="Tip  X %+.2f  Y %+.2f  Z %+.2f" % tip,
                         icon='EMPTY_AXIS')


# --------------------------------------------------------------------------
# inverse kinematics
# --------------------------------------------------------------------------

class ARM_PT_ik(_ArmPanel, Panel):
    bl_idname = "ARM_PT_ik"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Inverse Kinematics"

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            layout.label(text="No controller", icon='ERROR')
            return

        layout.enabled = not st.estopped

        # ---- target ----------------------------------------------------
        box = layout.box()
        box.label(text="End effector target", icon='EMPTY_AXIS')
        col = box.column(align=True)
        col.prop(st, "ik_target_x")
        col.prop(st, "ik_target_y")
        col.prop(st, "ik_target_z")
        box.operator("robot_arm.ik_get_current", icon='EYEDROPPER')
        box.prop(st, "ik_elbow", text="")

        # ---- actions ---------------------------------------------------
        col = layout.column(align=True)
        col.operator("robot_arm.ik_solve", icon='PLAY')
        col.operator("robot_arm.ik_preview", icon='HIDE_OFF')

        go = layout.row()
        go.scale_y = 1.5
        live = (not st.ik_preview_only) and st.ik_allow_hardware
        go.alert = live
        go.operator("robot_arm.ik_go",
                    text="GO TO TARGET" + ("" if live else "  (model only)"),
                    icon='PLAY')

        # ---- result ----------------------------------------------------
        box = layout.box()
        head = box.row(align=True)
        head.label(text="Result", icon='DRIVER')
        if st.ik_verified:
            head.label(text="verified", icon='CHECKMARK')

        if not st.ik_reachable:
            sub = box.box()
            sub.alert = True
            sub.label(text=st.ik_status or "No solve yet", icon='ERROR')
            for line in _wrap(st.ik_detail, 40):
                if line:
                    sub.label(text=line)
        else:
            grid = box.column(align=True)
            grid.label(text="Target   %+.3f  %+.3f  %+.3f"
                            % (st.ik_target_x, st.ik_target_y, st.ik_target_z))
            grid.label(text="Base     %+8.3f deg" % st.ik_base)
            grid.label(text="Middle   %+8.3f deg" % st.ik_middle)
            grid.label(text="Upper    %+8.3f deg" % st.ik_upper)
            grid.label(text="Residual  %.2e" % st.ik_residual)

            if not st.ik_in_limits:
                sub = box.box()
                sub.alert = True
                sub.label(text="OUTSIDE JOINT LIMITS", icon='ERROR')
                for line in _wrap(st.ik_detail, 40):
                    sub.label(text=line)
                sub.label(text="Raise Travel limit in Safety & Speed,")
                sub.label(text="or pick a nearer target.")
            elif not st.ik_verified:
                sub = box.box()
                sub.alert = True
                sub.label(text=st.ik_status, icon='ERROR')
                for line in _wrap(st.ik_detail, 40):
                    sub.label(text=line)
            else:
                box.label(text=st.ik_detail, icon='INFO')

        # ---- safety gates ----------------------------------------------
        box = layout.box()
        box.label(text="Motion gate", icon='LOCKED')
        box.prop(st, "ik_preview_only",
                 icon='CHECKBOX_HLT' if st.ik_preview_only else 'CHECKBOX_DEHLT')

        row = box.row()
        row.enabled = st.ik_verified
        row.prop(st, "ik_allow_hardware",
                 icon='CHECKBOX_HLT' if st.ik_allow_hardware else 'CHECKBOX_DEHLT')

        previewed = (st.ik_previewed_for ==
                     "%.4f,%.4f,%.4f" % (st.ik_target_x, st.ik_target_y,
                                         st.ik_target_z))

        if not st.ik_verified:
            box.label(text="Verify a solve to unlock hardware.", icon='INFO')
        elif st.ik_preview_only:
            box.label(text="Preview on: motors will not move.", icon='INFO')
        elif not st.ik_allow_hardware:
            box.label(text="Hardware not allowed: model only.", icon='INFO')
        elif not previewed:
            box.label(text="Not previewed at this target yet.", icon='INFO')
            box.label(text="Press GO once to pose the model,")
            box.label(text="check it, then press GO again.")
        elif st.link_state != sl.CONNECTED:
            box.label(text="Armed for hardware, but not connected.", icon='ERROR')
        else:
            sub = box.box()
            sub.alert = True
            sub.label(text="LIVE: GO TO TARGET will move the arm", icon='ERROR')

        # ---- workspace --------------------------------------------------
        geom, err = state.get_geometry(context)
        if geom is not None:
            info = layout.box()
            info.label(text="Workspace", icon='MESH_CIRCLE')
            info.label(text="links  L2 %.3f   L3 %.3f" % (geom.L2, geom.L3))
            info.label(text="reach  %.3f .. %.3f" % (geom.reach_min, geom.reach_max))
            info.label(text="dead cylinder r = %.3f" % geom.dead_radius)


# --------------------------------------------------------------------------
# mouse target mode
# --------------------------------------------------------------------------

class ARM_PT_mouse(_ArmPanel, Panel):
    bl_idname = "ARM_PT_mouse"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Mouse Target"

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            layout.label(text="No controller", icon='ERROR')
            return

        from . import mouse_target as mtmod
        plane = bpy.data.objects.get(mtmod.PLANE_NAME)

        # ---- the two switches -----------------------------------------
        box = layout.box()

        row = box.row()
        row.scale_y = 1.4
        if st.mouse_target_active:
            row.operator("robot_arm.mouse_target_stop",
                         text="MOUSE TARGET: ON", icon='CHECKBOX_HLT')
        else:
            sub = row.row()
            sub.enabled = not st.estopped
            sub.operator("robot_arm.mouse_target",
                         text="MOUSE TARGET: OFF", icon='CHECKBOX_DEHLT')

        can_sync = (st.mouse_target_active and not st.estopped
                    and st.link_state == sl.CONNECTED and st.armed)
        row = box.row()
        row.scale_y = 1.2
        row.enabled = can_sync
        row.alert = st.mouse_physical_sync
        row.prop(st, "mouse_physical_sync",
                 text="PHYSICAL SYNC: %s" % ("ON" if st.mouse_physical_sync else "OFF"),
                 toggle=True,
                 icon='CHECKBOX_HLT' if st.mouse_physical_sync else 'CHECKBOX_DEHLT')

        if st.mouse_physical_sync:
            warn = box.box()
            warn.alert = True
            warn.label(text="LIVE: the mouse is driving the arm", icon='ERROR')
        elif not can_sync and st.mouse_target_active:
            box.label(text="Blender only. Physical Sync needs a",
                      icon='INFO')
            box.label(text="connected, armed, un-stopped arm.")

        # ---- status ----------------------------------------------------
        box = layout.box()
        box.label(text="Status", icon='INFO')
        col = box.column(align=True)
        col.label(text="MOUSE TARGET   %s"
                       % ("ON" if st.mouse_target_active else "OFF"))
        col.label(text="PHYSICAL SYNC  %s"
                       % ("ON" if st.mouse_physical_sync else "OFF"))

        row = col.row()
        row.alert = st.mouse_target_active and not st.mouse_ik_ok
        row.label(text="IK STATUS      %s" % (st.mouse_ik_status or "idle"))

        col = box.column(align=True)
        if st.mouse_have_hit:
            col.label(text="TARGET X %+.3f" % st.mouse_target_x)
            col.label(text="TARGET Y %+.3f" % st.mouse_target_y)
            col.label(text="TARGET Z %+.3f" % st.mouse_target_z)
        else:
            col.label(text="TARGET  --  cursor not on the plane")

        col = box.column(align=True)
        if st.mouse_ik_ok:
            col.label(text="BASE   %+8.3f deg" % st.mouse_base)
            col.label(text="MIDDLE %+8.3f deg" % st.mouse_middle)
            col.label(text="UPPER  %+8.3f deg" % st.mouse_upper)
        else:
            col.label(text="BASE / MIDDLE / UPPER  --")

        if st.mouse_status:
            sub = box.box()
            sub.alert = st.mouse_target_active and not st.mouse_ik_ok
            for line in _wrap(st.mouse_status, 40):
                sub.label(text=line)

        # ---- plane + rate ----------------------------------------------
        box = layout.box()
        box.label(text="Target plane", icon='MESH_PLANE')
        if plane is None:
            sub = box.box()
            sub.alert = True
            sub.label(text="No target plane in the scene", icon='ERROR')
        else:
            box.label(text="Using '%s'" % plane.name, icon='CHECKMARK')
            box.label(text="Move or scale it like any object.")

        row = box.row(align=True)
        row.enabled = not st.mouse_target_active
        row.operator("robot_arm.mouse_create_plane", icon='ADD')
        row.operator("robot_arm.mouse_remove_plane", text="", icon='TRASH')
        sub = box.row()
        sub.enabled = not st.mouse_target_active
        sub.prop(st, "mouse_plane_half_size")

        box.prop(st, "mouse_update_hz")
        box.label(text="= every %d ms" % int(1000.0 / max(1, st.mouse_update_hz)))

        if st.mouse_target_active:
            layout.label(text="Press ESC in the viewport to stop.", icon='INFO')


# --------------------------------------------------------------------------
# safety
# --------------------------------------------------------------------------

class ARM_PT_safety(_ArmPanel, Panel):
    bl_idname = "ARM_PT_safety"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Safety & Speed"

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin

        col = layout.column(align=True)
        col.prop(st, "max_travel_deg")
        col.prop(st, "max_command_delta_deg")

        col = layout.column(align=True)
        col.prop(st, "step_interval_us")
        col.label(text="= %.2f RPM at %.0f steps/rev"
                       % (_rpm(st), _first_spr(st)), icon='DRIVER_ROTATIONAL_DIFFERENCE')

        col = layout.column(align=True)
        col.prop(st, "watchdog_ms")
        col.prop(st, "hold_torque")

        col = layout.column(align=True)
        col.prop(st, "zero_on_connect")
        col.prop(st, "arm_on_connect")

        row = layout.row(align=True)
        row.operator("robot_arm.push_settings", icon='EXPORT')
        row.operator("robot_arm.set_home", icon='HOME')

        if st.step_interval_us < 2000:
            warn = layout.box()
            warn.alert = True
            warn.label(text="Very fast for a 28BYJ-48", icon='ERROR')
            warn.label(text="Below ~1200 us it will stall.")


# --------------------------------------------------------------------------
# calibration
# --------------------------------------------------------------------------

class ARM_PT_calibration(_ArmPanel, Panel):
    bl_idname = "ARM_PT_calibration"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Calibration"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin

        layout.label(text="Per-motor step counts differ slightly.", icon='INFO')

        for jp in st.joints:
            box = layout.box()
            box.label(text=jp.label, icon='SETTINGS')
            col = box.column(align=True)
            col.prop(jp, "steps_per_rev")
            col.prop(jp, "gear_ratio")
            col.prop(jp, "direction", text="Dir")
            row = col.row(align=True)
            row.prop(jp, "min_deg")
            row.prop(jp, "max_deg")
            col.label(text="%.3f steps per degree" % _steps_per_deg(jp))

        row = layout.row(align=True)
        row.operator("robot_arm.save_config", icon='FILE_TICK')
        row.operator("robot_arm.reload_config", icon='FILE_REFRESH')
        layout.operator("robot_arm.push_settings", icon='EXPORT')


# --------------------------------------------------------------------------
# model
# --------------------------------------------------------------------------

class ARM_PT_model(_ArmPanel, Panel):
    bl_idname = "ARM_PT_model"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Model"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        st = context.scene.robot_arm_twin
        ctrl = state.get_controller(create=False)

        layout.label(text="Rest: %s" % st.rest_text, icon='ARMATURE_DATA')

        col = layout.column(align=True)
        col.operator("robot_arm.restore_rest", icon='LOOP_BACK')
        col.operator("robot_arm.capture_rest", icon='PINNED')
        col.operator("robot_arm.clear_rest", icon='TRASH')

        layout.label(text="Restore before saving the .blend.", icon='INFO')

        if ctrl is None:
            return
        from . import rig
        missing = rig.missing_objects(ctrl.cfg, context.scene)
        if missing:
            box = layout.box()
            box.alert = True
            box.label(text="Objects missing from scene:", icon='ERROR')
            for n in missing:
                box.label(text=n)
        else:
            box = layout.box()
            for j in ctrl.joints:
                box.label(text="%s: %s axis @ %s"
                              % (j.label, _axis_name(j.axis),
                                 _short_vec(j.pivot)), icon='CON_ROTLIKE')


# --------------------------------------------------------------------------
# diagnostics
# --------------------------------------------------------------------------

class ARM_PT_diagnostics(_ArmPanel, Panel):
    bl_idname = "ARM_PT_diagnostics"
    bl_parent_id = "ARM_PT_main"
    bl_label = "Diagnostics"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            layout.label(text="No controller")
            return

        layout.label(text="Timer: %s" % ("running" if state.timer_running() else "stopped"))
        layout.label(text="Config: %s" % ctrl.cfg.get("_path", "?"))

        box = layout.box()
        box.label(text="Serial traffic", icon='CONSOLE')
        tail = ctrl.link.traffic_tail(12)
        if not tail:
            box.label(text="(nothing yet)")
        for _ts, direction, text in tail:
            box.label(text="%s %s" % (direction, text[:44]))


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _state_label(s):
    return {sl.CONNECTED: "Connected", sl.CONNECTING: "Connecting",
            sl.ERROR: "Error", sl.DISCONNECTED: "Offline"}.get(s, s)


def _state_icon(s):
    return {sl.CONNECTED: 'LINKED', sl.CONNECTING: 'SORTTIME',
            sl.ERROR: 'ERROR', sl.DISCONNECTED: 'UNLINKED'}.get(s, 'QUESTION')


def _first_spr(st):
    return st.joints[0].steps_per_rev if st.joints else 4076.0


def _rpm(st):
    spr = _first_spr(st)
    if st.step_interval_us <= 0 or spr <= 0:
        return 0.0
    steps_per_sec = 1e6 / float(st.step_interval_us)
    return steps_per_sec * 60.0 / spr


def _steps_per_deg(jp):
    return (jp.steps_per_rev * jp.gear_ratio) / 360.0


def _axis_name(axis):
    for name, vec in (("X", (1, 0, 0)), ("Y", (0, 1, 0)), ("Z", (0, 0, 1))):
        if all(abs(a - b) < 1e-6 for a, b in zip(axis, vec)):
            return "+" + name
        if all(abs(a + b) < 1e-6 for a, b in zip(axis, vec)):
            return "-" + name
    return "(%.2f, %.2f, %.2f)" % tuple(axis)


def _short_vec(v):
    return "(%.2f, %.2f, %.2f)" % tuple(v)


def _wrap(text, width):
    words, lines, cur = str(text).split(), [], ""
    for w in words:
        if len(cur) + len(w) + 1 > width:
            if cur:
                lines.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).strip()
    if cur:
        lines.append(cur)
    return lines or [""]


CLASSES = (ARM_PT_main, ARM_PT_joints, ARM_PT_ik, ARM_PT_mouse,
           ARM_PT_safety, ARM_PT_calibration, ARM_PT_model,
           ARM_PT_diagnostics)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
