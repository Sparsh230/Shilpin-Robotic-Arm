"""
operators.py -- the buttons.

Connect / Disconnect / Emergency STOP are the three the panel puts up front.
The rest support calibration and recovery.

E-STOP never asks for confirmation, never blocks on the link, and works even
when the link is already broken: it latches locally first, then tries the
wire.  A stop button that can fail because the port is busy is not a stop
button.
"""

from __future__ import annotations

import os
import subprocess
import sys

import bpy
from bpy.types import Operator

from . import config as cfgmod
from . import inverse_kinematics as ikmod
from . import mouse_target as mtmod
from . import properties as props
from . import rig
from . import serial_link as sl
from . import state


def _controller_or_report(op):
    ctrl = state.get_controller()
    if ctrl is None:
        op.report({'ERROR'}, state.config_error() or "controller unavailable")
    return ctrl


# --------------------------------------------------------------------------
# connection
# --------------------------------------------------------------------------

class ARM_OT_connect(Operator):
    bl_idname = "robot_arm.connect"
    bl_label = "Connect"
    bl_description = "Open the serial port and push calibration to the firmware"

    @classmethod
    def poll(cls, context):
        ctrl = state.get_controller(create=False)
        return ctrl is None or not ctrl.link.is_busy

    def execute(self, context):
        ctrl = _controller_or_report(self)
        if ctrl is None:
            return {'CANCELLED'}
        st = context.scene.robot_arm_twin

        if not sl.have_serial():
            self.report({'ERROR'},
                        "pyserial is missing. Press 'Install pyserial' first")
            return {'CANCELLED'}

        ok, msg = state.ensure_rest(context)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        st.rest_text = msg

        port = props.selected_port(st)
        if not port:
            self.report({'ERROR'}, "no serial port selected")
            return {'CANCELLED'}

        # panel values win over whatever was last saved to disk
        props.write_back_to_config(context.scene, ctrl)
        ctrl.zero_on_connect = bool(st.zero_on_connect)

        if not ctrl.connect(port=port, arm_on_connect=st.arm_on_connect):
            self.report({'ERROR'}, ctrl.last_error or "connect failed")
            return {'CANCELLED'}

        state.start_timer()
        state.pull_pose_to_sliders(context)
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        self.report({'INFO'}, "Connecting to %s..." % port)
        return {'FINISHED'}


class ARM_OT_disconnect(Operator):
    bl_idname = "robot_arm.disconnect"
    bl_label = "Disconnect"
    bl_description = "Disarm the motors and close the serial port"

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is not None:
            ctrl.disconnect()
        state.stop_timer()
        st = context.scene.robot_arm_twin
        with state.suspend_updates():
            st.link_state = sl.DISCONNECTED
            st.status_text = "Disconnected"
            st.armed = False
            st.busy = False
        rig.tag_redraw(context)
        self.report({'INFO'}, "Disconnected")
        return {'FINISHED'}


# --------------------------------------------------------------------------
# safety
# --------------------------------------------------------------------------

class ARM_OT_estop(Operator):
    bl_idname = "robot_arm.estop"
    bl_label = "EMERGENCY STOP"
    bl_description = ("Halt all motion immediately, cut the coils and latch. "
                      "Requires Resume before the arm will move again")

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        st = context.scene.robot_arm_twin

        if ctrl is None:
            with state.suspend_updates():
                st.estopped = True
                st.armed = False
                st.mouse_physical_sync = False
                st.status_text = "EMERGENCY STOP (no link)"
            self.report({'WARNING'}, "E-STOP latched locally: no controller")
            return {'FINISHED'}

        reached = ctrl.emergency_stop()
        with state.suspend_updates():
            st.estopped = True
            st.armed = False
            # Mouse tracking must not resume driving the arm after a stop.
            st.mouse_physical_sync = False
            st.mouse_sent = False
            st.status_text = ctrl.status
            for jp, value in zip(st.joints, ctrl.desired):
                jp.angle = value
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        rig.tag_redraw(context)

        if reached:
            self.report({'WARNING'}, "EMERGENCY STOP sent")
        else:
            self.report({'WARNING'},
                        "E-STOP latched locally; no live link to the Arduino")
        return {'FINISHED'}


class ARM_OT_resume(Operator):
    bl_idname = "robot_arm.resume"
    bl_label = "Resume"
    bl_description = "Clear the emergency stop latch and re-arm the motors"

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            return {'CANCELLED'}
        if not ctrl.resume(rearm=True):
            self.report({'ERROR'}, ctrl.last_error or "resume failed")
            return {'CANCELLED'}
        with state.suspend_updates():
            for jp, value in zip(context.scene.robot_arm_twin.joints, ctrl.desired):
                jp.angle = value
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        rig.tag_redraw(context)
        self.report({'INFO'}, "Resumed")
        return {'FINISHED'}


class ARM_OT_home(Operator):
    bl_idname = "robot_arm.home"
    bl_label = "Go Home"
    bl_description = "Drive every joint back to 0 degrees"

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            return {'CANCELLED'}
        ctrl.go_home()
        with state.suspend_updates():
            for jp in context.scene.robot_arm_twin.joints:
                jp.angle = 0.0
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        rig.tag_redraw(context)
        return {'FINISHED'}


class ARM_OT_set_home(Operator):
    bl_idname = "robot_arm.set_home"
    bl_label = "Set Home Here"
    bl_description = ("Declare the arm's current physical pose to be zero. "
                      "Use after positioning it by hand")

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is None or not ctrl.is_connected:
            self.report({'ERROR'}, "not connected")
            return {'CANCELLED'}
        if not ctrl.set_zero_here():
            self.report({'ERROR'}, ctrl.last_error or "could not set home")
            return {'CANCELLED'}
        with state.suspend_updates():
            for jp in context.scene.robot_arm_twin.joints:
                jp.angle = 0.0
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        rig.tag_redraw(context)
        self.report({'INFO'}, "Home set at the current pose")
        return {'FINISHED'}


# --------------------------------------------------------------------------
# rest pose / viewport
# --------------------------------------------------------------------------

class ARM_OT_capture_rest(Operator):
    bl_idname = "robot_arm.capture_rest"
    bl_label = "Capture Rest Pose"
    bl_description = ("Snapshot the current object transforms as the zero pose. "
                      "Only do this with the model unposed")

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        ctrl = _controller_or_report(self)
        if ctrl is None:
            return {'CANCELLED'}
        ok, msg = rig.ensure_rest(ctrl.model, ctrl.cfg, context.scene, recapture=True)
        context.scene.robot_arm_twin.rest_text = msg
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class ARM_OT_restore_rest(Operator):
    bl_idname = "robot_arm.restore_rest"
    bl_label = "Restore Model"
    bl_description = ("Put every object back to its captured rest transform. "
                      "Use before saving the .blend")

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            return {'CANCELLED'}
        ok, msg = state.ensure_rest(context)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        rig.restore_rest(ctrl.model, context.scene)
        ctrl.pose_for_viewport = [0.0] * ctrl.n_joints
        with state.suspend_updates():
            for jp in context.scene.robot_arm_twin.joints:
                jp.angle = 0.0
        rig.tag_redraw(context)
        self.report({'INFO'}, "Model restored to rest")
        return {'FINISHED'}


class ARM_OT_clear_rest(Operator):
    bl_idname = "robot_arm.clear_rest"
    bl_label = "Clear Stored Rest"
    bl_description = ("Remove the rest-pose custom property this add-on stores "
                      "on the scene, leaving the .blend untouched by it")

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        rig.clear_rest(context.scene)
        context.scene.robot_arm_twin.rest_text = "not captured"
        self.report({'INFO'}, "Stored rest pose removed")
        return {'FINISHED'}


# --------------------------------------------------------------------------
# configuration
# --------------------------------------------------------------------------

class ARM_OT_save_config(Operator):
    bl_idname = "robot_arm.save_config"
    bl_label = "Save Calibration"
    bl_description = "Write the panel's calibration and safety values to arm_config.json"

    def execute(self, context):
        ctrl = _controller_or_report(self)
        if ctrl is None:
            return {'CANCELLED'}
        cfg = props.write_back_to_config(context.scene, ctrl)
        try:
            cfgmod.validate(cfg)
            path = cfgmod.save(cfg)
        except Exception as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        self.report({'INFO'}, "Saved %s" % os.path.basename(path))
        return {'FINISHED'}


class ARM_OT_reload_config(Operator):
    bl_idname = "robot_arm.reload_config"
    bl_label = "Reload Config"
    bl_description = "Re-read arm_config.json from disk. Disconnects first"

    def execute(self, context):
        ctrl = state.reload_config()
        if ctrl is None:
            self.report({'ERROR'}, state.config_error() or "reload failed")
            return {'CANCELLED'}
        props.populate_from_config(context.scene, ctrl)
        ok, msg = state.ensure_rest(context)
        context.scene.robot_arm_twin.rest_text = msg
        rig.tag_redraw(context)
        self.report({'INFO'}, "Config reloaded")
        return {'FINISHED'}


class ARM_OT_push_settings(Operator):
    bl_idname = "robot_arm.push_settings"
    bl_label = "Apply to Firmware"
    bl_description = "Send the current calibration, limits and speed to the Arduino"

    def execute(self, context):
        ctrl = state.get_controller(create=False)
        if ctrl is None or not ctrl.is_connected:
            self.report({'ERROR'}, "not connected")
            return {'CANCELLED'}
        props.write_back_to_config(context.scene, ctrl)
        # CAL resets the firmware's step counter, so this re-homes by definition
        cmds = cfgmod.build_init_commands(ctrl.cfg, zero_on_connect=True,
                                          arm_on_connect=context.scene.
                                          robot_arm_twin.arm_on_connect)
        failed = []
        for c in cmds:
            r = ctrl.link.send(c, timeout=2.0)
            if not r.ok:
                failed.append("%s -> %s" % (c, r.text))
        ctrl.desired = [0.0] * ctrl.n_joints
        ctrl.commanded = [0.0] * ctrl.n_joints
        ctrl.actual = [0.0] * ctrl.n_joints
        ctrl.pose_for_viewport = [0.0] * ctrl.n_joints
        with state.suspend_updates():
            for jp in context.scene.robot_arm_twin.joints:
                jp.angle = 0.0
        rig.apply_pose(ctrl.model, ctrl.pose_for_viewport, context.scene)
        rig.tag_redraw(context)

        if failed:
            self.report({'WARNING'}, "; ".join(failed[:3]))
            return {'FINISHED'}
        self.report({'INFO'}, "Settings applied; position re-zeroed")
        return {'FINISHED'}


class ARM_OT_refresh_ports(Operator):
    bl_idname = "robot_arm.refresh_ports"
    bl_label = "Refresh Ports"
    bl_description = "Rescan for serial ports"

    def execute(self, context):
        ports = sl.list_ports()
        rig.tag_redraw(context)
        self.report({'INFO'}, "Found %d port(s): %s"
                    % (len(ports), ", ".join(p for p, _ in ports) or "none"))
        return {'FINISHED'}


class ARM_OT_install_pyserial(Operator):
    bl_idname = "robot_arm.install_pyserial"
    bl_label = "Install pyserial"
    bl_description = ("Install pyserial into a private folder beside this add-on. "
                      "Needs an internet connection. No admin rights required")

    def execute(self, context):
        if sl.have_serial():
            self.report({'INFO'}, "pyserial is already available")
            return {'FINISHED'}

        target = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_vendor")
        os.makedirs(target, exist_ok=True)
        exe = sys.executable
        try:
            subprocess.run([exe, "-m", "ensurepip", "--user"],
                           capture_output=True, timeout=180)
            proc = subprocess.run(
                [exe, "-m", "pip", "install", "--upgrade",
                 "--target", target, "pyserial"],
                capture_output=True, text=True, timeout=300)
        except Exception as exc:
            self.report({'ERROR'}, "pip failed: %s" % exc)
            return {'CANCELLED'}

        if proc.returncode != 0:
            tail = (proc.stderr or proc.stdout or "").strip().splitlines()
            self.report({'ERROR'}, "pip failed: %s" % (tail[-1] if tail else "unknown"))
            return {'CANCELLED'}

        if target not in sys.path:
            sys.path.insert(0, target)
        if sl.have_serial():
            self.report({'INFO'}, "pyserial installed into %s" % target)
            rig.tag_redraw(context)
            return {'FINISHED'}
        self.report({'ERROR'}, "pyserial installed but still not importable; "
                               "restart Blender")
        return {'CANCELLED'}



# --------------------------------------------------------------------------
# inverse kinematics
# --------------------------------------------------------------------------

# How closely the solution must reproduce the target before it counts as
# verified.  The solver is exact, so anything above this means the geometry
# or the rest pose is not what the solver thinks it is.
IK_VERIFY_TOLERANCE = 1e-4


def _target_key(st):
    return "%.4f,%.4f,%.4f" % (st.ik_target_x, st.ik_target_y, st.ik_target_z)


def _mark_previewed(st):
    """Record that the model is standing at the current target."""
    with state.suspend_updates():
        st.ik_previewed_for = _target_key(st)


def _ik_clear(st, status, detail=""):
    with state.suspend_updates():
        st.ik_verified = False
        st.ik_reachable = False
        st.ik_in_limits = False
        st.ik_status = status
        st.ik_detail = detail
        st.ik_solved_for = ""


def _ik_solve(context, op):
    """
    Shared front half: build geometry, solve, verify, publish to the panel.

    Returns (result, angles); angles is None if there is nothing usable.
    Moves absolutely nothing -- each caller decides what to do with the answer.
    """
    st = context.scene.robot_arm_twin
    ctrl = state.get_controller()
    if ctrl is None:
        _ik_clear(st, "No controller")
        op.report({'ERROR'}, state.config_error() or "no controller")
        return None, None

    geom, err = state.get_geometry(context)
    if geom is None:
        _ik_clear(st, "Geometry unavailable", err)
        op.report({'ERROR'}, err)
        return None, None

    target = (st.ik_target_x, st.ik_target_y, st.ik_target_z)
    result = ikmod.solve(geom, target, joints=ctrl.joints,
                         current_angles=list(ctrl.desired),
                         elbow=st.ik_elbow)

    if not result.reachable or result.best is None:
        with state.suspend_updates():
            st.ik_verified = False
            st.ik_reachable = False
            st.ik_in_limits = False
            st.ik_status = "UNREACHABLE"
            st.ik_detail = ikmod.reach_report(geom, target)
            st.ik_solved_for = ""
        op.report({'WARNING'}, result.message)
        return result, None

    best = result.best

    # Independent check: push the answer back through the matrix forward
    # kinematics that drives the viewport, not the closed form that produced
    # it.  Agreement means the twin really will land on the target.
    fk = ctrl.model.tip_position(
        best.angles, ctrl.cfg.get("end_effector_object", "end_effector"))
    residual = (max(abs(a - b) for a, b in zip(fk, target))
                if fk else float("inf"))
    verified = residual <= IK_VERIFY_TOLERANCE

    with state.suspend_updates():
        st.ik_reachable = True
        st.ik_in_limits = best.in_limits
        st.ik_base, st.ik_middle, st.ik_upper = best.angles
        st.ik_residual = residual
        st.ik_verified = bool(verified and best.in_limits)
        st.ik_solved_for = "%.4f,%.4f,%.4f" % target
        if not best.in_limits:
            st.ik_status = "OUTSIDE JOINT LIMITS"
            st.ik_detail = "; ".join(best.violations)
        elif not verified:
            st.ik_status = "VERIFICATION FAILED"
            st.ik_detail = ("forward kinematics lands %.5f from the target"
                            % residual)
        else:
            st.ik_status = "Solved and verified"
            st.ik_detail = ("branch %s, residual %.2e"
                            % (best.elbow_branch, residual))
    return result, best.angles


def _ik_hardware_ready():
    ctrl = state.get_controller(create=False)
    return bool(ctrl and ctrl.is_connected)


class ARM_OT_ik_get_current(Operator):
    bl_idname = "robot_arm.ik_get_current"
    bl_label = "Get Current Position"
    bl_description = ("Fill the target boxes with the end effector's current "
                      "position, as a starting point")

    def execute(self, context):
        ctrl = state.get_controller()
        if ctrl is None:
            self.report({'ERROR'}, state.config_error() or "no controller")
            return {'CANCELLED'}
        ok, msg = state.ensure_rest(context)
        if not ok:
            self.report({'ERROR'}, msg)
            return {'CANCELLED'}
        tip = ctrl.tip_position()
        if tip is None:
            self.report({'ERROR'}, "end effector position unavailable")
            return {'CANCELLED'}
        st = context.scene.robot_arm_twin
        # assigning these fires the update callback, which clears verification
        st.ik_target_x, st.ik_target_y, st.ik_target_z = tip
        self.report({'INFO'}, "Target set to (%.3f, %.3f, %.3f)" % tuple(tip))
        return {'FINISHED'}


class ARM_OT_ik_solve(Operator):
    bl_idname = "robot_arm.ik_solve"
    bl_label = "Solve Only"
    bl_description = ("Calculate the joint angles for the target. Moves "
                      "nothing at all -- neither the model nor the motors")

    def execute(self, context):
        result, angles = _ik_solve(context, self)
        if result is None or angles is None:
            return {'CANCELLED'}
        st = context.scene.robot_arm_twin
        self.report({'INFO'} if st.ik_verified else {'WARNING'}, st.ik_status)
        return {'FINISHED'}


class ARM_OT_ik_preview(Operator):
    bl_idname = "robot_arm.ik_preview"
    bl_label = "Preview in Blender"
    bl_description = ("Solve and pose the Blender model only. The physical "
                      "motors are never touched by this button")

    def execute(self, context):
        result, angles = _ik_solve(context, self)
        if angles is None:
            return {'CANCELLED'}
        st = context.scene.robot_arm_twin
        if not st.ik_in_limits:
            self.report({'ERROR'}, "Not previewed: " + st.ik_detail)
            return {'CANCELLED'}
        state.apply_ik_pose(context, angles, send=False)
        _mark_previewed(st)
        self.report({'INFO'}, "Previewed in Blender; motors untouched")
        return {'FINISHED'}


class ARM_OT_ik_go(Operator):
    bl_idname = "robot_arm.ik_go"
    bl_label = "GO TO TARGET"
    bl_description = ("Solve, check joint limits, pose the model, and -- only "
                      "when Preview is off and hardware is allowed -- move "
                      "the physical arm")

    def execute(self, context):
        st = context.scene.robot_arm_twin
        result, angles = _ik_solve(context, self)
        if result is None:
            return {'CANCELLED'}

        # Nothing has moved yet.  Everything below is a refusal gate, and
        # every one of them returns before any motor command is possible.
        if angles is None:
            self.report({'ERROR'}, "Unreachable target: " + st.ik_detail)
            return {'CANCELLED'}
        if not st.ik_in_limits:
            self.report({'ERROR'},
                        "Joint limits would be violated: " + st.ik_detail)
            return {'CANCELLED'}
        if not st.ik_verified:
            self.report({'ERROR'},
                        "Solution failed verification: " + st.ik_detail)
            return {'CANCELLED'}

        # The model must already be standing at this exact target from a
        # previous action. Re-solving inside this call cannot satisfy that,
        # so the gate cannot be short-circuited by the check that reads it.
        previewed = st.ik_previewed_for == _target_key(st)

        want_hardware = ((not st.ik_preview_only)
                         and st.ik_allow_hardware
                         and previewed)
        state.apply_ik_pose(context, angles, send=want_hardware)
        _mark_previewed(st)

        if not want_hardware:
            if st.ik_preview_only:
                why = "Preview only is on"
            elif not st.ik_allow_hardware:
                why = "IK is not allowed to drive the motors yet"
            else:
                why = "not yet previewed at this target - press GO again to move"
            self.report({'INFO'}, "Model moved; motors NOT moved (%s)" % why)
            return {'FINISHED'}

        if not _ik_hardware_ready():
            self.report({'WARNING'},
                        "Model moved; no serial link, so motors did not move")
            return {'FINISHED'}

        self.report({'INFO'}, "Moving arm to (%.3f, %.3f, %.3f)"
                    % (st.ik_target_x, st.ik_target_y, st.ik_target_z))
        return {'FINISHED'}



# --------------------------------------------------------------------------
# mouse target mode
# --------------------------------------------------------------------------

def _physical_block_reason(st, ctrl, angles):
    """
    Why this pose must NOT go to the motors, or None if it may.

    Checked on every single update, immediately before the send decision.
    Reachability and joint limits are already settled by the time this runs;
    what is left is link state and the per-command delta cap.
    """
    if not st.mouse_physical_sync:
        return "Physical Sync off"
    if st.estopped:
        return "emergency stop latched"
    if ctrl is None or not ctrl.is_connected:
        return "not connected"
    if not ctrl.armed:
        return "motors disarmed"

    cap = float(st.max_command_delta_deg)
    worst, worst_name = 0.0, ""
    for joint, want, have in zip(ctrl.joints, angles, ctrl.actual):
        d = abs(float(want) - float(have))
        if d > worst:
            worst, worst_name = d, joint.label
    if worst > cap:
        return "%s would jump %.1f deg, cap is %.1f" % (worst_name, worst, cap)
    return None


def _mouse_tick(context, st, ctrl, region, rv3d, mouse_x, mouse_y):
    """
    One update: mouse -> plane -> IK -> Blender (-> Arduino, only if allowed).

    Never raises; the modal loop must survive a bad frame.
    """
    scene = context.scene

    plane = bpy.data.objects.get(mtmod.PLANE_NAME)
    if plane is None:
        st.mouse_have_hit = False
        st.mouse_status = "Target plane missing - press Create Target Plane"
        return

    geom, err = state.get_geometry(context)
    if geom is None:
        st.mouse_have_hit = False
        st.mouse_status = err or "geometry unavailable"
        return

    hit = mtmod.mouse_to_plane(region, rv3d, mouse_x, mouse_y, plane)
    if hit is None:
        st.mouse_have_hit = False
        st.mouse_ik_ok = False
        st.mouse_status = "Cursor is not over the target plane"
        return

    st.mouse_have_hit = True
    st.mouse_target_x, st.mouse_target_y, st.mouse_target_z = hit

    # The marker shows where the arm is *trying* to reach, so it follows the
    # cursor even when the pose turns out to be impossible.
    mtmod.move_marker(scene, hit)

    result = ikmod.solve(geom, hit, joints=ctrl.joints,
                         current_angles=list(ctrl.desired), elbow=st.ik_elbow)

    if not result.reachable or result.best is None:
        st.mouse_ik_ok = False
        st.mouse_sent = False
        st.mouse_ik_status = "UNREACHABLE"
        st.mouse_status = mtmod_reach_note(geom, hit)
        return

    best = result.best
    if not best.in_limits:
        st.mouse_ik_ok = False
        st.mouse_sent = False
        st.mouse_ik_status = "OUT OF LIMITS"
        st.mouse_status = "; ".join(best.violations)[:70]
        return

    st.mouse_ik_ok = True
    st.mouse_ik_status = "solved"
    st.mouse_base, st.mouse_middle, st.mouse_upper = best.angles

    block = _physical_block_reason(st, ctrl, best.angles)
    send = block is None

    # One path to the hardware, shared with the XYZ IK panel and the sliders.
    state.apply_ik_pose(context, best.angles, send=send)

    st.mouse_sent = send
    if send:
        st.mouse_status = "Tracking - driving the arm"
    elif st.mouse_physical_sync:
        st.mouse_status = "Held: " + block
    else:
        st.mouse_status = "Tracking - Blender only"


def mtmod_reach_note(geom, target):
    try:
        return ikmod.reach_report(geom, target)[:70]
    except Exception:
        return "unreachable"


class ARM_OT_mouse_create_plane(Operator):
    bl_idname = "robot_arm.mouse_create_plane"
    bl_label = "Create Target Plane"
    bl_description = ("Add a visible target plane and aim marker, sized to the "
                      "arm's workspace. Move or scale them like any object")

    def execute(self, context):
        geom, err = state.get_geometry(context)
        if geom is None:
            self.report({'ERROR'}, err or "geometry unavailable")
            return {'CANCELLED'}
        st = context.scene.robot_arm_twin
        plane, marker = mtmod.ensure_targets(context.scene, geom,
                                             half_size=st.mouse_plane_half_size)
        rig.tag_redraw(context)
        self.report({'INFO'}, "Created %s and %s" % (plane.name, marker.name))
        return {'FINISHED'}


class ARM_OT_mouse_remove_plane(Operator):
    bl_idname = "robot_arm.mouse_remove_plane"
    bl_label = "Remove Target Plane"
    bl_description = "Delete the target plane and marker from the scene"

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        st = context.scene.robot_arm_twin
        if st.mouse_target_active:
            self.report({'ERROR'}, "stop Mouse Target mode first")
            return {'CANCELLED'}
        removed = mtmod.remove_targets(context.scene)
        rig.tag_redraw(context)
        self.report({'INFO'}, "Removed: %s" % (", ".join(removed) or "nothing"))
        return {'FINISHED'}


class ARM_OT_mouse_target_stop(Operator):
    bl_idname = "robot_arm.mouse_target_stop"
    bl_label = "Mouse Target: ON"
    bl_description = "Stop mouse tracking"

    def execute(self, context):
        st = context.scene.robot_arm_twin
        # The running modal loop watches this flag and shuts itself down.
        st.mouse_target_active = False
        st.mouse_physical_sync = False
        st.mouse_status = "Stopping..."
        rig.tag_redraw(context)
        return {'FINISHED'}


class ARM_OT_mouse_target(Operator):
    bl_idname = "robot_arm.mouse_target"
    bl_label = "Mouse Target: OFF"
    bl_description = ("Start mouse tracking: the end effector follows the "
                      "cursor across the target plane. Blender only until "
                      "Physical Sync is switched on. ESC stops it")

    _timer = None
    _hz = 0
    _mx = 0
    _my = 0
    _dirty = False

    def invoke(self, context, event):
        st = context.scene.robot_arm_twin
        if st.mouse_target_active:
            self.report({'WARNING'}, "Mouse Target is already running")
            return {'CANCELLED'}

        ctrl = state.get_controller()
        if ctrl is None:
            self.report({'ERROR'}, state.config_error() or "no controller")
            return {'CANCELLED'}

        geom, err = state.get_geometry(context)
        if geom is None:
            self.report({'ERROR'}, err or "geometry unavailable")
            return {'CANCELLED'}

        if bpy.data.objects.get(mtmod.PLANE_NAME) is None:
            mtmod.ensure_targets(context.scene, geom,
                                 half_size=st.mouse_plane_half_size)

        state.clear_mouse_abort()
        with state.suspend_updates():
            st.mouse_target_active = True
            # Always start Blender-only, whatever was left set last time.
            st.mouse_physical_sync = False
            st.mouse_sent = False
            st.mouse_ik_status = "waiting for the cursor"
            st.mouse_status = "Tracking - Blender only"

        self._mx, self._my = event.mouse_x, event.mouse_y
        self._dirty = True
        self._hz = max(1, int(st.mouse_update_hz))

        wm = context.window_manager
        self._timer = wm.event_timer_add(1.0 / self._hz, window=context.window)
        wm.modal_handler_add(self)
        self.report({'INFO'}, "Mouse Target ON (Blender only). ESC to stop")
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        st = getattr(context.scene, "robot_arm_twin", None)
        if st is None or state.mouse_abort() or not st.mouse_target_active:
            return self._finish(context, "Off")

        if event.type in {'ESC'}:
            return self._finish(context, "Off (ESC)")

        if event.type == 'MOUSEMOVE':
            self._mx, self._my = event.mouse_x, event.mouse_y
            self._dirty = True
            return {'PASS_THROUGH'}

        if event.type == 'TIMER':
            self._retime_if_needed(context, st)
            if self._dirty:
                self._dirty = False
                self._safe_tick(context, st)
            return {'PASS_THROUGH'}

        # Everything else belongs to Blender: never swallow ordinary input.
        return {'PASS_THROUGH'}

    def _retime_if_needed(self, context, st):
        want = max(1, int(st.mouse_update_hz))
        if want == self._hz:
            return
        wm = context.window_manager
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
        self._hz = want
        self._timer = wm.event_timer_add(1.0 / want, window=context.window)

    def _safe_tick(self, context, st):
        ctrl = state.get_controller(create=False)
        if ctrl is None:
            st.mouse_status = "no controller"
            return
        region, rv3d = mtmod.region_under_mouse(context.window, self._mx, self._my)
        if region is None:
            st.mouse_have_hit = False
            st.mouse_status = "Cursor is outside the 3D viewport"
            return
        try:
            _mouse_tick(context, st, ctrl, region, rv3d, self._mx, self._my)
        except Exception:
            import traceback
            traceback.print_exc()
            st.mouse_status = "update failed - see the system console"

    def _finish(self, context, why):
        wm = context.window_manager
        if self._timer is not None:
            try:
                wm.event_timer_remove(self._timer)
            except Exception:
                pass
            self._timer = None
        st = getattr(context.scene, "robot_arm_twin", None)
        if st is not None:
            with state.suspend_updates():
                st.mouse_target_active = False
                st.mouse_physical_sync = False
                st.mouse_sent = False
                st.mouse_ik_status = "idle"
                st.mouse_status = why
        rig.tag_redraw(context)
        return {'CANCELLED'}


CLASSES = (
    ARM_OT_connect, ARM_OT_disconnect, ARM_OT_estop, ARM_OT_resume,
    ARM_OT_home, ARM_OT_set_home,
    ARM_OT_capture_rest, ARM_OT_restore_rest, ARM_OT_clear_rest,
    ARM_OT_save_config, ARM_OT_reload_config, ARM_OT_push_settings,
    ARM_OT_refresh_ports, ARM_OT_install_pyserial,
    ARM_OT_ik_get_current, ARM_OT_ik_solve,
    ARM_OT_ik_preview, ARM_OT_ik_go,
    ARM_OT_mouse_create_plane, ARM_OT_mouse_remove_plane,
    ARM_OT_mouse_target, ARM_OT_mouse_target_stop,
)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(CLASSES):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
