"""
rig.py -- the only module that touches the Blender scene.

Design rule: the .blend is treated as read-only reference geometry.  We never
parent, never add modifiers, never edit meshes, never move origins.  Posing is
done by writing `matrix_world` on the four existing objects, computed from
their captured rest transforms.  `restore_rest()` puts everything back exactly.

The one thing written into the file is a scene custom property holding the
captured rest transforms.  Without it, saving the .blend while posed and
reopening it would make the posed state look like the rest state, and every
angle afterwards would be measured from the wrong place.  It is a handful of
floats, it touches no geometry, and 'Clear Stored Rest' removes it.
"""

from __future__ import annotations

import json

import bpy
from mathutils import Matrix, Vector

from . import kinematics as kin

REST_KEY = "robot_arm_twin_rest_pose"
REST_VERSION = 1


# --------------------------------------------------------------------------
# conversions
# --------------------------------------------------------------------------

def to_blender(m):
    """kinematics Mat4 (nested list) -> mathutils.Matrix"""
    return Matrix([[float(v) for v in row] for row in m])


def from_blender(m):
    """mathutils.Matrix -> kinematics Mat4"""
    return [[float(m[i][j]) for j in range(4)] for i in range(4)]


# --------------------------------------------------------------------------
# scene inspection
# --------------------------------------------------------------------------

def referenced_objects(cfg):
    """Every object name the config expects to find in the scene."""
    names = []
    for j in cfg.get("joints", []):
        for n in j.get("blender", {}).get("moves", []):
            if n not in names:
                names.append(n)
    for n in cfg.get("static_objects", []):
        if n not in names:
            names.append(n)
    tip = cfg.get("end_effector_object")
    if tip and tip not in names:
        names.append(tip)
    return names


def missing_objects(cfg, scene=None):
    """Config entries with no matching object -- the usual cause of 'nothing moves'."""
    scene = scene or bpy.context.scene
    present = {o.name for o in scene.objects}
    return [n for n in referenced_objects(cfg) if n not in present]


def representative_object(joint):
    """The object a user would grab to rotate this joint by hand."""
    return joint.moves[0] if joint.moves else None


# --------------------------------------------------------------------------
# rest pose: capture, persist, restore
# --------------------------------------------------------------------------

def capture_rest(cfg, scene=None):
    """
    Snapshot the current world transforms as the zero pose.

    Only meaningful when the arm is unposed -- call it on a freshly opened
    file, or after `restore_rest()`.
    """
    scene = scene or bpy.context.scene
    rest = {}
    for name in referenced_objects(cfg):
        ob = scene.objects.get(name)
        if ob is not None:
            rest[name] = from_blender(ob.matrix_world)
    return rest


def store_rest(rest, scene=None):
    scene = scene or bpy.context.scene
    scene[REST_KEY] = json.dumps({"version": REST_VERSION, "matrices": rest})


def load_rest(scene=None):
    scene = scene or bpy.context.scene
    raw = scene.get(REST_KEY)
    if not raw:
        return None
    try:
        data = json.loads(raw)
    except Exception:
        return None
    if data.get("version") != REST_VERSION:
        return None
    return data.get("matrices") or None


def clear_rest(scene=None):
    scene = scene or bpy.context.scene
    if REST_KEY in scene.keys():
        del scene[REST_KEY]


def ensure_rest(model, cfg, scene=None, recapture=False):
    """
    Populate `model.rest`, preferring the stored snapshot.

    Returns (ok, message).
    """
    scene = scene or bpy.context.scene
    missing = missing_objects(cfg, scene)
    if missing:
        return False, "objects not found in scene: %s" % ", ".join(missing)

    rest = None if recapture else load_rest(scene)
    source = "stored"
    if rest is None:
        rest = capture_rest(cfg, scene)
        store_rest(rest, scene)
        source = "captured"

    model.rest = {k: [list(map(float, row)) for row in v] for k, v in rest.items()}
    if not model.has_rest():
        return False, "rest pose is incomplete"
    return True, "rest pose %s (%d objects)" % (source, len(model.rest))


def restore_rest(model, scene=None):
    """Put every object back to its captured rest transform."""
    scene = scene or bpy.context.scene
    for name, m in model.rest.items():
        ob = scene.objects.get(name)
        if ob is not None:
            ob.matrix_world = to_blender(m)


# --------------------------------------------------------------------------
# posing
# --------------------------------------------------------------------------

def apply_pose(model, angles_deg, scene=None):
    """
    Drive the viewport to `angles_deg`.

    Returns the number of objects moved.  Objects named in the config but
    absent from the scene are skipped rather than raising -- a partially
    renamed scene should degrade, not explode.
    """
    scene = scene or bpy.context.scene
    if not model.has_rest():
        return 0
    world = model.forward(list(angles_deg))
    moved = 0
    for name, m in world.items():
        ob = scene.objects.get(name)
        if ob is None:
            continue
        ob.matrix_world = to_blender(m)
        moved += 1
    return moved


# --------------------------------------------------------------------------
# reading a hand-made viewport rotation back out
# --------------------------------------------------------------------------

AXIS_TOLERANCE = 0.02       # 1 - |dot| between measured and expected axis
MIN_ANGLE_RAD = 1.0e-4      # ignore numerical dust


def read_manual_angles(model, current_angles, scene=None, tolerance=AXIS_TOLERANCE):
    """
    Detect that the user rotated a joint object by hand in the viewport.

    For each joint we compare its representative object's actual transform
    against the transform forward kinematics predicts for `current_angles`.
    The residual is converted to a signed rotation about the joint axis.

    A residual that is not a rotation about the expected axis is rejected --
    that means the user grabbed the object and did something the mechanism
    cannot do, and silently reinterpreting it as joint motion would command
    the hardware somewhere unintended.

    Returns (angles, changed_indices, rejected_indices).
    """
    scene = scene or bpy.context.scene
    angles = list(current_angles)
    changed, rejected = [], []
    if not model.has_rest():
        return angles, changed, rejected

    predicted = model.forward(list(current_angles))

    for j in model.joints:
        name = representative_object(j)
        if not name:
            continue
        ob = scene.objects.get(name)
        exp = predicted.get(name)
        if ob is None or exp is None:
            continue

        expected = to_blender(exp)
        actual = ob.matrix_world
        if _matrices_close(actual, expected):
            continue

        try:
            residual = actual @ expected.inverted()
        except ValueError:
            rejected.append(j.index)
            continue

        axis, angle = residual.to_quaternion().to_axis_angle()
        if abs(angle) < MIN_ANGLE_RAD:
            continue

        want = Vector(j.axis).normalized()
        got = Vector(axis)
        if got.length < 1e-9:
            continue
        got.normalize()
        dot = got.dot(want)
        if abs(abs(dot) - 1.0) > tolerance:
            rejected.append(j.index)          # rotated about the wrong axis
            continue

        signed = angle if dot > 0 else -angle
        new_angle = j.clamp_angle(current_angles[j.index] + _degrees(signed))
        if abs(new_angle - angles[j.index]) > 1e-4:
            angles[j.index] = new_angle
            changed.append(j.index)

    return angles, changed, rejected


def _degrees(rad):
    import math
    return math.degrees(rad)


def _matrices_close(a, b, tol=1e-6):
    for i in range(4):
        for k in range(4):
            if abs(a[i][k] - b[i][k]) > tol:
                return False
    return True


# --------------------------------------------------------------------------
# viewport feedback
# --------------------------------------------------------------------------

def tip_location(model, angles_deg, cfg):
    """World-space end effector position for the given pose, or None."""
    tip = cfg.get("end_effector_object", "end_effector")
    if not model.has_rest() or tip not in model.rest:
        return None
    pos = model.tip_position(list(angles_deg), tip)
    return Vector(pos) if pos else None


def tag_redraw(context=None):
    """Force the 3D viewport and its sidebar to repaint."""
    context = context or bpy.context
    wm = getattr(context, "window_manager", None)
    if wm is None:
        return
    for window in wm.windows:
        for area in window.screen.areas:
            if area.type in {'VIEW_3D', 'PROPERTIES'}:
                area.tag_redraw()
