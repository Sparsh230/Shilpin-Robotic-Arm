"""
kinematics.py -- forward kinematics for the 3-DOF arm.

Deliberately free of `bpy` AND `mathutils`, so it can be imported by a plain
CPython process (tests, the future dashboard, an IK solver) as well as by
Blender.  It knows nothing about how the pose is displayed.

Matrix convention
-----------------
A transform is a 4x4 row-major nested list.  Points are column vectors, so a
point is transformed as ``M @ p``.  That matches ``mathutils.Matrix`` exactly,
which makes handing these to Blender a one-line conversion.

Model
-----
The arm has no parenting in the .blend file, so every object's rest transform
is its own world transform.  A joint is defined by an axis, a pivot point on
that axis, and the list of objects it carries.  Posing is therefore:

    world[obj] = R_n @ ... @ R_1 @ rest[obj]

where R_k is the rotation of joint k about its axis through its pivot, applied
only to the objects that joint carries.  Joints are ordered base-outwards, so
applying them in order composes the chain correctly.
"""

from __future__ import annotations

import math

# --------------------------------------------------------------------------
# minimal 4x4 / vec3 maths
# --------------------------------------------------------------------------

Mat4 = list  # list[list[float]] of shape 4x4
Vec3 = tuple  # tuple[float, float, float]


def mat_identity() -> Mat4:
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def mat_mul(a: Mat4, b: Mat4) -> Mat4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def mat_translate(t: Vec3) -> Mat4:
    m = mat_identity()
    m[0][3], m[1][3], m[2][3] = float(t[0]), float(t[1]), float(t[2])
    return m


def mat_rotate_axis(axis: Vec3, angle_rad: float) -> Mat4:
    """Rotation about `axis` through the origin (Rodrigues). Axis is normalised."""
    x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return mat_identity()
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    C = 1.0 - c
    return [[c + x * x * C,     x * y * C - z * s, x * z * C + y * s, 0.0],
            [y * x * C + z * s, c + y * y * C,     y * z * C - x * s, 0.0],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C,     0.0],
            [0.0,               0.0,               0.0,               1.0]]


def mat_about_pivot(axis: Vec3, pivot: Vec3, angle_rad: float) -> Mat4:
    """Rotation about an axis line that passes through `pivot`."""
    return mat_mul(mat_translate(pivot),
                   mat_mul(mat_rotate_axis(axis, angle_rad),
                           mat_translate((-pivot[0], -pivot[1], -pivot[2]))))


def mat_translation_of(m: Mat4) -> Vec3:
    return (m[0][3], m[1][3], m[2][3])


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else (hi if value > hi else value)


# --------------------------------------------------------------------------
# joint + arm model
# --------------------------------------------------------------------------

class JointSpec:
    """One revolute joint: an axis through a pivot, plus what it carries."""

    __slots__ = ("index", "name", "label", "axis", "pivot", "moves",
                 "min_deg", "max_deg", "offset_deg")

    def __init__(self, index, name, label, axis, pivot, moves,
                 min_deg=-10.0, max_deg=10.0, offset_deg=0.0):
        self.index = int(index)
        self.name = str(name)
        self.label = str(label)
        self.axis = (float(axis[0]), float(axis[1]), float(axis[2]))
        self.pivot = (float(pivot[0]), float(pivot[1]), float(pivot[2]))
        self.moves = list(moves)
        self.min_deg = float(min_deg)
        self.max_deg = float(max_deg)
        self.offset_deg = float(offset_deg)

    def clamp_angle(self, deg: float) -> float:
        return clamp(float(deg), self.min_deg, self.max_deg)

    def __repr__(self):
        return ("JointSpec(%d %s axis=%s pivot=%s moves=%s limits=[%.2f,%.2f])"
                % (self.index, self.name, self.axis, self.pivot,
                   self.moves, self.min_deg, self.max_deg))


class ArmModel:
    """Forward kinematics over a set of rest transforms."""

    def __init__(self, joints, rest_matrices=None):
        self.joints = list(joints)
        # name -> Mat4 of the object's transform at the zero pose
        self.rest = dict(rest_matrices or {})

    # -- rest pose ---------------------------------------------------------

    def set_rest(self, name: str, matrix: Mat4) -> None:
        self.rest[name] = [row[:] for row in matrix]

    def has_rest(self) -> bool:
        needed = set()
        for j in self.joints:
            needed.update(j.moves)
        return bool(needed) and needed.issubset(self.rest.keys())

    # -- forward kinematics -------------------------------------------------

    def joint_transforms(self, angles_deg):
        """Return [Mat4] -- the world-space rotation contributed by each joint."""
        out = []
        for j, a in zip(self.joints, angles_deg):
            out.append(mat_about_pivot(j.axis, j.pivot, math.radians(float(a))))
        return out

    def forward(self, angles_deg):
        """
        Map joint angles (degrees, base-outwards) to world transforms.

        Returns {object_name: Mat4} for every object any joint carries.
        Objects not carried by any joint (the static base) are simply absent.
        """
        result = {name: [row[:] for row in m] for name, m in self.rest.items()}

        # Compose tip-first.  Each joint's axis and pivot are recorded in the
        # REST frame, so a proximal joint must be applied *after* (i.e. to the
        # left of) every joint it carries:  world = R1 @ R2 @ R3 @ rest.
        # Pre-multiplying base-first would rotate joint 2 about the axis
        # joint 2 had before the base moved, which separates the shoulder.
        # Exact either way for single-joint moves; only combined poses differ.
        pairs = list(zip(self.joints, angles_deg))
        for j, a in reversed(pairs):
            a = float(a)
            if abs(a) < 1e-12:
                continue
            R = mat_about_pivot(j.axis, j.pivot, math.radians(a))
            for name in j.moves:
                base = result.get(name)
                if base is not None:
                    result[name] = mat_mul(R, base)
        return result

    def tip_position(self, angles_deg, tip_name="end_effector"):
        """World position of the end effector at the given pose, or None."""
        m = self.forward(angles_deg).get(tip_name)
        return mat_translation_of(m) if m else None

    # -- safety -------------------------------------------------------------

    def clamp_pose(self, angles_deg):
        """Clamp every angle into its joint's soft limits."""
        return [j.clamp_angle(a) for j, a in zip(self.joints, angles_deg)]

    def limit_delta(self, current_deg, target_deg, max_delta_deg):
        """
        Restrict how far each joint may move in one command.

        Returns (angles, was_limited).  Applied per joint, after clamping,
        so a large slider drag becomes a sequence of bounded moves rather
        than one lurch.
        """
        out, limited = [], False
        for cur, tgt in zip(current_deg, target_deg):
            d = float(tgt) - float(cur)
            if d > max_delta_deg:
                tgt, limited = cur + max_delta_deg, True
            elif d < -max_delta_deg:
                tgt, limited = cur - max_delta_deg, True
            out.append(float(tgt))
        return out, limited


# --------------------------------------------------------------------------
# construction from the config dict
# --------------------------------------------------------------------------

def joints_from_config(cfg: dict):
    """Build JointSpec objects from the parsed arm_config.json."""
    joints = []
    for jc in cfg.get("joints", []):
        b = jc.get("blender", {})
        lim = jc.get("limits", {})
        joints.append(JointSpec(
            index=jc.get("id", len(joints)),
            name=jc.get("name", "J%d" % len(joints)),
            label=jc.get("label", jc.get("name", "Joint")),
            axis=b.get("axis", (0.0, 0.0, 1.0)),
            pivot=b.get("pivot", (0.0, 0.0, 0.0)),
            moves=b.get("moves", []),
            min_deg=lim.get("min_deg", -10.0),
            max_deg=lim.get("max_deg", 10.0),
            offset_deg=lim.get("offset_deg", 0.0),
        ))
    joints.sort(key=lambda j: j.index)
    return joints
