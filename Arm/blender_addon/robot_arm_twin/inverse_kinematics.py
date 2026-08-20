"""
inverse_kinematics.py -- closed-form IK for this specific 3-DOF arm.

No `bpy`, no `mathutils`, no solver library.  The geometry admits an exact
analytical solution, so that is what this is.

The model
---------
Measured from the .blend, the chain is:

    tip(theta) = b + Rz(t1) . [ u + Ry(t2) . ( v + Ry(t3) . w ) ]

    b = a point on the base axis        (P1.x, P1.y, 0)
    u = P2 - b        shoulder pivot, relative to the base axis
    v = P3 - P2       upper-arm link
    w = tip - P3      fore-arm / tool link

This is *not* a textbook planar arm.  Both pitch joints turn about +Y, and a
rotation about Y leaves the Y component of any vector untouched.  So the Y
parts of u, v and w are constant offsets that no joint can change:

    C = u.y + v.y + w.y

is invariant.  Everything else happens in the X-Z plane, where the effective
link lengths are the *projected* ones:

    L2 = |(v.x, v.z)|      not |v|
    L3 = |(w.x, w.z)|      not |w|

Using the 3-D lengths here would be wrong, and would put the solution
consistently off.

Solving it
----------
Write q = u + Ry(t2)(v + Ry(t3) w), so tip = b + Rz(t1) q.  Let d = target - b.

1. Rz preserves Z and preserves radius in XY, and q.y == C always, so

       q.z = d.z
       q.x = +/- sqrt(d.x^2 + d.y^2 - C^2)          (two base branches)
       t1  = atan2(d.y, d.x) - atan2(C, q.x)

   The square root needs d.x^2 + d.y^2 >= C^2: targets nearer the base axis
   than |C| are unreachable at any joint angle -- a dead cylinder.

2. What is left is a planar 2-link problem in X-Z.  Treating (x, z) as the
   complex number x + iz, a rotation Ry(t) is exactly multiplication by
   e^-it, so with P = (q - u) projected to X-Z, V = v_xz, W = w_xz:

       P = e^-i.t2 ( V + e^-i.t3 W )

   Taking moduli kills t2 and leaves the law of cosines:

       |P|^2 = L2^2 + L3^2 + 2 L2 L3 cos(psi - t3),   psi = arg W - arg V

   giving t3 = psi -/+ acos(kappa)                    (two elbow branches)
   and then t2 = arg(V + e^-i.t3 W) - arg(P).

Four candidates in total.  All are returned; the caller picks one that is
inside the joint limits and closest to where the arm already is.
"""

from __future__ import annotations

import cmath
import math

EPS = 1e-9

# Which branch reproduces the modelled rest pose.  Named A/B rather than
# "elbow up/down" because which one looks "up" depends on the target.
BRANCH_A = "A"
BRANCH_B = "B"


def wrap_deg(a):
    """Fold an angle into (-180, 180]."""
    a = math.fmod(float(a) + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


class GeometryError(ValueError):
    """The arm is not the shape this solver was derived for."""


# --------------------------------------------------------------------------
# geometry, derived from the live model
# --------------------------------------------------------------------------

class ArmGeometry(object):
    """
    b, u, v, w, L2, L3, C -- all measured from the loaded model, never
    hard-coded, so editing the .blend or the config flows straight through.
    """

    __slots__ = ("b", "u", "v", "w", "L2", "L3", "C", "tip_rest",
                 "pivots", "axes")

    def __init__(self, b, u, v, w, tip_rest, pivots, axes):
        self.b = tuple(map(float, b))
        self.u = tuple(map(float, u))
        self.v = tuple(map(float, v))
        self.w = tuple(map(float, w))
        self.tip_rest = tuple(map(float, tip_rest))
        self.pivots = [tuple(map(float, p)) for p in pivots]
        self.axes = [tuple(map(float, a)) for a in axes]

        self.L2 = math.hypot(self.v[0], self.v[2])
        self.L3 = math.hypot(self.w[0], self.w[2])
        self.C = self.u[1] + self.v[1] + self.w[1]

        if self.L2 < EPS or self.L3 < EPS:
            raise GeometryError("degenerate link length (L2=%.6f L3=%.6f)"
                                % (self.L2, self.L3))

    # -- construction ------------------------------------------------------

    @classmethod
    def from_model(cls, model, tip_name="end_effector"):
        """
        Build from an ArmModel that already has its rest pose captured.

        Raises GeometryError if the axes are not the Z / Y / Y arrangement
        this closed form was derived for -- better a clear refusal than a
        silently wrong answer.
        """
        if len(model.joints) != 3:
            raise GeometryError("expected 3 joints, found %d" % len(model.joints))
        if not model.has_rest():
            raise GeometryError("rest pose not captured yet")
        rest = model.rest.get(tip_name)
        if rest is None:
            raise GeometryError("no rest transform for %r" % tip_name)

        j1, j2, j3 = model.joints
        _require_axis(j1, (0.0, 0.0, 1.0), "base")
        _require_axis(j2, (0.0, 1.0, 0.0), "middle")
        _require_axis(j3, (0.0, 1.0, 0.0), "upper")

        p1, p2, p3 = j1.pivot, j2.pivot, j3.pivot
        tip = (rest[0][3], rest[1][3], rest[2][3])

        b = (p1[0], p1[1], 0.0)
        u = (p2[0] - b[0], p2[1] - b[1], p2[2] - b[2])
        v = (p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2])
        w = (tip[0] - p3[0], tip[1] - p3[1], tip[2] - p3[2])
        return cls(b, u, v, w, tip, [p1, p2, p3], [j1.axis, j2.axis, j3.axis])

    # -- forward, in closed form ------------------------------------------

    def tip(self, angles_deg):
        """
        Tip position from angles, straight from the equation above.

        Deliberately independent of ArmModel.forward()'s matrix chain: if the
        two ever disagree, one of them is wrong and the tests will say so.
        """
        t1, t2, t3 = (math.radians(float(a)) for a in angles_deg[:3])
        wx, wy, wz = self.w
        rx, rz = _roty(wx, wz, t3)
        sx, sy, sz = self.v[0] + rx, self.v[1] + wy, self.v[2] + rz
        qx, qz = _roty(sx, sz, t2)
        qx += self.u[0]
        qy = self.u[1] + sy
        qz += self.u[2]
        c, s = math.cos(t1), math.sin(t1)
        return (self.b[0] + qx * c - qy * s,
                self.b[1] + qx * s + qy * c,
                self.b[2] + qz)

    # -- reach -------------------------------------------------------------

    @property
    def reach_min(self):
        return abs(self.L2 - self.L3)

    @property
    def reach_max(self):
        return self.L2 + self.L3

    @property
    def dead_radius(self):
        """Targets closer than this to the base axis can never be reached."""
        return abs(self.C)

    def describe(self):
        return ("L2=%.4f L3=%.4f  lateral offset C=%.4f  "
                "shoulder reach %.4f..%.4f" %
                (self.L2, self.L3, self.C, self.reach_min, self.reach_max))


def _require_axis(joint, expected, label):
    got = joint.axis
    n = math.sqrt(sum(c * c for c in got)) or 1.0
    unit = tuple(c / n for c in got)
    if max(abs(a - e) for a, e in zip(unit, expected)) > 1e-6:
        raise GeometryError(
            "%s joint turns about %s; this closed-form solver needs %s"
            % (label, _fmt(got), _fmt(expected)))


def _fmt(v):
    return "(%.3f, %.3f, %.3f)" % tuple(v)


def _roty(x, z, t):
    """Rotate the (x, z) part of a vector about +Y by t radians."""
    c, s = math.cos(t), math.sin(t)
    return (x * c + z * s, -x * s + z * c)


# --------------------------------------------------------------------------
# a candidate solution
# --------------------------------------------------------------------------

class IKSolution(object):

    __slots__ = ("angles", "base_branch", "elbow_branch", "reachable",
                 "in_limits", "violations", "position_error", "travel", "note")

    def __init__(self, angles, base_branch, elbow_branch):
        self.angles = [wrap_deg(a) for a in angles]
        self.base_branch = base_branch          # +1 or -1
        self.elbow_branch = elbow_branch        # BRANCH_A / BRANCH_B
        self.reachable = True
        self.in_limits = True
        self.violations = []                    # ["Base -14.2 < -10.0", ...]
        self.position_error = 0.0
        self.travel = 0.0
        self.note = ""

    def check_limits(self, joints):
        """Compare against the same soft limits the sliders and firmware use."""
        self.violations = []
        for j, a in zip(joints, self.angles):
            if a < j.min_deg - 1e-6:
                self.violations.append("%s %+.2f below min %+.2f"
                                       % (j.label, a, j.min_deg))
            elif a > j.max_deg + 1e-6:
                self.violations.append("%s %+.2f above max %+.2f"
                                       % (j.label, a, j.max_deg))
        self.in_limits = not self.violations
        return self.in_limits

    def __repr__(self):
        return ("<IK %s base%+d [%.2f, %.2f, %.2f] err=%.4f %s>"
                % (self.elbow_branch, self.base_branch, self.angles[0],
                   self.angles[1], self.angles[2], self.position_error,
                   "ok" if self.in_limits else "LIMIT"))


class IKResult(object):
    """Everything a caller (or a panel) needs to explain what happened."""

    __slots__ = ("target", "best", "candidates", "reachable", "message")

    def __init__(self, target):
        self.target = tuple(map(float, target))
        self.best = None
        self.candidates = []
        self.reachable = False
        self.message = ""

    @property
    def ok(self):
        return self.best is not None and self.best.in_limits

    @property
    def angles(self):
        return list(self.best.angles) if self.best else [0.0, 0.0, 0.0]


# --------------------------------------------------------------------------
# the solver
# --------------------------------------------------------------------------

def solve(geom, target, joints=None, current_angles=None, elbow="AUTO"):
    """
    Analytical IK.

    geom            ArmGeometry
    target          (x, y, z) in Blender world space
    joints          JointSpec list, for limit checking (optional)
    current_angles  used to prefer the solution needing least motion
    elbow           "AUTO", "A" or "B"

    Returns IKResult.  `best` may be None (nothing reachable), or may be a
    solution with in_limits False -- callers must check before commanding
    anything.
    """
    res = IKResult(target)
    tx, ty, tz = (float(c) for c in target)
    dx, dy, dz = tx - geom.b[0], ty - geom.b[1], tz - geom.b[2]

    # --- stage 1: base rotation --------------------------------------------
    radial_sq = dx * dx + dy * dy
    C = geom.C
    if radial_sq < C * C - 1e-9:
        res.message = (
            "Unreachable: target is %.3f from the base axis but the arm's "
            "fixed lateral offset is %.3f, so nothing closer than that can "
            "be reached." % (math.sqrt(radial_sq), abs(C)))
        return res

    qx_mag = math.sqrt(max(0.0, radial_sq - C * C))
    base_options = [1] if qx_mag < 1e-9 else [-1, 1]

    V = complex(geom.v[0], geom.v[2])
    W = complex(geom.w[0], geom.w[2])
    L2, L3 = geom.L2, geom.L3
    psi = cmath.phase(W) - cmath.phase(V)

    reach_notes = []
    for sign in base_options:
        qx = sign * qx_mag
        t1 = math.atan2(dy, dx) - math.atan2(C, qx)

        # --- stage 2: planar two-link in X-Z ------------------------------
        P = complex(qx - geom.u[0], dz - geom.u[2])
        r = abs(P)
        if r < 1e-9:
            reach_notes.append("degenerate: target sits on the shoulder pivot")
            continue
        if r > L2 + L3 + 1e-9:
            reach_notes.append("too far: needs %.3f, max %.3f" % (r, L2 + L3))
            continue
        if r < abs(L2 - L3) - 1e-9:
            reach_notes.append("too close: needs %.3f, min %.3f"
                               % (r, abs(L2 - L3)))
            continue

        kappa = (r * r - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
        kappa = max(-1.0, min(1.0, kappa))
        acos_k = math.acos(kappa)

        for branch, s in ((BRANCH_A, 1.0), (BRANCH_B, -1.0)):
            if elbow in (BRANCH_A, BRANCH_B) and branch != elbow:
                continue
            t3 = psi - s * acos_k
            S = V + cmath.exp(-1j * t3) * W
            if abs(S) < 1e-9:
                continue
            t2 = cmath.phase(S) - cmath.phase(P)

            sol = IKSolution(
                [math.degrees(t1), math.degrees(t2), math.degrees(t3)],
                sign, branch)

            # Verify by pushing the answer back through forward kinematics.
            got = geom.tip(sol.angles)
            sol.position_error = math.sqrt(sum((g - t) ** 2 for g, t
                                               in zip(got, (tx, ty, tz))))
            if current_angles:
                sol.travel = max(abs(a - c) for a, c
                                 in zip(sol.angles, current_angles))
            if joints:
                sol.check_limits(joints)
            res.candidates.append(sol)

    if not res.candidates:
        res.message = "Unreachable: " + ("; ".join(reach_notes) or "no solution")
        return res

    res.reachable = True

    # Prefer: inside limits, then accurate, then least movement.
    res.candidates.sort(key=lambda s: (not s.in_limits,
                                       s.position_error > 1e-4,
                                       s.travel))
    res.best = res.candidates[0]

    if res.best.in_limits:
        res.message = ("Reachable. Branch %s, base%+d, residual %.5f"
                       % (res.best.elbow_branch, res.best.base_branch,
                          res.best.position_error))
    else:
        res.message = ("Reachable geometrically, but outside joint limits: "
                       + "; ".join(res.best.violations))
    return res


# --------------------------------------------------------------------------
# diagnostics for the panel
# --------------------------------------------------------------------------

def reach_report(geom, target):
    """One short line explaining a target's status, limits aside."""
    dx = float(target[0]) - geom.b[0]
    dy = float(target[1]) - geom.b[1]
    dz = float(target[2]) - geom.b[2]
    radial = math.hypot(dx, dy)
    if radial < abs(geom.C) - 1e-9:
        return "inside the %.3f dead cylinder around the base axis" % abs(geom.C)
    qx = math.sqrt(max(0.0, radial * radial - geom.C * geom.C))
    r = math.hypot(qx - geom.u[0], dz - geom.u[2])
    if r > geom.reach_max:
        return "%.3f beyond full stretch (%.3f)" % (r - geom.reach_max, geom.reach_max)
    if r < geom.reach_min:
        return "%.3f inside the folded minimum (%.3f)" % (geom.reach_min - r, geom.reach_min)
    return "within reach (shoulder distance %.3f of %.3f..%.3f)" % (
        r, geom.reach_min, geom.reach_max)
