"""
selftest.py -- check the host stack without Blender and without hardware.

    python tools/selftest.py

Exercises the config loader, the kinematics, the serial transport and the
protocol against the firmware emulator.  Run it after editing arm_config.json
or any of the core modules; if this passes, the only things left to go wrong
are wiring and calibration.
"""

from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "blender_addon"))

from robot_arm_twin import config as cfgmod          # noqa: E402
from robot_arm_twin import controller as ctrlmod     # noqa: E402
from robot_arm_twin import inverse_kinematics as ik   # noqa: E402
from robot_arm_twin import kinematics as kin         # noqa: E402
from robot_arm_twin import mouse_target as mt        # noqa: E402
from robot_arm_twin import serial_link as sl         # noqa: E402
from tools.firmware_emulator import install_emulator  # noqa: E402

PASS, FAIL = [], []


def check(name, condition, detail=""):
    (PASS if condition else FAIL).append(name)
    mark = "  ok  " if condition else " FAIL "
    print("[%s] %s%s" % (mark, name, ("  -- " + detail) if detail else ""))
    return condition


def near(a, b, tol=1e-3):
    return abs(float(a) - float(b)) <= tol


# --------------------------------------------------------------------------

def test_config():
    print("\n--- config ---")
    cfg = cfgmod.load()
    check("config loads", True, os.path.basename(cfg["_path"]))
    check("three joints", len(cfg["joints"]) == 3)
    check("joint names", [j["name"] for j in cfg["joints"]] == ["BASE", "MIDDLE", "UPPER"])

    pins = [p for j in cfg["joints"] for p in j["motor"]["pins"]]
    check("12 distinct pins", len(set(pins)) == 12, str(pins))
    check("pins match wiring",
          pins == [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13])

    saf = cfg["safety"]
    check("travel limit is 10 deg", near(saf["max_travel_deg"], 10.0))
    check("command delta is 10 deg", near(saf["max_command_delta_deg"], 10.0))
    check("speed is slow", saf["step_interval_us"] >= 4000,
          "%d us/step" % saf["step_interval_us"])
    check("speed above stall floor",
          saf["step_interval_us"] >= saf["min_step_interval_us"])

    # a config that should be rejected
    bad = cfgmod.load()
    bad["joints"][1]["motor"]["pins"] = [2, 3, 4, 5]     # clash with BASE
    try:
        cfgmod.validate(bad)
        check("duplicate pins rejected", False)
    except cfgmod.ConfigError as exc:
        check("duplicate pins rejected", True, str(exc)[:48])
    return cfg


def test_kinematics(cfg):
    print("\n--- kinematics ---")
    joints = kin.joints_from_config(cfg)
    model = kin.ArmModel(joints)

    check("BASE turns about +Z", joints[0].axis == (0.0, 0.0, 1.0))
    check("MIDDLE turns about +Y", joints[1].axis == (0.0, 1.0, 0.0))
    check("UPPER turns about +Y", joints[2].axis == (0.0, 1.0, 0.0))
    check("BASE carries 4 objects", len(joints[0].moves) == 4)
    check("UPPER carries 2 objects", len(joints[2].moves) == 2)
    check("base link is static",
          all("base" not in j.moves for j in joints))

    origins = {
        "base": (0.0, 0.0, 0.0),
        "bottom_joint": (-0.401576, 0.0, 1.384290),
        "lowerarm_join": (-0.784127, -1.105872, 2.897107),
        "upperarm_joint": (-0.410570, 0.0, 7.835087),
        "end_effector": (-4.107260, 0.313711, 7.583349),
    }
    for name, o in origins.items():
        model.set_rest(name, kin.mat_translate(o))
    check("rest pose complete", model.has_rest())

    # values measured from Blender itself
    expected = {
        (0, 0, 0): (-4.107, 0.314, 7.583),
        (25, 0, 0): (-3.893, -1.282, 7.583),
        (0, 20, 0): (-2.304, 0.314, 8.437),
        (0, 0, 30): (-3.738, 0.314, 9.465),
        (25, 15, -25): (-2.655, -0.705, 6.680),
        (10, 10, 10): (-3.132, -0.163, 8.723),
        (-8, 6, -9): (-3.500, 0.752, 7.324),
    }
    worst = 0.0
    for ang, exp in expected.items():
        got = model.tip_position(list(ang))
        worst = max(worst, max(abs(g - e) for g, e in zip(got, exp)))
    check("FK matches Blender (parented ground truth)", worst < 1e-3,
          "worst error %.5f" % worst)

    check("zero pose is identity",
          model.tip_position([0, 0, 0]) == model.tip_position([0, 0, 0]))
    check("soft limits clamp", model.clamp_pose([90, -90, 3]) == [10.0, -10.0, 3.0])

    stepped, limited = model.limit_delta([0, 0, 0], [45, -2, 0], 10.0)
    check("delta guard bounds a big move", stepped == [10.0, -2.0, 0.0] and limited)
    return model


def test_link(cfg):
    print("\n--- serial link + protocol ---")
    install_emulator(sl)
    cfg["serial"]["boot_settle_s"] = 0.2

    ctrl = ctrlmod.ArmController(config=cfg)
    ok = ctrl.connect(port="EMU")
    check("connect starts", ok)
    for _ in range(100):
        ctrl.service()
        if ctrl.state != sl.CONNECTING:
            break
        time.sleep(0.02)
    check("link reaches CONNECTED", ctrl.state == sl.CONNECTED, ctrl.state)
    check("armed after handshake", ctrl.armed)

    r = ctrl.link.send("ID")
    check("firmware identifies", r.ok and "ARM3DOF" in r.text, r.text)

    # absolute move
    ctrl.request_pose([6.0, -4.0, 2.0])
    for _ in range(200):
        ctrl.service()
        if not ctrl.busy and near(ctrl.actual[0], 6.0, 0.05):
            break
        time.sleep(0.01)
    check("absolute move lands",
          near(ctrl.actual[0], 6.0, 0.05) and near(ctrl.actual[1], -4.0, 0.05),
          str([round(v, 3) for v in ctrl.actual]))
    check("step quantisation is small",
          all(abs(e) < 0.06 for e in ctrl.tracking_error()),
          str([round(e, 4) for e in ctrl.tracking_error()]))

    # limits
    check("beyond-limit move refused",
          not ctrl.link.send("MOVE 45 0 0").ok)
    # the arm is at about +6, so -9.9 is a ~15.9 deg jump: over the 10 deg cap
    r = ctrl.link.send("MOVEJ 0 -9.9")
    check("oversized delta refused", not r.ok and "DELTA" in r.text, r.text)

    # e-stop
    ctrl.request_pose([9.0, 9.0, 9.0])
    ctrl.service()
    check("estop reaches the link", ctrl.emergency_stop())
    for _ in range(20):
        ctrl.service()
        time.sleep(0.01)
    check("estop latched", ctrl.estopped)
    check("move refused while latched", not ctrl.link.send("MOVE 0 0 0").ok)
    check("disarmed by estop", not ctrl.armed)

    check("resume clears latch", ctrl.resume(rearm=True))
    check("armed after resume", ctrl.armed)
    check("move accepted after resume", ctrl.link.send("MOVE 6 -4 2").ok)

    ctrl.disconnect()
    check("disconnect closes", ctrl.state == sl.DISCONNECTED)



def test_ik(cfg, model):
    """Closed-form inverse kinematics."""
    print("\n--- inverse kinematics ---")
    joints = kin.joints_from_config(cfg)
    geom = ik.ArmGeometry.from_model(model)

    check("geometry builds from the model", True, geom.describe())
    check("L2 is the PROJECTED upper-arm length", near(geom.L2, 4.952089, 1e-5),
          "%.6f" % geom.L2)
    check("L3 is the PROJECTED fore-arm length", near(geom.L3, 3.705252, 1e-5),
          "%.6f" % geom.L3)
    check("lateral offset C", near(geom.C, 0.313711, 1e-6), "%.6f" % geom.C)
    check("projected lengths differ from 3-D lengths",
          not near(geom.L2, 5.074066, 1e-3),
          "using |v| would be wrong by %.4f" % abs(geom.L2 - 5.074066))

    # closed-form FK must agree with the matrix FK that drives the viewport
    worst = 0.0
    for pose in [(0, 0, 0), (10, 10, 10), (25, 15, -25), (-8, 6, -9),
                 (45, 30, -40), (-60, 25, 55)]:
        a = geom.tip(pose)
        b = model.tip_position(list(pose))
        worst = max(worst, max(abs(x - y) for x, y in zip(a, b)))
    check("closed-form FK == matrix FK", worst < 1e-9, "worst %.3e" % worst)

    # round trip over the whole joint range
    import random
    random.seed(11)
    worst_res, failures = 0.0, 0
    cases = [(0, 0, 0), (5, 5, 5), (10, -10, 10), (-10, 10, -10), (25, 15, -25)]
    cases += [(random.uniform(-80, 80), random.uniform(-45, 45),
               random.uniform(-60, 60)) for _ in range(150)]
    for pose in cases:
        target = geom.tip(pose)
        res = ik.solve(geom, target, current_angles=list(pose))
        if not res.reachable:
            failures += 1
            continue
        worst_res = max(worst_res, min(c.position_error for c in res.candidates))
        recovered = min(res.candidates,
                        key=lambda c: max(abs(x - y) for x, y in zip(c.angles, pose)))
        if max(abs(x - y) for x, y in zip(recovered.angles, pose)) > 1e-6:
            failures += 1
    check("IK round trip over %d poses" % len(cases), failures == 0,
          "worst tip residual %.3e, %d failures" % (worst_res, failures))

    # unreachable cases must be refused, not fudged
    on_axis = (geom.b[0], geom.b[1], 7.6)
    check("target on the base axis refused",
          not ik.solve(geom, on_axis).reachable,
          ik.reach_report(geom, on_axis))
    far = (geom.b[0] - 40.0, 0.0, 7.6)
    check("target beyond full stretch refused",
          not ik.solve(geom, far).reachable, ik.reach_report(geom, far))

    # limits are reported, never silently clamped
    rest_tip = geom.tip((0, 0, 0))
    hard = (rest_tip[0] + 0.60, rest_tip[1] - 0.90, rest_tip[2] + 0.50)
    res = ik.solve(geom, hard, joints=joints, current_angles=[0, 0, 0])
    check("out-of-limit solution flagged, not clamped",
          res.reachable and res.best is not None and not res.best.in_limits,
          "; ".join(res.best.violations) if res.best else "no solution")

    # a small nearby target should be legal at the default +/-10 limits
    easy = (rest_tip[0] + 0.05, rest_tip[1] + 0.08, rest_tip[2] + 0.11)
    res = ik.solve(geom, easy, joints=joints, current_angles=[0, 0, 0])
    check("nearby target solves inside limits", res.ok,
          "angles %s" % [round(a, 3) for a in res.angles])
    if res.best:
        got = model.tip_position(res.best.angles)
        check("IK answer verified through matrix FK",
              max(abs(x - y) for x, y in zip(got, easy)) < 1e-6,
              "residual %.2e" % max(abs(x - y) for x, y in zip(got, easy)))

    # both elbow branches must reproduce the target
    res = ik.solve(geom, easy, joints=joints)
    check("both branches returned", len({c.elbow_branch for c in res.candidates}) == 2,
          "%d candidates" % len(res.candidates))
    check("every candidate reproduces the target",
          all(c.position_error < 1e-9 for c in res.candidates),
          "worst %.2e" % max(c.position_error for c in res.candidates))



def test_mouse_projection():
    """The mouse -> world XYZ conversion, independent of Blender."""
    print("\n--- mouse target projection ---")

    # straight down onto a horizontal plane
    h = mt.ray_plane_intersect((1.0, 2.0, 20.0), (0, 0, -1), (0, 0, 7.5), (0, 0, 1))
    check("perpendicular ray lands directly below",
          h is not None and near(h[0], 1.0, 1e-9) and near(h[1], 2.0, 1e-9)
          and near(h[2], 7.5, 1e-9), str(h))

    # 45 degrees: dropping 10 in Z must travel 10 in X
    h = mt.ray_plane_intersect((0.0, 0.0, 17.5), (1, 0, -1), (0, 0, 7.5), (0, 0, 1))
    check("45 deg ray offsets by the drop",
          h is not None and near(h[0], 10.0, 1e-6), str(h))

    # everything that must be refused rather than guessed at
    check("ray parallel to the plane misses",
          mt.ray_plane_intersect((0, 0, 20), (1, 0, 0), (0, 0, 7.5), (0, 0, 1)) is None)
    check("plane behind the viewer misses",
          mt.ray_plane_intersect((0, 0, 20), (0, 0, 1), (0, 0, 7.5), (0, 0, 1)) is None)
    check("zero direction refused",
          mt.ray_plane_intersect((0, 0, 20), (0, 0, 0), (0, 0, 7.5), (0, 0, 1)) is None)
    check("zero normal refused",
          mt.ray_plane_intersect((0, 0, 20), (0, 0, -1), (0, 0, 7.5), (0, 0, 0)) is None)
    check("absurdly distant hit refused",
          mt.ray_plane_intersect((0, 0, 0), (1, 0, -1e-5), (0, 0, -1e9),
                                 (0, 0, 1)) is None)

    # inputs need not be unit vectors
    h = mt.ray_plane_intersect((0, 0, 20), (0, 0, -7), (0, 0, 7.5), (0, 0, 5))
    check("unnormalised inputs handled", h is not None and near(h[2], 7.5, 1e-9))

    # a tilted plane still resolves
    h = mt.ray_plane_intersect((0, 0, 20), (0, 0, -1), (0, 0, 7.5), (1, 0, 1))
    check("tilted plane solves", h is not None and near(h[2], 7.5, 1e-6), str(h))


def main():
    print("=" * 62)
    print("3-DOF arm host self-test")
    print("=" * 62)
    cfg = test_config()
    model = test_kinematics(cfg)
    test_ik(cfg, model)
    test_mouse_projection()
    test_link(cfgmod.load())

    print("\n" + "=" * 62)
    print("%d passed, %d failed" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("   FAILED: %s" % f)
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
