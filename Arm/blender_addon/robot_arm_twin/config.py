"""
config.py -- load / save / validate config/arm_config.json.

No `bpy`.  The add-on, the emulator and any future dashboard all read the same
file, so calibration lives in exactly one place.
"""

from __future__ import annotations

import json
import os

DEFAULT_FILENAME = "arm_config.json"

# Point this at a file (or a folder containing config/arm_config.json) to
# override discovery entirely.
ENV_VAR = "ROBOT_ARM_TWIN_CONFIG"

# Directories registered at runtime, searched before anything else.  The
# Blender half fills this in with the open .blend's folder, which is what
# lets a ZIP-installed add-on -- living under AppData, nowhere near the
# project -- still find the project's config.  Kept as plain paths so this
# module stays free of bpy.
_EXTRA_ROOTS = []


def add_search_root(path, ancestors=3):
    """
    Register `path` and a few of its ancestors as places to look for
    config/arm_config.json.  Later lookups see them first.
    """
    if not path:
        return
    probe = os.path.abspath(path)
    for _ in range(ancestors + 1):
        if os.path.isdir(probe) and probe not in _EXTRA_ROOTS:
            _EXTRA_ROOTS.append(probe)
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent


def clear_search_roots():
    del _EXTRA_ROOTS[:]


def _ancestors(start, levels=5):
    out, probe = [], os.path.abspath(start)
    for _ in range(levels):
        out.append(probe)
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    return out


def bundled_config_path():
    """The copy shipped inside the add-on, used only as a last resort."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "default_config", DEFAULT_FILENAME)


def is_bundled(cfg):
    """True when the loaded config is the add-on's shipped fallback."""
    path = (cfg or {}).get("_path", "")
    if not path:
        return False
    here = os.path.normcase(os.path.abspath(path))
    bundled = os.path.normcase(os.path.abspath(bundled_config_path()))
    return here == bundled


def candidate_paths():
    """Every place the config might be, most specific first."""
    cands = []

    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        cands.append(os.path.join(env, "config", DEFAULT_FILENAME)
                     if os.path.isdir(env) else env)

    here = os.path.dirname(os.path.abspath(__file__))
    for root in list(_EXTRA_ROOTS) + _ancestors(here):
        cands.append(os.path.join(root, "config", DEFAULT_FILENAME))
        cands.append(os.path.join(root, DEFAULT_FILENAME))

    # Shipped fallback: guarantees a ZIP-only install still starts up.
    cands.append(bundled_config_path())

    seen, ordered = set(), []
    for c in cands:
        key = os.path.normcase(os.path.abspath(c))
        if key not in seen:
            seen.add(key)
            ordered.append(c)
    return ordered


def default_config_path():
    """The first config that actually exists, else where one ought to live."""
    for c in candidate_paths():
        if os.path.isfile(c):
            return c
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(os.path.dirname(here)),
                        "config", DEFAULT_FILENAME)


# --------------------------------------------------------------------------
# defaults + validation
# --------------------------------------------------------------------------

SERIAL_DEFAULTS = {
    "port": "",
    "baud": 115200,
    "read_timeout_s": 1.0,
    "boot_settle_s": 2.5,
    "poll_interval_s": 0.25,
}

SAFETY_DEFAULTS = {
    "max_travel_deg": 10.0,
    "max_command_delta_deg": 10.0,
    "step_interval_us": 8000,
    "min_step_interval_us": 1200,
    "watchdog_ms": 4000,
    "hold_torque": False,
    "require_arm_before_move": True,
}

MOTOR_DEFAULTS = {
    "pins": [2, 3, 4, 5],
    "steps_per_rev": 4076.0,
    "gear_ratio": 1.0,
    "direction": 1,
    "backlash_steps": 0,
}


class ConfigError(ValueError):
    pass


def _merge_defaults(section, defaults):
    out = dict(defaults)
    out.update(section or {})
    return out


def load(path=None):
    """Read the config, filling in defaults.  Raises ConfigError on bad input."""
    path = path or default_config_path()
    if not os.path.isfile(path):
        looked = candidate_paths()
        raise ConfigError(
            "%s not found. Looked in: %s"
            % (DEFAULT_FILENAME,
               " | ".join(os.path.dirname(p) for p in looked[:6])))
    try:
        with open(path, "r", encoding="utf-8") as fh:
            cfg = json.load(fh)
    except Exception as exc:
        raise ConfigError("cannot parse %s: %s" % (path, exc))

    cfg["_path"] = path
    cfg["serial"] = _merge_defaults(cfg.get("serial"), SERIAL_DEFAULTS)
    cfg["safety"] = _merge_defaults(cfg.get("safety"), SAFETY_DEFAULTS)

    joints = cfg.get("joints")
    if not joints:
        raise ConfigError("config has no 'joints'")
    for i, j in enumerate(joints):
        j.setdefault("id", i)
        j.setdefault("name", "J%d" % i)
        j.setdefault("label", j["name"])
        j["motor"] = _merge_defaults(j.get("motor"), MOTOR_DEFAULTS)
        j.setdefault("limits", {})
        j["limits"].setdefault("min_deg", -SAFETY_DEFAULTS["max_travel_deg"])
        j["limits"].setdefault("max_deg", SAFETY_DEFAULTS["max_travel_deg"])
        j["limits"].setdefault("offset_deg", 0.0)
        b = j.setdefault("blender", {})
        if "axis" not in b or "pivot" not in b:
            raise ConfigError("joint %s is missing blender axis/pivot" % j["name"])
        b.setdefault("moves", [])

    validate(cfg)
    return cfg


def validate(cfg):
    """Sanity checks that would otherwise show up as physical misbehaviour."""
    problems = []
    saf = cfg["safety"]

    if saf["step_interval_us"] < saf["min_step_interval_us"]:
        problems.append(
            "step_interval_us (%s) is below min_step_interval_us (%s); the "
            "28BYJ-48 will stall and lose position"
            % (saf["step_interval_us"], saf["min_step_interval_us"]))
    if saf["max_command_delta_deg"] <= 0:
        problems.append("max_command_delta_deg must be > 0")
    if saf["max_travel_deg"] <= 0:
        problems.append("max_travel_deg must be > 0")

    used_pins = {}
    for j in cfg["joints"]:
        lim, mot = j["limits"], j["motor"]
        if lim["min_deg"] > lim["max_deg"]:
            problems.append("%s: min_deg > max_deg" % j["name"])
        if mot["steps_per_rev"] <= 0:
            problems.append("%s: steps_per_rev must be > 0" % j["name"])
        if mot["gear_ratio"] <= 0:
            problems.append("%s: gear_ratio must be > 0" % j["name"])
        if int(mot["direction"]) not in (-1, 1):
            problems.append("%s: direction must be 1 or -1" % j["name"])
        if len(mot["pins"]) != 4:
            problems.append("%s: needs exactly 4 pins" % j["name"])
        for p in mot["pins"]:
            if p in used_pins:
                problems.append("pin D%s used by both %s and %s"
                                % (p, used_pins[p], j["name"]))
            used_pins[p] = j["name"]

    if problems:
        raise ConfigError("; ".join(problems))
    return True


def save(cfg, path=None):
    """Write the config back, preserving key order and dropping runtime keys."""
    path = path or cfg.get("_path") or default_config_path()
    out = {k: v for k, v in cfg.items() if not k.startswith("_path")}
    tmp = path + ".tmp"
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, path)
    return path


# --------------------------------------------------------------------------
# handshake construction
# --------------------------------------------------------------------------

def build_init_commands(cfg, zero_on_connect=True, arm_on_connect=True):
    """
    The opening handshake pushed to the firmware right after connect.

    Order matters: CAL resets the firmware's step counter, so calibration must
    precede ZERO, and ZERO must precede ARM.
    """
    saf = cfg["safety"]
    cmds = ["ID"]

    for j in cfg["joints"]:
        m = j["motor"]
        cmds.append("CAL %d %.4f %.6f %d"
                    % (j["id"], float(m["steps_per_rev"]),
                       float(m["gear_ratio"]), int(m["direction"])))
    for j in cfg["joints"]:
        lim = j["limits"]
        cmds.append("LIM %d %.4f %.4f"
                    % (j["id"], float(lim["min_deg"]), float(lim["max_deg"])))

    cmds.append("SPEED %d" % max(int(saf["step_interval_us"]),
                                 int(saf["min_step_interval_us"])))
    cmds.append("DELTA %.4f" % float(saf["max_command_delta_deg"]))
    cmds.append("WD %d" % int(saf["watchdog_ms"]))
    cmds.append("HOLD %d" % (1 if saf["hold_torque"] else 0))

    if zero_on_connect:
        cmds.append("ZERO")
    if arm_on_connect:
        cmds.append("ARM 1")
    return cmds
