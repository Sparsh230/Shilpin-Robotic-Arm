# 3-DOF Robotic Arm — Digital Twin

Blender drives three 28BYJ-48 steppers over USB. Move a joint in Blender, the
matching motor moves. Commands are **absolute joint angles**, not `+`/`-` nudges.

Built and verified against **Blender 5.2.0 LTS** and an **Arduino Uno**
(`COM4`, USB-SERIAL CH340). Includes closed-form inverse kinematics.

---

## What was found in the model

Inspected `blender file/3-DOF-robotic_arm.blend` — 5 objects, no armature, and
**no parenting at all**. Every object had `parent: None`, zero rotation, unit
scale. Rotating one object moved only that object; the kinematic chain did not
exist. The add-on supplies it without altering the file.

| Object | Type | Role |
|---|---|---|
| `base` | MESH | Fixed pedestal + base motor. **Never moves.** |
| `bottom_joint` | MESH | Shoulder motor body |
| `lowerarm_join` | MESH | Lower arm + elbow motor |
| `upperarm_joint` | MESH | Upper arm link |
| `end_effector` | EMPTY | Tip marker |

### Joint axes and pivots

Each was derived from **mating geometry**, not assumed. The modeller had
placed every object's origin exactly on its joint axis, confirmed against the
opposing part in each case.

| Joint | Motor pins | Axis | Pivot (world) | Carries |
|---|---|---|---|---|
| **Base** | D2–D5 | **+Z** | `(−0.4016, 0.0, 1.3843)` | bottom_joint, lowerarm_join, upperarm_joint, end_effector |
| **Middle** | D6–D9 | **+Y** | `(−0.7841, −1.1059, 2.8971)` | lowerarm_join, upperarm_joint, end_effector |
| **Upper** | D10–D13 | **+Y** | `(−0.4106, 0.0, 7.8351)` | upperarm_joint, end_effector |

Evidence:

- **Base** — `base` has a square post at z 1.309→1.663 centred `(−0.40103, −0.00001)`;
  `bottom_joint`'s origin is `(−0.40158, 0.0, 1.38429)`, matching to 0.0006.
  The base axis is vertical but **offset from the world origin** — it is the
  28BYJ-48's off-centre output shaft. Assuming `(0,0)` would have been wrong.
- **Middle** — `bottom_joint` has a stub protruding in **−Y** (y −1.208→−0.855)
  centred x −0.7847, z 2.89589; `lowerarm_join`'s origin matches in x and z to
  ~0.001, and its mounting collar (0.5 thick in Y) wraps that stub.
- **Upper** — `lowerarm_join`'s shaft face sits at y≈0.44, centred x −0.40297,
  z 7.82755; `upperarm_joint`'s hub is the mating disc (y 0.114→0.514) and its
  origin matches to 0.008.

Link lengths: shoulder→elbow **5.074**, elbow→tip **3.719**.

### Forward kinematics correction

The original `forward()` composed the joint rotations in reverse order
(R3.R2.R1 instead of R1.R2.R3). It was *exact* for single-joint moves, which is
why it went unnoticed, but for combined poses it rotated each joint about the
axis that joint had *before* its parent moved. At the ±10° limits that put the
tip up to **0.23 units** off and separated the shoulder joint by **0.034 units**.

Fixed by iterating tip-first. Verified against an independently built, genuinely
parented Blender chain: **0.000000** error at every pose tested, including
45°/30°/−40°. Single-joint results are bit-identical to before, and no angle
sent to a motor changed — only the viewport was wrong.

### Your geometry is not modified

Posing writes `matrix_world` on the four existing objects, computed from
captured rest transforms. No parenting, no armature, no modifiers, no mesh
edits, no origin changes.

The one thing written into the `.blend` is a scene custom property holding the
rest transforms. Without it, saving mid-pose and reopening would make the posed
state look like rest, and every angle afterwards would be measured from the
wrong place. **Model → Clear Stored Rest** removes it. A `save_pre` handler
also restores the rest pose before every save, so the file on disk always looks
the way you modelled it.

---

## Layout

```
Arm/
├── blender file/3-DOF-robotic_arm.blend   your model, untouched
├── config/arm_config.json                 all calibration + safety values
├── firmware/arm_firmware/arm_firmware.ino Arduino firmware
├── blender_addon/
│   ├── robot_arm_twin/                    the add-on source
│   │   ├── inverse_kinematics.py          closed-form IK (no bpy)
│   │   └── mouse_target.py                mouse -> plane projection
│   └── robot_arm_twin.zip                 installable build
├── tools/
│   ├── selftest.py                        verify the stack, no hardware
│   ├── firmware_emulator.py               software stand-in for the Arduino
│   └── package_addon.py                   rebuild the zip
└── control arduino/sketch_aug19a.ino      your original sketch (superseded)
```

### Module boundaries

`kinematics.py`, `config.py`, `serial_link.py` and `controller.py` import
cleanly into plain CPython — no `bpy`, no `mathutils`. Only `rig.py`,
`state.py`, `properties.py`, `operators.py` and `panel.py` touch Blender. That
split is what makes the dashboard, homing switches and teach-and-repeat
additions rather than a rewrite. `inverse_kinematics.py` is bpy-free too, and
was dropped in without touching the existing control path.

---

## Setup

### 1. Flash the firmware

Open `firmware/arm_firmware/arm_firmware.ino`, select **Arduino Uno**, upload.
Verified build: **9720 bytes flash (30%), 476 bytes RAM (23%)**.

> **Why not `Stepper.h`?** Your original sketch used it, but `Stepper::step()`
> busy-waits until the whole move completes and never reads the serial port
> meanwhile — so an emergency stop cannot arrive. This firmware advances one
> step per `loop()` pass, so STOP is honoured within a single step interval.
> It also drives IN1–IN4 in natural pin order with a proper 8-step half-step
> sequence, which is why the pins are declared `2,3,4,5` here rather than the
> `2,4,3,5` swap `Stepper.h` required.

### 2. Install the Blender add-on

**Edit → Preferences → Add-ons → Install from Disk…** → pick
`blender_addon/robot_arm_twin.zip` → tick it on.

Then open the 3D viewport, press **N**, choose the **Robot Arm** tab.

#### How the add-on finds `arm_config.json`

Installed from the ZIP, the add-on lives under `AppData`, nowhere near this
project — so it cannot find the config by looking around its own folder. It
searches, most specific first:

1. `%ROBOT_ARM_TWIN_CONFIG%` — a file, or a folder containing `config/arm_config.json`
2. The **open `.blend`'s folder and its ancestors** — this is the normal path.
   `blender file/*.blend` sits inside the project, so `<project>/config/arm_config.json` is found.
3. Folders above `config.py` — this is what works when running from the project tree
4. `robot_arm_twin/default_config/arm_config.json` — a copy shipped inside the ZIP

Step 4 means the panel always starts, even with no project open. When that
fallback is in use the panel says so in red, because **calibration saved then
goes into the add-on folder, not your project**. Open your project `.blend` and
the add-on switches to the project config automatically.

**Model → Diagnostics** shows the resolved path if you ever need to confirm it.

### 3. Install pyserial

Blender's bundled Python has no `serial` module. The panel shows an **Install
pyserial** button — it installs into a private `_vendor` folder beside the
add-on. No admin rights needed. Restart Blender if it asks.

### 4. Check it without moving anything

```bash
python tools/selftest.py
```

60 checks covering config validation, forward kinematics against a parented
Blender ground-truth chain, the closed-form IK (155 round trips), reachability
and limit refusals, the serial protocol, soft limits, the delta guard and the
e-stop latch, plus the mouse-to-XYZ ray/plane projection.

---

## Using it

1. Position the arm by hand where you want "zero" to be.
2. Pick the port (**COM4**), press **Connect**.
3. Drag the **Base / Middle / Upper** sliders.

The handshake pushes calibration, limits, speed and watchdog, then `ZERO`
(adopting the arm's current physical pose as home) and `ARM 1`.

### Controls

| Control | What it does |
|---|---|
| **EMERGENCY STOP** | Freezes motion, cuts all coils, latches. Always enabled. |
| **Connect / Disconnect** | Open/close the port. Disconnect disarms first. |
| **Resume** | Clears the latch. Re-arming is deliberate and separate. |
| **Go Home** | Drive every joint to 0°. |
| **Set Home Here** | Declare the current physical pose to be zero. |
| **Live sync** | Off = move the twin only, hardware stays put. |
| **Follow viewport edits** | Also pick up joints you rotate by hand in the 3D view. |
| **Restore Model** | Put the model back to rest. |

**Follow viewport edits** compares each joint object against what forward
kinematics predicts and converts the residual into a rotation about that
joint's axis. If you rotate an object about an axis the mechanism does not
have, the edit is **rejected** and the object snaps back rather than being
reinterpreted as joint motion — verified in testing.

---

## Safety

Starting values are deliberately timid.

| Setting | Default | Meaning |
|---|---|---|
| Travel limit | **±10°** | Soft limit per joint from home |
| Max step | **10°** | Largest change accepted in one command |
| Step interval | **8000 µs** | 125 steps/s ≈ **1.8 RPM** |
| Watchdog | **4000 ms** | Firmware halts if the host goes quiet mid-move |
| Hold torque | **off** | Coils released at rest — cool, but no holding torque |

Limits are enforced in **three independent places**: the slider range, the
controller's clamp, and the firmware's own `LIM` check. A bug in any one still
leaves two standing. The firmware validates *all* joints of a `MOVE` before
moving *any*, so a rejected joint means nothing moves.

**Never set the step interval below 1200 µs** — a 28BYJ-48 stalls and silently
loses position. The firmware clamps it regardless.

The e-stop latches: after a stop, `MOVE` returns `ERR ESTOPPED` until `RESUME`,
and `RESUME` deliberately leaves the motors disarmed so recovery is a conscious
two-step act.

### Before you power the motors

- Common ground between the Arduino and the 5 V motor supply — non-negotiable.
- Never power three 28BYJ-48s from the Arduino's 5 V pin. Use the external supply.
- Keep the first session at ±10° and 8000 µs. Raise limits only once each joint
  moves the direction you expect.

---

## Calibration

28BYJ-48 step counts differ slightly between motors — that is what the
**Calibration** panel is for.

- **Steps / rev** — default **4076**, the exact half-step figure
  (63.68395 gearbox × 64). Many people use 4096; yours may differ by a few counts.
- **Gear ratio** — `1.0` for direct drive.
- **Direction** — flip to **Reversed** if a joint turns opposite to the model.
- **Min / Max** — per-joint soft limits.

To calibrate a joint: home it, command +10° ten times (100° total), measure the
real angle, then

```
new_steps_per_rev = old_steps_per_rev × commanded / measured
```

**Save Calibration** writes to `config/arm_config.json`. **Apply to Firmware**
pushes it live — note this re-zeroes position, because changing steps-per-rev
invalidates the step counter.

### Which copy wins

`config/arm_config.json` is the **source of truth at load time**. Blender also
persists the panel's calibration inside the `.blend`, so on every file load and
add-on enable the panel is re-populated from the JSON. The panel is a *view* of
the file; **Save Calibration** is how edits travel the other way.

This matters: connecting lets the panel's values win over disk, so a stale
panel would otherwise push old calibration to the firmware and silently undo
any edit made to `arm_config.json` between sessions. To confirm what the
firmware actually received, open **Diagnostics** and look for the `CAL` lines —
the fourth field is the direction.

Expect a small residual: at 4076 steps/rev a joint resolves to ~0.088°, so a
commanded 5° lands at 5.035°. The panel shows commanded, actual and error.

---

## Serial protocol

115200 baud, newline-terminated ASCII. Replies start `OK` or `ERR`;
unsolicited events start `EV`.

| Command | Reply | Purpose |
|---|---|---|
| `PING` | `OK PONG` | Liveness |
| `ID` | `OK ARM3DOF 1.0 JOINTS 3` | Identify |
| `MOVE <a0> <a1> <a2>` | `OK POS …` | **Absolute** angles, degrees |
| `MOVEJ <i> <deg>` | `OK POS …` | One joint, absolute |
| `GET` | `OK POS <a0> <a1> <a2> busy <b> estop <e> armed <a>` | State |
| `STOP` | `OK STOP` | Emergency stop, latches |
| `RESUME` | `OK RESUME` | Clear latch (leaves disarmed) |
| `ARM <0\|1>` | `OK ARM <n>` | Enable/disable motion |
| `ZERO` | `OK ZERO` | Current pose becomes home |
| `CAL <i> <steps_per_rev> <gear> <dir>` | `OK CAL <i>` | Calibration |
| `LIM <i> <min> <max>` | `OK LIM <i>` | Soft limits |
| `DELTA <deg>` | `OK DELTA <n>` | Max change per command |
| `SPEED <µs>` | `OK SPEED <n>` | Step interval (clamped ≥1200) |
| `WD <ms>` | `OK WD <n>` | Watchdog, 0 disables |
| `HOLD <0\|1>` | `OK HOLD <n>` | Hold torque at rest |

Events: `EV READY <name> <ver>`, `EV ESTOP <reason>`.

`tools/firmware_emulator.py` implements this same protocol in Python, so the
host can be exercised with no hardware attached.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `pyserial not installed` | Press **Install pyserial**, restart Blender. |
| No ports listed | Plug the Uno in, press the refresh icon. Serial Monitor must be closed. |
| `ERR LIMIT` | Target is outside ±10°. Raise the travel limit. |
| `ERR DELTA` | Jump larger than 10°. The controller normally hops in bounded steps; this appears if you send raw commands. |
| `ERR DISARMED` | After `RESUME`. Press **Resume** in the panel, which re-arms. |
| Motor buzzes, doesn't turn | Step interval too low, or IN1–IN4 order wrong. |
| Joint moves backwards | Set **Direction → Reversed** for that joint. |
| Motion drifts over time | Steps lost to stalling. Slow down, then **Set Home Here**. |
| Twin moves, motor doesn't | **Live sync** is off, or not armed. |
| `EV ESTOP WATCHDOG` | Host stopped talking mid-move. Raise the watchdog or set 0. |

---

## Inverse kinematics

Give it an X/Y/Z target, it returns Base/Middle/Upper angles. Closed-form, no
solver library — the geometry has an exact solution.

### The model

    tip(t1,t2,t3) = b + Rz(t1) . [ u + Ry(t2) . ( v + Ry(t3) . w ) ]

| Term | Meaning | Measured value |
|---|---|---|
| `b` | point on the base axis | `(−0.401576, 0, 0)` |
| `u` | base axis → shoulder pivot | `(−0.382551, −1.105872, 2.897107)` |
| `v` | shoulder → elbow | `(0.373557, 1.105872, 4.937980)` |
| `w` | elbow → tip | `(−3.696690, 0.313711, −0.251738)` |

**This is not a textbook planar arm.** Both pitch joints turn about **+Y**, and a
Y-rotation cannot change a vector's Y component. So the Y parts of `u`, `v`, `w`
are fixed offsets, and the effective link lengths are the **projected** ones:

| | Projected (correct) | Full 3-D (wrong) |
|---|---|---|
| `L2` upper arm | **4.952089** | 5.074066 |
| `L3` fore arm | **3.705252** | 3.718509 |

Using the 3-D lengths would put every solution consistently off by ~0.12.

The invariant `C = u.y + v.y + w.y = 0.313711` is the arm's fixed lateral
offset, and it creates a **dead cylinder of radius 0.3137** around the base axis
that no joint combination can reach.

### Solving it

1. **Base.** `Rz` preserves Z and radius in XY, and `q.y == C` always, so
   `q.z = d.z`, `q.x = ±√(d.x² + d.y² − C²)`, and
   `t1 = atan2(d.y, d.x) − atan2(C, q.x)`. The square root gives the dead
   cylinder; the ± gives two base branches.
2. **Planar two-link in X-Z.** Treating `(x, z)` as the complex number `x + iz`,
   a rotation `Ry(t)` is exactly multiplication by `e^-it`. Taking moduli
   eliminates `t2` and leaves the law of cosines:
   `|P|² = L2² + L3² + 2·L2·L3·cos(ψ − t3)` with `ψ = arg W − arg V`.
   Then `t3 = ψ ∓ acos(κ)` (two elbow branches) and
   `t2 = arg(V + e^-i·t3·W) − arg(P)`.

Four candidates. All are returned; the solver picks one that is inside the
joint limits and needs least motion.

### Coordinate system and conventions

- Blender world axes, Z up, arm units (~1 unit ≈ one Blender unit).
- Angles in **degrees**, right-hand rule about each joint's stated axis.
- Zero = the modelled rest pose, tip at `(−4.1073, 0.3137, 7.5833)`.
- Positive **Base** rotates counter-clockwise seen from above.
- Positive **Middle** and **Upper** raise the tip.
- `offset_deg` in the config is a placeholder and is currently **0 and unused**;
  home is set by `ZERO` on connect, not by an offset.

### Panel

**Inverse Kinematics** sits under **Joints**: X/Y/Z targets,
**Get Current Position**, branch choice, then **Solve Only**,
**Preview in Blender** and **GO TO TARGET**. Results show target, the three
calculated angles and the residual, with red warnings for unreachable targets
and for limit violations.

### Safety gates

Nothing reaches a motor until **all** of these pass, checked in this order:

1. Target is reachable (dead cylinder, min and max reach)
2. Solution is inside every joint's soft limits — checked **before** any command
3. Solution verified: pushed back through the matrix FK, residual ≤ 1e-4
4. **Preview only** is off
5. **Allow IK to drive the motors** is on (only tickable after a verified solve)
6. The model is already standing at this exact target from a previous action

Rule 6 is why **the first GO always previews and the second GO moves.** Changing
the target resets it. IK then goes through `request_pose()` like the sliders, so
the clamp, the 10° delta guard and the firmware's own `LIM` check all still
apply — IK gets no privileged path to the hardware.

Direct joint control, E-STOP, calibration and serial are untouched.

### Testing IK safely

1. **Disconnect the Arduino.** Everything below works with no hardware.
2. Press **Get Current Position** — you get the rest tip.
3. Nudge one number by ~0.05, press **Solve Only**. Nothing moves; check the
   angles look small and the residual is ~1e-15.
4. Press **Preview in Blender**. Watch the model. Confirm the joints stay
   mated and the arm goes where you expect.
5. Try a deliberately silly target (X = −40) and confirm the red unreachable
   warning appears and nothing moves.
6. Only then connect, tick **Allow IK to drive the motors**, untick
   **Preview only**, and press **GO** twice — first press poses the model,
   second press moves the arm. Keep the E-STOP in reach.

At the default **±10°** limits the reachable workspace is a small pocket around
the rest pose. Most targets will report *outside joint limits* — that is the
guard working, not a bug. Raise **Travel limit** in *Safety & Speed* only once
you trust the mechanism.


---

## Mouse Target mode

Move the cursor over a target plane in the viewport and the end effector
follows it, through the **same** IK solver the XYZ panel uses. There is one IK
implementation in this project and mouse mode does not add another.

    mouse pixel -> viewport ray -> intersection with the target plane
                -> world XYZ    -> existing IK -> joint angles
                -> Blender model -> (only when Physical Sync is ON) Arduino

### How the mouse becomes X/Y/Z

1. The modal handler records the cursor in **window** coordinates.
2. `region_under_mouse()` finds which 3D viewport it is over, so the mode
   behaves correctly with several viewports open.
3. `view3d_utils.region_2d_to_origin_3d` / `..._to_vector_3d` turn the pixel
   into a **ray** — this is correct in both perspective and orthographic views.
4. `plane_frame()` reads the plane object's live transform: its world origin is
   a point on the plane, its local **+Z** column is the normal. Moving,
   rotating or scaling the plane in Blender changes the target surface with no
   code changes.
5. `ray_plane_intersect()` solves `t = ((P−O)·n)/(d·n)` and returns `O + d·t`.

It returns **None** — meaning "no target", never a guess — when the ray is
parallel to the plane (`|d·n| < 1e-4`), when the plane is behind the viewer
(`t < 0`), or when the hit is absurdly far away.

### Scene objects

Created on demand in a **`RobotArmTargets`** collection:

| Object | What it is |
|---|---|
| `ArmTargetPlane` | The surface the cursor is projected onto. Blue, wireframe-on so it shows in Solid shading. |
| `ArmTargetMarker` | A small orange sphere empty, drawn in front, showing exactly where the arm is aiming. |

Neither is referenced by `arm_config.json`, so forward kinematics, the rest-pose
capture and the save handlers ignore them completely. **Remove Target Plane**
deletes both.

The default plane sits at the rest tip. At the stock **±10°** limits, **880 of
1681** sampled points on it are reachable (X −4.96…−3.21, Y −0.44…1.06) — a
usable patch, with the rest correctly reported unreachable.

### The two switches

| | Behaviour |
|---|---|
| Mouse Target **ON**, Physical Sync **OFF** | mouse → target → IK → **Blender only** |
| Mouse Target **ON**, Physical Sync **ON** | mouse → target → IK → Blender → Arduino |

Mouse Target **always starts Blender-only**: `invoke` forces Physical Sync off
regardless of what was set before.

### Safety, checked on every single update

Before anything is sent:

1. Ray actually hit the plane
2. Target is reachable (dead cylinder, min and max reach)
3. Every joint is inside its soft limits
4. Physical Sync is ON
5. Not e-stopped, link connected, motors armed
6. No joint would move more than **Max step** from where the hardware *actually* is

Fail any of 2–3 and the model does not move either. Fail 4–6 and Blender still
follows while the motors are held, with the reason shown. Sending goes through
`apply_ik_pose()` → `request_pose()`, the same path as the sliders, so the
clamp, the delta guard and the firmware's `LIM` check all still apply.

**The emergency stop forces Physical Sync OFF** — in both the linked and
no-link paths.

### Testing it WITHOUT the motors

This is the default; you have to work to make it touch hardware.

1. **Unplug the Arduino**, or just do not press Connect.
2. Panel → **Mouse Target** → **Create Target Plane**.
3. Press **MOUSE TARGET: OFF** to switch it on. It reports
   *"Mouse Target ON (Blender only)"*.
4. Move the cursor over the blue plane. The orange marker follows, the model
   follows, and **TARGET X/Y/Z** and **BASE/MIDDLE/UPPER** update live.
5. Move the cursor off the plane, or to a far corner — status should read
   *Cursor is not over the target plane* or *UNREACHABLE / OUT OF LIMITS*, and
   the arm should stop following rather than lunging.
6. Press **ESC** in the viewport to stop.

Physical Sync cannot even be switched on until the arm is connected and armed —
the button is greyed out, and the property refuses to latch.

Headless checks, no Blender and no hardware:

```bash
python tools/selftest.py
```

### Update rate

**Update rate** (default **15 Hz**) sets how often the cursor is turned into an
IK solve. The timer re-arms itself when you change it mid-run. Mouse movement
is only *recorded* on `MOUSEMOVE`; the work happens on the timer, so flicking
the cursor cannot flood the solver or the serial link.

All other viewport input passes straight through — mouse mode never swallows
your navigation.

---

## Still not implemented

- **Homing switches** — firmware gains a `HOME` command replacing the
  `ZERO`-on-connect assumption; `build_init_commands()` is where the handshake
  changes.
- **Teach and repeat** — `ArmController.request_pose()` and `describe()` are the
  record/playback hooks.
- **Full dashboard** — import `controller.py` in any CPython process; it has no
  Blender dependency.

---

## Verification performed

- Model inspected headlessly in Blender 5.2; axes and pivots derived from mating
  geometry and confirmed by rendering rest, base+25°, mid+20°, upper+30° and a
  combined pose — joints stayed mated, only correct downstream links moved.
- Pure-Python forward kinematics matches Blender's own matrix maths to
  **4×10⁻⁴** across five poses.
- Firmware compiles for Arduino Uno: 30% flash, 23% RAM.
- Full add-on lifecycle driven headlessly against the real `.blend`: register,
  pose, clamp, connect, slider→motor, e-stop, resume, hand-edit detection
  (including correct rejection of an impossible axis), disconnect, unregister.
- `tools/selftest.py`: 60/60.

**Not verified:** anything requiring the physical arm — real motor direction,
true steps-per-revolution, mechanical limits. Those are the calibration step
above, and they are why the defaults start at ±10° and 1.8 RPM.
