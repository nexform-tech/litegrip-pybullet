# LiteGrip PyBullet examples

Five runnable programs that build on one another: read the model, move it in
simulation, teach the gripper a motion by hand and replay it into both, then
couple the two in each direction.

**English** · [简体中文](README.zh-CN.md)

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`01_hello_sim.py`](01_hello_sim.py) | reads the state, moves nothing | **No** |
| [`02_move_sim.py`](02_move_sim.py) | simulation only | **No** |
| [`03_trajectory.py`](03_trajectory.py) | records on the gripper, replays into both | Yes, to record or to replay onto it |
| [`04_mirror_real.py`](04_mirror_real.py) | gripper → simulation | Yes, to actually mirror |
| [`05_dual_control.py`](05_dual_control.py) | simulation → gripper | Yes, to actually move |

## Prerequisites

```bash
python3 -m pip install pybullet                       # examples 01–02
python3 -m pip install -e /path/to/lite-grip          # examples 04–05
python3 -m pip install -e /path/to/litegrip-python    # example 03
```

The `litegrip` SDK is **not on PyPI**: `pip install litegrip` installs something
else, and no released version carries the API these examples use. Take it from a
checkout — either an editable install as above, or a sibling directory next to
this repository, or an environment variable:

| Examples | Checkout | Sibling directory | Environment variable |
| --- | --- | --- | --- |
| 04, 05 | `lite-grip` | `../lite-grip` | `LITEGRIP_SDK_DIR` |
| 03 | `litegrip-python` | `../litegrip-python` | `LITEGRIP_TRAJ_SDK_DIR` |

**Two checkouts, because the API is split across them and both report
`__version__ == 2.2.0`.** Read the capability, not the version number:

| API | `lite-grip` | `litegrip-python` |
| --- | --- | --- |
| `LiteGrip.refresh_status`, `GripperState.has_data` / `is_stale` / `data_age_s` | yes | **no** |
| `LiteGrip.record_start` / `record_stop` / `play_start` / `play_stop` / `trajectory_status`, `litegrip.trajectory` | **no** | yes |

Missing its own checkout, each example stops at startup and names the members it
needs (`LiteGrip.refresh_status`, `GripperState.data_age_s` / `has_data` /
`is_stale` for 04/05; `LiteGrip.record_start`, `Trajectory.load` and the rest for
03) instead of failing somewhere inside a control loop. **Do not** point
`LITEGRIP_SDK_DIR` at `litegrip-python`: 04 and 05 read it, fail their own check
against it and refuse to start. The two variables name two capabilities and are
not interchangeable.

If `pybullet` is missing, the examples re-exec themselves into `./.venv/bin/python`
when that exists. Nothing else is required: the URDF and its meshes are bundled
under `src/litegrip_pybullet/assets/litegrip_urdf/`, so a fresh clone runs as-is.

Bring up CAN before running 03, 04 or 05:

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

> **Examples 03, 04 and 05 move real hardware.** Read
> [Before you drive the hardware](#before-you-drive-the-hardware) first.

## One opening, three notations

The gripper can be described three ways, and mixing them up is the usual source
of confusion:

| Notation | Range | Meaning |
| --- | --- | --- |
| Joint value | `0 … 0.042726` m | prismatic travel of one finger; `0` is wide open |
| **Normalised opening** | `0 … 1` | **the one shared quantity**; `0` closed, `1` wide open |
| SDK millimetres | `0 … 120` mm | the *calibrated* scale the hardware reports, not a jaw gap |
| Jaw gap | `1.548 … 87` mm | physical distance between the finger faces |

SDK millimetres are what the gripper's own `goto(mm)` uses; the physical gap is
what you would measure with calipers. They are not the same number and they are
not a unit conversion apart — only the normalised opening means the same thing
on both sides, which is why every example converts through it.

**The examples convert between radians and the normalised opening directly, not
through millimetres**, because `120` is the SDK's *default* `max_stroke_mm` and a
calibration file carries whatever scale it was saved with. `rad_to_mm` is derived
at calibration time from a stroke length the loader never writes back, so the two
can disagree; when they do, `position_mm / max_stroke_mm` saturates partway and
the top of the slider does nothing. See `_common.fraction_to_target_rad`.

## The five examples

### 01 — hello, simulation

Reads the model and moves nothing. No hardware, no CAN, no SDK — and no motion
call anywhere in the file: `sim.step()` only pumps the window's events and
advances the clock, so the jaws stay where they are however long you leave it.

1. **The model constants** — the finger count, the per-finger stroke, the jaw
   opening the bundled URDF actually has and the force cap in force.
2. **The current opening, in three notations** — the normalised fraction, the
   jaw gap in millimetres and the SDK's calibrated millimetres, side by side.
3. **A live status line** — with a window open it refreshes until you press
   **Esc** or **Q**.

```bash
python3 examples/01_hello_sim.py                # window, full demo
python3 examples/01_hello_sim.py --headless     # no window, exits 0
```

Read-only is checked, not promised: `tests/test_examples_cli.py` parses this file
and refuses it any motion call (`command_fraction`, `command_joint`,
`reset_fraction`, `settle`, `run_for`, `add_box`), and separately asserts its
output never claims a motion happened.

### 02 — moving, in simulation

The same gripper, now driven, still with nothing but PyBullet:

1. **Speed-limited travel** — a full stroke takes ~1 s: each finger ramps at
   42.73 mm/s, and because the two fingers close on each other the *gap* changes
   at twice that — the hardware's rated 85 mm/s.
2. **Midpoint positioning** — command a normalised opening, and `settle()` waits
   for the jaws to actually arrive.

```bash
python3 examples/02_move_sim.py                 # window, full demo
python3 examples/02_move_sim.py --headless      # no window, exits 0
python3 examples/02_move_sim.py --speed 0.02    # slow: ~2 s for a full stroke
python3 examples/02_move_sim.py --force 20      # 20 N grip cap
```

**Do not** reach for `--object-mm`, `--hold`, `--pull` or `--slip` here: this
example has no part to grasp or pull on, and argparse rejects those flags with
exit 2 rather than ignoring them. They belonged to the old `03_grasp.py`, which
no longer exists.

### 03 — recording a motion and replaying it

Teach the gripper a motion by hand, save it, and play it back into the hardware
**and** the window at the same time:

1. **Record.** The motor drops into zero gravity — its force is switched off and
   you push the fingers through the motion yourself — while the SDK samples the
   opening at 100 Hz in the background. Press **Esc** / **Q** to stop, or pass
   `--record 6` to stop by itself. The window follows your hand as you go.
2. **Save.** The recording goes to `~/.litegrip/trajectories/` as a `.lgt` named
   after the example and the time, so a second recording cannot overwrite the
   first. Nothing is written into this repository: a trajectory is data measured
   on one machine.
3. **Replay.** The same trajectory drives the motor and teleports the simulated
   fingers onto its **measured** position, so what the window shows is what the
   gripper is doing. The fingers stop where the trajectory ends and do not return
   to the start by themselves.

```bash
python3 examples/03_trajectory.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py --record 6 --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --real
python3 examples/03_trajectory.py --play 03_hand_taught-20260929-120000 --speed 0.5 --headless
```

The trajectory stores the **normalised opening**, not an angle, so a motion
taught on one gripper replays on another, and the file replayed through a
different machine's calibration still means the same opening. A bare name is
resolved inside `~/.litegrip/trajectories` with `.lgt` appended; a path with a `/`
in it is used as written.

`--play` on its own touches nothing: no connection, no enable, no frame. It reads
the file and drives the window from it, which is the one path this example has
that runs without a gripper (and the only one CI can run). Add `--real` to send
the same trajectory to the hardware as well.

Replay commands **position**, not force. The recorded torque is kept in the file
as a diagnostic and is never fed forward, so a squeeze that was recorded against
a part replays as a position path that presses with whatever `kp` yields — the
grip force you taught is not preserved. For a repeatable grip, replay the motion
and then call the SDK's `grasp(force_n=...)`.

#### Between the phases, this example feeds the motor itself

`record_stop()` puts the motor back under closed-loop control and then stops: the
SDK's recorder was the only thing streaming frames, and the replay has not started
yet. An enabled motor that hears nothing latches the communication-loss fault
(0xD) — see [Why "just watching" still has to send
frames](#why-just-watching-still-has-to-send-frames). So this example streams
"**locked at the measured position**" hold frames at 200 Hz through every gap:
after `enable()`, between the recording and the replay, and after the replay ends
until you quit. It commands no motion; the target *is* the position the motor
reports, with zero feed-forward.

**Do not** stream frames of your own while a recording or a replay is running.
Both are SDK background threads streaming the bus, and a second stream on the same
wire tears the trajectory apart. The example's hold frames are therefore strictly
between phases, never inside one.

### 04 — the gripper drives the simulation

The hardware is the source of truth; the simulation is a display. Each frame
reads the hardware position once and teleports the simulated fingers onto it with
`reset_fraction()` — pure kinematics, no dynamics — so the window shows where the
hardware is right now, with no lag and no drift of its own.

Two ways to use it:

- **Push it by hand** (`--zero-gravity`, recommended): the motor goes slack and
  you can move the fingers yourself. The window follows your hand. Press **Z**
  while running to toggle slack/enabled.
- **Watch another program drive it** (`--passive`): this example connects, reads,
  and leaves the feeding to that program — it does not enable the motor and sends
  nothing at all.

```bash
python3 examples/04_mirror_real.py --zero-gravity --calib ~/.litegrip/litegrip_calibration.json
python3 examples/04_mirror_real.py                  # mirror (hold frames keep it alive)
python3 examples/04_mirror_real.py --passive        # read only, let someone else feed it
python3 examples/04_mirror_real.py --headless       # terminal readings only
python3 examples/04_mirror_real.py --duration 10    # stop after 10 s
```

Like 05, this needs a calibration file on every run — see
[Before you drive the hardware](#before-you-drive-the-hardware).

#### Why "just watching" still has to send frames

An **enabled** motor that hears nothing for about **0.9 s** latches the
communication-loss fault (0xD) — again a blinking red LED, positions that still
read, and commands that are silently ignored. The SDK measured that 0.9 s on this
hardware; the `TIMEOUT` register (RID 9) disagrees with it (8000 ms on one read,
0 on another) and is marked unresolved, so the timing here follows the
measurement. Watching the PyBullet window without feeding it is enough to wedge
the gripper.

So the default mode is not read-only: it sends "**locked at the measured
position**" hold frames at 200 Hz — target where the fingers already are, zero
velocity, zero feed-forward. It commands no motion; it just gives the fingers
stiffness and keeps the watchdog fed. To back-drive it instead, add
`--zero-gravity` (or press Z), which streams the same way with kp/kd zeroed.

Use `--passive` when a **different program** is driving the hardware: this example
then sends no frames, does not enable the motor, and disables the Z key, so the
two never fight over the bus. The cost is that the other program has to feed the
motor itself, or it latches 0xD about 0.9 s later anyway. **Do not combine
`--passive` with running this example alone.**

`--zero-gravity` (or Z) makes the gripper *soft*, so the fingers can be
back-driven and can also sag under gravity. Support the gripper before enabling
it.

Quitting **disables** the motor (0xFD) rather than sending one last relock frame:
a single frame has nothing after it, so an enabled motor goes quiet and latches
0xD within the second — with nobody left to clear it. The gripper therefore goes
limp on exit and the fingers may drift under their own weight.

### 05 — simulation drives the gripper

The window is two things at once: three sliders that *command* the hardware, and
a mirror that *shows* where the hardware is. Enabling the motor does not move
anything — the example reads the position you are already at, shows it in the
window and holds there, and the hardware only moves once you drag a slider.

1. Drag the **opening** slider. The drag *is* the command: there is no send key.
   The hardware ramps toward the target at 200 Hz, and the simulated jaws mirror
   its **measured** position, so they lag the slider rather than jumping to it.
   That gap is the tracking error, and it stays open while the fingers are
   blocked by a part — which is how you tell you have gripped something.
2. Drag the **speed** slider at any time, before or mid-move. It caps how fast
   the position target may grow, as a percentage of the fingers' rated 85 mm/s.
   100 % means "at most 85 mm/s"; values above 100 % are treated as 100 %.
3. Drag the **force** slider. It becomes the feed-forward torque sent with the
   frame, and it is only applied in the closing direction (a grip force on an
   opening move would fight the motor) and only once the target is reached.
4. Press **Esc** or **Q** to quit. Frame sending stops and the motor is
   **disabled** (0xFD): the fingers go limp and a gripped part will drop, but no
   communication-loss fault is left latched on the motor.

> **There is no confirmation step.** A slider value *is* a command, so brushing a
> slider with the mouse is a real command to a real motor. Park the pointer away
> from the sliders when you are not driving, and expect the gripper to move the
> moment you touch the opening slider.

Between drags the example keeps streaming the current position at 200 Hz. That is
not a keep-alive nicety: an enabled motor that hears nothing for about 0.9 s
latches a communication-loss fault (0xD), so standing still is the thing that
fails.

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/05_dual_control.py --dry-run         # window only, never touches CAN
python3 examples/05_dual_control.py --speed 40        # start the speed slider at 40 %
python3 examples/05_dual_control.py --force 20 --channel can1
```

`--calib` is required on every run, `--dry-run` included; without it the example
lists the calibration candidates and asks (see
[Before you drive the hardware](#before-you-drive-the-hardware)).

`--dry-run` runs the same loop with no CAN traffic and no measured position, so
the window follows the *commanded* opening instead of mirroring the hardware and
the status line says `dry-run` to remind you.

`--headless` is refused: the sliders are the input device, and a DIRECT
connection has no sliders. Use 01 for a headless run.

#### Why it ramps

The MIT position term is `kp × (q_target − q_actual)`, and `kp` is an entry in
the calibration file rather than a constant. At the SDK's default `kp = 100
Nm/rad`, sending the target as a step asks for `100 × 1.845 rad ≈ 185 Nm` from a
motor rated around 10 Nm; at this machine's present `kp = 5.0` the same step asks
for about 9 Nm, inside the rating. The current saturates and the motor latches an
under-voltage/over-current fault, after which it keeps reporting its position
while ignoring every command — the blinking red LED and "it reads but I can't
control it" symptom.

The ramp is kept because it does not depend on that number: it bounds how far the
*position target* may jump in one frame, whatever `kp` is configured to. Each
frame advances one tick from where the motor *is*, so a single frame demands well
under the rated torque. The SDK's own `goto_rad` ramps for the same reason.

This is also why the speed limit is a limit on the *target*, not on the slider.
Yanking the opening slider from fully open to fully closed still costs the motor
one bounded increment per frame — the drag's own speed never reaches the bus.

#### Why it slows down at the end

The ramp reaches the target *and then the fingers move back a little*, most
visibly when opening. The cause is the velocity field of an MIT frame: `dq` is a
**target** velocity, not a measurement, so the frame that drops it from full speed
to `0` reverses the damping term into a torque step of `kd × v`. At this machine's
`kd = 2.0` and a full-speed `1.39 rad/s` that is about `2.8 Nm`; absorbing it needs
`2.8 / kp = 0.56 rad` of position error at `kp = 5.0`, which is 40 % of the travel
— more than the position loop can find, so the mechanism recoils to rebuild it.

Example 05 therefore decelerates before it arrives: while the remaining distance
is short, the commanded speed is `√(2·a·remaining)` with `a = full speed / 0.15 s`
(`RAMP_DOWN_S`). The speed is never *above* the rate limit, so the limit stays a
hard bound; it only makes the last few frames slower, and the frame before the
stop is under 7 % of full speed — about `0.17 Nm` of reversed damping instead of
`2.8 Nm`. The whole move costs at most `RAMP_DOWN_S` more than walking the
distance at the rate limit.

The SDK's own `_move_at_speed_rad` does **not** do this — it holds `dq` at the
full speed and then sets it to `0` on the frame it stops — so the same recoil
appears when a script drives the gripper with the SDK directly.

#### If a wedged gripper reads but won't move

```bash
python3 examples/05_dual_control.py --status                  # read only, sends nothing
python3 examples/05_dual_control.py --status --clear-fault    # clear the latched fault
```

`--status` opens no window, does not enable the motor and **sends no motion
command at all** — it just reads the error code and translates it, so it is safe
to run while the gripper holds a part or is in someone's hands. It touches the
hardware, so it needs the calibration file too: the two lines above each take a
`--calib <path>`, or let the example list the candidates on a terminal. Only
`--clear-fault` sends frames, and those are all zero-torque; but the SDK's clear
sequence is disable → clear → enable, so the motor goes limp for an instant and
the fingers may drift under their own weight. Support the gripper first.

The fault is latched: it will not clear itself until the gripper is power-cycled.

##### Two faults look identical and are not

| Code | Meaning | What triggers it |
| --- | --- | --- |
| 0x9 / 0xA | under-voltage / over-current | a step command: ~185 Nm in one frame on a ~10 Nm motor at the default kp, ~9 Nm at this machine's 5.0 |
| **0xD** | **communication loss** (`通讯丢失 (CAN 超时)`, named by the SDK's own `describe_error`) | an enabled motor left silent for about 0.9 s — including "just watching with the window open" |

0xD is the motor's CAN watchdog, and it is measured rather than configured:
**an enabled motor latches it after roughly 0.9 s of silence** (the SDK's own
figure, taken on this hardware). **Idling is itself the fault cause.** This
example therefore keeps sending hold frames at 200 Hz through its idle periods
(target = measured position, zero feed-forward, no motion commanded) — see
`IdleKeeper`. Earlier revisions sent nothing, so the quiet stretch after
`enable()` — its 50 ms hold stream ends, then PyBullet starts up and the operator
looks at the window for a few seconds — went past the watchdog and wedged the
motor. That, not the ramp, was the main cause of "simulation can read the gripper
but not control it."

Do not derive that duration from the `TIMEOUT` register (RID 9): it reads 8000 ms
on one run and 0 (no watchdog) on another, and neither agrees with the measured
0.9 s, so the SDK marks it as unresolved. `--status` prints the register's live
value for the record and says as much next to it:

```text
[2] 读寄存器（只发读请求）
   TIMEOUT=0 · CTRL_MODE=1 · UV_Value=15 · OC_Value=0.8 · OT_Value=100
   通信超时保护（TIMEOUT, RID 9）= 0（这个寄存器当前不生效）
   别拿这个寄存器当依据：实测**使能态**的电机静默约 0.9 s 就锁 0xD 通信丢失故障，与寄存器读数对不上（这台机器读到过 8000，也读到过 0），SDK 自己把这条标成「待查」。
      所以 03/04/05 空闲时照 200 Hz 持续发帧，不赌这个数字；只读不喂帧（或跑了别的只读脚本）同样会把它看哑。
```

(Captured on this bench on 2026-09-28, motor disabled and healthy: exit 0. One
phrase has since changed: that last line listed `04/05` until 03 gained the same
hold frames. Everything else is the run's own text.)

The examples keep feeding either way — a different motor, or someone editing that
register, changes nothing about the timings these examples rely on.

A **second master** on the same bus makes the symptom messier still: the two
streams fight and neither side wins. Before running a hardware example, check
nothing else (say `litegrip_console --backend real`) is on the same CAN
interface:

```bash
pgrep -af python3 | grep -i litegrip        # who has CAN open
ip -details -statistics link show can0      # busy bus? counters climbing while you run nothing means someone else is talking
```

`ip link set can0 down` / `up` does **not** evict that program — its socket
survives and it resumes when the interface returns. The process has to exit.

The example drives the CAN loop itself, with the SDK's public
`send_mit_frame()` + `poll()`, rather than calling `move_to()`/`goto_rad()`. Those
run `control_mit_stream()` internally, which sleeps in its own 5 ms loop and never
yields — the PyBullet window would freeze and keystrokes would go unread. The
public frame-level API exists for exactly this case.

## Before you drive the hardware

All three hardware examples print a safety banner and are meant to be run with the
gripper in hand or clamped to a bench, **with the travel clear**, and the power
switch within reach. The first run of 05 should be `--dry-run`.

A first real run of 05 looks like nothing happening, and that is correct: the
motor is enabled, the window shows where the gripper already is, and it stays
there until you drag the opening slider. Keep the fingers clear the whole time —
the motion starts on the drag, not on a keypress.

**Only run 03 with your hands where they should be.** Recording means the motor's
force is off and your hand is on the fingers, so nothing can pinch you — the risk
there is that you push the fingers somewhere they cannot go, or that the recording
starts before you are holding it. Replay is the opposite: the motor is under
closed-loop control and follows the trajectory at whatever `kp` and `kd` the
calibration carries. Keep clear of the travel during the replay, and remember that
the fingers stay at the end of the trajectory, at force, until you quit or the
example is stopped. **Do not** run a replay in front of someone who is not
expecting the gripper to move: there is no confirmation step, and the window shows
the motion at the same time as the hardware does it.

Confirm the CAN interface before anything moves:

```bash
ip -details link show can0
```

**Pick this gripper's calibration file first.** Every path that touches the
hardware starts there — `--status`, `--dry-run` and 04's `--passive` included —
and the default is deliberately not an option:

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/05_dual_control.py            # no --calib: it lists candidates
```

Without `--calib` the candidates in `~/.litegrip` are listed with their mtime and
key values (closed/open angles, `rad_to_mm`, `kp`, `mst_id`) and you pick one by
number, or type a path. The `*.sim.json` file the studio writes for its simulator
backend and the `*.bak` backups are never offered, and a `*.sim.json` named
explicitly is refused — its scale belongs to the simulated gripper. With no
terminal to ask on (a pipe, a script, CI) or nothing to offer, the run stops and
says how to get a calibration and how to pass one.

The file comes from calibrating *this* gripper in the host software
(`litegrip-studio` / `litegrip-console`, or the SDK's own
`tools/gui/litegrip_gui.py`) and saving it. That matters because the SDK's
`load_calibration` loads the shipped factory calibration *silently* when the path
it was given cannot be read, and still returns `True` — so a mistyped path would
otherwise drive the motor with another machine's angles. The examples therefore
read the file themselves and check it took effect field by field, and they refuse
a file whose `can_id`/`mst_id` name a different motor. They also check the angles
are self-consistent: the SDK's factory defaults ship an opening angle that
contradicts its own `goto()` convention, and an uncalibrated unit is stopped with
an explanation rather than driven with meaningless angles.

## Notes

**Where the grasp centre is.** `[0.0, 0.0, 0.0665]` m — between the finger faces,
22.5 mm clear of the base's top surface. No example puts a part there any more;
`GripperSim.grasp_center` and the contact helpers are still part of the library and
covered by `tests/test_sim.py`. `getAABB` inflates each link by roughly 3 mm, so
never read the jaw opening from it; use `aperture_mm()`.

**Shared helpers.** [`_common.py`](_common.py) holds the argument parsers, the two
SDK discoveries (the freshness one for 04/05 and the trajectory one for 03), the
calibration choice (`choose_calibration_file`, the candidate listing, the "did the
file actually take effect" check), the connect/enable sequence, the unit
conversions and the status line. It is not a fourth example — it is imported by the
other four, which is why each starts with `from _common import ...` *before*
importing `litegrip_pybullet`.

**03 does not use `open_real_gripper`.** That helper runs `check_sdk_api` before it
connects — the check for the *freshness* API, which the trajectory checkout does
not have. 03 therefore repeats the connect/enable/calibrate sequence against the
steps `_common` exports, minus that one check. If the two SDKs are ever merged
into one checkout, this and `trajectory_sdk_dir()` both disappear.
