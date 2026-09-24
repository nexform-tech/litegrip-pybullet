# LiteGrip PyBullet examples

Three runnable programs that show the same gripper model from three directions:
simulation only, simulation driving the hardware, and hardware driving the
simulation.

**English** · [简体中文](README.zh-CN.md)

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`01_sim_only.py`](01_sim_only.py) | — (pure PyBullet) | **No** |
| [`02_sim_to_real.py`](02_sim_to_real.py) | simulation → gripper | Yes, to actually move |
| [`03_real_to_sim.py`](03_real_to_sim.py) | gripper → simulation | Yes, to actually mirror |

## Prerequisites

```bash
python3 -m pip install pybullet          # examples 01–03
python3 -m pip install litegrip          # examples 02–03 (hardware)
```

The `litegrip` SDK is also picked up from a sibling `lite-grip` checkout or from
`$LITEGRIP_SDK_DIR`, so a source checkout works without installing:

```bash
export LITEGRIP_SDK_DIR=/path/to/lite-grip
```

If `pybullet` is missing, the examples re-exec themselves into `./.venv/bin/python`
when that exists. Nothing else is required: the URDF and its meshes are bundled
under `src/litegrip_pybullet/assets/litegrip_urdf/`, so a fresh clone runs as-is.

Bring up CAN before running 02 or 03:

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

> **Examples 02 and 03 move real hardware.** Read
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

## The examples

### 01 — simulation only

No hardware, no CAN, no SDK. A four-part walkthrough of the model:

1. **Speed-limited travel** — a full stroke takes ~1 s, because the ramp is
   limited to 85 mm/s, the hardware's rated speed. The fingers close on each
   other, so the *gap* changes at twice that.
2. **Midpoint positioning** — command a normalised opening, and `settle()` waits
   for the jaws to actually arrive.
3. **Grasping** — drop a box between the fingers, close at a force cap, and read
   back the contact points and the motor force. `settle()` returning `False` here
   is the correct answer, not a failure: the fingers are blocked by the part.
4. **Pulling** — add a downward force and find the friction limit. It holds 5 N
   and slips at 15 N.

```bash
python3 examples/01_sim_only.py                 # window, full demo
python3 examples/01_sim_only.py --headless      # no window, exits 0
python3 examples/01_sim_only.py --object-mm 60 --force 20
python3 examples/01_sim_only.py --slip 40       # a pull that definitely slips
```

The grip in step 3 is real friction, not a ledge: the part is held in mid-air and
*touches only the two fingers*. See [Notes](#notes) for how that is arranged.

### 02 — simulation drives the gripper

You set the target in the PyBullet window; pressing Enter sends it.

1. Drag the **opening** slider. The simulated jaws follow it live — that is a
   *preview*, and it obeys the same 85 mm/s limit, so the speed you see is the
   speed the hardware will move at. The hardware has not moved yet.
2. Drag the **force** slider. It becomes the feed-forward torque sent with the
   frame.
3. Press **Enter** or **Space** to send. The hardware **ramps** toward the target
   at 200 Hz — slow enough by default to keep the fingers at or under 85 mm/s
   (`--duration` can ask for slower, never for faster) — then holds and pushes.
   The simulation shows the *commanded* opening while the hardware's measured
   opening is displayed next to it; the gap between the two is the tracking
   error, and it stays open while the fingers are blocked by a part — which is
   how you tell you have gripped something.
4. Press **Esc** or **Q** to quit. Frame sending stops and the motor holds its
   position.

```bash
python3 examples/02_sim_to_real.py --dry-run         # window only, never touches CAN
python3 examples/02_sim_to_real.py                   # can0, 10 N, automatic speed
python3 examples/02_sim_to_real.py --force 20 --duration 2
python3 examples/02_sim_to_real.py --channel can1    # a different CAN port
```

`--headless` is refused: the sliders are the input device, and a DIRECT
connection has no sliders. Use 01 for a headless run.

#### Why it ramps

The MIT position term is `kp × (q_target − q_actual)` with `kp = 100 Nm/rad`.
Sending the target as a step means the first frame asks for
`100 × 1.845 rad ≈ 185 Nm` on a motor rated around 10 Nm. The current saturates,
the motor latches an under-voltage/over-current fault, and it then keeps
reporting its position while ignoring every command — the blinking red LED and
"it reads but I can't control it" symptom. This example ramps like the SDK's
`goto_rad` does: each frame advances one tick from where the motor *is*, so a
single frame demands under 1 Nm.

#### If a wedged gripper reads but won't move

```bash
python3 examples/02_sim_to_real.py --status                  # read only, sends nothing
python3 examples/02_sim_to_real.py --status --clear-fault    # clear the latched fault
```

`--status` opens no window, does not enable the motor and **sends no motion
command at all** — it just reads the error code and translates it, so it is safe
to run while the gripper holds a part or is in someone's hands. Only
`--clear-fault` sends frames, and those are all zero-torque; but the SDK's clear
sequence is disable → clear → enable, so the motor goes limp for an instant and
the fingers may drift under their own weight. Support the gripper first.

The fault is latched: it will not clear itself until the gripper is power-cycled.

##### Two faults look identical and are not

| Code | Meaning | What triggers it |
| --- | --- | --- |
| 0x9 / 0xA | under-voltage / over-current | a step command: ~185 Nm in one frame on a ~10 Nm motor |
| **0xD** | **communication loss** — the SDK's `describe_error` does not know this code, so the example names it itself (`EXTRA_ERRORS`) | 8 s with no frame received — including "just watching with the window open" |

0xD is the `TIMEOUT` register's CAN watchdog: **idling is itself the fault
cause.** This example therefore keeps sending hold frames at 200 Hz through its
idle periods (target = measured position, zero feed-forward, no motion
commanded) — see `IdleKeeper`. Earlier revisions sent nothing, so the quiet
stretch after `enable()`'s priming frame — PyBullet starting up plus a few
seconds of looking at the window — went past 8000 ms and wedged the motor. That,
not the ramp, was the main cause of "simulation can read the gripper but not
control it."

`--status` prints the register rather than leaving you to guess:

```text
   ⏱  通信超时保护 = 8000 ms：连续这么久收不到帧，电机会锁进通信丢失故障
      （位置照读、指令不执行、红灯闪）
```

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

### 03 — the gripper drives the simulation

The hardware is the source of truth; the simulation is a display. Each frame
reads the hardware position once and teleports the simulated fingers onto it with
`reset_fraction()` — pure kinematics, no dynamics — so the window shows where the
hardware is right now, with no lag and no drift of its own.

Two ways to use it:

- **Push it by hand** (`--zero-gravity`, recommended): the motor goes slack and
  you can move the fingers yourself. The window follows your hand. Press **Z**
  while running to toggle slack/enabled.
- **Watch another program drive it** (`--passive`): this example sends nothing at
  all and leaves the feeding to that program.

```bash
python3 examples/03_real_to_sim.py --zero-gravity   # push it by hand
python3 examples/03_real_to_sim.py                  # mirror (hold frames keep it alive)
python3 examples/03_real_to_sim.py --passive        # send nothing, let someone else feed it
python3 examples/03_real_to_sim.py --headless       # terminal readings only
python3 examples/03_real_to_sim.py --duration 10    # stop after 10 s
```

#### Why "just watching" still has to send frames

The motor's `TIMEOUT` register (RID 9, measured **8000 ms** on this machine) is a
CAN watchdog: **that long with no frame received latches a communication-loss
fault** — again a blinking red LED, positions that still read, and commands that
are silently ignored. A few seconds of watching the PyBullet window is enough to
wedge the gripper.

So the default mode is not read-only: it sends "**locked at the measured
position**" hold frames at 200 Hz — target where the fingers already are, zero
velocity, zero feed-forward. It commands no motion; it just gives the fingers
stiffness and keeps the watchdog fed. To back-drive it instead, add
`--zero-gravity` (or press Z), which streams the same way with kp/kd zeroed.

Use `--passive` when a **different program** is driving the hardware: this example
then sends no frames and disables the Z key, so the two never fight over the bus.
The cost is that the other program has to feed the motor itself, or it latches
after 8 s anyway. **Do not combine `--passive` with running this example alone.**

`--zero-gravity` (or Z) makes the gripper *soft*, so the fingers can be
back-driven and can also sag under gravity. Support the gripper before enabling
it.

Quitting restores the motor (`exit_zero_gravity()`) so the gripper holds its
position rather than going limp.

## Before you drive the hardware

Both hardware examples print a safety banner and are meant to be run with the
gripper in hand or clamped to a bench, **with the travel clear**, and the power
switch within reach. The first run of 02 should be `--dry-run`.

Confirm the CAN interface and the calibration before anything moves:

```bash
ip -details link show can0
```

The examples check the calibration themselves and refuse to start if it is
inconsistent — the SDK's factory defaults ship an opening angle that contradicts
its own `goto()` convention, so an uncalibrated unit is stopped with an
explanation rather than driven with meaningless angles.

## Notes

**Why the part is "held" while the jaws close.** Closing takes about a second,
and a 40 mm part released at the grasp centre falls ~24 mm onto the gripper's base
in that time. It would then be *resting on the base* rather than gripped by the
fingers, and a friction test on it would measure nothing. So example 01 sets the
part's mass to zero (PyBullet treats it as static) until the jaws have closed,
then restores it — after which only the fingers touch it. This is a simulation
device, not a physical effect; the same trick is used in
`tests/test_sim.py::close_in`.

**Where the grasp centre is.** `[0.0, 0.0, 0.0665]` m — between the finger faces,
22.5 mm clear of the base's top surface. `getAABB` inflates each link by roughly
3 mm, so never read the jaw opening from it; use `aperture_mm()`.

**Shared helpers.** [`_common.py`](_common.py) holds the argument parsers, the
SDK discovery, the connect/enable sequence, the unit conversions and the status
line. It is not a fourth example — it is imported by the other three, which is why
each starts with `from _common import ...` *before* importing
`litegrip_pybullet`.
