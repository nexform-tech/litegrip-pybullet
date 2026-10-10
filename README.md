# litegrip-pybullet

PyBullet simulation environment for the **LiteGrip lightweight robotic gripper
series**: a physical model of the gripper that speaks the same *opening* as the
real one, plus five runnable examples that go from reading the model to teaching
the hardware a motion by hand and replaying it into the simulation and the
gripper at the same time.

**English** · [简体中文](README.zh-CN.md)

| Property | Value |
| --- | --- |
| Product | LiteGrip lightweight robotic gripper series |
| Repository role | PyBullet simulation environment |
| Model | Two parallel prismatic fingers, 42.726 mm travel each |
| Jaw opening | 1.548 mm closed … 87.0 mm open |
| Rated finger speed | 85 mm/s |
| Hardware SDK | One `litegrip-python` checkout, shared by examples 03, 04 and 05 |
| Python | 3.8+ |

## Contents

- [Features](#features)
- [Architecture](#architecture)
- [Install](#install)
- [Quick start](#quick-start)
- [Core API](#core-api)
- [The five examples](#the-five-examples)
- [Calibration](#calibration)
- [Safety](#safety)
- [Verification status](#verification-status)
- [Troubleshooting](#troubleshooting)
- [Related repositories](#related-repositories)
- [Documentation](#documentation)
- [Development](#development)
- [License](#license)
- [Contact](#contact)

## Features

### Simulation

- **Bundled model**: the upstream ROS 2 xacro from `litegrip-description`, normalised
  for PyBullet at load time (mesh paths rewritten, the massless root link
  stripped) and shipped inside the package, so a fresh clone loads it with no
  extra setup.
- **One shared opening**: the only quantity the simulation and the hardware
  agree on is the normalised opening (0 closed … 1 wide open). Every other
  notation — joint travel, jaw gap, the SDK's calibrated millimetres — is
  derived from it in `litegrip_pybullet.model`.
- **Speed-limited travel**: the fingers ramp at the rated 42.73 mm/s each, so a
  full stroke takes about 1 s and a commanded opening is reached without
  overshoot.
- **Force cap and friction grasping**: `command_fraction(..., force_n=...)` caps
  the grip, and `add_box()` plus `contacts()` let a test check that the jaws
  hold a part and slip when it pulls too hard.
- **Kinematic mirroring**: `reset_fraction()` teleports the jaws (no dynamics,
  no lag) so the simulation can display where a real gripper is right now.
- **Headless and windowed**: `gui=False` runs the same physics through PyBullet's
  DIRECT connection, which is what CI and `--headless` use.

### Hardware examples

- **Record and replay (03)**: teach a motion by hand in zero gravity, save it as
  a `.lgt` file, then replay it into the hardware and the window together, or
  into the window alone with `--play` (no connection, no frames sent).
- **Mirror (04)**: the gripper drives the simulation, one read per frame, with
  `--zero-gravity` for back-driving by hand and `--passive` for watching another
  program drive it.
- **Dual control (05)**: sliders command the hardware while the simulated jaws
  mirror its measured position, with a speed cap and an optional feed-forward
  grip force.
- **Idle keep-alive**: all three hardware examples keep streaming hold frames
  between moves, because an enabled motor that hears nothing latches a
  communication-loss fault — see [Safety](#safety).
- **Ramped targets**: position targets move one tick per frame and decelerate
  before arrival, so the torque a frame demands does not depend on the
  calibration's `kp` or on how fast a slider was dragged.
- **Diagnostics without motion**: `05 --status` reads the error code and the DM
  registers without enabling the motor or sending a motion command.

### Calibration and startup checks

- **Factory default**: with no `--calib`, the examples use the calibration
  shipped inside the SDK package, resolved relative to the package directory, so
  the same invocation works on another machine.
- **Explicit override**: `--calib <path>` uses this gripper's own calibration.
- **Verified before use**: the file is read and checked field by field, because
  the SDK's own loader falls back to the shipped factory values *silently* when
  the path it was given cannot be read.
- **No scanning**: with no `--calib` and no readable factory file the example
  stops and says to pass `--calib`. It never guesses which JSON on the machine
  belongs to this gripper. `--list-calibrations` prints the candidates when you
  need a path to pass.
- **SDK API check**: each hardware example verifies at startup that the SDK
  carries the members it calls and stops with the list of what is missing,
  rather than failing inside a control loop.

## Architecture

```text
┌──────────────────────────────────────────────────────────────┐
│  your program · examples/01–05 · examples/_common.py          │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.sim — GripperSim                           │
│  ├── command_fraction / command_joint / reset_fraction        │
│  ├── settle / run_for / step                                  │
│  ├── fraction / aperture_mm / finger_force_n                  │
│  ├── add_box / contacts / grasp_center / link_aabb            │
│  └── keyboard_events / mouse_events / status_text             │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.model — the unit boundary                  │
│  fraction ↔ joint travel ↔ jaw gap ↔ SDK millimetres          │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│  litegrip_pybullet.urdf — resolve and normalise               │
│  bundled xacro / $LITEGRIP_URDF_PATH / $LITEGRIP_URDF_DIR     │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    PyBullet (GUI or DIRECT) → URDF + STL
```

The three hardware examples add one more layer on top of `GripperSim`: the CAN
loop they run themselves, over the SDK's public frame-level API
(`send_mit_frame()` + `poll()`).

### Core modules

| Module | Responsibility |
| --- | --- |
| `src/litegrip_pybullet/sim.py` | `GripperSim`, `pressed()`, `clicked()`, the event and key constants |
| `src/litegrip_pybullet/model.py` | `STROKE_M`, `N_FINGERS`, the aperture constants and every conversion between the notations |
| `src/litegrip_pybullet/urdf.py` | URDF resolution (`resolve_urdf()`, `bundle_dir()`) and the PyBullet normalisation (`normalize_urdf()`) |
| `src/litegrip_pybullet/assets/litegrip_urdf/` | The bundled xacro and STL meshes |
| [`examples/_common.py`](examples/_common.py) | Argument parsers, SDK discovery, calibration choice, connect/enable, unit helpers, the status line |
| [`examples/03_trajectory.py`](examples/03_trajectory.py) | Record on the gripper, replay into both |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | Gripper → simulation |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | Simulation → gripper |

## Install

### 1. From a checkout (development)

```bash
git clone https://github.com/nexform-tech/litegrip-pybullet.git
cd litegrip-pybullet
python3 -m venv .venv && source .venv/bin/activate
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

### 2. Plain install

```bash
python3 -m pip install .
```

### 3. Build a wheel

```bash
python3 -m pip install build
python3 -m build --wheel
# the artifact lands in dist/
```

### Dependencies

- **`pybullet>=3.2`** — the simulator.
- **`numpy>=1.20`** — required explicitly because pybullet's wheel declares no
  requirements of its own, yet importing it fails without numpy.
- **`pytest>=7.0`** (`test` extra) — the test suite.

The model needs nothing else: the URDF, the xacro and the STL meshes are bundled
inside the package.

### The hardware SDK (examples 03, 04 and 05)

`litegrip` is **not on PyPI** — `pip install litegrip` installs something else,
and no released version carries the API these examples use. All three hardware
examples take it from one checkout,
[`nexform-tech/litegrip-python`](https://github.com/nexform-tech/litegrip-python),
resolved in this order:

1. `$LITEGRIP_SDK_DIR` — pointing at the directory that *contains* the
   `litegrip` package (`<repo>/src` in a src layout).
2. A sibling checkout next to this repository: `../litegrip-python`, tried as
   `src/` and then as the repository root.
3. Whatever `litegrip` is installed in the running interpreter.

```bash
python3 -m pip install -e /path/to/litegrip-python
# or
export LITEGRIP_SDK_DIR=/path/to/litegrip-python/src
```

**Do not** point `LITEGRIP_SDK_DIR` at a directory without a `litegrip` package
in it: the examples stop with an explanation instead of quietly importing a
same-named package from somewhere else. Another repository ships a package
called `litegrip` whose `__version__` is also 2.2.0 but which has no trajectory
API, so the version number cannot tell them apart — the startup API check can,
and it names every member that is missing.

## Quick start

### Simulation only

```bash
python3 examples/01_hello_sim.py --headless
```

```python
from litegrip_pybullet import GripperSim

sim = GripperSim()                  # gui=False by default; pass gui=True for a window
sim.command_fraction(0.0)           # 0 = closed, 1 = wide open
sim.settle()                        # ~1 s, speed-limited at the rated 85 mm/s
print(sim.aperture_mm(), sim.fraction(), sim.finger_force_n())
sim.disconnect()
```

### CAN bus setup

Every hardware example needs a 1 Mbit/s SocketCAN interface (default `can0`).

You do not have to configure it yourself. Before connecting, examples 03, 04 and 05
read the interface with `ip -details link show` and run the privileged `ip` commands
**only when its state is actually wrong**. An interface that is already right runs
no command at all and asks for no password. `--no-can-setup` turns the whole step
off, for when you manage the interface yourself.

The manual recipe, which is also what the automatic step runs:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 fd off
sudo ip link set can0 up
ip -details link show can0
```

Do not leave out `restart-ms`. The kernel default is `restart-ms 0`, which means the
controller never leaves bus-off on its own: one bad frame leaves the interface up,
at the right bitrate, and unable to send anything. Some USB adapters reject the
option; the examples retry that one command without it and leave the rest alone.

### Real hardware

```bash
python3 examples/04_mirror_real.py                     # read only enough to mirror
python3 examples/05_dual_control.py --dry-run           # the whole flow, no CAN traffic
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
python3 examples/03_trajectory.py                       # record, then replay
```

> **Examples 03, 04 and 05 move real hardware.** Read
> [Safety](#safety) and
> [examples/README.md](examples/README.md#before-you-drive-the-hardware) first.

## Core API

### GripperSim

| Method | Description |
| --- | --- |
| `GripperSim(urdf_path=None, *, gui=False, gravity=(0,0,-9.81), max_force_n=..., velocity_m_s=...)` | Connects PyBullet and loads the gripper; `gui=True` opens a window, `gui=False` uses DIRECT |
| `command_fraction(fraction, *, force_n=None, velocity_m_s=None)` | Command a normalised opening, clamped to `[0, 1]` |
| `command_joint(joint, *, force_n=None, velocity_m_s=None)` | Command a raw finger travel in metres, clamped to the URDF limits |
| `reset_fraction(fraction)` | Teleport the jaws kinematically — the mirror path |
| `step(n=1)` | Advance the simulation; returns `False` when the window was closed |
| `run_for(seconds)` | Step for a wall-clock duration |
| `settle(*, tolerance_m=..., timeout_s=3.0)` | Step until the target is reached; returns `(seconds, reached)` |
| `fraction()` / `aperture_mm()` / `joint_values()` / `finger_force_n()` | The current state |
| `add_box(half_extents, position, ...)` | Add a graspable part |
| `contacts(other_body)` | Contacts between a part and the gripper |
| `status_text(text)` | Write a line into the window |
| `keyboard_events()` / `mouse_events()` | Poll input; `pressed()` and `clicked()` interpret them |
| `disconnect()` | Close the window and the PyBullet connection |

### Unit conversions

The simulation and the hardware meet at the normalised opening, so the
conversions live in one module rather than in each example.

| Function | Description |
| --- | --- |
| `fraction_to_joint(fraction)` / `joint_to_fraction(joint)` | Normalised opening ↔ finger travel in metres |
| `fraction_to_aperture_mm(fraction)` / `joint_to_aperture_mm(joint)` | Jaw gap in millimetres, the distance a pair of calipers would measure |
| `sdk_mm_to_fraction(mm, max_stroke_mm=120.0)` / `fraction_to_sdk_mm(...)` | The SDK's *calibrated* millimetres |
| `clamp_fraction(fraction)` | Clamp to `[0, 1]` |

| Constant | Value | Meaning |
| --- | --- | --- |
| `STROKE_M` | `0.042726` m | Travel of one finger, full stroke |
| `N_FINGERS` | `2` | Parallel fingers |
| `APERTURE_CLOSED_MM` / `APERTURE_OPEN_MM` | `1.548` / `87.0` mm | Jaw gap at either end |
| `MAX_GRIP_FORCE_N` | `40.0` N | The library's force ceiling |
| `DEFAULT_MAX_STROKE_MM` | `120.0` mm | The SDK's default calibrated scale |

**SDK millimetres and the jaw gap are different quantities, not a unit
conversion apart.** `120` is the SDK's *default* `max_stroke_mm`; a calibration
file carries whatever scale it was saved with, and `rad_to_mm` is derived at
calibration time from a stroke length the loader never writes back. The examples
therefore convert between radians and the normalised opening directly. See
[One opening, three notations](examples/README.md#one-opening-three-notations).

### URDF resolution

`resolve_urdf(path)` tries, in order: an explicit path, `$LITEGRIP_URDF_PATH`,
`$LITEGRIP_URDF_DIR`, the bundled copy, then a sibling `litegrip-description` checkout.
Pointing the library at a different gripper model is therefore an environment
variable, not a code change. An unfindable model raises `UrdfError`.

## The five examples

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`examples/01_hello_sim.py`](examples/01_hello_sim.py) | reads the state, moves nothing | **No** |
| [`examples/02_move_sim.py`](examples/02_move_sim.py) | simulation only | **No** |
| [`examples/03_trajectory.py`](examples/03_trajectory.py) | records on the gripper, replays into both | Yes, to record or to replay onto it |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | gripper → simulation | Yes, to actually mirror |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | simulation → gripper | Yes, to actually move |

01 moves nothing at all: `tests/test_examples_cli.py` parses it and refuses it
any motion call. 03's `--play` is the one hardware-example path that runs without
a gripper, and the only one CI can run — in the runs where a checkout is present
to write the fixture with.

### Suggested learning path

```text
01_hello_sim.py        read the model, move nothing        (no hardware)
  ↓
02_move_sim.py         speed-limited motion                (no hardware)
  ↓
03_trajectory.py --play <file>   offline replay            (no hardware)
  ↓
04_mirror_real.py      the gripper drives the window       (hardware, read-mostly)
  ↓
05_dual_control.py --dry-run     the flow, no CAN traffic
  ↓
05_dual_control.py     the window drives the gripper       (hardware, moves)
  ↓
03_trajectory.py       record by hand, replay to both      (hardware, moves)
```

See [examples/README.md](examples/README.md) for what each one demonstrates, and
for the notation they all share.

## Calibration

Every path that touches the hardware has to decide which calibration to use,
because the angles and the millimetre scale are measured per gripper.

### The factory default

With no `--calib`, the examples use the calibration shipped inside the SDK
package — `factory_calibration.json`, next to the `litegrip` package's own
`__init__.py`. The path is derived from the package directory, never hard-coded,
so the same command works on another machine or in another virtual environment.

**That file holds the factory's bench-fixture measurements, not measurements of
your gripper.** It is a usable default, and its travel endpoints may not match
the unit in front of you. For anything beyond a first look, pass `--calib`.

### Overriding it

```bash
python3 examples/05_dual_control.py --calib ~/.litegrip/litegrip_calibration.json
```

The file comes from calibrating *this* gripper in the host software
(`litegrip-studio` / `litegrip-console`, or the SDK's own
`tools/gui/litegrip_gui.py`) and saving it. The examples read it themselves and
check it took effect field by field, refuse a file whose `can_id`/`mst_id` name a
different motor, and stop with an explanation on a file whose angles contradict
each other. The SDK's own `load_calibration()` is not trusted for this: with an
unreadable path it loads the shipped factory values *silently* and still returns
`True`, so a mistyped path would otherwise drive the motor with another
machine's angles.

### There is no picker

Changing calibration means naming a file: `--calib <path>`. The examples do not
scan the machine and do not ask. If neither `--calib` nor a readable factory file
is available, the run stops and says to pass `--calib` — an incomplete SDK
install is not a reason to drive the motor with numbers nobody chose.

`--list-calibrations` prints the candidates in `~/.litegrip` with their key
values (closed/open angles, `rad_to_mm`, `kp`, `mst_id`) and exits 0. It is a
query: it connects to nothing, sends nothing, and changes no default. The
`*.sim.json` file the studio writes for its simulator backend and the `*.bak`
backups are never listed, and a `*.sim.json` named explicitly is refused — that
scale belongs to the simulated gripper.

## Safety

### Speed and force limits

| Limit | Value | Where it is enforced |
| --- | --- | --- |
| Finger speed | 42.73 mm/s each, 85 mm/s jaw | `velocity_m_s` in `GripperSim`, default 42.73 mm/s |
| Grip force | 10 N by default, 40 N rated maximum | `DEFAULT_MAX_FORCE_N`, `MAX_GRIP_FORCE_N`; override per command with `force_n` |
| Position-target step | one ramp tick per frame | examples 03, 04 and 05 |

### Hold frames and the 0xD communication-loss fault

An **enabled** motor that hears nothing for about **0.9 s** latches the
communication-loss fault (`0xD`, which the SDK's `describe_error` names
`通讯丢失 (CAN 超时)`). The symptom is the one people report as "it reads the
position but won't take commands, red LED blinking": the position still reads,
and every command is silently ignored. **A quiet idle period is itself the fault
cause.**

The 0.9 s is a measurement on this hardware. Do **not** derive it from the
`TIMEOUT` register (RID 9): that register read 8000 ms on one occasion and 0
(currently inactive) on another, and the SDK marks it unresolved. `--status`
prints the register's live value for the record and says so next to it.

All three hardware examples therefore stream hold frames at 200 Hz through every
gap in which nothing else is feeding the motor — target = the measured position,
zero feed-forward, no motion commanded. 03 does it between its recording and
replay phases; 04's `--passive` opts out entirely when another program is driving
the bus, and does not enable the motor either.

While a recording or a replay is running, the SDK is streaming the bus from its
own thread. **Do not** insert frames of your own at the same time: a second
stream on the same wire tears the trajectory apart.

### Step commands and the 0x9/0xA fault

The MIT position term is `kp × (q_target − q_actual)`, and `kp` is an entry in
the calibration file rather than a constant. Sending a whole target in one frame
asks the motor for `kp × 1.845 rad` — about 185 Nm at the SDK's default
`kp = 100`, against a motor rated around 10 Nm. The current saturates and the
motor latches an under-voltage/over-current fault (0x9/0xA), after which it keeps
reporting its position while ignoring commands.

Example 05 ramps the *position target* one tick per frame instead, which bounds
the demand whatever `kp` is configured to — at this machine's `kp = 5.0` the same
step asks for about 9 Nm. It also decelerates before arrival, because a frame
that drops the target velocity from full speed to zero reverses the damping term
into a `kd × v` torque step.

Both faults are latched: they do not clear by themselves until the gripper is
power-cycled. They can be read and cleared without moving the motor:

```bash
python3 examples/05_dual_control.py --status                  # read only, no motion command
python3 examples/05_dual_control.py --status --clear-fault    # disable → clear → enable
```

`--status` does not open a window, does not enable the motor and sends no
command frame. It also **probes** the CAN interface without repairing it: it prints
what it read and leaves the interface exactly as it found it. Changing the interface
underneath would delete the evidence `--status` exists to collect. 04's `--passive`
behaves the same way. A DM motor answers commands — one status frame per command frame,
in any enable state — so with nothing sent there is nothing answered and no
position can be read at all; that is reported as "cannot tell", exit 0, not as a
fault, and the DM registers are still read. Only `--clear-fault` sends frames,
and its disable → clear → enable sequence leaves the fingers limp for an
instant.

## Verification status

What has been verified, and what has not:

| Capability | Status | Evidence |
| --- | --- | --- |
| URDF/xacro normalisation | ✅ Verified | `tests/test_urdf.py`: the upstream ROS 2 xacro loads in PyBullet, 2 prismatic joints, meshes present |
| Model geometry (stroke ↔ opening) | ✅ Verified | `tests/test_model.py`, including a check that the bundled xacro's `stroke` default matches `STROKE_M` |
| Speed-limited motion | ✅ Verified | `tests/test_sim.py`: full stroke 1.00 s of ramp + ~0.13 s of servo settling, no overshoot |
| Force cap and friction grasping | ✅ Verified | `tests/test_sim.py`: holds 5 N against a 10 N grip, slips at 15 N; only the fingers touch the part |
| Examples 01–02 (simulation only) | ✅ Verified | Both run headless and exit 0, with their output asserted in `tests/test_examples_cli.py`; 01 is additionally parsed and refused any motion call |
| Example 03 `--play` (offline replay) | ✅ Verified | Reads a `.lgt`, drives the window from it, sends nothing: run headless and exit 0, asserted end to end in `tests/test_example03_loop.py` and against a real file in `tests/test_examples_cli.py` |
| One SDK checkout, factory-calibration default | ✅ Verified (no hardware) | `tests/test_common.py` covers the resolution order (`$LITEGRIP_SDK_DIR` → sibling → installed), the loud failure on a wrong `LITEGRIP_SDK_DIR`, `factory_calibration_path()` resolving through the package, and the three-tier calibration choice; `tests/test_examples_cli.py` covers the paths end to end |
| Example 03 recording and hardware replay | ⚠️ **Not verified** | Neither has been run against a gripper: each one moves real hardware. The sample-rate, pose-conversion, hold-frame and "confirm before the replay" logic is covered by `tests/test_example03_loop.py` against a stand-in SDK |
| Example 03's "click the window" go-ahead | ⚠️ **Partly verified** | The keyboard half is unit-tested; on the mouse half `getMouseEvents()` was called in a real window (0 events, no error) but **nobody has actually clicked** — the 5-tuple `clicked()` reads is taken from pybullet's own `pybullet_examples/createVisualShapeArray.py` |
| Example 04 (hardware → simulation) | ⚠️ **Partially verified** | The read-only mirror path was run against a real gripper on `can0` **with the other SDK checkout** — the one this repository no longer uses. `--zero-gravity` and `--passive` were not run at all, and the same paths have not been re-run against `litegrip-python` |
| Example 05 (simulation → hardware) | ⚠️ **Partially verified** | Run against a real gripper on `can0`, where it latched a motor fault; the ramp fix is covered by `tests/test_example05_stream.py` but has **not** itself been run against hardware. The drag-driven interaction, the travel-normalised slider mapping and the deceleration before arrival have only been exercised in tests |
| Example 05 `--status` diagnostics | ⚠️ **Partially verified** | On a real gripper on 2026-09-28 it read a live status frame, the position, the error code and the DM registers without enabling or moving the motor (exit 0) — again with the other SDK checkout. The reporting path for a **latched fault**, clearing one, and the new "disabled motor, no position readable, exit 0" behaviour have not been exercised on hardware |
| Real-hardware motion commands | ⚠️ **Partially verified** | A step-command stream was sent to hardware from an earlier revision of example 05 and faulted the motor; the ramped replacement has not been run yet |
| Idle keep-alive (`IdleKeeper`, 04's hold frames) | ⚠️ **Partially verified** | Unit-tested for cadence and for the gap staying under the timeout; **never run against hardware** |
| The CAN probe (`ensure_can_link`, `--no-can-setup`) | ⚠️ **Partly verified** | Reading is verified: the parser is pinned against real `ip -details link show` transcripts (including a bus-off one where every flag looks healthy), and `probe_can_link('can0')` was run read-only on this machine and read the interface correctly. The **repair** — the `sudo ip` sequence that brings a wrong interface up — has only been exercised against a stand-in `run`; nobody has watched it fix a real interface |

The simulation dynamics are PyBullet's, with the URDF's own inertias. The finger
speed limit and the force cap are enforced by this library; the reported servo
settling time is a measured property of the simulated position servo, not a
hardware measurement.

## Troubleshooting

### CAN does not come up

Read it with `-details`. The flag list alone is not enough, because the two causes
below look identical without `can state`:

```bash
ip -details link show can0
```

- **The interface is not up.** The flags read `<NOARP>` — no `UP`. Examples 03, 04
  and 05 fix this themselves before connecting; by hand it is the recipe under
  [CAN bus setup](#can-bus-setup).
- **The controller is bus-off.** The flags read `<NOARP,UP,LOWER_UP>` and the bitrate
  is right, so every flag looks healthy — but not one frame can be sent. Only
  `can state BUS-OFF` says so. With `restart-ms 0` it stays that way until something
  reconfigures the interface.

`enable()` failing with `[Errno 100] Network is down` is a host link problem, not a
gripper one. A CAN socket binds happily on an interface that is down, so `connect()`
succeeds and the failure only surfaces when the first frame is sent — which is why
it used to be reported as "the gripper may not be powered". The examples now name
the link instead; check `can state` before touching the wiring.

The examples default to `can0` and CAN ID `0x08`; `--channel` and `--can-id`
change both. A USB-CAN adapter that disappears mid-run shows up as `ENOBUFS`
from `enable()` — that is the adapter, not the gripper.

### "It reads the position but won't take commands", red LED blinking

Two lookalike causes, both described under [Safety](#safety): a step command
(0x9/0xA) or an idle gap longer than the watchdog (0xD). Diagnose without moving
the motor:

```bash
python3 examples/05_dual_control.py --status
python3 examples/05_dual_control.py --status --clear-fault
```

### A second master on the same bus

Two streams on one interface fight, and neither side wins. Check nothing else
has the interface open before running a hardware example:

```bash
pgrep -af python3 | grep -i litegrip        # who has CAN open
ip -details -statistics link show can0      # counters climbing while you run nothing?
```

`ip link set can0 down` / `up` does **not** evict that program — its socket
survives and resumes when the interface returns. The process has to exit.

### The examples cannot find the SDK

```text
找不到真机 SDK（litegrip 包）：LITEGRIP_SDK_DIR 指到 /path,但那里没有 litegrip/__init__.py。
```

The variable names the directory that *contains* the `litegrip` package, not the
package directory and not the repository root. Either unset it, or point it at
`<repo>/src`. `$LITEGRIP_TRAJ_SDK_DIR` no longer exists: all three hardware
examples use the same checkout now, so a script that still sets it should set
`LITEGRIP_SDK_DIR` instead.

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |
| [litegrip-description](https://github.com/nexform-tech/litegrip-description) | URDF/xacro description package |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |

## Documentation

- [Example guide](examples/README.md) — what each of the five examples
  demonstrates, the notation they share, and the checklist before driving
  hardware.

## Development

```bash
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

The suite needs no CAN interface and no display: it skips hardware entirely and
runs PyBullet headless. The no-SDK case is part of it — with no checkout
anywhere, `tests/test_examples_cli.py` still asserts that each example stops with
a readable message instead of a traceback. That is the state CI runs in, where
`litegrip` is not installed at all.

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).

## Contact

- Repository: [github.com/nexform-tech/litegrip-pybullet](https://github.com/nexform-tech/litegrip-pybullet)
- Issues: [github.com/nexform-tech/litegrip-pybullet/issues](https://github.com/nexform-tech/litegrip-pybullet/issues)
