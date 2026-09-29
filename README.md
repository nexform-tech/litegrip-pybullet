# litegrip-pybullet

PyBullet simulation environment for the **LiteGrip lightweight robotic gripper
series** — a physical model of the gripper, plus the five examples that go from
reading the model to driving real hardware in both directions.

**English** · [简体中文](README.zh-CN.md)

| Property | Value |
| --- | --- |
| Product | LiteGrip lightweight robotic gripper series |
| Repository role | PyBullet simulation environment |
| Model | Two parallel prismatic fingers, 42.726 mm travel each |
| Jaw opening | 1.548 mm closed … 87.0 mm open |
| Rated finger speed | 85 mm/s |
| Python | 3.8+ |

## Install

```bash
python3 -m pip install pybullet      # simulation
python3 -m pip install -e .          # this package
```

The gripper model (URDF + STL meshes) is bundled, so nothing else is needed to
load it. To drive real hardware, examples 04 and 05 additionally need the
`litegrip` SDK, which is **not on PyPI** — take it from a checkout, either
`python3 -m pip install -e /path/to/lite-grip`, a `lite-grip` directory beside
this repository, or `$LITEGRIP_SDK_DIR`. Without one of those they stop at
startup and name the SDK members they require.

## Quick start

```bash
python3 examples/01_hello_sim.py --headless
```

```python
from litegrip_pybullet import GripperSim

sim = GripperSim()                  # opens a window by default
sim.command_fraction(0.0)           # 0 = closed, 1 = wide open
sim.settle()                        # ~1 s, speed-limited at the rated 85 mm/s
print(sim.aperture_mm(), sim.fraction(), sim.finger_force_n())
sim.disconnect()
```

## The five examples

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`examples/01_hello_sim.py`](examples/01_hello_sim.py) | reads the state, moves nothing | **No** |
| [`examples/02_move_sim.py`](examples/02_move_sim.py) | simulation only | **No** |
| [`examples/03_grasp.py`](examples/03_grasp.py) | simulation only | **No** |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | gripper → simulation | Yes, to actually mirror |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | simulation → gripper | Yes, to actually move |

See [examples/README.md](examples/README.md) for what each one demonstrates and
the notation they all share.

## One opening, three notations

Mixing these up is the usual source of confusion, so the library converts through
a single normalised opening (0 closed … 1 wide open):

| Notation | Range | Meaning |
| --- | --- | --- |
| Joint value | `0 … 0.042726` m | prismatic travel of one finger; `0` is wide open |
| Normalised opening | `0 … 1` | the one shared quantity |
| SDK millimetres | `0 … 120` mm | the *calibrated* scale the hardware reports |
| Jaw gap | `1.548 … 87` mm | physical distance between the finger faces |

SDK millimetres and the physical jaw gap are not related by a unit conversion —
they are different quantities. Only the normalised opening means the same thing
on both sides.

The `120` is the SDK's *default* `max_stroke_mm`; a calibration file carries
whatever scale it was saved with, so the examples convert between radians and the
normalised opening directly rather than through millimetres. See
[One opening, three notations](examples/README.md#one-opening-three-notations).

## Status

What has been verified, and what has not:

| Capability | Status | Evidence |
| --- | --- | --- |
| URDF/xacro normalisation | ✅ Verified | `tests/test_urdf.py`: the upstream ROS 2 xacro loads in PyBullet, 2 prismatic joints, meshes present |
| Model geometry (stroke ↔ opening) | ✅ Verified | `tests/test_model.py`, including a check that the bundled xacro's `stroke` default matches `STROKE_M` |
| Speed-limited motion | ✅ Verified | `tests/test_sim.py`: full stroke 1.00 s of ramp + ~0.13 s of servo settling, no overshoot |
| Force cap and friction grasping | ✅ Verified | Holds 5 N against a 10 N grip, slips at 15 N; only the fingers touch the part |
| Examples 01–03 (simulation only) | ✅ Verified | All three run headless and exit 0, with their output asserted in `tests/test_examples_cli.py`; 01 is additionally parsed and refused any motion call |
| Example 04 (hardware → simulation) | ⚠️ **Partially verified** | The read-only mirror path was run against a real gripper on `can0`; `--zero-gravity` and `--passive` were not |
| Example 05 (simulation → hardware) | ⚠️ **Partially verified** | Run against a real gripper on `can0`, where it latched a motor fault; the ramp fix below is covered by `tests/test_example05_stream.py` but has **not** itself been run against hardware. The drag-driven interaction (no send key, an enabled-but-stationary start, the speed slider) has likewise only been exercised in tests, as have the travel-normalised slider mapping and the deceleration before arrival |
| Example 05 `--status` diagnostics | ⚠️ **Partially verified** | On a real gripper on 2026-09-28 it read a live status frame, the position, the error code and the DM registers without enabling or moving the motor (exit 0); the reporting path for a **latched fault**, and clearing one, have not been exercised on hardware |
| Real-hardware motion commands | ⚠️ **Partially verified** | A step-command stream was sent to hardware from an earlier revision of example 05 and faulted the motor; the ramped replacement has not been run yet |
| Idle keep-alive (`IdleKeeper`, 04's hold frames) | ⚠️ **Partially verified** | Unit-tested for cadence and for the gap staying under the timeout; **never run against hardware** |

"Reads the position but won't take commands, red LED blinking" has two lookalike
causes on this hardware:

- **A step command** (the whole target in one frame): the MIT position term
  `kp × (q_target − q_actual)` over a 1.845 rad travel asks for ~185 Nm from a
  ~10 Nm motor at the SDK's default `kp = 100 Nm/rad`, which latches an
  under-voltage/over-current fault (0x9/0xA). `kp` is a calibration entry, not a
  constant — at this machine's present 5.0 the same step asks for ~9 Nm — so
  example 05 ramps the *position target* like the SDK's own `goto_rad`, one tick
  per frame, which bounds the demand whatever `kp` is.
- **Idling too long**: an enabled motor latches a communication-loss fault
  (**0xD**, which the SDK's `describe_error` now names `通讯丢失 (CAN 超时)`)
  after roughly **0.9 s** of silence — the SDK's own measurement on this
  hardware. **A quiet idle period is itself the fault cause**, and it was the
  dominant one behind "simulation can read the gripper but not control it." The
  `TIMEOUT` register (RID 9) does not give that duration — it read 8000 ms on
  2026-09-24 and 0 ms (watchdog off) on 2026-09-28, and the SDK marks it
  unresolved; `--status` prints the live value without drawing a conclusion from
  it. Both examples keep sending hold frames (target = measured position, zero
  feed-forward) while idle regardless; 04's `--passive` opts out of that when
  another program is driving the bus.

Both faults are diagnosed and cleared without moving the motor via
`examples/05_dual_control.py --status [--clear-fault]`.

Every example path that touches the hardware also has to be told which
calibration file to use: pass `--calib <path>`, or let it list the candidates on
a terminal and pick one. The default is deliberately not accepted, because the
SDK's `load_calibration` falls back to the shipped factory calibration *silently*
when the path it was given cannot be read. Calibrate this gripper in the host
software and save the file from there — see
[examples/README.md](examples/README.md#before-you-drive-the-hardware).

The simulation dynamics are PyBullet's, with the URDF's own inertias. The
finger speed limit and the force cap are enforced by this library; the reported
servo settling time is a measured property of the simulated position servo, not a
hardware measurement.

## Related repositories

| Repository | Role |
| --- | --- |
| [litegrip-python](https://github.com/nexform-tech/litegrip-python) | Python SDK |
| [litegrip-cpp](https://github.com/nexform-tech/litegrip-cpp) | C++ SDK |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver |
| [litegrip-urdf](https://github.com/nexform-tech/litegrip-urdf) | URDF/xacro description package |
| [litegrip-docs](https://github.com/nexform-tech/litegrip-docs) | Product documentation |

## Development

```bash
python3 -m pip install -e ".[test]"
python3 -m pytest -q
```

The URDF is resolved from a bundled copy by default. To point the library at a
different model, in order of precedence: an explicit `urdf_path`, `$LITEGRIP_URDF_PATH`,
`$LITEGRIP_URDF_DIR`, the bundled copy, then a sibling `litegrip-urdf` checkout.

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
