# litegrip-pybullet

PyBullet simulation environment for the **LiteGrip lightweight robotic gripper
series** — a physical model of the gripper, plus the three examples that show it
being driven in both directions.

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
python3 -m pip install litegrip      # optional: to drive real hardware
```

The gripper model (URDF + STL meshes) is bundled, so nothing else is needed to
load it.

## Quick start

```bash
python3 examples/01_sim_only.py --headless
```

```python
from litegrip_pybullet import GripperSim

sim = GripperSim()                  # opens a window by default
sim.command_fraction(0.0)           # 0 = closed, 1 = wide open
sim.settle()                        # ~1 s, speed-limited at the rated 85 mm/s
print(sim.aperture_mm(), sim.fraction(), sim.finger_force_n())
sim.disconnect()
```

## The three examples

| File | Direction | Hardware needed? |
| --- | --- | --- |
| [`examples/01_sim_only.py`](examples/01_sim_only.py) | — (pure PyBullet) | **No** |
| [`examples/02_sim_to_real.py`](examples/02_sim_to_real.py) | simulation → gripper | Yes, to actually move |
| [`examples/03_real_to_sim.py`](examples/03_real_to_sim.py) | gripper → simulation | Yes, to actually mirror |

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

## Status

What has been verified, and what has not:

| Capability | Status | Evidence |
| --- | --- | --- |
| URDF/xacro normalisation | ✅ Verified | `tests/test_urdf.py`: the upstream ROS 2 xacro loads in PyBullet, 2 prismatic joints, meshes present |
| Model geometry (stroke ↔ opening) | ✅ Verified | `tests/test_model.py`, including a check that the bundled xacro's `stroke` default matches `STROKE_M` |
| Speed-limited motion | ✅ Verified | `tests/test_sim.py`: full stroke 1.00 s of ramp + ~0.13 s of servo settling, no overshoot |
| Force cap and friction grasping | ✅ Verified | Holds 5 N against a 10 N grip, slips at 15 N; only the fingers touch the part |
| Example 01 | ✅ Verified | Runs headless, exits 0, all four demos asserted in `tests/test_examples_cli.py` |
| Example 02 (simulation → hardware) | ⚠️ **Partially verified** | Run against a real gripper on `can0`, where it latched a motor fault; the ramp fix below is covered by `tests/test_example02_stream.py` but has **not** itself been run against hardware |
| Example 02 `--status` diagnostics | ⚠️ **Partially verified** | `--status` on a real gripper reaches `connect()` and reports a missing CAN interface correctly; reading and clearing a latched fault has not been exercised on hardware |
| Example 03 (hardware → simulation) | ⚠️ **Partially verified** | The read-only mirror path was run against a real gripper on `can0`; `--zero-gravity` and `--passive` were not |
| Real-hardware motion commands | ⚠️ **Partially verified** | A step-command stream was sent to hardware from an earlier revision of example 02 and faulted the motor; the ramped replacement has not been run yet |
| Idle keep-alive (`IdleKeeper`, 03's hold frames) | ⚠️ **Partially verified** | Unit-tested for cadence and for the gap staying under the timeout; **never run against hardware** |

"Reads the position but won't take commands, red LED blinking" has two lookalike
causes on this hardware:

- **A step command** (the whole target in one frame): the MIT position term
  `kp × (q_target − q_actual)` at `kp = 100 Nm/rad` over a 1.845 rad travel asks
  for ~185 Nm from a ~10 Nm motor, which latches an under-voltage/over-current
  fault (0x9/0xA). Example 02 ramps like the SDK's own `goto_rad`, one tick per
  frame, so a single frame demands under 1 Nm.
- **Idling too long**: the `TIMEOUT` register (RID 9, measured 8000 ms here) is a
  CAN watchdog — that long with no frame received latches a communication-loss
  fault (**0xD**, a code the SDK's `describe_error` does not yet know). **A quiet
  idle period is itself the fault cause**, and it was the dominant one behind
  "simulation can read the gripper but not control it." Both examples now send
  hold frames (target = measured position, zero feed-forward) while idle; 03's
  `--passive` opts out of that when another program is driving the bus.

Both faults are diagnosed and cleared without moving the motor via
`examples/02_sim_to_real.py --status [--clear-fault]`.

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
