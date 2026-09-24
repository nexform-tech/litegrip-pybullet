# -*- coding: utf-8 -*-
"""Example 02's MIT stream: the ramp, the rate limit, and fault handling.

This file exists because of a real failure: the first version of example 02
streamed a *constant* target rad for the whole move, so the very first frame
asked a ~10 Nm motor for ``kp × 1.845 rad ≈ 185 Nm``.  The gripper latched an
under-voltage/over-current fault (blinking red LED), kept reporting its
position, and ignored every command afterwards — "it reads fine but I can't
control it".

The tests below pin the shape of the stream, not its timing, by driving
``StreamMove.at()`` with synthetic timestamps: no sleeping, no CAN, no
hardware.  The key assertion is that no single frame moves the position target
far enough for ``kp`` to demand more torque than the motor can deliver.
"""

import importlib.util
import os
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("pybullet")

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

#: ``_common`` re-execs into the repo .venv when pybullet is missing; under
#: pytest that would replace the test process.
os.environ.setdefault("LITEGRIP_PYBULLET_REEXEC", "1")

if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))


def _load_example_02():
    """Import ``02_sim_to_real.py`` as a module (its name is not an identifier)."""
    path = EXAMPLES / "02_sim_to_real.py"
    spec = importlib.util.spec_from_file_location("example02", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["example02"] = module   # so dataclass/pickle lookups work
    spec.loader.exec_module(module)
    return module


ex02 = _load_example_02()

#: The DM4310 in the gripper is rated around 10 Nm.  A frame whose commanded
#: position sits far enough from where the motor actually is that ``kp ×
#: error`` exceeds this is a frame the motor cannot execute — it either
#: saturates into a fault or just lags, which is what caused the bug.
MOTOR_RATED_NM = 10.0

KP = 100.0       # the SDK's MIT position stiffness
KD = 2.0
OPEN_RAD = -1.731
CLOSED_RAD = 0.114


class RecordingGripper:
    """Stands in for ``LiteGrip``: records frames instead of sending CAN."""

    def __init__(self, ok: bool = True) -> None:
        self.frames: list[dict] = []
        self.ok = ok
        self.position_rad = OPEN_RAD
        self.config = _config().config

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return self.ok

    def get_state(self, wait: bool = True):
        return SimpleNamespace(position_rad=self.position_rad, force_n=0.0,
                               position_mm=0.0, is_moving=False,
                               error_code=1, is_error=False)


def _config(**overrides):
    """A calibrated gripper config, as ``load_calibration`` leaves it."""
    values = dict(
        max_stroke_mm=120.0,
        rad_to_mm=120.0 / (CLOSED_RAD - OPEN_RAD),
        pos_closed_rad=CLOSED_RAD,
        pos_open_rad=OPEN_RAD,
        kp=KP,
        kd=KD,
    )
    values.update(overrides)
    return SimpleNamespace(config=SimpleNamespace(**values))


def _run_stream(move, frames: int | None = None) -> RecordingGripper:
    """Drive ``move`` at :data:`ex02.FRAME_HZ` from t=0 and return the frames."""
    move.started = 0.0                      # synthetic clock, no sleeping
    total = move.duration_s + move.hold_s
    count = frames if frames is not None else int(total * ex02.FRAME_HZ) + 1
    gripper = RecordingGripper()
    for i in range(count):
        move.send(gripper, i * ex02.FRAME_DT)
    return gripper


def worst_torque_demand(frames, kp: float, start_rad: float) -> float:
    """The hardest single correction the stream asks the servo to make [Nm].

    The servo only sees where it *is* and where it is *told to go*: ``kp ×
    (q_target − q_actual)``.  Between two consecutive frames the motor has had
    at most one tick to follow, so the distance the target jumps since the last
    frame is the error the servo has to make up — and before the first frame
    the motor is sitting at ``start_rad``.

    A jump of the whole travel is what latched the fault: ``100 Nm/rad ×
    1.845 rad ≈ 185 Nm`` on a ~10 Nm motor.
    """
    worst = 0.0
    previous = start_rad
    for frame in frames:
        worst = max(worst, kp * abs(frame["q"] - previous))
        previous = frame["q"]
    return worst


# ═══════════════════════════════════════════════════════════════════════════
# The ramp — the regression test for the fault this file is named after
# ═══════════════════════════════════════════════════════════════════════════


class TestStreamRamp:
    def _move(self, start=OPEN_RAD, target=CLOSED_RAD, duration=1.0, hold=0.5,
              tau=1.0):
        return ex02.StreamMove(
            start_rad=start, target_rad=target, duration_s=duration,
            hold_s=hold, kp=KP, kd=KD, tau_nm=tau)

    def test_no_frame_asks_the_motor_for_more_than_it_can_give(self):
        """The bug in one assertion.

        Every frame must stay close enough to the previous one (and the first
        one close enough to where the motor actually is) that ``kp`` turns the
        jump into a torque the motor can deliver.
        """
        move = self._move()
        gripper = _run_stream(move)
        assert len(gripper.frames) > 100, "expected a full 200 Hz stream"

        demanded = worst_torque_demand(gripper.frames, KP, move.start_rad)
        assert demanded < MOTOR_RATED_NM, (
            f"the stream asks for {demanded:.1f} Nm in a single frame from a "
            f"{MOTOR_RATED_NM:g} Nm motor — that is a step, not a ramp")

    def test_that_assertion_would_have_caught_the_original_bug(self):
        """Feed the pre-fix stream through the same predicate, and fail it.

        The old code sent the target rad on every frame: the frames never
        differed from each other, so only the jump from *where the motor
        actually was* reveals it.  Keeping that counter-example here is what
        stops the assertion above from quietly becoming toothless again.
        """
        frames = [dict(q=CLOSED_RAD, tau=0.0) for _ in range(200)]
        demanded = worst_torque_demand(frames, KP, OPEN_RAD)
        assert demanded == pytest.approx(KP * abs(CLOSED_RAD - OPEN_RAD))
        assert demanded > MOTOR_RATED_NM * 10, (
            "the old step should be flagged as wildly over the rating")

    def test_the_first_frame_is_the_start_not_the_target(self):
        """The exact old behaviour: frame #1 already sat at the target."""
        move = self._move()
        gripper = _run_stream(move, frames=1)
        assert gripper.frames[0]["q"] == pytest.approx(OPEN_RAD)
        assert gripper.frames[0]["q"] != pytest.approx(CLOSED_RAD)

    def test_the_ramp_keeps_up_with_a_200_hz_stream(self):
        """Commanded positions advance by exactly one tick per frame."""
        move = self._move()
        gripper = _run_stream(move)
        steps = [abs(b["q"] - a["q"])
                 for a, b in zip(gripper.frames, gripper.frames[1:])]
        assert max(steps) <= move.speed_rad_s * ex02.FRAME_DT * 1.01

    def test_the_ramp_is_monotonic_from_start_to_target(self):
        gripper = _run_stream(self._move())
        ramp = [f["q"] for f in gripper.frames
                if f["tau"] == 0.0 and f["q"] != CLOSED_RAD]
        assert ramp[0] == pytest.approx(OPEN_RAD)
        assert ramp == sorted(ramp)
        assert max(ramp) < CLOSED_RAD

    def test_the_ramp_carries_velocity_feed_forward_and_no_force(self):
        """Tau is held back for the hold phase — pushing mid-travel is wrong."""
        gripper = _run_stream(self._move())
        ramp = [f for f in gripper.frames if f["q"] < CLOSED_RAD - 1e-9]
        assert ramp, "no frame landed strictly inside the ramp"
        assert all(f["tau"] == 0.0 for f in ramp)
        assert all(f["dq"] > 0.0 for f in ramp)      # opening → closing

    def test_the_hold_phase_sits_still_and_pushes(self):
        move = self._move(tau=1.5)
        gripper = _run_stream(move)
        hold = [f for f in gripper.frames if f["q"] == CLOSED_RAD]
        assert len(hold) == pytest.approx(ex02.FRAME_HZ * move.hold_s,
                                          abs=2)
        assert all(f["tau"] == 1.5 for f in hold)
        assert all(f["dq"] == 0.0 for f in hold)

    def test_a_closing_move_runs_the_ramp_in_the_other_direction(self):
        move = self._move(start=CLOSED_RAD, target=OPEN_RAD)
        gripper = _run_stream(move)
        assert gripper.frames[0]["q"] == pytest.approx(CLOSED_RAD)
        ramp = [f for f in gripper.frames if f["q"] != OPEN_RAD]
        assert all(f["dq"] < 0.0 for f in ramp)

    def test_a_zero_length_move_does_not_divide_by_zero(self):
        move = self._move(start=OPEN_RAD, target=OPEN_RAD)
        assert move.speed_rad_s == 0.0
        gripper = _run_stream(move, frames=10)
        assert [f["q"] for f in gripper.frames] == [OPEN_RAD] * 10

    def test_a_dropped_frame_is_reported_rather_than_counted(self):
        """``send_mit_frame`` returns False when the gripper is not enabled."""
        move = self._move()
        move.started = 0.0
        assert move.send(RecordingGripper(ok=False), 0.0) is False
        assert move.frames == 0
        assert move.send(RecordingGripper(ok=True), 0.0) is True
        assert move.frames == 1


# ═══════════════════════════════════════════════════════════════════════════
# The idle keep-alive
# ═══════════════════════════════════════════════════════════════════════════


class TestIdleKeeper:
    """A gripper that hears nothing for TIMEOUT ms latches a comms-loss fault.

    Read off the motor itself: register ``TIMEOUT`` (RID 9) = 8000, so eight
    seconds of silence is enough to wedge it — position still readable, every
    command ignored, LED blinking. An interactive example that only transmits
    while a move is in flight therefore breaks the hardware by being *looked
    at* for eight seconds. These tests pin down that the idle path keeps
    talking, and that what it says cannot move anything.
    """

    #: The motor's own CAN timeout, in seconds, as the hardware reports it.
    WATCHDOG_S = 8.0

    #: 64 Hz = 1/64 s per tick, a power of two, so every test timestamp below is
    #: exact in binary and the cadence assertions cannot drift.
    HZ = 64.0
    TICK = 1.0 / HZ

    def _keeper(self, gripper=None):
        return ex02.IdleKeeper(gripper or RecordingGripper(), hz=self.HZ)

    def test_it_sends_at_its_own_cadence(self):
        keeper = self._keeper()
        for i in range(int(self.HZ) + 1):          # 1.0 s
            keeper.maybe_send(i * self.TICK)
        assert keeper.frames == int(self.HZ) + 1

    def test_the_first_frame_goes_out_immediately(self):
        """Waiting one interval before the first frame is a needless silence."""
        keeper = self._keeper()
        assert keeper.maybe_send(0.0) is True
        assert keeper.maybe_send(0.0) is None      # ...then not again

    def test_it_never_leaves_a_gap_long_enough_to_trip_the_watchdog(self):
        """The regression test for the fault this whole file is about.

        Whatever the main loop's cadence does — and a PyBullet loop is much
        slower than 200 Hz — the gap between consecutive frames has to stay far
        below the motor's own timeout. The loop below ticks at half the idle
        rate, which is the worst case the keeper can be asked to cover.
        """
        keeper = self._keeper()
        step = self.TICK / 2
        for i in range(int(20 / step)):            # 20 s of idling
            keeper.maybe_send(i * step)
        assert keeper.frames == int(20 / self.TICK), "one frame per interval"
        assert keeper.interval * 2 < self.WATCHDOG_S
        assert step < self.WATCHDOG_S

    def test_the_idle_frame_cannot_move_anything(self):
        """Hold at the measured position: zero torque, zero velocity.

        ``kp × (q_target − q_measured)`` is zero by construction, so the frame
        is a no-op that only feeds the timeout counter.
        """
        gripper = RecordingGripper()
        gripper.position_rad = CLOSED_RAD
        keeper = ex02.IdleKeeper(gripper, hz=50.0)

        keeper.maybe_send(0.0)
        frame = gripper.frames[0]
        assert frame["q"] == pytest.approx(CLOSED_RAD)
        assert frame["dq"] == 0.0
        assert frame["tau"] == 0.0

    def test_it_repeats_the_last_command_so_the_grip_survives(self):
        """Idling must not silently drop a grip force the user asked for."""
        gripper = RecordingGripper()
        keeper = ex02.IdleKeeper(gripper, hz=50.0)
        move = ex02.StreamMove(
            start_rad=OPEN_RAD, target_rad=CLOSED_RAD, duration_s=1.0,
            hold_s=0.5, kp=KP, kd=KD, tau_nm=1.7)
        move.started = 0.0
        keeper.remember(move)
        keeper.maybe_send(0.0)
        assert gripper.frames[0]["q"] == pytest.approx(CLOSED_RAD)
        assert gripper.frames[0]["tau"] == pytest.approx(1.7)

    def test_dropped_keepalives_are_counted(self):
        keeper = self._keeper(RecordingGripper(ok=False))
        for i in range(5):
            keeper.maybe_send(i * 1 / 50.0)
        assert keeper.frames == 0
        assert keeper.dropped == 5


# ═══════════════════════════════════════════════════════════════════════════
# The rate limit
# ═══════════════════════════════════════════════════════════════════════════


class TestPlanDuration:
    def test_auto_duration_keeps_the_fingers_at_the_rated_speed(self):
        gripper = _config()
        distance = CLOSED_RAD - OPEN_RAD
        duration = ex02.plan_duration(gripper, distance, None)
        travel_mm = distance * gripper.config.rad_to_mm
        assert travel_mm / duration == pytest.approx(ex02.RATED_SPEED_MM_S)

    def test_a_slower_duration_is_honoured(self):
        assert ex02.plan_duration(_config(), CLOSED_RAD - OPEN_RAD, 5.0) == 5.0

    def test_a_duration_faster_than_the_motor_is_slowed_down(self, capsys):
        """Refusing to go faster is the point: that is what faults the motor."""
        gripper = _config()
        distance = CLOSED_RAD - OPEN_RAD
        floor = ex02.plan_duration(gripper, distance, None)
        duration = ex02.plan_duration(gripper, distance, floor / 10.0)
        assert duration == pytest.approx(floor)
        assert "超过额定" in capsys.readouterr().out

    def test_a_short_move_is_not_stretched_by_a_long_duration(self):
        assert ex02.plan_duration(_config(), 0.0, 3.0) == 3.0


# ═══════════════════════════════════════════════════════════════════════════
# Fault detection
# ═══════════════════════════════════════════════════════════════════════════


def _stub_sdk(monkeypatch, describe=lambda code: f"故障 0x{code:X}"):
    """A stand-in for ``litegrip.constants`` so no SDK install is needed."""
    constants = types.ModuleType("litegrip.constants")
    constants.describe_error = describe
    package = types.ModuleType("litegrip")
    package.constants = constants
    monkeypatch.setitem(sys.modules, "litegrip", package)
    monkeypatch.setitem(sys.modules, "litegrip.constants", constants)


@pytest.mark.parametrize("error_code", [0, 1])
def test_a_healthy_state_has_no_fault(error_code, monkeypatch):
    _stub_sdk(monkeypatch)
    assert ex02.fault_of(SimpleNamespace(error_code=error_code,
                                         is_error=False)) is None


@pytest.mark.parametrize("error_code", [0x9, 0xA, 0xB, 0xC])
def test_a_latched_fault_is_described(error_code, monkeypatch):
    """UV / OC / OT all read fine and ignore commands — they must be named."""
    _stub_sdk(monkeypatch)
    fault = ex02.fault_of(SimpleNamespace(error_code=error_code, is_error=True))
    assert fault is not None
    assert f"0x{error_code:X}" in fault


# ═══════════════════════════════════════════════════════════════════════════
# Argument handling
# ═══════════════════════════════════════════════════════════════════════════


class TestArgs:
    def _parse(self, argv, monkeypatch):
        monkeypatch.setattr(sys, "argv", ["02_sim_to_real.py", *argv])
        return ex02.parse_args()

    def test_duration_defaults_to_auto(self, monkeypatch):
        assert self._parse([], monkeypatch).duration is None

    def test_duration_can_be_given(self, monkeypatch):
        assert self._parse(["--duration", "2.5"], monkeypatch).duration == 2.5

    def test_status_is_off_by_default(self, monkeypatch):
        args = self._parse([], monkeypatch)
        assert not args.status and not args.clear_fault

    def test_status_and_clear_fault_are_accepted_together(self, monkeypatch):
        args = self._parse(["--status", "--clear-fault"], monkeypatch)
        assert args.status and args.clear_fault


class TestStatusRefusesBadCombinations:
    """``--status`` is the do-not-move path; it must not be used by accident."""

    def _refusal(self, argv, monkeypatch) -> str:
        """Run ``main()`` and return the message it exits with.

        ``SystemExit("…")`` carries the text as its ``code``; the interpreter
        prints it and exits 1 (see the CLI-level test for the exit code).
        """
        monkeypatch.setattr(sys, "argv", ["02_sim_to_real.py", *argv])
        with pytest.raises(SystemExit) as excinfo:
            ex02.main()
        return str(excinfo.value.code)

    def test_clear_fault_alone_is_refused(self, monkeypatch):
        message = self._refusal(["--clear-fault"], monkeypatch)
        assert "--status" in message

    def test_status_with_dry_run_is_refused(self, monkeypatch):
        message = self._refusal(["--status", "--dry-run"], monkeypatch)
        assert "--dry-run" in message
