# -*- coding: utf-8 -*-
"""Example 05's MIT stream: the follow-the-slider rate limit, and fault handling.

This file exists because of a real failure: the first version of example 05
streamed a *constant* target rad for the whole move, so the very first frame
asked a ~10 Nm motor for ``kp × 1.845 rad ≈ 185 Nm``.  The gripper latched an
under-voltage/over-current fault (blinking red LED), kept reporting its
position, and ignored every command afterwards — "it reads fine but I can't
control it".

The rewrite that made the slider drive the gripper directly had to keep that
protection, and it moved the protection's home: it used to bound the *ramp*'s
slope over a fixed duration, and it now bounds what one frame may do to the
position target (:class:`ex02.SliderDrive`).  So the tests below pin the shape
of the stream, not its timing, by driving the drive with synthetic timestamps:
no sleeping, no CAN, no hardware.  The key assertion is unchanged — no single
frame may move the position target far enough for ``kp`` to demand more torque
than the motor can deliver, *however fast the user yanks the slider*.
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
    """Import ``05_dual_control.py`` as a module (its name is not an identifier)."""
    path = EXAMPLES / "05_dual_control.py"
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

    def __init__(self, ok: bool = True, answering: bool = True) -> None:
        self.frames: list[dict] = []
        self.ok = ok
        self.position_rad = OPEN_RAD
        #: False = this motor is not sending status frames, so nothing it says
        #: about where it is can be trusted (``LiteGrip.poll`` answers that
        #: question, and it is the only thing that does).
        self.answering = answering
        self.config = _config().config

    def poll(self, timeout_s: float = 0.0) -> bool:
        return self.answering

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


def full_speed(gripper=None) -> float:
    """The rate limit at 100 % — the fastest the fingers are allowed to go."""
    return ex02.plan_speed(gripper or _config(), 100.0)


def drive_at(start_rad: float, *, speed_rad_s: float | None = None,
             want_rad: float | None = None) -> ex02.SliderDrive:
    """A drive sitting at ``start_rad``, already yanked to ``want_rad``."""
    drive = ex02.SliderDrive(
        start_rad=start_rad,
        speed_rad_s=full_speed() if speed_rad_s is None else speed_rad_s,
        kp=KP, kd=KD)
    if want_rad is not None:
        drive.retarget(want_rad)
    return drive


def run_drive(drive, frames: int, tau_nm: float = 0.0) -> RecordingGripper:
    """Drive ``drive`` at :data:`ex02.FRAME_HZ` from t=0 and return the frames.

    No sleeping: the timestamps are synthetic.  ``send`` is called once per
    tick, which is what the main loop's ``due()`` gate amounts to.
    """
    gripper = RecordingGripper()
    for i in range(frames):
        drive.send(gripper, i * ex02.FRAME_DT, tau_nm)
    return gripper


def frames_to_arrive(drive) -> int:
    """How many ticks the drive needs to reach ``q_want`` from where it is."""
    gripper = RecordingGripper()
    for i in range(int(60 * ex02.FRAME_HZ)):        # 60 s is far more than enough
        drive.send(gripper, i * ex02.FRAME_DT)
        if drive.arrived():
            return i + 1
    raise AssertionError("这条驱动 60 s 都没走到目标")


def worst_torque_demand(frames, kp: float, start_rad: float) -> float:
    """The hardest single correction the stream asks the servo to make [Nm].

    The servo only sees where it *is* and where it is *told to go*: ``kp ×
    (q_target − q_actual)``, plus whatever feed-forward torque the frame
    carries.  Between two consecutive frames the motor has had at most one tick
    to follow, so the distance the target jumps since the last frame is the
    error the servo has to make up — and before the first frame the motor is
    sitting at ``start_rad``.

    A jump of the whole travel is what latched the fault: ``100 Nm/rad ×
    1.845 rad ≈ 185 Nm`` on a ~10 Nm motor.
    """
    worst = 0.0
    previous = start_rad
    for frame in frames:
        worst = max(worst, kp * abs(frame["q"] - previous) + abs(frame.get("tau", 0.0)))
        previous = frame["q"]
    return worst


# ═══════════════════════════════════════════════════════════════════════════
# The slider drive — the regression test for the fault this file is named after
# ═══════════════════════════════════════════════════════════════════════════


class TestSliderDrive:
    def test_no_frame_asks_the_motor_for_more_than_it_can_give(self):
        """The bug in one assertion — now with the slider yanked to the far end.

        Every frame must stay close enough to the previous one (and the first
        one close enough to where the motor actually is) that ``kp`` turns the
        jump into a torque the motor can deliver.  Dragging the slider across
        the whole stroke in one window tick is the worst case a user can
        produce, and it is exactly what this test performs.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400, tau_nm=1.5)
        assert len(gripper.frames) > 100, "expected a full 200 Hz stream"

        demanded = worst_torque_demand(gripper.frames, KP, OPEN_RAD)
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
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        first = run_drive(drive, 1).frames[0]["q"]
        assert abs(first - OPEN_RAD) <= drive.max_step_rad * 1.01, \
            "第一帧就离开了电机所在的位置，比一个限速步还远"
        assert first != pytest.approx(CLOSED_RAD), \
            "第一帧就停在目标上——这就是那条阶跃指令"

    def test_a_full_stroke_yank_takes_at_least_the_travel_over_one_step(self):
        """Dragging is not teleporting: the far end needs many frames.

        A step command reaches the target on frame #1.  A rate-limited one
        cannot arrive before it has walked the whole distance a step at a time,
        and that count is what makes the two distinguishable.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        travel = abs(CLOSED_RAD - OPEN_RAD)
        needed = frames_to_arrive(drive)
        assert needed >= travel / (full_speed() * ex02.FRAME_DT) - 1
        assert needed > 100, "全行程一拽到底，却是几步就走到了"

    def test_the_ramp_keeps_up_with_a_200_hz_stream(self):
        """Commanded positions advance by at most one tick per frame."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400)
        steps = [abs(b["q"] - a["q"])
                 for a, b in zip(gripper.frames, gripper.frames[1:])]
        assert max(steps) <= full_speed() * ex02.FRAME_DT * 1.01

    def test_the_ramp_is_monotonic_from_start_to_target(self):
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400)
        ramp = [f["q"] for f in gripper.frames if f["q"] != CLOSED_RAD]
        assert abs(ramp[0] - OPEN_RAD) <= drive.max_step_rad * 1.01, \
            "第一帧比一个限速步还远离起点"
        assert ramp == sorted(ramp)
        assert max(ramp) < CLOSED_RAD

    def test_the_ramp_carries_velocity_feed_forward_and_no_force(self):
        """Tau is held back for the hold phase — pushing mid-travel is wrong."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400, tau_nm=1.5)
        ramp = [f for f in gripper.frames if f["q"] < CLOSED_RAD - 1e-9]
        assert ramp, "no frame landed strictly inside the ramp"
        assert all(f["tau"] == 0.0 for f in ramp)
        assert all(f["dq"] > 0.0 for f in ramp)      # opening → closing

    def test_the_hold_phase_sits_still_and_pushes(self):
        """Once it arrives it holds the target and keeps pushing, forever.

        There is no hold *duration* any more: the frames that hold the position
        are the same frames that keep the motor's watchdog fed, so they run as
        long as the drive does.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400, tau_nm=1.5)
        hold = [f for f in gripper.frames if f["q"] == CLOSED_RAD]
        assert len(hold) > 100, "到位之后没有继续发帧"
        assert all(f["tau"] == 1.5 for f in hold), \
            "到位那一帧就该开始加力，而不是等到下一帧"
        # 到位的那一帧还在走完最后一步（dq≠0），从下一帧起才是静止保持。
        assert all(f["dq"] == 0.0 for f in hold[1:])

    def test_the_speed_comes_down_over_the_last_frames(self):
        """``dq`` has to decay into the target, not fall off a cliff.

        This is the fix for "it reaches the target and then bounces": ``dq`` in
        a MIT frame is a *target* velocity, so the frame that drops it to 0
        reverses the damping term into a torque step of ``kd × v``.  At full
        speed with this machine's ``kd = 2.0`` that is ``2.0 × 1.39 ≈ 2.8 Nm``,
        and the position loop needs ``2.8 / kp`` = 0.56 rad of error to absorb
        it — 40 % of the travel, which the mechanism supplies by moving
        backwards.  With the tail slowed down, the last real move is a small
        fraction of full speed instead.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400)
        moving = [f["dq"] for f in gripper.frames if f["q"] != CLOSED_RAD]
        assert len(moving) > 100

        # Not a constant-speed run into a wall: the tail has to be slower than
        # the body of the move, and slower each frame.
        tail = moving[-8:]
        assert all(b < a for a, b in zip(tail, tail[1:])), \
            f"收尾速度没有逐帧降下来：{[round(v, 4) for v in tail]}"
        assert tail[0] < 0.9 * max(moving), "全程匀速——收尾没有减速"
        # And the frame before the stop is a small fraction of full speed, so
        # the damping term it reverses is small too.
        assert max(abs(v) for v in moving[-2:]) <= 0.15 * full_speed(), \
            f"最后一帧仍在以 {max(abs(v) for v in moving[-2:]):.3f} rad/s 撞上去"
        assert KD * abs(moving[-1]) < 0.2, \
            f"停下那一刻 dq 反转出的力矩仍有 {KD * abs(moving[-1]):.3f} Nm"

    def test_the_slower_tail_does_not_make_the_move_drag(self):
        """It arrives, and the deceleration costs a bounded fraction of a second.

        ``RAMP_DOWN_S`` is the time the tail is allowed to take, so the whole
        move is the rate-limited walk plus at most that — the slider still feels
        immediate, and nothing asymptotes towards the target forever.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        travel = abs(CLOSED_RAD - OPEN_RAD)
        ideal = travel / (full_speed() * ex02.FRAME_DT)
        needed = frames_to_arrive(drive)
        assert needed <= ideal + ex02.RAMP_DOWN_S / ex02.FRAME_DT, \
            f"到位用了 {needed} 帧，限速走完只要 {ideal:.0f} 帧"

    def test_the_rate_limit_still_bounds_the_decelerating_frames(self):
        """The tail is slower than the limit, never faster than it."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 400)
        steps = [abs(b["q"] - a["q"])
                 for a, b in zip(gripper.frames, gripper.frames[1:])]
        assert max(steps) <= drive.max_step_rad * 1.01
        assert worst_torque_demand(gripper.frames, KP, OPEN_RAD) < MOTOR_RATED_NM

    def test_a_short_hop_decelerates_too(self):
        """A move shorter than the deceleration distance is all tail.

        Dragging one notch of the slider is this case, and it has to land on the
        target without ever exceeding the limit — the distance it covers is
        below ``2·a·dt²`` here, so the last frame is a clean landing.
        """
        want = OPEN_RAD + 0.02
        drive = drive_at(OPEN_RAD, want_rad=want)
        gripper = run_drive(drive, 200)
        moving = [f for f in gripper.frames if f["q"] != want]
        assert moving, "20 mrad 一帧就走完了——没有减速段"
        assert all(abs(f["dq"]) < full_speed() for f in gripper.frames)
        assert gripper.frames[-1]["q"] == want
        assert max(abs(f["dq"]) for f in moving) <= full_speed()

    def test_a_closing_move_runs_the_ramp_in_the_other_direction(self):
        drive = drive_at(CLOSED_RAD, want_rad=OPEN_RAD)
        gripper = run_drive(drive, 400)
        assert gripper.frames[0]["q"] == pytest.approx(
            CLOSED_RAD, abs=drive.max_step_rad * 1.01)
        ramp = [f for f in gripper.frames if f["q"] != OPEN_RAD]
        assert all(f["dq"] < 0.0 for f in ramp)

    def test_only_a_closing_target_may_carry_force(self):
        """Pushing while opening holds the motor back — that is not a grasp."""
        opening = drive_at(CLOSED_RAD, want_rad=OPEN_RAD)
        assert opening.closing is False
        closing = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        assert closing.closing is True

    def test_a_zero_length_move_does_not_divide_by_zero(self):
        drive = drive_at(OPEN_RAD, want_rad=OPEN_RAD)
        assert drive.max_step_rad > 0.0
        gripper = run_drive(drive, 10)
        assert [f["q"] for f in gripper.frames] == [OPEN_RAD] * 10
        assert [f["dq"] for f in gripper.frames] == [0.0] * 10

    def test_a_target_it_is_already_sitting_on_moves_nothing(self):
        """The behaviour the whole redesign is for.

        ``retarget`` to where the drive already is must not produce a single
        frame that differs from the last — this is what a slider that has not
        been touched looks like, and it has to be a no-op.
        """
        drive = drive_at(CLOSED_RAD)
        gripper = run_drive(drive, 200, tau_nm=1.5)
        assert [f["q"] for f in gripper.frames] == [CLOSED_RAD] * 200
        assert [f["dq"] for f in gripper.frames] == [0.0] * 200
        # Arriving is what gates the force, and a drive that never left is
        # already there — so a clamp asked for before the drag still applies.
        assert all(f["tau"] == 1.5 for f in gripper.frames)

    def test_a_second_target_mid_ramp_keeps_the_rate_limit(self):
        """The slider is live: a new target arrives before the old one does.

        What must not change is the per-frame budget — a retarget while moving
        is exactly where a naive implementation would jump.
        """
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = RecordingGripper()
        for i in range(120):
            if i == 40:
                drive.retarget(CLOSED_RAD - 0.2)
            if i == 80:
                drive.retarget(OPEN_RAD + 0.1)
            drive.send(gripper, i * ex02.FRAME_DT)
        worst = worst_torque_demand(gripper.frames, KP, OPEN_RAD)
        assert worst < MOTOR_RATED_NM
        assert len({round(f["q"], 9) for f in gripper.frames}) > 50, \
            "拖动过程中目标换了两次，位置目标却没怎么动"

    def test_the_speed_limit_can_be_changed_while_it_runs(self):
        """The speed slider is live, and it lowers the per-frame budget."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        gripper = RecordingGripper()
        for i in range(200):
            if i == 100:
                drive.speed_rad_s = ex02.plan_speed(_config(), 10.0)
            drive.send(gripper, i * ex02.FRAME_DT)
        slow = full_speed() * ex02.FRAME_DT
        steps = [abs(b["q"] - a["q"])
                 for a, b in zip(gripper.frames[100:], gripper.frames[101:])]
        assert steps and max(steps) <= slow * 0.1001, \
            "调慢之后每帧的增量没有跟着变小"

    def test_zero_speed_commands_no_motion_at_all(self):
        """0 % has to mean 0, not "as fast as it takes to get there"."""
        drive = drive_at(OPEN_RAD, speed_rad_s=0.0, want_rad=CLOSED_RAD)
        gripper = run_drive(drive, 200)
        assert [f["q"] for f in gripper.frames] == [OPEN_RAD] * 200
        assert not drive.arrived(), "速度 0 却当成已经到位了"

    def test_done_is_about_standing_still_not_about_the_target(self):
        """``done`` hands the drive back only after it has stopped moving."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        needed = frames_to_arrive(drive)
        now = needed * ex02.FRAME_DT
        assert not drive.done(now + 0.1, 0.5), "刚到位就交回保活了"
        assert drive.done(now + 0.6, 0.5)
        # A drive that is still travelling is never done, however long it has
        # been running: the clock starts at the last movement, not the start.
        moving = drive_at(OPEN_RAD, speed_rad_s=full_speed() / 100.0,
                          want_rad=CLOSED_RAD)
        run_drive(moving, 200)
        assert not moving.arrived()
        assert not moving.done(200 * ex02.FRAME_DT + 10.0, 0.5)

    def test_a_dropped_frame_is_reported_rather_than_counted(self):
        """``send_mit_frame`` returns False when the gripper is not enabled."""
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        assert drive.send(RecordingGripper(ok=False), 0.0) is False
        assert drive.frames == 0
        assert drive.dropped == 1
        # A frame that never reached the bus must not advance the commanded
        # position either: otherwise the next one would cover two steps and
        # walk around the rate limit it exists to enforce.
        assert drive.q == pytest.approx(OPEN_RAD)
        assert drive.send(RecordingGripper(ok=True), 0.0) is True
        assert drive.frames == 1
        assert drive.q > OPEN_RAD


# ═══════════════════════════════════════════════════════════════════════════
# The idle keep-alive
# ═══════════════════════════════════════════════════════════════════════════


class TestIdleKeeper:
    """A gripper that hears nothing for long enough latches a fault.

    Measured on the motor itself: an *enabled* motor goes quiet for ~0.9 s and
    reports 0xD — position still readable, every command ignored, LED blinking.
    (The ``TIMEOUT`` register, RID 9, says 8000 ms and has also said 0; neither
    matches the measurement, so it is not what the cadence is held against.)
    An interactive example that only transmits while a move is in flight
    therefore breaks the hardware by being *looked at*.  These tests pin down
    that the idle path keeps talking, and that what it says cannot move
    anything.
    """

    #: The measured latch time the keep-alive cadence has to beat, in seconds.
    WATCHDOG_S = 0.9

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
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        run_drive(drive, frames_to_arrive(drive))
        keeper.remember(drive)
        keeper.set_force(1.7)
        keeper.maybe_send(0.0)
        assert gripper.frames[0]["q"] == pytest.approx(CLOSED_RAD)
        assert gripper.frames[0]["tau"] == pytest.approx(1.7)

    def test_the_remembered_grip_force_follows_the_slider(self):
        """The force slider stays live after the drive hands over.

        The alternative — freezing the force at the value it had when the drag
        stopped — would make "clamp a part, then raise the force" impossible
        without dragging the position again.
        """
        gripper = RecordingGripper()
        keeper = ex02.IdleKeeper(gripper, hz=50.0)
        drive = drive_at(OPEN_RAD, want_rad=CLOSED_RAD)
        run_drive(drive, frames_to_arrive(drive))
        keeper.remember(drive)
        keeper.set_force(0.0)
        keeper.maybe_send(0.0)
        assert gripper.frames[-1]["tau"] == 0.0
        keeper.set_force(2.5)
        keeper.maybe_send(1.0)
        assert gripper.frames[-1]["tau"] == pytest.approx(2.5)
        assert gripper.frames[-1]["q"] == gripper.frames[-2]["q"], \
            "改夹持力把位置目标也改了"

    def test_an_opening_command_never_gets_a_grip_force(self):
        """Only closing directions may push; the keeper has to remember which."""
        gripper = RecordingGripper()
        keeper = ex02.IdleKeeper(gripper, hz=50.0)
        keeper.remember(drive_at(CLOSED_RAD, want_rad=OPEN_RAD))
        assert keeper.closing is False
        keeper.remember(drive_at(OPEN_RAD, want_rad=CLOSED_RAD))
        assert keeper.closing is True

    def test_dropped_keepalives_are_counted(self):
        keeper = self._keeper(RecordingGripper(ok=False))
        for i in range(5):
            keeper.maybe_send(i * 1 / 50.0)
        assert keeper.frames == 0
        assert keeper.dropped == 5


# ═══════════════════════════════════════════════════════════════════════════
# The rate limit
# ═══════════════════════════════════════════════════════════════════════════


class TestPlanSpeed:
    def test_a_hundred_percent_is_the_rated_finger_speed(self):
        gripper = _config()
        speed_rad_s = ex02.plan_speed(gripper, 100.0)
        assert speed_rad_s * gripper.config.rad_to_mm == pytest.approx(
            ex02.RATED_SPEED_MM_S)

    def test_a_lower_percentage_is_proportionally_slower(self):
        gripper = _config()
        full = ex02.plan_speed(gripper, 100.0)
        assert ex02.plan_speed(gripper, 40.0) == pytest.approx(full * 0.4)
        assert ex02.plan_speed(gripper, 1.0) == pytest.approx(full * 0.01)

    def test_zero_percent_means_zero(self):
        assert ex02.plan_speed(_config(), 0.0) == 0.0

    def test_a_percentage_over_a_hundred_is_clamped_and_says_so(self, capsys):
        """Refusing to go faster is the point: that is what faults the motor."""
        gripper = _config()
        full = ex02.plan_speed(gripper, 100.0)
        assert ex02.plan_speed(gripper, 250.0) == pytest.approx(full)
        assert "超过额定" in capsys.readouterr().out

    def test_the_clamped_speed_is_never_above_the_rating(self):
        """Whatever the percentage, the fingers never exceed the rating."""
        for percent in (0.0, 1.0, 50.0, 100.0, 100.1, 1000.0):
            speed_rad_s = ex02.plan_speed(_config(), percent)
            assert speed_rad_s * _config().config.rad_to_mm \
                <= ex02.RATED_SPEED_MM_S + 1e-9


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
        monkeypatch.setattr(sys, "argv", ["05_dual_control.py", *argv])
        return ex02.parse_args()

    def test_speed_defaults_to_the_full_rating(self, monkeypatch):
        assert self._parse([], monkeypatch).speed == 100.0

    def test_speed_can_be_given(self, monkeypatch):
        assert self._parse(["--speed", "25"], monkeypatch).speed == 25.0

    def test_status_is_off_by_default(self, monkeypatch):
        args = self._parse([], monkeypatch)
        assert not args.status and not args.clear_fault

    def test_status_and_clear_fault_are_accepted_together(self, monkeypatch):
        args = self._parse(["--status", "--clear-fault"], monkeypatch)
        assert args.status and args.clear_fault

    def test_the_drag_model_has_no_confirm_key(self, monkeypatch):
        """Dragging is the only input now — there is no key to press."""
        assert not hasattr(ex02, "CONFIRM_KEYS")


class TestStatusRefusesBadCombinations:
    """``--status`` is the do-not-move path; it must not be used by accident."""

    def _refusal(self, argv, monkeypatch) -> str:
        """Run ``main()`` and return the message it exits with.

        ``SystemExit("…")`` carries the text as its ``code``; the interpreter
        prints it and exits 1 (see the CLI-level test for the exit code).
        """
        monkeypatch.setattr(sys, "argv", ["05_dual_control.py", *argv])
        with pytest.raises(SystemExit) as excinfo:
            ex02.main()
        return str(excinfo.value.code)

    def test_clear_fault_alone_is_refused(self, monkeypatch):
        message = self._refusal(["--clear-fault"], monkeypatch)
        assert "--status" in message

    def test_status_with_dry_run_is_refused(self, monkeypatch):
        message = self._refusal(["--status", "--dry-run"], monkeypatch)
        assert "--dry-run" in message
