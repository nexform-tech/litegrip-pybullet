# -*- coding: utf-8 -*-
"""Example 05's *main loop*, driven end to end against a fake gripper.

``tests/test_example02_stream.py`` covers the follow-the-slider maths in
isolation.  This file covers the part that only ever ran on real hardware
before: ``main()`` itself — slider reading, the drag detection, the idle
keep-alive, fault detection, the exit path — with a stand-in for ``LiteGrip``
and a stand-in for the PyBullet window.

The interaction these tests pin down is the one the example was rewritten for:
the motor holds its position when the example starts, the window mirrors where
it actually is, and nothing moves until the slider is dragged.  A stationary
slider is never a command, whatever it reads.

Nothing here touches CAN or opens a window, so it is safe on a machine with a
gripper attached, and it runs in milliseconds.  The values are the real ones
from this machine's ``litegrip_calibration.json``, so a unit or sign error in
the calibration arithmetic shows up here instead of on the motor.
"""

import importlib.util
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("pybullet")

#: ``pressed()`` masks each event against pybullet's trigger bit, and on this
#: build that is 2 — *not* the 1 a "key is down" event carries.  A fake that
#: hands out a bare ``1`` therefore makes every key press a silent no-op, which
#: is exactly what these tests are here to notice.
from pybullet import KEY_WAS_TRIGGERED  # noqa: E402  (after importorskip)

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

os.environ.setdefault("LITEGRIP_PYBULLET_REEXEC", "1")
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))


def _load_example_02():
    path = EXAMPLES / "05_dual_control.py"
    spec = importlib.util.spec_from_file_location("example02_loop", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["example02_loop"] = module
    spec.loader.exec_module(module)
    return module


ex02 = _load_example_02()

#: This machine's real calibration (litegrip_calibration.json), so the
#: arithmetic below is exercised with the values the hardware actually uses.
POS_CLOSED_RAD = 1.775959
POS_OPEN_RAD = -0.064279
RAD_TO_MM = 65.21
MAX_STROKE_MM = 120.0
KP = 100.0
KD = 2.0

#: The motor is rated around 10 Nm; a sustained command past that stalls it.
MOTOR_RATED_NM = 10.0

#: How long an enabled motor can go silent before it latches the
#: communication-loss fault, as measured on this hardware (the SDK's own
#: number).  The motor's ``TIMEOUT`` register says 8000 ms — and has also said 0
#: — which does not match the measurement, so the cadence is held against the
#: measured value rather than the register.
WATCHDOG_S = 0.9


class FakeClock:
    """A monotonic clock the fake window advances — no sleeping in tests."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def monotonic(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


class FakeGripper:
    """Stands in for a connected, enabled ``LiteGrip``.

    Reads back whatever position the test sets and records every frame instead
    of putting it on the bus.
    """

    def __init__(self, position_rad: float = 0.0, error_code: int = 1,
                 answering: bool = True) -> None:
        self.position_rad = position_rad
        self.error_code = error_code
        #: False = the motor is not sending status frames (a wedged motor, a
        #: deaf master, a second program on the bus).  ``poll`` and
        #: ``refresh_status`` are the public ways to tell that apart from a live
        #: one, and neither can answer on a bus that carries nothing.
        self.answering = answering
        self.frames: list[dict] = []
        self.stopped = False
        self.disabled = False
        self.refreshes: list[float] = []
        self.disconnected = False
        self.config = SimpleNamespace(
            pos_closed_rad=POS_CLOSED_RAD,
            pos_open_rad=POS_OPEN_RAD,
            rad_to_mm=RAD_TO_MM,
            max_stroke_mm=MAX_STROKE_MM,
            travel_range=POS_CLOSED_RAD - POS_OPEN_RAD,
            kp=KP,
            kd=KD,
        )

    # ── the LiteGrip surface example 05 uses ────────────────────────────
    def poll(self, timeout_s: float = 0.0) -> bool:
        """``LiteGrip.poll``: True = a *new* status frame arrived just now."""
        return self.answering

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        """``LiteGrip.refresh_status``: sends 0xCC, then waits for the reply."""
        self.refreshes.append(timeout_s)
        return self.answering

    def get_state(self, wait: bool = True):
        return SimpleNamespace(
            position_rad=self.position_rad,
            position_mm=(POS_CLOSED_RAD - self.position_rad) * RAD_TO_MM,
            force_n=0.0,
            velocity_rad_s=0.0,
            # A frame just arrived (``answering``), so the snapshot is backed by
            # data and young — the two signals ``fresh_state`` cross-checks.
            data_age_s=0.0,
            has_data=True,
            is_stale=False,
            is_moving=False,
            error_code=self.error_code,
            is_error=self.error_code not in (0, 1),
            is_enabled=self.error_code == 1,
        )

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True

    def stop(self) -> None:
        """Zero-torque but still *enabled* — the exit path must not use this."""
        self.stopped = True

    def disable(self) -> None:
        self.disabled = True

    def disconnect(self) -> None:
        self.disconnected = True

    def read_param(self, rid, timeout_s: float = 0.5) -> float:
        return 8000.0


class FakeSim:
    """Stands in for ``GripperSim``: no window, no physics, a fixed step budget."""

    def __init__(self, clock: FakeClock, steps: int, quit_at: int | None = None) -> None:
        self.clock = clock
        self.steps_left = steps
        self.quit_at = quit_at
        self.tick = 0
        self.urdf_path = "fake.urdf"
        #: Every ``reset_fraction`` the loop asked for — the mirror.  There is no
        #: ``command_fraction`` on this fake on purpose: the loop must not be
        #: driving the simulated jaws, only placing them where the real ones are.
        self.mirrored: list[float] = []
        self.status: list[str] = []
        self.keys_emitted = 0
        self.disconnected = False
        self.focused = False

    def connected(self) -> bool:
        return self.steps_left > 0

    def focus_camera(self) -> None:
        self.focused = True

    def keyboard_events(self):
        # pybullet's shape: {key: bitmask}; ``pressed`` tests it with ``.get``.
        self.tick += 1
        if self.quit_at is not None and self.tick == self.quit_at:
            self.keys_emitted += 1
            return {key: KEY_WAS_TRIGGERED for key in ex02.QUIT_KEYS}
        return {}

    def reset_fraction(self, fraction: float) -> float:
        self.mirrored.append(fraction)
        return fraction

    def status_text(self, text: str) -> None:
        self.status.append(text)

    def step(self) -> bool:
        self.steps_left -= 1
        self.clock.advance(0.005)          # a 200 Hz main loop
        return self.steps_left > 0

    def disconnect(self) -> None:
        self.disconnected = True


class FakeSliders:
    """Stands in for the pybullet calls example 05 uses for its sliders.

    The opening slider starts wherever ``addUserDebugParameter`` was told to
    start it — the example computes that from the gripper's own position — and
    jumps to ``drag_to`` once the loop has run ``drag_tick`` times.  That is
    what a mouse drag looks like from the loop's side: the value simply
    changes.  Nothing else about it changes, which is how a slider the user has
    *not* touched stays a non-command.
    """

    def __init__(self, drag_to: float | None = None, drag_tick: int | None = None,
                 tick_fn=None) -> None:
        self.values: dict[int, float] = {}
        self.created: list[tuple] = []
        self.drag_to = drag_to
        self.drag_tick = drag_tick
        self.tick_fn = tick_fn

    def addUserDebugParameter(self, name, lo, hi, start):   # noqa: N802 (pybullet)
        index = len(self.created) + 1
        self.created.append((name, lo, hi, start))
        self.values[index] = start
        return index

    def readUserDebugParameter(self, index):                # noqa: N802 (pybullet)
        if index == 1 and self.drag_to is not None and self.tick_fn is not None \
                and self.tick_fn() >= self.drag_tick:
            self.values[1] = self.drag_to * 100.0
        return self.values[index]


#: A calibration file as ``read_calibration_file`` returns it, for ``--dry-run``
#: (which never connects, so there is no ``gripper.config`` to borrow).
CALIB_FILE = dict(zero_position_rad=POS_CLOSED_RAD, max_position_rad=POS_OPEN_RAD,
                  rad_to_mm=RAD_TO_MM, kp=KP, kd=KD)


def _run(monkeypatch, gripper=None, steps=40, drag_to=None, drag_tick=3,
         quit_at=None, clock=None, speed=100.0, dry_run=False):
    """Run ``example 05``'s ``main()`` against fakes; return the pieces."""
    clock = clock or FakeClock()
    gripper = gripper or FakeGripper(position_rad=POS_OPEN_RAD + 0.3)
    sim = FakeSim(clock, steps, quit_at=quit_at)
    sliders = FakeSliders(drag_to=drag_to, drag_tick=drag_tick,
                          tick_fn=lambda: sim.tick)

    args = SimpleNamespace(
        channel="can0", can_id=0x08, mst_id=0x18, calib=None,
        urdf=None, headless=False, dry_run=dry_run, status=False,
        clear_fault=False, speed=speed, force=10.0,
    )

    monkeypatch.setattr(ex02, "parse_args", lambda: args)
    monkeypatch.setattr(ex02, "open_real_gripper", lambda a, enable=True: gripper)
    monkeypatch.setattr(ex02, "GripperSim", lambda **kw: sim)
    monkeypatch.setattr(ex02, "p", sliders)
    monkeypatch.setattr(ex02, "time", SimpleNamespace(
        monotonic=clock.monotonic, sleep=lambda s: None))
    monkeypatch.setattr(ex02, "import_litegrip", lambda: SimpleNamespace(
        UnitConversion=SimpleNamespace(N_TO_NM=0.1)))
    if dry_run:
        monkeypatch.setattr(ex02, "choose_calibration_file",
                            lambda requested: Path("/fake/calibration.json"))
        monkeypatch.setattr(ex02, "read_calibration_file",
                            lambda path: dict(CALIB_FILE))

    code = ex02.main()
    return SimpleNamespace(code=code, sim=sim, gripper=gripper, clock=clock,
                           sliders=sliders)


def _worst_frame_demand(frames, kp: float, start_rad: float) -> float:
    """The hardest single correction the loop asks the servo to make [Nm]."""
    worst = 0.0
    previous = start_rad
    for frame in frames:
        worst = max(worst, kp * abs(frame["q"] - previous) + abs(frame["tau"]))
        previous = frame["q"]
    return worst


class TestIdleKeepAlive:
    """The regression this loop was rewritten for: never let the CAN
    watchdog expire, even when nothing is being commanded."""

    def test_it_streams_frames_while_idle(self, monkeypatch):
        run = _run(monkeypatch, steps=200)          # 1.0 s at 200 Hz
        assert run.gripper.frames, "空闲时一帧都没发——电机会锁通信超时"
        # Not exactly 200: the keeper only sends when a whole interval has
        # elapsed, so a loop running at the same rate as the keeper can land on
        # either side of the comparison.  Anything in the tens of Hz is far
        # more than the watchdog needs; what must not happen is silence.
        assert len(run.gripper.frames) >= 50

    def test_no_gap_between_frames_can_reach_the_watchdog(self, monkeypatch):
        run = _run(monkeypatch, steps=400)          # 2.0 s
        assert len(run.gripper.frames) >= 2
        # 200 Hz for 2 s: even if the loop stalls, the frame count shows the
        # mean gap.  A stall long enough to matter would show up as far fewer.
        mean_gap = 2.0 / len(run.gripper.frames)
        assert mean_gap < WATCHDOG_S / 10, f"平均间隔 {mean_gap:.3f} s 太接近超时"

    def test_the_idle_frame_holds_the_measured_position(self, monkeypatch):
        run = _run(monkeypatch, steps=100)
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(run.gripper.position_rad)
            assert frame["kp"] == KP
            assert frame["kd"] == KD
            assert frame["tau"] == 0.0, "空闲帧不该带前馈力"

    def test_it_holds_a_current_command_without_stepping(self, monkeypatch):
        """Every frame has to be within the motor's reach of the last one."""
        run = _run(monkeypatch, steps=400)
        demand = _worst_frame_demand(run.gripper.frames, KP,
                                     run.gripper.position_rad)
        assert demand <= MOTOR_RATED_NM, \
            f"一帧就要 {demand:.1f} Nm，超过电机额定的 {MOTOR_RATED_NM} Nm"


class TestItDoesNotMoveUntilTheSliderIsTouched:
    """The behaviour the example was rewritten for.

    Enabling the motor must not command anything: the example holds the
    position it measured, the window shows that measurement, and the gripper
    only moves once the slider is dragged.
    """

    def test_starting_up_commands_no_motion_at_all(self, monkeypatch):
        run = _run(monkeypatch, steps=400)          # 2 s with the slider untouched
        assert run.gripper.frames
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(run.gripper.position_rad), \
                "滑条没动，目标却换了地方"
            assert frame["dq"] == 0.0
            assert frame["tau"] == 0.0

    def test_the_window_shows_where_the_gripper_is(self, monkeypatch):
        """The mirror, not a preview of the target.

        The gripper is parked somewhere the slider does not point at, so a
        window showing the *commanded* value would look different from one
        showing the measurement.
        """
        parked = POS_OPEN_RAD + 0.3
        run = _run(monkeypatch, steps=100,
                   gripper=FakeGripper(position_rad=parked))
        expected = ex02.rad_to_fraction(run.gripper, parked)
        assert run.sim.mirrored, "窗口一帧都没被摆到真机的位置上"
        assert all(f == pytest.approx(expected) for f in run.sim.mirrored), \
            "窗口显示的不是实测位置"

    def test_the_window_keeps_showing_the_measurement_while_it_moves(
            self, monkeypatch):
        """Even mid-drag: the window is a mirror, never the setpoint."""
        run = _run(monkeypatch, steps=600, drag_to=0.9, drag_tick=3)
        measured = ex02.rad_to_fraction(run.gripper, run.gripper.position_rad)
        assert run.sim.mirrored
        assert all(f == pytest.approx(measured) for f in run.sim.mirrored)

    def test_the_slider_starting_value_is_not_a_command(self, monkeypatch):
        """A slider parked away from the measured position moves nothing.

        The start value comes off the measurement, but it does not have to:
        what makes something a command is that its value *changed*.
        """
        gripper = FakeGripper(position_rad=POS_OPEN_RAD + 0.3)
        run = _run(monkeypatch, gripper=gripper, steps=300)
        start = ex02.rad_to_fraction(gripper, gripper.position_rad) * 100.0
        assert run.sliders.created[0][3] == pytest.approx(start), \
            "开度滑条没有从真机当前位置起步"
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(gripper.position_rad)

    def test_the_gripper_only_starts_moving_after_the_drag(self, monkeypatch):
        """The frames up to the drag all hold; the first that does not, moves.

        Written against the frames themselves rather than against a slice of
        them: how many frames the keeper gets out per loop iteration depends on
        the clock's float arithmetic, and that is not what is being tested.
        """
        run = _run(monkeypatch, steps=600, drag_to=0.9, drag_tick=50)
        parked = run.gripper.position_rad
        moving = [i for i, f in enumerate(run.gripper.frames)
                  if f["q"] != pytest.approx(parked)]
        assert run.gripper.frames, "一帧都没发"
        assert moving, "拖了之后也没动"
        assert moving[0] > 10, \
            f"拖动之前第 {moving[0]} 帧就开始动了"
        assert all(f["q"] == pytest.approx(parked)
                   for f in run.gripper.frames[:moving[0]])


class TestDragDispatch:
    """No key press: the drag is the command."""

    def test_dragging_sends_a_limited_stream_then_keeps_the_link_alive(
            self, monkeypatch):
        run = _run(monkeypatch, steps=600, drag_to=0.9, drag_tick=3)
        assert run.gripper.frames
        assert run.sim.keys_emitted == 0, "这一档按过键？拖动本该是唯一输入"
        demand = _worst_frame_demand(run.gripper.frames, KP,
                                     run.gripper.position_rad)
        assert demand <= MOTOR_RATED_NM, \
            f"整个过程里最狠的一帧要 {demand:.1f} Nm，超过额定 {MOTOR_RATED_NM} Nm"

    def test_it_never_jumps_to_the_target_in_one_frame(self, monkeypatch):
        run = _run(monkeypatch, steps=600, drag_to=0.1, drag_tick=3)
        if not run.gripper.frames:
            pytest.skip("这一档没有下发")
        first = run.gripper.frames[0]["q"]
        assert first == pytest.approx(run.gripper.position_rad, abs=0.05), \
            "第一帧就该从电机当前位置起步"

    def test_the_target_is_inside_the_calibrated_travel(self, monkeypatch):
        run = _run(monkeypatch, steps=600, drag_to=1.0, drag_tick=3)
        for frame in run.gripper.frames:
            assert POS_OPEN_RAD - 1e-9 <= frame["q"] <= POS_CLOSED_RAD + 1e-9

    def test_a_slow_speed_cap_reaches_the_motor(self, monkeypatch):
        """--speed lowers the per-frame budget, and the frames show it."""
        fast = _run(monkeypatch, steps=600, drag_to=0.9, drag_tick=3)
        slow = _run(monkeypatch, steps=600, drag_to=0.9, drag_tick=3, speed=10.0)
        assert fast.gripper.frames and slow.gripper.frames

        def biggest_step(frames):
            return max(abs(b["q"] - a["q"])
                       for a, b in zip(frames, frames[1:]))

        assert biggest_step(slow.gripper.frames) \
            < biggest_step(fast.gripper.frames) / 5, \
            "速度 % 调小了，每帧的增量却没跟着小下去"

    def test_a_drag_force_is_only_pushed_in_the_closing_direction(self, monkeypatch):
        """Opening with a feed-forward torque would fight the motor."""
        gripper = FakeGripper(position_rad=POS_CLOSED_RAD - 0.2)
        run = _run(monkeypatch, gripper=gripper, steps=600, drag_to=1.0,
                   drag_tick=3)                       # drag towards open
        moved = [f for f in run.gripper.frames
                 if f["q"] != pytest.approx(gripper.position_rad)]
        assert moved, "没动，这条用例没测到东西"
        assert all(f["tau"] == 0.0 for f in moved), "张开方向也加了夹持力"


class PollSchedule(FakeGripper):
    """A gripper whose status frames follow a schedule.

    ``answer`` gets the attempt count and decides whether that frame arrived, so
    a test can say "answers for a while, then goes quiet" — the dangerous case,
    because the cached position is then a *plausible old* value rather than the
    SDK's 0.0, and nothing looks wrong until the value is used.

    Both ways of asking share the count, because they are the same question: a
    bus that has gone quiet answers neither a poll nor a 0xCC request.
    """

    def __init__(self, answer, **kwargs) -> None:
        super().__init__(**kwargs)
        self.answer = answer
        self.polls = 0

    def _answers(self) -> bool:
        self.polls += 1
        return bool(self.answer(self.polls))

    def poll(self, timeout_s: float = 0.0) -> bool:
        return self._answers()

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        self.refreshes.append(timeout_s)
        return self._answers()


class TestItWillNotActOnAnUnmeasuredPosition:
    """Every command has to be built from a position that came off the bus.

    ``MotorState._position`` starts at ``0.0`` (``litegrip/can/motor.py``) and is
    only moved by a real status frame, while ``GripperState.timestamp`` is the
    *local* clock — so the public snapshot cannot say how old it is, or whether it
    was ever filled in.  Build a drag's start from one and the rate limit is
    measuring from the wrong place, which is a step aimed at 0 rad (5.5 % open on
    this calibration) — ``kp × 1.7 rad ≈ 170 Nm`` against a ~10 Nm servo.  That is
    the shape of "一开夹爪就起飞", so the reading is checked before it is trusted.
    """

    def test_no_frame_at_all_when_the_motor_never_answers(self, monkeypatch, capsys):
        run = _run(monkeypatch, steps=300, drag_to=0.9, drag_tick=5,
                   gripper=FakeGripper(POS_OPEN_RAD + 0.3, answering=False))
        assert run.gripper.frames == [], (
            f"读不到状态帧还是发了 {len(run.gripper.frames)} 帧"
            f"（第一帧 q={run.gripper.frames[0]['q']:.4f}）")
        assert "不下发" in capsys.readouterr().out

    def test_the_unread_zero_is_never_commanded(self, monkeypatch):
        """``0.0`` means "never read", not "the jaws are at 0 rad"."""
        run = _run(monkeypatch, steps=300, drag_to=0.9, drag_tick=5,
                   gripper=FakeGripper(POS_OPEN_RAD + 0.3, answering=False))
        zeros = [f for f in run.gripper.frames if abs(f["q"]) < 1e-6]
        assert not zeros, f"把「从没读到过」的 0.0 当目标发了 {len(zeros)} 帧"

    def test_it_holds_once_the_motor_starts_answering(self, monkeypatch):
        gripper = PollSchedule(lambda n: n > 3, position_rad=POS_OPEN_RAD + 0.3)
        run = _run(monkeypatch, gripper=gripper, steps=300)
        assert run.gripper.frames, "恢复应答之后还是不发帧"
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(gripper.position_rad)

    def test_a_read_that_goes_quiet_refuses_the_dispatch(self, monkeypatch, capsys):
        """The keeper's already-latched frame is still safe to repeat — a *new*
        command is not, and must not be invented from the frozen cache."""
        gripper = PollSchedule(lambda n: n <= 2, position_rad=POS_OPEN_RAD + 0.3)
        run = _run(monkeypatch, gripper=gripper, steps=300, drag_to=0.9,
                   drag_tick=3)
        assert "不下发" in capsys.readouterr().out
        assert run.gripper.frames, "保活帧是锁位帧，读数冻结了也该照常重发"
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(gripper.position_rad), \
                "读数冻结后又造了新的目标"


class TestFaultHandling:
    def test_a_latched_fault_stops_the_frames_and_fails_the_run(self, monkeypatch):
        gripper = FakeGripper(position_rad=POS_OPEN_RAD + 0.3)

        def state_with_fault(wait: bool = True):
            state = FakeGripper.get_state(gripper, wait)
            state.error_code = 0xA
            state.is_error = True
            return state

        monkeypatch.setattr(gripper, "get_state", state_with_fault)
        run = _run(monkeypatch, gripper=gripper, steps=200)
        assert run.code == 1, "报故障却没有失败退出"
        assert run.gripper.disconnected

    def test_a_drag_against_a_latched_fault_is_refused(self, monkeypatch, capsys):
        """Nothing may be built on a position from a motor that ignores us."""
        gripper = FakeGripper(position_rad=POS_OPEN_RAD + 0.3, error_code=0xD)
        run = _run(monkeypatch, gripper=gripper, steps=300, drag_to=0.9,
                   drag_tick=5)
        out = capsys.readouterr().out
        assert "故障" in out
        assert run.code == 1

    def test_it_disconnects_even_when_nothing_was_sent(self, monkeypatch):
        run = _run(monkeypatch, steps=20)
        assert run.gripper.disconnected
        assert run.sim.disconnected

    def test_the_exit_disables_rather_than_leaving_the_motor_enabled(self, monkeypatch):
        """Stopping the frames is not enough.

        ``stop()`` sends one kp=0 frame and leaves the motor *enabled*; an
        enabled motor that hears nothing latches 0xD within about a second, and
        a process that has exited cannot clear it — the next run then starts
        looking at a wedged gripper.  ``disable()`` needs no frames at all.
        """
        run = _run(monkeypatch, steps=20)
        assert run.gripper.disabled, "退出时没有失能——电机会在无人喂帧时锁故障"
        assert not run.gripper.stopped, \
            "用了 stop()：它只发一帧零力矩，电机仍是使能态"


class TestDryRun:
    """``--dry-run`` runs the same loop with no SDK, no CAN and no window motor.

    It has no measurement to mirror, so the window shows the commanded value —
    and it must not pretend otherwise, nor send a single frame.
    """

    def test_it_sends_no_frame_and_needs_no_gripper(self, monkeypatch):
        run = _run(monkeypatch, steps=300, drag_to=0.9, drag_tick=3, dry_run=True)
        assert run.gripper.frames == [], "dry-run 下发了帧"
        assert run.gripper.refreshes == [], "dry-run 碰了真机"

    def test_it_still_follows_the_slider(self, monkeypatch):
        """With no hardware, the run still walks the commanded position.

        There is nothing to read here, so the drive starts from the slider's own
        starting value — which is what ``make_sliders`` falls back to without a
        measurement — and ends up at what was dragged to.
        """
        run = _run(monkeypatch, steps=400, drag_to=0.9, drag_tick=3, dry_run=True)
        assert run.sim.mirrored[0] == pytest.approx(1.0), "起点不是滑条的起始开度"
        assert run.sim.mirrored[-1] == pytest.approx(0.9, abs=0.02), \
            "拖到 90% 之后命令值没有走到 90%"
        assert all(0.0 <= f <= 1.0 for f in run.sim.mirrored), "开度跑出了 [0,1]"
        assert len(run.sim.mirrored) > 100, "窗口没跟着跑"


class TestStatusReportsMeasuredValues:
    """``--status`` is the tool for "it reads but I can't control it", so the
    numbers it prints have to be measurements.

    A disabled motor sends no status frames, so a plain poll returns the cache —
    and ``MotorState._position`` starts at ``0.0`` and stays there until a frame
    arrives.  On this calibration that printed ``5.5 % / 6.63 mm`` for a gripper
    that was really at ``−0.370222 rad`` (≈ 27.5 %): a fabricated readout, and a
    very convincing one.  ``--status`` now asks for a frame (0xCC, which the SDK
    documents as "does not change motor output") and, failing that, says it has
    no reading instead of inventing one.
    """

    def _status(self, monkeypatch, capsys, gripper, timeout_ms: float = 0.0):
        args = SimpleNamespace(
            channel="can0", can_id=0x08, mst_id=0x18, calib=None, urdf=None,
            headless=False, dry_run=False, status=True, clear_fault=False,
            speed=100.0, force=10.0,
        )
        monkeypatch.setattr(ex02, "parse_args", lambda: args)
        monkeypatch.setattr(ex02, "open_real_gripper",
                            lambda a, enable=False: gripper)
        # The register table lives in the SDK; what is being tested is how the
        # value is reported, so feed it in rather than require an install.
        monkeypatch.setattr(ex02, "read_registers",
                            lambda g: {"TIMEOUT": timeout_ms})
        code = ex02.main()
        return code, capsys.readouterr().out

    def test_it_asks_for_a_frame_before_printing_a_position(self, monkeypatch, capsys):
        gripper = FakeGripper(position_rad=POS_OPEN_RAD + 0.3)
        code, out = self._status(monkeypatch, capsys, gripper)
        assert gripper.refreshes == [ex02.STATUS_WAIT_S], \
            "没有先请它回一帧就读了缓存（或者没用 --status 那个更宽的等待预算）"
        expected = ex02.rad_to_fraction(gripper, gripper.position_rad) * 100
        assert f"{expected:5.1f}%" in out
        assert "6.63 mm" not in out, "又把「从没读到过」的 0.0 当成位置打印了"
        # The healthy line, spelled out here so the assertion below cannot pass by
        # the string having simply disappeared from the source.
        assert "没有故障" in out
        assert code == 0

    def test_no_frame_means_no_readout_instead_of_a_fake_one(self, monkeypatch, capsys):
        code, out = self._status(monkeypatch, capsys,
                                 FakeGripper(answering=False))
        assert code == 1, "读不到状态帧却报了「健康」"
        assert "读不到状态帧" in out
        # No status line at all: no bar, no aperture, no position — the readout
        # is the thing that must not appear half-invented.  (The text does
        # mention 「5.5%」, but only to explain what the old fabricated print was.)
        assert "开口" not in out and "6.63 mm" not in out, \
            "读不到实测值时还是把 SDK 的初值当读数打印了"
        assert "没有故障" not in out

    def test_the_watchdog_registers_read_value_is_what_gets_reported(self, monkeypatch, capsys):
        """The register is read live, so both of its values come out as read.

        It used to be reported as *the* timeout: "hold this many ms and the
        motor latches".  That was wrong — the register has read 8000 and 0, and
        neither matches the ~0.9 s measured on this hardware — so the reading is
        printed together with the measurement it contradicts.
        """
        _, off = self._status(monkeypatch, capsys, FakeGripper(), timeout_ms=0.0)
        assert "通信超时保护（TIMEOUT, RID 9）= 0" in off
        _, armed = self._status(monkeypatch, capsys, FakeGripper(),
                                timeout_ms=8000.0)
        assert "通信超时保护（TIMEOUT, RID 9）= 8000" in armed

    def test_neither_register_value_is_passed_off_as_the_trip_time(self, monkeypatch, capsys):
        """Whatever the register says, the reported trip time is the measured
        one — a stale register must not become the operative number again."""
        for timeout_ms in (0.0, 8000.0):
            _, out = self._status(monkeypatch, capsys, FakeGripper(),
                                  timeout_ms=timeout_ms)
            assert "8000 ms 就锁" not in out
            assert f"静默约 {ex02.MEASURED_COMM_LOSS_S:g} s 就锁" in out, \
                "没把实测的闩锁时间说出来，读者只能拿寄存器当依据"


class TestDescribingAFault:
    """The fault path must not be able to fail.

    A wedged motor is diagnosed by what this prints, so a crash in here costs the
    operator the one piece of information that explains the fault. CI has no SDK
    and ``--dry-run`` is documented to work without one, so the SDK import has to
    be optional — and the code it *doesn't* know has to be spelled out anyway.
    """

    def test_the_communication_watchdog_code_is_named(self):
        """0xD is what this hardware latches; the SDK now names it itself."""
        text = ex02.describe_code(0xD)
        assert "0xD" not in text and "未知错误" not in text, \
            f"0xD 又被打回「未知错误」了：{text}"
        assert "超时" in text or "丢失" in text

    def test_a_stale_sdk_falls_back_to_our_own_table(self, monkeypatch):
        """An SDK old enough to call 0xD unknown must not win over the table.

        ``describe_error``'s only fallback wording is 未知错误, so that string is
        the signal that *this* SDK's table does not have the code — the case the
        local table exists for.
        """
        stub = SimpleNamespace(describe_error=lambda code: f"未知错误 (0x{code:X})")
        monkeypatch.setitem(sys.modules, "litegrip", SimpleNamespace())
        monkeypatch.setitem(sys.modules, "litegrip.constants", stub)
        text = ex02.describe_code(0xD)
        assert "未知错误" not in text, f"旧 SDK 一句话就把 0xD 顶掉了：{text}"
        assert "超时" in text or "丢失" in text

    def test_the_sdk_still_gets_the_codes_it_knows(self):
        """Duplicating the table must not shadow the SDK's own wording."""
        assert "过流" in ex02.describe_code(0xA)

    def test_it_works_without_the_sdk_installed(self, monkeypatch):
        """Exactly CI's situation: ``from litegrip... import`` raises."""
        monkeypatch.setitem(sys.modules, "litegrip", None)
        monkeypatch.setitem(sys.modules, "litegrip.constants", None)
        for code in (0x0, 0x1, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE):
            text = ex02.describe_code(code)
            assert "未知错误" not in text, \
                f"没装 SDK 就翻译不出 0x{code:X} 了：{text!r}"
        # A code nobody knows still has to come back as text, not as a raise.
        assert "0x7F" in ex02.describe_code(0x7F)


class TestItNeverLeavesTheMotorUnfed:
    """``enable()`` holds the motor, but only for the 50 ms stream it sends.

    Everything the example does between that and its first loop frame is dead
    time, and enough of it latches a communication-loss fault on an enabled,
    unattended motor — measured at about 0.9 s of silence, not the ``TIMEOUT``
    register's 8000 ms, which reads 0 as often as not."""

    def test_a_frame_goes_out_before_the_window_is_even_used(self, monkeypatch):
        run = _run(monkeypatch, steps=0)      # the window dies immediately
        assert len(run.gripper.frames) >= 1, \
            "使能之后、主循环之前一帧都没发——这段空窗就是通信超时故障的来源"

    def test_a_setup_failure_still_puts_the_motor_back(self, monkeypatch):
        """If the window cannot be built, the motor must not be left enabled."""
        def explode(**kwargs):
            raise RuntimeError("PyBullet 起不来")

        monkeypatch.setattr(ex02, "GripperSim", explode)
        clock = FakeClock()
        gripper = FakeGripper()
        monkeypatch.setattr(ex02, "parse_args", lambda: SimpleNamespace(
            channel="can0", can_id=0x08, mst_id=0x18, calib=None, urdf=None,
            headless=False, dry_run=False, status=False, clear_fault=False,
            speed=100.0, force=10.0))
        monkeypatch.setattr(ex02, "open_real_gripper", lambda a, enable=True: gripper)
        monkeypatch.setattr(ex02, "p", FakeSliders())
        monkeypatch.setattr(ex02, "time", SimpleNamespace(
            monotonic=clock.monotonic, sleep=lambda s: None))
        monkeypatch.setattr(ex02, "import_litegrip", lambda: SimpleNamespace(
            UnitConversion=SimpleNamespace(N_TO_NM=0.1)))

        with pytest.raises(RuntimeError):
            ex02.main()
        assert gripper.disabled, "建窗口失败后没让电机失能"
        assert not gripper.stopped, "只停发帧：电机还是「使能 + 没人喂帧」"
        assert gripper.disconnected, "建窗口失败后没有断开真机——电机会被晾着"


class TestCalibrationArithmetic:
    """The slider → rad path, with the hardware's real numbers."""

    def test_closed_and_open_map_to_the_travel_ends(self):
        """Both ends land on the calibrated angles, to float round-off.

        They used to be mapped through millimetres, which put the open end a
        hair short (and, on a file whose ``rad_to_mm`` belongs to a different
        ``max_stroke_mm`` than the SDK's default, stopped the slider reaching
        the open end at all).  Normalising over the travel makes the ends the
        ends; the 1e-12 tolerance is the subtraction's own round-off, not a
        physical shortfall.
        """
        gripper = FakeGripper()
        assert ex02.fraction_to_target_rad(gripper, 0.0) == POS_CLOSED_RAD
        assert ex02.fraction_to_target_rad(
            gripper, 1.0) == pytest.approx(POS_OPEN_RAD, abs=1e-12)

    def test_the_mapping_round_trips(self):
        gripper = FakeGripper()
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            rad = ex02.fraction_to_target_rad(gripper, fraction)
            assert ex02.rad_to_fraction(gripper, rad) == pytest.approx(fraction)
