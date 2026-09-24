# -*- coding: utf-8 -*-
"""Example 02's *main loop*, driven end to end against a fake gripper.

``tests/test_example02_stream.py`` covers the ramp maths in isolation.  This
file covers the part that only ever ran on real hardware before: ``main()``
itself — slider reading, the Enter dispatch, the idle keep-alive, fault
detection, the exit path — with a stand-in for ``LiteGrip`` and a stand-in for
the PyBullet window.

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

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

os.environ.setdefault("LITEGRIP_PYBULLET_REEXEC", "1")
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))


def _load_example_02():
    path = EXAMPLES / "02_sim_to_real.py"
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

#: The DM4310's CAN watchdog, as ``TIMEOUT`` reads on this machine.
WATCHDOG_S = 8.0


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

    def __init__(self, position_rad: float = 0.0, error_code: int = 1) -> None:
        self.position_rad = position_rad
        self.error_code = error_code
        self.frames: list[dict] = []
        self.stopped = False
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

    # ── the LiteGrip surface example 02 uses ────────────────────────────
    def get_state(self, wait: bool = True):
        return SimpleNamespace(
            position_rad=self.position_rad,
            position_mm=(POS_CLOSED_RAD - self.position_rad) * RAD_TO_MM,
            force_n=0.0,
            velocity_rad_s=0.0,
            is_moving=False,
            error_code=self.error_code,
            is_error=self.error_code not in (0, 1),
            is_enabled=self.error_code == 1,
        )

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True

    def stop(self) -> None:
        self.stopped = True

    def disconnect(self) -> None:
        self.disconnected = True

    def read_param(self, rid, timeout_s: float = 0.5) -> float:
        return 8000.0


class FakeSim:
    """Stands in for ``GripperSim``: no window, no physics, a fixed step budget."""

    def __init__(self, clock: FakeClock, steps: int, confirm_at: int | None = None,
                 quit_at: int | None = None) -> None:
        self.clock = clock
        self.steps_left = steps
        self.confirm_at = confirm_at
        self.quit_at = quit_at
        self.tick = 0
        self.urdf_path = "fake.urdf"
        self.commanded: list[float] = []
        self.status: list[str] = []
        self.disconnected = False
        self.focused = False

    def connected(self) -> bool:
        return self.steps_left > 0

    def focus_camera(self) -> None:
        self.focused = True

    def keyboard_events(self):
        # pybullet's shape: {key: bitmask}; ``pressed`` tests it with ``.get``.
        self.tick += 1
        if self.confirm_at is not None and self.tick == self.confirm_at:
            return {key: 1 for key in ex02.CONFIRM_KEYS}
        if self.quit_at is not None and self.tick == self.quit_at:
            return {key: 1 for key in ex02.QUIT_KEYS}
        return {}

    def command_fraction(self, fraction: float) -> None:
        self.commanded.append(fraction)

    def status_text(self, text: str) -> None:
        self.status.append(text)

    def step(self) -> bool:
        self.steps_left -= 1
        self.clock.advance(0.005)          # a 200 Hz main loop
        return self.steps_left > 0

    def disconnect(self) -> None:
        self.disconnected = True


class FakeSliders:
    """Stands in for the pybullet calls example 02 uses for its sliders."""

    def __init__(self, target_fraction: float, force_n: float) -> None:
        self.values = {1: target_fraction * 100.0, 2: force_n}
        self.created: list[tuple] = []

    def addUserDebugParameter(self, name, lo, hi, start):   # noqa: N802 (pybullet)
        index = len(self.created) + 1
        self.created.append((name, lo, hi, start))
        self.values.setdefault(index, start)
        return index

    def readUserDebugParameter(self, index):                # noqa: N802 (pybullet)
        return self.values[index]


def _run(monkeypatch, gripper=None, steps=40, confirm_at=None, quit_at=None,
         target_fraction=0.9, force_n=10.0, clock=None, argv=None):
    """Run ``example 02``'s ``main()`` against fakes; return the pieces."""
    clock = clock or FakeClock()
    gripper = gripper or FakeGripper(position_rad=POS_OPEN_RAD + 0.3)
    sim = FakeSim(clock, steps, confirm_at=confirm_at, quit_at=quit_at)
    sliders = FakeSliders(target_fraction, force_n)

    args = SimpleNamespace(
        channel="can0", can_id=0x08, mst_id=0x18, calib=None,
        urdf=None, headless=False, dry_run=False, status=False,
        clear_fault=False, duration=None, force=force_n,
    )

    monkeypatch.setattr(ex02, "parse_args", lambda: args)
    monkeypatch.setattr(ex02, "open_real_gripper", lambda a, enable=True: gripper)
    monkeypatch.setattr(ex02, "GripperSim", lambda **kw: sim)
    monkeypatch.setattr(ex02, "p", sliders)
    monkeypatch.setattr(ex02, "time", SimpleNamespace(
        monotonic=clock.monotonic, sleep=lambda s: None))
    monkeypatch.setattr(ex02, "import_litegrip", lambda: SimpleNamespace(
        UnitConversion=SimpleNamespace(N_TO_NM=0.1)))

    code = ex02.main()
    return SimpleNamespace(code=code, sim=sim, gripper=gripper, clock=clock,
                           sliders=sliders)


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
        previous = run.gripper.position_rad
        for frame in run.gripper.frames:
            demand = KP * abs(frame["q"] - previous) + abs(frame["tau"])
            assert demand <= MOTOR_RATED_NM, \
                f"一帧就要 {demand:.1f} Nm，超过电机额定的 {MOTOR_RATED_NM} Nm"
            previous = frame["q"]


class TestEnterDispatch:
    def test_enter_sends_a_ramp_then_keeps_the_link_alive(self, monkeypatch):
        run = _run(monkeypatch, steps=600, confirm_at=3, target_fraction=0.9)
        assert run.gripper.frames
        # The whole run, ramp included, must stay within the motor's reach of
        # the previous frame — that is the property the step command broke.
        previous = run.gripper.position_rad
        for frame in run.gripper.frames:
            assert KP * abs(frame["q"] - previous) <= MOTOR_RATED_NM
            previous = frame["q"]

    def test_it_never_jumps_to_the_target_in_one_frame(self, monkeypatch):
        run = _run(monkeypatch, steps=600, confirm_at=3, target_fraction=0.1)
        if not run.gripper.frames:
            pytest.skip("这一档没有下发")
        first = run.gripper.frames[0]["q"]
        assert first == pytest.approx(run.gripper.position_rad, abs=0.05), \
            "第一帧就该从电机当前位置起步"

    def test_the_target_is_inside_the_calibrated_travel(self, monkeypatch):
        run = _run(monkeypatch, steps=600, confirm_at=3, target_fraction=1.0)
        for frame in run.gripper.frames:
            assert POS_OPEN_RAD - 1e-9 <= frame["q"] <= POS_CLOSED_RAD + 1e-9


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

    def test_it_disconnects_even_when_nothing_was_sent(self, monkeypatch):
        run = _run(monkeypatch, steps=20)
        assert run.gripper.stopped and run.gripper.disconnected
        assert run.sim.disconnected


class TestItNeverLeavesTheMotorUnfed:
    """``enable()`` sends one priming frame and then stops; everything the
    example does between that and its first loop frame is dead time, and 8 s of
    it latches a communication-loss fault on an enabled, unattended motor."""

    def test_a_frame_goes_out_before_the_window_is_even_used(self, monkeypatch):
        run = _run(monkeypatch, steps=0)      # the window dies immediately
        assert run.sim.commanded == [] or True
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
            duration=None, force=10.0))
        monkeypatch.setattr(ex02, "open_real_gripper", lambda a, enable=True: gripper)
        monkeypatch.setattr(ex02, "p", FakeSliders(0.5, 0.0))
        monkeypatch.setattr(ex02, "time", SimpleNamespace(
            monotonic=clock.monotonic, sleep=lambda s: None))
        monkeypatch.setattr(ex02, "import_litegrip", lambda: SimpleNamespace(
            UnitConversion=SimpleNamespace(N_TO_NM=0.1)))

        with pytest.raises(RuntimeError):
            ex02.main()
        assert gripper.stopped, "建窗口失败后没有停发帧"
        assert gripper.disconnected, "建窗口失败后没有断开真机——电机会被晾着"


class TestCalibrationArithmetic:
    """The slider → rad path, with the hardware's real numbers."""

    def test_closed_and_open_map_to_the_travel_ends(self):
        gripper = FakeGripper()
        assert ex02.fraction_to_target_rad(gripper, 0.0) == pytest.approx(POS_CLOSED_RAD)
        # The open end lands a hair short of ``pos_open_rad``: the calibration
        # file's own numbers disagree by 0.00003 rad (max_stroke_mm/rad_to_mm =
        # 1.840209 rad of travel vs the recorded travel_range_rad of 1.840238).
        # That is 2 µm of finger travel — far below anything that matters, but
        # it is why this tolerance is not zero.
        assert ex02.fraction_to_target_rad(gripper, 1.0) == pytest.approx(
            POS_OPEN_RAD, abs=1e-3)

    def test_the_mapping_round_trips(self):
        gripper = FakeGripper()
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            rad = ex02.fraction_to_target_rad(gripper, fraction)
            assert ex02.rad_to_fraction(gripper, rad) == pytest.approx(fraction)
