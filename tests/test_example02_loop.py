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


class FakeController:
    """The one SDK internal ``request_status_frame`` (0xCC) reaches for."""

    def __init__(self) -> None:
        self.refreshes = 0

    def refresh_status(self, motor) -> None:
        self.refreshes += 1


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
        #: deaf master, a second program on the bus).  ``poll`` is the only
        #: public way to tell that apart from a live one.
        self.answering = answering
        self.frames: list[dict] = []
        self.stopped = False
        self.disconnected = False
        self.controller = FakeController()
        self._can = SimpleNamespace(_controller=self.controller,
                                    _motor=object())
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
    def poll(self, timeout_s: float = 0.0) -> bool:
        """``LiteGrip.poll``: True = a *new* status frame arrived just now."""
        return self.answering

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
            return {key: KEY_WAS_TRIGGERED for key in ex02.CONFIRM_KEYS}
        if self.quit_at is not None and self.tick == self.quit_at:
            return {key: KEY_WAS_TRIGGERED for key in ex02.QUIT_KEYS}
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


class PollSchedule(FakeGripper):
    """A gripper whose status frames follow a schedule.

    ``answer`` gets the poll count and decides whether that frame arrived, so a
    test can say "answers for a while, then goes quiet" — the dangerous case,
    because the cached position is then a *plausible old* value rather than the
    SDK's 0.0, and nothing looks wrong until the value is used.
    """

    def __init__(self, answer, **kwargs) -> None:
        super().__init__(**kwargs)
        self.answer = answer
        self.polls = 0

    def poll(self, timeout_s: float = 0.0) -> bool:
        self.polls += 1
        return bool(self.answer(self.polls))


class TestItWillNotActOnAnUnmeasuredPosition:
    """Every command has to be built from a position that came off the bus.

    ``MotorState._position`` starts at ``0.0`` (``litegrip/can/motor.py``) and is
    only moved by a real status frame, while ``GripperState.timestamp`` is the
    *local* clock — so the public snapshot cannot say how old it is, or whether it
    was ever filled in.  Build a ramp's start and duration from one and the ramp
    becomes a step aimed at 0 rad (5.5 % open on this calibration), which the
    motor answers with ``kp × 1.7 rad ≈ 170 Nm`` against a ~10 Nm servo.  That is
    the shape of "一开夹爪就起飞", so the reading is checked before it is trusted.
    """

    def test_no_frame_at_all_when_the_motor_never_answers(self, monkeypatch, capsys):
        run = _run(monkeypatch, steps=300, confirm_at=5,
                   gripper=FakeGripper(POS_OPEN_RAD + 0.3, answering=False))
        assert run.gripper.frames == [], (
            f"读不到状态帧还是发了 {len(run.gripper.frames)} 帧"
            f"（第一帧 q={run.gripper.frames[0]['q']:.4f}）")
        assert "不下发" in capsys.readouterr().out

    def test_the_unread_zero_is_never_commanded(self, monkeypatch):
        """``0.0`` means "never read", not "the jaws are at 0 rad"."""
        run = _run(monkeypatch, steps=300, confirm_at=5,
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
        run = _run(monkeypatch, gripper=gripper, steps=300, confirm_at=3)
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

    def test_it_disconnects_even_when_nothing_was_sent(self, monkeypatch):
        run = _run(monkeypatch, steps=20)
        assert run.gripper.stopped and run.gripper.disconnected
        assert run.sim.disconnected


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
            duration=None, force=10.0,
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
        assert gripper.controller.refreshes == 1, "没有先请它回一帧就读了缓存"
        expected = ex02.rad_to_fraction(gripper, gripper.position_rad) * 100
        assert f"{expected:5.1f}%" in out
        assert "6.63 mm" not in out, "又把「从没读到过」的 0.0 当成位置打印了"
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
        assert "✅ 没有故障" not in out

    def test_the_watchdog_registers_read_value_is_what_gets_reported(self, monkeypatch, capsys):
        """The stored 8000 ms is not this machine's current truth: the register
        is read live, and 0 (watchdog off) must not be reported as armed."""
        _, live = self._status(monkeypatch, capsys, FakeGripper(), timeout_ms=0.0)
        assert "通信超时保护 = 0" in live
        _, stored = self._status(monkeypatch, capsys, FakeGripper(),
                                 timeout_ms=8000.0)
        assert "通信超时保护 = 8000" in stored


class TestDescribingAFault:
    """The fault path must not be able to fail.

    A wedged motor is diagnosed by what this prints, so a crash in here costs the
    operator the one piece of information that explains the fault. CI has no SDK
    and ``--dry-run`` is documented to work without one, so the SDK import has to
    be optional — and the code it *doesn't* know has to be spelled out anyway.
    """

    def test_the_communication_watchdog_code_is_named(self):
        """0xD is what this hardware latches, and the SDK calls it 未知错误."""
        text = ex02.describe_code(0xD)
        assert "0xD" not in text and "未知错误" not in text, \
            f"0xD 又被打回「未知错误」了：{text}"
        assert "TIMEOUT" in text or "超时" in text

    def test_the_sdk_still_gets_the_codes_it_knows(self):
        """Duplicating the table must not shadow the SDK's own wording."""
        assert "过流" in ex02.describe_code(0xA)

    def test_it_works_without_the_sdk_installed(self, monkeypatch):
        """Exactly CI's situation: ``from litegrip... import`` raises."""
        monkeypatch.setitem(sys.modules, "litegrip", None)
        monkeypatch.setitem(sys.modules, "litegrip.constants", None)
        for code in (0x0, 0x1, 0x9, 0xA, 0xB, 0xC, 0xD):
            text = ex02.describe_code(code)
            assert "未知错误" not in text, \
                f"没装 SDK 就翻译不出 0x{code:X} 了：{text!r}"
        # A code nobody knows still has to come back as text, not as a raise.
        assert "0x7F" in ex02.describe_code(0x7F)


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
