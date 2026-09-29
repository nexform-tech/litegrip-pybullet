# -*- coding: utf-8 -*-
"""Example 03's mirror loop, driven end to end against a fake gripper.

03 is the example where the *readout* is the product: the window is supposed to
show where the hardware is right now.  If that pose comes from a frozen cache
instead of a live status frame, the window shows a pose the gripper is not in —
and the hold frame it streams at 200 Hz is a position command built from the same
number, so a frozen or never-filled cache turns "hold still" into "go to 0 rad".

Both are checked here against a stand-in for ``LiteGrip`` and a stand-in for the
PyBullet window.  Nothing here touches CAN or opens a window, so it is safe to run
on a machine with a gripper attached.
"""

import importlib.util
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("pybullet")

#: ``pressed()`` masks each event against pybullet's trigger bit; on this build
#: that is 2, so a fake handing out a bare ``1`` makes every key press a no-op.
from pybullet import KEY_WAS_TRIGGERED  # noqa: E402  (after importorskip)

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

os.environ.setdefault("LITEGRIP_PYBULLET_REEXEC", "1")
if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))


def _load_example_03():
    path = EXAMPLES / "03_real_to_sim.py"
    spec = importlib.util.spec_from_file_location("example03_loop", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["example03_loop"] = module
    spec.loader.exec_module(module)
    return module


ex03 = _load_example_03()

#: This machine's calibration, as ``litegrip_calibration.json`` has it: the
#: closed end is the numerically larger angle and opening drives it negative.
POS_CLOSED_RAD = 1.775959
POS_OPEN_RAD = -0.064279
RAD_TO_MM = 65.21
MAX_STROKE_MM = 120.0
KP = 100.0
KD = 2.0

#: The position the fake motor reports: open, but not all the way.
START_RAD = POS_OPEN_RAD + 0.3

#: ...and where a hand pushes it to while the motor is limp.
PUSHED_RAD = POS_OPEN_RAD + 1.1


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

    ``answer`` decides, per attempt, whether a status frame arrived — that is
    the public signal saying the cached position is current, and a bus that has
    gone quiet gives the same answer to a poll and to a 0xCC request.
    """

    def __init__(self, position_rad: float = START_RAD, answer=None) -> None:
        self.position_rad = position_rad
        self.answer = answer or (lambda n: True)
        self.polls = 0
        self.frames: list[dict] = []
        self.refreshes: list[float] = []
        self.zero_gravity_calls: list[str] = []
        self.disabled = False
        self.disconnected = False
        self.config = SimpleNamespace(
            pos_closed_rad=POS_CLOSED_RAD,
            pos_open_rad=POS_OPEN_RAD,
            rad_to_mm=RAD_TO_MM,
            max_stroke_mm=MAX_STROKE_MM,
            kp=KP,
            kd=KD,
        )

    # ── the LiteGrip surface example 03 uses ────────────────────────────
    def _answers(self) -> bool:
        self.polls += 1
        return bool(self.answer(self.polls))

    def poll(self, timeout_s: float = 0.0) -> bool:
        return self._answers()

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        self.refreshes.append(timeout_s)
        return self._answers()

    def get_state(self, wait: bool = True):
        return SimpleNamespace(
            position_rad=self.position_rad,
            position_mm=(POS_CLOSED_RAD - self.position_rad) * RAD_TO_MM,
            force_n=0.0,
            velocity_rad_s=0.0,
            data_age_s=0.0,
            has_data=True,
            is_stale=False,
            is_moving=False,
            error_code=1,
            is_error=False,
        )

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True

    def enter_zero_gravity(self, duration: float = 0.0) -> None:
        self.zero_gravity_calls.append("enter")

    def exit_zero_gravity(self) -> None:
        self.zero_gravity_calls.append("exit")

    def disable(self) -> None:
        self.disabled = True

    def disconnect(self) -> None:
        self.disconnected = True


class FakeSim:
    """Stands in for ``GripperSim``: no window, no physics, a step budget.

    Each loop iteration advances the clock by 20 ms — deliberately longer than
    ``FRAME_DT`` (5 ms), so an iteration sends exactly one frame no matter how the
    float comparison lands, and frame N is the frame of loop tick N+1.
    """

    def __init__(self, clock: FakeClock, steps: int, keys_at=None,
                 on_tick=None, dt: float = 0.02) -> None:
        self.clock = clock
        self.steps_left = steps
        self.keys_at = keys_at or {}
        self.on_tick = on_tick or {}
        self.dt = dt
        self.tick = 0
        self.urdf_path = "fake.urdf"
        self.mirrored: list[float] = []
        self.status: list[str] = []
        self.disconnected = False

    def connected(self) -> bool:
        return self.steps_left > 0

    def focus_camera(self) -> None:
        pass

    def keyboard_events(self):
        self.tick += 1
        if self.tick in self.on_tick:
            self.on_tick[self.tick]()
        return {key: KEY_WAS_TRIGGERED for key in self.keys_at.get(self.tick, ())}

    def reset_fraction(self, fraction: float) -> None:
        self.mirrored.append(fraction)

    def status_text(self, text: str) -> None:
        self.status.append(text)

    def step(self) -> bool:
        self.steps_left -= 1
        self.clock.advance(self.dt)
        return self.steps_left > 0

    def disconnect(self) -> None:
        self.disconnected = True


def _run(monkeypatch, gripper=None, steps=60, keys_at=None, on_tick=None,
         zero_gravity=False, passive=False):
    """Run example 03's ``main()`` against fakes; return the pieces."""
    clock = FakeClock()
    gripper = gripper or FakeGripper()
    sim = FakeSim(clock, steps, keys_at=keys_at, on_tick=on_tick)

    args = SimpleNamespace(
        channel="can0", can_id=0x08, mst_id=0x18, calib=None,
        urdf=None, headless=False, zero_gravity=zero_gravity, passive=passive,
        duration=0.0,
    )

    # ``open_real_gripper`` is where the motor would be enabled; the fake records
    # how it was asked, so a mode that must not enable can be told apart from one
    # that must.
    opened: list[dict] = []

    def fake_open(a, enable=True):
        opened.append(dict(enable=enable))
        return gripper

    monkeypatch.setattr(ex03, "parse_args", lambda: args)
    monkeypatch.setattr(ex03, "open_real_gripper", fake_open)
    monkeypatch.setattr(ex03, "GripperSim", lambda **kw: sim)
    monkeypatch.setattr(ex03, "time", SimpleNamespace(
        monotonic=clock.monotonic, sleep=lambda s: None))

    code = ex03.main()
    return SimpleNamespace(code=code, sim=sim, gripper=gripper, clock=clock,
                           opened=opened)


def fraction_of(rad: float) -> float:
    """The mirror value example 03 should render for ``rad``.

    Normalised over the *calibrated travel* — the two angles the calibration
    actually measured.  Deliberately not ``(closed − rad) × rad_to_mm /
    max_stroke_mm``: that route saturates partway when the file's ``rad_to_mm``
    was derived from a different ``max_stroke_mm`` than the SDK's default, and
    the mirror then cannot show the jaws fully open.  See
    ``_common.rad_to_fraction``.
    """
    return max(0.0, min(1.0,
                       (POS_CLOSED_RAD - rad)
                       / (POS_CLOSED_RAD - POS_OPEN_RAD)))


class TestMirroring:
    def test_the_window_shows_the_measured_position(self, monkeypatch):
        run = _run(monkeypatch, steps=40)
        assert run.sim.mirrored, "一次都没刷新仿真"
        assert run.sim.mirrored[-1] == pytest.approx(fraction_of(START_RAD))

    def test_the_mirror_reaches_both_ends_of_the_travel(self, monkeypatch):
        """A gripper at either end has to read 0.0 / 1.0 in the window.

        This is the regression test for the saturating mapping: on a
        calibration whose mm scale is not the SDK's nominal 120, the mirror
        used to top out around 72 % and the simulated jaws could never be shown
        fully open, whatever the real ones did.
        """
        gripper = FakeGripper()
        assert ex03.rad_to_fraction(gripper, POS_CLOSED_RAD) == pytest.approx(0.0)
        assert ex03.rad_to_fraction(gripper, POS_OPEN_RAD) == pytest.approx(1.0)
        # ...and the middle of the travel reads 0.5, not 0.7.
        middle = (POS_CLOSED_RAD + POS_OPEN_RAD) / 2.0
        assert ex03.rad_to_fraction(gripper, middle) == pytest.approx(0.5)

    def test_it_holds_the_measured_position_without_moving_it(self, monkeypatch):
        """A hold frame commands no motion: target = where the motor already is."""
        run = _run(monkeypatch, steps=40)
        assert run.gripper.frames
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(START_RAD)
            assert frame["kp"] == KP and frame["kd"] == KD
            assert frame["tau"] == 0.0

    def test_passive_sends_no_frame_at_all(self, monkeypatch):
        run = _run(monkeypatch, passive=True, steps=40)
        assert run.gripper.frames == []
        assert run.gripper.refreshes == [], \
            "--passive 说好了一帧都不发，0xCC 请求也是 CAN 帧"
        assert not run.gripper.disabled, \
            "--passive 下一帧都没发过，退出时也不该补一帧 0xFD 失能"

    def test_passive_does_not_enable_the_motor(self, monkeypatch):
        """Enabling and then sending nothing is what latches 0xD: about a second
        later the motor reports communication loss. ``--passive`` is "watch
        someone else drive it", so it must not enable either."""
        run = _run(monkeypatch, passive=True, steps=40)
        assert run.opened == [dict(enable=False)], \
            "--passive 还是把电机使能了——使能了又没人喂帧，约 1 s 就锁 0xD"

    def test_the_default_mode_does_enable(self, monkeypatch):
        """The mirror mode holds the fingers with stiffness, which needs the
        motor enabled."""
        run = _run(monkeypatch, steps=40)
        assert run.opened == [dict(enable=True)]


class TestExiting:
    """Leaving the motor enabled and silent is what latches 0xD.

    ``exit_zero_gravity()`` is a single frame, so the old exit path left an
    enabled motor with nobody feeding it: about a second later it latched the
    communication-loss fault, and the process that could have cleared it was
    already gone.
    """

    def test_the_exit_disables_the_motor(self, monkeypatch):
        run = _run(monkeypatch, steps=40)
        assert run.gripper.disabled, "退出时没有失能——电机会在无人喂帧时锁故障"

    def test_it_disables_even_when_it_was_left_soft(self, monkeypatch):
        """Coming out of --zero-gravity, one relock frame is not enough: nothing
        follows it, so the enabled motor goes quiet and latches a fault."""
        run = _run(monkeypatch, zero_gravity=True, steps=40)
        assert run.gripper.disabled
        assert run.gripper.zero_gravity_calls == ["enter"], \
            "退出时又发了一帧 exit_zero_gravity——那一帧之后还是没人喂"


class TestItWillNotCommandAnUnmeasuredPosition:
    """The hold frame's target has to come off the bus.

    ``MotorState._position`` starts at ``0.0`` and only a status frame moves it,
    and the motor does not speak unless it is spoken to — so "hold still" built
    from an unread cache is a step command to 0 rad (5.5 % open here), which the
    servo answers with ``kp × 1.7 rad ≈ 170 Nm``.
    """

    def test_no_control_frame_when_the_motor_never_answers(self, monkeypatch):
        gripper = FakeGripper(answer=lambda n: False)
        run = _run(monkeypatch, gripper=gripper, steps=40)
        assert run.gripper.frames == [], (
            f"读不到状态帧还是发了 {len(run.gripper.frames)} 帧"
            f"（第一帧 q={run.gripper.frames[0]['q']:.4f}）")

    def test_it_asks_for_a_frame_with_a_read_only_request(self, monkeypatch):
        """Waiting forever is not an option either: an unfed motor stays quiet."""
        gripper = FakeGripper(answer=lambda n: False)
        run = _run(monkeypatch, gripper=gripper, steps=40)
        assert run.gripper.refreshes, \
            "读不到帧也不叫它一声——那就永远读不到了"

    def test_it_holds_once_the_motor_starts_answering(self, monkeypatch):
        gripper = FakeGripper(answer=lambda n: n > 3)
        run = _run(monkeypatch, gripper=gripper, steps=60)
        assert run.gripper.frames, "恢复应答之后还是不发帧"
        for frame in run.gripper.frames:
            assert frame["q"] == pytest.approx(START_RAD)

    def test_a_frozen_read_is_said_out_loud(self, monkeypatch, capsys):
        """A stale pose rendered as if live is the failure this guards."""
        run = _run(monkeypatch, gripper=FakeGripper(answer=lambda n: False),
                   steps=40)
        assert "读不到状态帧" in capsys.readouterr().out
        assert any("未读到状态帧" in text for text in run.sim.status), \
            "窗口里还在把冻结的读数当实时位姿显示"


class TestSwitchingBackFromZeroGravity:
    def test_it_relocks_where_the_hand_left_the_gripper(self, monkeypatch):
        """Pushing the fingers is the point of ``--zero-gravity``; what the loop
        does next must not be a command back to where they started."""
        gripper = FakeGripper()

        def push() -> None:
            gripper.position_rad = PUSHED_RAD

        run = _run(monkeypatch, gripper=gripper, steps=60,
                   keys_at={3: (ex03.ZERO_GRAVITY_KEY,),
                            6: (ex03.ZERO_GRAVITY_KEY,)},
                   on_tick={4: push})
        assert gripper.zero_gravity_calls == ["enter", "exit"]

        # The two modes are told apart by kp: zero gravity streams kp=0, holding
        # streams the calibrated stiffness.  The first two holding frames are
        # before the push (ticks 1-2, Z at tick 3); everything from the second Z
        # (tick 6) on has to hold where the hand left the fingers.
        holding = [f["q"] for f in gripper.frames if f["kp"] == KP]
        assert holding[:2] == pytest.approx([START_RAD] * 2)
        assert holding[2:] == pytest.approx([PUSHED_RAD] * len(holding[2:])), \
            f"失力恢复后又按失力之前的目标发帧了：{[f'{q:+.3f}' for q in holding]}"
