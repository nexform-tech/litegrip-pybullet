# -*- coding: utf-8 -*-
"""Example 03's record/replay loop, driven end to end against a fake gripper.

03 is where the two halves of the gripper's life have to line up: the motion is
recorded on the hardware, and replaying it drives the hardware *and* the window
from one trajectory.  What this file pins down is that those two never fight
over the bus, and that nothing is commanded from a number that was not measured.

Three of these are failure modes with a physical cost:

* the window has to show the **measured** position, not the trajectory's own
  commanded opening — the whole claim of the example is that the window shows
  what the gripper is doing;
* no frame of our own may go out while the SDK's recorder or player is running —
  the trajectory is being streamed by a background thread, and a second stream
  on the same bus corrupts it;
* the gaps between phases (which the SDK leaves unfed) have to be held, or the
  enabled motor latches its communication-loss fault about a second later.

Nothing here touches CAN or opens a window, so it is safe to run on a machine
with a gripper attached.
"""

import importlib.util
import os
import sys
import time as real_time
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
    path = EXAMPLES / "03_trajectory.py"
    spec = importlib.util.spec_from_file_location("example03_trajectory", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["example03_trajectory"] = module
    spec.loader.exec_module(module)
    return module


ex03 = _load_example_03()

#: A calibration of the shape this bench has: the closed end is the numerically
#: larger angle, opening drives it negative.
POS_CLOSED_RAD = 1.775959
POS_OPEN_RAD = -0.064279
KP = 100.0
KD = 2.0

#: Where the motor is when the loop starts reading, and where a hand has pushed
#: it by the time the recording is stopped.
START_RAD = POS_OPEN_RAD + 0.3
PUSHED_RAD = POS_OPEN_RAD + 1.1

#: ...and where it is while the replayed trajectory is driving it.  Deliberately
#: a different opening from the trajectory's own last sample (0.8), so mirroring
#: the commanded value instead of the measured one is visible in the assertions.
REPLAYED_RAD = POS_OPEN_RAD + 0.55


class FakeClock:
    """A monotonic clock the fake window advances — no sleeping in tests."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def monotonic(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


class FakeTrajectory:
    """A recorded motion: just enough of ``Trajectory`` for the loop."""

    def __init__(self, n: int = 50, duration: float = 1.0) -> None:
        self.n = n
        self.duration = duration
        self.samples = [SimpleNamespace(openness=0.2 + 0.6 * i / (n - 1))
                        for i in range(n)]
        self.can_id = 0x08
        self.mount = "normal"
        self.saved_as = None

    def __len__(self) -> int:
        return self.n

    @property
    def last_openness(self) -> float:
        return self.samples[-1].openness

    def openness_at(self, t: float) -> float:
        span = max(self.duration, 1e-9)
        return 0.2 + 0.6 * max(0.0, min(1.0, t / span))

    def save(self, name: str) -> str:
        self.saved_as = name
        return f"/tmp/{name}.lgt"

    @classmethod
    def load(cls, name: str) -> "FakeTrajectory":
        if "missing" in name:
            raise FileNotFoundError(name)
        return cls()


class FakeGripper:
    """Stands in for a connected, enabled ``LiteGrip`` running the replay SDK.

    ``calls`` is the ordered log of everything the example did to the gripper:
    the point of several tests here is *when* a frame was sent relative to
    ``record_start`` / ``play_start``, which a per-call list answers directly.
    """

    def __init__(self, position_rad: float = START_RAD, answers: bool = True,
                 play_calls: int = 4, record_error=None) -> None:
        self.position_rad = position_rad
        self.answers = answers
        self.play_calls = play_calls
        self.record_error = record_error
        self.calls: list[str] = []
        self.frames: list[dict] = []
        self.config = SimpleNamespace(pos_closed_rad=POS_CLOSED_RAD,
                                      pos_open_rad=POS_OPEN_RAD,
                                      kp=KP, kd=KD)
        self.recording = False
        self.playing = False
        self.status_calls = 0
        self.record_args: dict = {}
        self.play_args: dict = {}
        self.played = None
        self.trajectory = FakeTrajectory()
        self.disabled = False
        self.disconnected = False

    # ── the LiteGrip surface example 03 uses ────────────────────────────
    def poll(self, timeout_s: float = 0.0) -> bool:
        self.calls.append("poll")
        return self.answers

    def get_state(self, wait: bool = True):
        return SimpleNamespace(
            position_rad=self.position_rad,
            position_mm=(POS_CLOSED_RAD - self.position_rad),
            force_n=0.0,
            velocity_rad_s=0.0,
            is_moving=False,
            error_code=1,
            is_error=False,
        )

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        self.calls.append("frame")
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True

    def record_start(self, rate_hz=100.0, zero_gravity=True, max_samples=None):
        self.calls.append("record_start")
        self.record_args = dict(rate_hz=rate_hz, zero_gravity=zero_gravity)
        self.recording = True
        return {"active": True, "kind": "record", "samples": 0}

    def record_stop(self, allow_empty: bool = False):
        self.calls.append("record_stop")
        self.recording = False
        if self.record_error is not None:
            raise self.record_error
        self.position_rad = PUSHED_RAD      # 手推过它
        return self.trajectory

    def play_start(self, trajectory, speed=1.0, kp=None, kd=None, loop=False,
                   align=True):
        self.calls.append("play_start")
        self.playing = True
        self.status_calls = 0
        self.played = trajectory
        self.play_args = dict(speed=speed, loop=loop, align=align)
        self.position_rad = REPLAYED_RAD    # 回放线程在推它走
        return {"active": True, "kind": "play"}

    def play_stop(self, timeout: float = 2.0):
        self.calls.append("play_stop")
        self.playing = False
        return {"active": False, "kind": "play"}

    def trajectory_status(self):
        if self.recording:
            return {"active": True, "kind": "record", "samples": 7}
        if self.playing:
            self.status_calls += 1
            if self.status_calls >= self.play_calls:
                self.playing = False
                return {"active": False, "kind": "play", "frames": 42,
                        "loop_hz": 99.5, "completed": True}
            return {"active": True, "kind": "play", "frames": 10,
                    "loop_hz": 99.0, "openness": 0.9}
        return {"active": False, "kind": None}

    def disable(self) -> None:
        self.calls.append("disable")
        self.disabled = True

    def disconnect(self) -> None:
        self.calls.append("disconnect")
        self.disconnected = True


class FakeSim:
    """Stands in for ``GripperSim``: no window, no physics, a step budget."""

    def __init__(self, clock: FakeClock, steps: int, keys_at=None,
                 dt: float = 0.02) -> None:
        self.clock = clock
        self.steps_left = steps
        self.keys_at = keys_at or {}
        self.dt = dt
        self.tick = 0
        self.urdf_path = "fake.urdf"
        self.gui = False
        self.mirrored: list[float] = []
        self.status: list[str] = []
        self.disconnected = False

    def connected(self) -> bool:
        return self.steps_left > 0

    def focus_camera(self) -> None:
        pass

    def keyboard_events(self):
        self.tick += 1
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


class EmptyTrajectory(FakeTrajectory):
    """A file that holds no samples: the recording that captured nothing."""

    @classmethod
    def load(cls, name: str) -> "EmptyTrajectory":
        return cls(n=0)


def fake_sdk(trajectory_class=FakeTrajectory) -> SimpleNamespace:
    """The trajectory SDK's public surface, as 03 imports it."""
    return SimpleNamespace(Trajectory=trajectory_class,
                           trajectory_dir=lambda: "/tmp")


def _run(monkeypatch, gripper=None, steps=200, mode="record", record=0.1,
         speed=1.0, keys_at=None, sdk_trajectory=FakeTrajectory):
    """Run example 03's ``main()`` against fakes; return the pieces."""
    clock = FakeClock()
    gripper = gripper if gripper is not None else FakeGripper()
    sim = FakeSim(clock, steps, keys_at=keys_at)

    args = SimpleNamespace(
        channel="can0", can_id=0x08, mst_id=0x18, calib="/tmp/calib.json",
        urdf=None, headless=True, record=record, speed=speed, real=False,
        play=None,
    )
    if mode == "play":
        args.play = "/tmp/fixture"
    elif mode == "play-real":
        args.play = "/tmp/fixture"
        args.real = True

    opened: list[dict] = []

    def fake_open(a, litegrip):
        opened.append(dict(channel=a.channel))
        return gripper

    monkeypatch.setattr(ex03, "parse_args", lambda: args)
    monkeypatch.setattr(ex03, "import_trajectory_litegrip",
                        lambda: fake_sdk(sdk_trajectory))
    monkeypatch.setattr(ex03, "check_trajectory_sdk_api", lambda sdk: None)
    monkeypatch.setattr(ex03, "open_gripper", fake_open)
    monkeypatch.setattr(ex03, "GripperSim", lambda **kw: sim)
    monkeypatch.setattr(ex03, "time", SimpleNamespace(
        monotonic=clock.monotonic, sleep=lambda s: None,
        strftime=real_time.strftime))

    code = ex03.main()
    return SimpleNamespace(code=code, sim=sim, gripper=gripper, clock=clock,
                           opened=opened, args=args)


def fraction_of(rad: float) -> float:
    """The mirror value example 03 should render for ``rad`` (see 04's tests)."""
    return max(0.0, min(1.0,
                       (POS_CLOSED_RAD - rad)
                       / (POS_CLOSED_RAD - POS_OPEN_RAD)))


class TestReplayingAFileWithoutHardware:
    """``--play`` on its own is the offline viewer, and CI's only runnable path."""

    def test_it_runs_and_reports_success(self, monkeypatch):
        run = _run(monkeypatch, mode="play", steps=80)
        assert run.code == 0

    def test_it_never_touches_the_hardware(self, monkeypatch):
        run = _run(monkeypatch, mode="play", steps=80)
        assert run.opened == [], "--play 默认不该去连真机"
        assert run.gripper.calls == [], \
            f"不连真机却动了夹爪：{run.gripper.calls}"
        assert run.gripper.frames == []

    def test_the_window_follows_the_trajectory(self, monkeypatch):
        run = _run(monkeypatch, mode="play", steps=80)
        assert run.sim.mirrored, "一次都没刷新仿真"
        assert run.sim.mirrored[0] == pytest.approx(0.2, abs=0.02)
        assert run.sim.mirrored[-1] == pytest.approx(0.8, abs=0.02)
        assert run.sim.mirrored == sorted(run.sim.mirrored), "开度不是单调张开的"

    def test_the_speed_multiplier_is_honoured(self, monkeypatch):
        """``--speed 2`` covers the trajectory in half the wall clock."""
        fast = _run(monkeypatch, mode="play", speed=2.0, steps=80)
        slow = _run(monkeypatch, mode="play", speed=1.0, steps=80)
        assert len(fast.sim.mirrored) < len(slow.sim.mirrored)

    def test_it_says_it_is_not_connecting(self, monkeypatch, capsys):
        _run(monkeypatch, mode="play", steps=80)
        out = capsys.readouterr().out
        assert "不连真机" in out


class TestRecording:
    """The recording half: zero gravity, hand-taught, then saved."""

    def test_it_records_in_zero_gravity(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        assert run.gripper.record_args["zero_gravity"] is True, \
            "录制没进零重力——手推不动，就不是手拖录制了"
        assert run.gripper.record_args["rate_hz"] == ex03.RATE_HZ

    def test_it_saves_what_it_recorded(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        assert run.gripper.trajectory.saved_as, "录完没存盘"
        assert run.gripper.trajectory.saved_as.startswith(ex03.DEFAULT_NAME), \
            "存的名字里没有样例名，回头找不到是哪来的"

    def test_an_empty_recording_is_a_failure(self, monkeypatch):
        """No samples is not a demo: report it and exit non-zero."""
        gripper = FakeGripper(record_error=RuntimeError("没有采到样本"))
        run = _run(monkeypatch, gripper=gripper, record=0.1)
        assert run.code == 1
        assert not gripper.playing, "没录到东西还是去回放了"

    def test_it_does_not_feed_frames_during_the_recording(self, monkeypatch):
        """The recorder streams zero-torque frames itself; a second stream on the
        same bus is what corrupts a capture."""
        run = _run(monkeypatch, record=0.1)
        calls = run.gripper.calls
        start = calls.index("record_start")
        stop = calls.index("record_stop")
        assert "frame" not in calls[start:stop], \
            f"录制期间本样例自己发了帧：{calls[start:stop]}"


class TestReplayingOnTheHardware:
    """``--play`` with the recording path: one trajectory, both targets."""

    def test_the_replay_gets_the_trajectory_and_the_speed(self, monkeypatch):
        run = _run(monkeypatch, record=0.1, speed=0.5)
        assert run.gripper.played is run.gripper.trajectory
        assert run.gripper.play_args["speed"] == 0.5
        assert run.gripper.play_args["align"] is True, \
            "没对齐就回放，起手是一步从当前位置到首个样本的跳变"

    def test_it_does_not_feed_frames_while_the_replay_runs(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        calls = run.gripper.calls
        start = calls.index("play_start")
        stop = calls.index("play_stop")
        assert "frame" not in calls[start:stop], (
            f"回放期间本样例又发了自己的帧：{calls[start:stop]}")
        assert stop > start

    def test_the_window_shows_the_measured_position_not_the_command(
            self, monkeypatch):
        """The example's whole claim.  The fake reports an opening the
        trajectory never asks for (see REPLAYED_RAD), so a window driven by
        ``status()['openness']`` would show 0.9 instead."""
        run = _run(monkeypatch, record=0.1)
        after_replay = run.sim.mirrored[-1]
        assert after_replay == pytest.approx(fraction_of(REPLAYED_RAD)), (
            f"窗口显示的是命令值不是实测值：{after_replay:.3f} "
            f"（实测 {fraction_of(REPLAYED_RAD):.3f}）")

    def test_it_says_the_gripper_will_not_return_by_itself(self, monkeypatch,
                                                           capsys):
        _run(monkeypatch, record=0.1)
        assert "不会自己回起点" in capsys.readouterr().out

    def test_play_real_replays_a_loaded_file_on_the_hardware(self, monkeypatch):
        """``--play FILE --real``: no recording, but the gripper still moves."""
        run = _run(monkeypatch, mode="play-real", steps=80)
        assert run.code == 0
        assert run.opened, "--real 却没去连真机"
        assert run.gripper.played is not None, "读到的轨迹没下发给真机"
        assert run.gripper.play_args["align"] is True
        assert "record_start" not in run.gripper.calls, \
            "--play 不该现场再录一段"
        assert run.gripper.disabled, "退出时没有失能"


class TestTheMotorIsNeverLeftUnfed:
    """Between phases the SDK stops feeding, and an enabled motor silent for
    about a second latches its communication-loss fault (0xD)."""

    def test_it_holds_position_after_the_recording_stops(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        calls = run.gripper.calls
        stop = calls.index("record_stop")
        play = calls.index("play_start")
        assert "frame" in calls[stop:play], \
            "record_stop 到 play_start 之间一帧都没发——这段空档没人喂电机"
        assert run.gripper.frames, "一帧保持帧都没发过"

    def test_it_holds_position_after_the_replay_stops(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        calls = run.gripper.calls
        stop = calls.index("play_stop")
        assert "frame" in calls[stop:], \
            "回放结束之后没人喂帧——电机静默约 1 s 就锁 0xD"

    def test_the_hold_after_the_replay_targets_the_new_position(self,
                                                                monkeypatch):
        """Re-using the pre-replay target is a command to travel back to it."""
        run = _run(monkeypatch, record=0.1)
        held = [frame["q"] for frame in run.gripper.frames]
        assert held, "一帧保持帧都没有"
        assert held[-1] == pytest.approx(REPLAYED_RAD), (
            f"回放结束后还锁在回放之前的位置上：{held[-1]:+.4f} "
            f"（现在的位置 {REPLAYED_RAD:+.4f}）")

    def test_every_hold_frame_is_a_standing_still_command(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        for frame in run.gripper.frames:
            assert frame["kp"] == KP and frame["kd"] == KD
            assert frame["tau"] == 0.0, "保持帧带了前馈"
            assert frame["dq"] == 0.0


class TestItWillNotHoldAnUnmeasuredPosition:
    """A hold frame's target must come off the bus: the cache's initial value is
    ``0.0``, which is a position the gripper has never been in."""

    def test_no_frame_goes_out_when_the_motor_never_answers(self, monkeypatch):
        gripper = FakeGripper(answers=False)
        run = _run(monkeypatch, gripper=gripper, record=0.1)
        assert gripper.frames == [], (
            f"读不到状态帧还是发了 {len(gripper.frames)} 帧"
            f"（第一帧 q={gripper.frames[0]['q']:.4f}）")

    def test_it_asks_for_a_frame_before_giving_up(self, monkeypatch):
        gripper = FakeGripper(answers=False)
        run = _run(monkeypatch, gripper=gripper, record=0.1)
        assert run.gripper.calls.count("poll") > 1, "一次都没去问电机要状态帧"


class TestLoadingATrajectory:
    """Reading a ``.lgt`` is the one thing 03 does that touches no bus at all."""

    def test_a_missing_file_says_where_it_looked(self):
        """A bare name is not a path: it is resolved inside the SDK's own
        trajectory directory, so a reader looking in the working directory needs
        to be told that."""
        sdk = SimpleNamespace(Trajectory=FakeTrajectory,
                              trajectory_dir=lambda: "/home/o/.litegrip/trajectories")
        with pytest.raises(SystemExit) as excinfo:
            ex03.load_trajectory(sdk, "missing")
        message = str(excinfo.value)
        assert "找不到这段轨迹" in message
        assert "missing" in message
        assert ".lgt" in message
        assert "trajectories" in message

    def test_a_zero_sample_file_is_refused(self, monkeypatch, capsys):
        """A file with no samples would "replay" instantly and report success."""
        run = _run(monkeypatch, mode="play", steps=80,
                   sdk_trajectory=EmptyTrajectory)
        assert run.code == 1
        assert "一个样本都没有" in capsys.readouterr().out
        assert run.sim.mirrored == [], "空轨迹还是刷了仿真"


class TestExiting:
    def test_the_exit_disables_the_motor(self, monkeypatch):
        run = _run(monkeypatch, record=0.1)
        assert run.gripper.disabled, "退出时没有失能——电机会在无人喂帧时锁故障"
        assert run.gripper.disconnected
        assert run.sim.disconnected

    def test_the_offline_path_disables_nothing(self, monkeypatch):
        run = _run(monkeypatch, mode="play", steps=80)
        assert not run.gripper.disabled
        assert run.sim.disconnected
