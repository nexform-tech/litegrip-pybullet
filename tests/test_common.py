# -*- coding: utf-8 -*-
"""The shared real↔sim helpers in ``examples/_common.py``.

They are exercised against a stub config, so the tests run without hardware and
without the SDK installed: the arithmetic is what matters, and it is the same
arithmetic the SDK's own ``goto(mm)`` does.
"""

import argparse
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"

#: ``_common`` re-execs into the repo .venv when pybullet is missing.  Under
#: pytest that would replace the test process, so stand it down.
os.environ.setdefault("LITEGRIP_PYBULLET_REEXEC", "1")

if str(EXAMPLES) not in sys.path:
    sys.path.insert(0, str(EXAMPLES))

import _common  # noqa: E402


def _config(**overrides):
    """The SDK's ``GripperConfig``, as ``load_calibration`` leaves it.

    Calibration sets ``pos_closed_rad`` to the numerically larger value and
    ``pos_open_rad`` to the smaller one, and derives ``rad_to_mm`` from the
    measured travel.  The angles are the unit on can0 (≈1.85 rad of travel);
    ``rad_to_mm`` is computed from them so both directions are exact.
    """
    pos_closed, pos_open = 0.114, -1.731
    values = dict(
        max_stroke_mm=120.0,
        rad_to_mm=120.0 / (pos_closed - pos_open),
        pos_closed_rad=pos_closed,
        pos_open_rad=pos_open,
    )
    values.update(overrides)
    return SimpleNamespace(config=SimpleNamespace(**values))


class TestFractionToTargetRad:
    def test_full_open_reaches_pos_open(self):
        cfg = _config().config
        assert _common.fraction_to_target_rad(
            _config(), 1.0) == pytest.approx(cfg.pos_open_rad)

    def test_closed_reaches_pos_closed(self):
        cfg = _config().config
        assert _common.fraction_to_target_rad(
            _config(), 0.0) == pytest.approx(cfg.pos_closed_rad)

    def test_halfway_is_half_the_stroke(self):
        cfg = _config().config
        assert _common.fraction_to_target_rad(_config(), 0.5) == pytest.approx(
            cfg.pos_closed_rad - 60.0 / cfg.rad_to_mm
        )

    def test_matches_the_sdk_goto_formula(self):
        """Same arithmetic as ``goto(position_mm)`` — just spelled out."""
        gripper = _config()
        cfg = gripper.config
        for fraction in (0.0, 0.25, 0.6, 1.0):
            position_mm = fraction * cfg.max_stroke_mm
            expected = cfg.pos_closed_rad - position_mm / cfg.rad_to_mm
            assert _common.fraction_to_target_rad(
                gripper, fraction
            ) == pytest.approx(expected)

    def test_rad_decreases_as_the_gripper_opens(self):
        """Open is the *negative* direction — the sign the SDK's goto implies."""
        gripper = _config()
        readings = [_common.fraction_to_target_rad(gripper, f)
                    for f in (0.0, 0.5, 1.0)]
        assert readings[0] > readings[1] > readings[2]

    def test_the_whole_range_stays_inside_the_calibrated_travel(self):
        gripper = _config()
        cfg = gripper.config
        for fraction in (-1.0, 0.0, 0.3, 1.0, 2.0):
            rad = _common.fraction_to_target_rad(gripper, fraction)
            assert cfg.pos_open_rad <= rad <= cfg.pos_closed_rad

    def test_out_of_range_is_clamped(self):
        cfg = _config().config
        assert _common.fraction_to_target_rad(
            _config(), 5.0) == pytest.approx(cfg.pos_open_rad)
        assert _common.fraction_to_target_rad(
            _config(), -5.0) == pytest.approx(cfg.pos_closed_rad)

    def test_a_whole_stroke_does_not_exceed_the_measured_travel(self):
        """Commanding open must not reach past the calibrated open angle."""
        gripper = _config()
        cfg = gripper.config
        travel = cfg.pos_closed_rad - cfg.pos_open_rad
        commanded = (abs(_common.fraction_to_target_rad(gripper, 0.0)
                         - _common.fraction_to_target_rad(gripper, 1.0)))
        assert commanded == pytest.approx(travel)


class TestCheckCalibration:
    """The guard that stops the examples before they drive an uncalibrated unit."""

    def test_accepts_a_calibrated_config(self):
        _common.check_calibration(_config())  # must not raise

    def test_rejects_the_sdk_factory_defaults(self):
        """``GripperConfig`` ships ``pos_open_rad = +1.14`` with closed ``0.0``.

        That contradicts ``goto``'s own sign convention (closed must be the
        larger angle), so every target angle computed from it is meaningless.
        """
        with pytest.raises(SystemExit, match="标定"):
            _common.check_calibration(
                _config(rad_to_mm=105.26, pos_closed_rad=0.0,
                        pos_open_rad=1.14))

    def test_rejects_a_zero_travel_range(self):
        with pytest.raises(SystemExit, match="标定"):
            _common.check_calibration(_config(pos_open_rad=0.114))

    def test_rejects_a_zero_rad_to_mm(self):
        with pytest.raises(SystemExit, match="标定"):
            _common.check_calibration(_config(rad_to_mm=0.0))

    def test_rejects_a_zero_stroke(self):
        with pytest.raises(SystemExit, match="标定"):
            _common.check_calibration(_config(max_stroke_mm=0.0))

    def test_the_message_says_how_to_fix_it(self):
        with pytest.raises(SystemExit) as excinfo:
            _common.check_calibration(
                _config(pos_closed_rad=0.0, pos_open_rad=1.14))
        message = str(excinfo.value)
        assert "--calib" in message
        assert "calibrate()" in message


class TestRadToFraction:
    @pytest.mark.parametrize("fraction", [0.0, 0.125, 0.5, 0.9, 1.0])
    def test_round_trip(self, fraction):
        gripper = _config()
        rad = _common.fraction_to_target_rad(gripper, fraction)
        assert _common.rad_to_fraction(gripper, rad) == pytest.approx(
            fraction, abs=1e-9
        )

    def test_ends(self):
        gripper = _config()
        cfg = gripper.config
        assert _common.rad_to_fraction(gripper, cfg.pos_closed_rad) == 0.0
        assert _common.rad_to_fraction(gripper, cfg.pos_open_rad) == 1.0

    def test_out_of_range_is_clamped(self):
        gripper = _config()
        assert _common.rad_to_fraction(gripper, 100.0) == 0.0
        assert _common.rad_to_fraction(gripper, -100.0) == 1.0

    def test_zero_stroke_does_not_divide_by_zero(self):
        gripper = _config(max_stroke_mm=0.0)
        assert _common.rad_to_fraction(gripper, 0.0) == 0.0

    def test_the_reading_the_hardware_actually_gave(self):
        """``get_state`` on can0 reported 29.85 mm → 24.9 % of the SDK scale."""
        gripper = _config()
        cfg = gripper.config
        rad = cfg.pos_closed_rad - 29.85 / cfg.rad_to_mm
        assert _common.rad_to_fraction(gripper, rad) == pytest.approx(
            29.85 / 120.0
        )


class FakeGrip:
    """Just the ``LiteGrip`` surface the ``_common`` helpers use.

    ``data_age_s`` is the SDK's public freshness signal (``inf`` = no frame has
    ever been decoded); the two properties are derived from it exactly as the
    SDK derives them, so a test can put the snapshot on either side of
    ``STALE_AFTER_S`` by choosing one number.
    """

    #: ``litegrip.models.STALE_AFTER_S``: older than this reads as stale.
    STALE_AFTER_S = 0.5

    def __init__(self, answering: bool = True,
                 data_age_s: float = 0.0) -> None:
        self.answering = answering
        self.data_age_s = data_age_s
        self.refreshes: list[float] = []
        self.polls: list[float] = []
        self.reads = 0
        self.frames: list[dict] = []

    def refresh_status(self, timeout_s: float = 0.5) -> bool:
        self.refreshes.append(timeout_s)
        return self.answering

    def poll(self, timeout_s: float = 0.0) -> bool:
        self.polls.append(timeout_s)
        return self.answering

    def get_state(self, wait: bool = True):
        self.reads += 1
        return SimpleNamespace(
            position_rad=0.42,
            data_age_s=self.data_age_s,
            has_data=self.data_age_s != float("inf"),
            is_stale=self.data_age_s > self.STALE_AFTER_S,
        )

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        """Any control frame is recorded, so a read-only path can prove it sent
        none."""
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True


class TestFreshState:
    """``fresh_state`` is the only sanctioned way to read the real position.

    Two public signals have to agree before the cached position counts as a
    reading: ``poll()`` says a status frame arrived *just now*, and the
    snapshot's own ``has_data`` / ``is_stale`` say it is backed by data.
    """

    def test_a_fresh_frame_returns_the_state(self):
        gripper = FakeGrip()
        state = _common.fresh_state(gripper, timeout_s=0.25)
        assert state is not None
        assert state.position_rad == 0.42
        assert gripper.polls == [0.25]

    def test_no_frame_returns_none_without_reading_the_cache(self):
        """A cache known to be stale must not be read at all — reading it is how
        a frozen value gets mistaken for a measurement."""
        gripper = FakeGrip(answering=False)
        assert _common.fresh_state(gripper) is None
        assert gripper.reads == 0, "等不到帧还去读了缓存"

    def test_it_can_wake_the_motor_up_first(self):
        """A motor that is not being fed never speaks; 0xCC asks it to."""
        gripper = FakeGrip()
        _common.fresh_state(gripper, timeout_s=1.0, request=True)
        assert gripper.refreshes == [1.0]
        assert gripper.reads == 1

    def test_waking_the_motor_up_does_not_spend_the_budget_twice(self):
        """``refresh_status`` does its own waiting; polling again afterwards
        would double the worst-case stall of every caller that asks for a
        frame."""
        gripper = FakeGrip()
        _common.fresh_state(gripper, request=True)
        assert gripper.polls == [], "叫醒电机之后又 poll 了一次"

    def test_the_wake_up_call_is_read_only(self):
        """``refresh_status`` is documented "does not change motor output" — no
        control frame may be sent on the way to asking for one."""
        gripper = FakeGrip()
        _common.fresh_state(gripper, request=True)
        assert gripper.refreshes and gripper.frames == []

    def test_no_request_is_sent_unless_asked_for(self):
        gripper = FakeGrip()
        _common.fresh_state(gripper)
        assert gripper.refreshes == []

    def test_a_frame_the_sdk_calls_stale_is_refused(self):
        """``poll`` said a frame arrived, but the snapshot says it is old — the
        two disagree, and the reading must lose."""
        gripper = FakeGrip(data_age_s=FakeGrip.STALE_AFTER_S + 0.1)
        assert _common.fresh_state(gripper) is None
        assert gripper.reads == 1, "该读的还是读了，只是没敢用"

    def test_a_snapshot_with_no_data_behind_it_is_refused(self):
        """``inf`` is the SDK's "never received a frame" — the same
        ``MotorState._position = 0.0`` that this whole path exists to stop from
        being commanded."""
        gripper = FakeGrip(data_age_s=float("inf"))
        assert _common.fresh_state(gripper) is None

    def test_a_failing_transport_is_not_fatal(self):
        """A CAN error must read as "no frame", never as "here is the cache"."""
        class Exploding(FakeGrip):
            def refresh_status(self, timeout_s: float = 0.5) -> bool:
                raise OSError("CAN 掉线了")

        gripper = Exploding()
        assert _common.request_status_frame(gripper) is False
        assert _common.fresh_state(gripper, request=True) is None
        assert gripper.reads == 0, "传输层出错之后还是把缓存当读数了"


def _sdk(*absent: str) -> SimpleNamespace:
    """A stand-in for the ``litegrip`` module, lacking the named members.

    Built fresh on every call out of exactly the members the check looks for:
    the check is a ``hasattr`` walk, so an absent one has to be genuinely
    absent, and deleting it off a shared class would leak into other tests.
    """
    drop = set(absent)
    grip: dict = {}
    state: dict = {}
    for path, _why in _common.REQUIRED_SDK_API:
        if path in drop:
            continue
        owner, _, attr = path.partition(".")
        (grip if owner == "LiteGrip" else state)[attr] = True
    return SimpleNamespace(LiteGrip=SimpleNamespace(**grip),
                           GripperState=SimpleNamespace(**state))


class TestSdkApiCheck:
    """The examples refuse to run on an SDK that cannot answer "is this reading
    current?" — loudly, at startup, naming the member and where to get one.

    Silently degrading is what the previous version did (it borrowed
    ``gripper._can._controller``), and a guess about a measured position is the
    input to a step command.
    """

    def test_a_complete_sdk_passes(self):
        sdk = _sdk()
        assert _common.missing_sdk_api(sdk) == []
        _common.check_sdk_api(sdk)          # must not raise

    def test_every_missing_member_is_named(self):
        sdk = _sdk(*[path for path, _ in _common.REQUIRED_SDK_API])
        assert _common.missing_sdk_api(sdk) == [
            path for path, _ in _common.REQUIRED_SDK_API]

    @pytest.mark.parametrize("absent", [
        "LiteGrip.refresh_status",
        "GripperState.data_age_s",
        "GripperState.has_data",
        "GripperState.is_stale",
    ])
    def test_one_missing_member_is_reported_alone(self, absent):
        assert _common.missing_sdk_api(_sdk(absent)) == [absent]

    def test_it_says_which_sdk_to_use(self):
        with pytest.raises(SystemExit) as excinfo:
            _common.check_sdk_api(_sdk("LiteGrip.refresh_status"))
        message = str(excinfo.value)
        assert "LiteGrip.refresh_status" in message
        assert "PyPI" in message, "没说明这个包不在 PyPI 上，用户会去 pip install"
        assert "LITEGRIP_SDK_DIR" in message
        assert "pip install -e" in message


class TestStatusLine:
    def test_contains_the_opening_and_aperture(self):
        line = _common.status_line("仿真", fraction=0.5, aperture_mm=44.3)
        assert "50.0%" in line
        assert "44.30 mm" in line
        assert line.startswith("[仿真]")

    def test_bar_tracks_the_fraction(self):
        empty = _common.status_line("x", fraction=0.0, aperture_mm=1.5)
        full = _common.status_line("x", fraction=1.0, aperture_mm=87.0)
        assert empty.count("█") == 0
        assert full.count("█") == 20

    def test_optional_fields_appear_only_when_given(self):
        bare = _common.status_line("x", fraction=0.5, aperture_mm=44.3)
        assert "SDK" not in bare
        assert "力" not in bare
        assert "运动" not in bare

        full = _common.status_line(
            "真机", fraction=0.5, aperture_mm=44.3, sdk_mm=60.0, force_n=9.5,
            moving=True,
        )
        assert "SDK  60.00 mm" in full
        assert "力  9.50 N" in full
        assert "运动中" in full

    def test_moving_false_reads_as_stopped(self):
        line = _common.status_line(
            "真机", fraction=0.5, aperture_mm=44.3, moving=False)
        assert "已停住" in line

    def test_fraction_out_of_range_does_not_break_the_bar(self):
        wide = _common.status_line("x", fraction=5.0, aperture_mm=87.0)
        shut = _common.status_line("x", fraction=-1.0, aperture_mm=0.0)
        assert wide.count("█") == 20
        assert shut.count("█") == 0


class TestArgParsers:
    def test_common_args(self):
        parser = argparse.ArgumentParser()
        _common.add_common_args(parser)
        args = parser.parse_args([])
        assert args.urdf is None
        assert args.headless is False
        assert parser.parse_args(["--headless", "--urdf", "x.urdf"]).headless

    def test_hardware_args_defaults(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        args = parser.parse_args([])
        assert args.channel == "can0"
        assert args.can_id == 0x08
        assert args.mst_id == 0x18
        assert args.calib is None

    def test_hardware_args_accept_hex_and_decimal(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args(["--can-id", "0x0A"]).can_id == 10
        assert parser.parse_args(["--can-id", "10"]).can_id == 10
        assert parser.parse_args(["--mst-id", "0x20"]).mst_id == 0x20

    def test_safety_banner_warns_about_real_motion(self):
        assert "真机" in _common.SAFETY_BANNER
        assert "Esc" in _common.SAFETY_BANNER


class TestSdkDiscovery:
    def test_env_var_wins(self, monkeypatch, tmp_path):
        monkeypatch.setenv("LITEGRIP_SDK_DIR", str(tmp_path))
        assert _common.sdk_dir() == tmp_path

    def test_discovers_a_real_sdk_if_one_is_around(self, monkeypatch):
        """When nothing overrides it, whatever comes back must actually exist."""
        monkeypatch.delenv("LITEGRIP_SDK_DIR", raising=False)
        found = _common.sdk_dir()
        assert found is None or Path(found).exists()

    def test_bootstrap_makes_the_library_importable(self):
        _common.bootstrap_src()
        import litegrip_pybullet

        assert Path(litegrip_pybullet.__file__).is_file()
