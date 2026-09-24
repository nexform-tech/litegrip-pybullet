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
