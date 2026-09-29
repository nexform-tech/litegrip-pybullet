# -*- coding: utf-8 -*-
"""The shared real↔sim helpers in ``examples/_common.py``.

They are exercised against a stub config, so the tests run without hardware and
without the SDK installed: the arithmetic is what matters, and it is the same
arithmetic the SDK's own ``goto(mm)`` does.
"""

import argparse
import json
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

    def test_it_agrees_with_the_sdk_goto_formula_on_a_consistent_file(self):
        """Where the two can agree, they do.

        On a file whose ``rad_to_mm`` was derived from the same
        ``max_stroke_mm`` the SDK carries (``rad_to_mm == max_stroke_mm /
        travel``), going through millimetres and going through the travel are
        the same arithmetic.  The next test is the case where they are not.
        """
        gripper = _config()
        cfg = gripper.config
        assert cfg.rad_to_mm * (cfg.pos_closed_rad - cfg.pos_open_rad) == \
            pytest.approx(cfg.max_stroke_mm)
        for fraction in (0.0, 0.25, 0.6, 1.0):
            position_mm = fraction * cfg.max_stroke_mm
            expected = cfg.pos_closed_rad - position_mm / cfg.rad_to_mm
            assert _common.fraction_to_target_rad(
                gripper, fraction
            ) == pytest.approx(expected)

    def test_a_file_with_a_different_mm_scale_still_reaches_full_open(self):
        """The regression test for the saturating slider.

        The file this machine actually has has ``rad_to_mm = 61.01`` over a
        1.41 rad travel — a 86 mm scale — while ``load_calibration`` leaves
        ``max_stroke_mm`` at the SDK's default 120.  Mapping through
        ``position_mm / max_stroke_mm`` tops out at ``86 / 120 = 71.7 %``: the
        top of the slider did nothing, and 50 % of the slider asked for 70 % of
        the travel.  Normalising over the travel has no such premise, so the
        ends have to land exactly on the calibrated ends.
        """
        travel = 1.409552
        pos_closed, pos_open = 0.052071, 0.052071 - travel
        gripper = _config(pos_closed_rad=pos_closed, pos_open_rad=pos_open,
                          rad_to_mm=86.0 / travel, max_stroke_mm=120.0)

        assert _common.fraction_to_target_rad(
            gripper, 1.0) == pytest.approx(pos_open)
        assert _common.fraction_to_target_rad(
            gripper, 0.5) == pytest.approx(pos_closed - travel / 2.0)
        # Half the slider is half the travel — not the 70 % the old map gave.
        assert _common.fraction_to_target_rad(
            gripper, 0.5) != pytest.approx(pos_closed - 60.0 / gripper.config.rad_to_mm)

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

    def test_the_value_level_check_agrees_with_the_config_one(self):
        """``--dry-run`` has no ``GripperConfig`` to look at, only a file."""
        _common.check_calibration_values(0.114, -1.731, 65.21, 120.0)
        for values in ((0.0, 1.14, 105.26, 120.0),     # factory: inverted travel
                       (0.114, 0.114, 65.21, 120.0),   # no travel at all
                       (0.114, -1.731, 0.0, 120.0),    # no mm scale
                       (0.114, -1.731, 65.21, 0.0)):   # no stroke
            with pytest.raises(SystemExit, match="标定"):
                _common.check_calibration_values(*values)


#: A plausible calibration file's contents.  Not this bench's numbers -- the
#: tests below are about which file gets *used*, not about the arithmetic.
CALIB = dict(
    channel="can0", can_id=0x08, mst_id=0x18,
    zero_position_rad=0.114, max_position_rad=-1.731,
    travel_range_rad=1.845, rad_to_mm=120.0 / 1.845, kp=5.0, kd=2.0,
)


def _calib_file(path: Path, **overrides) -> Path:
    """Write a calibration file and return its path."""
    data = dict(CALIB)
    data.update(overrides)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _interactive(monkeypatch, on: bool) -> None:
    """Make ``_common`` see (or not see) an interactive stdin.

    ``_common``'s own ``sys`` reference is replaced, not the real module's, so
    pytest's own stdin keeps working.
    """
    monkeypatch.setattr(_common, "sys", SimpleNamespace(
        stdin=SimpleNamespace(isatty=lambda: on)))


class TestCalibrationConfig:
    """``--dry-run`` has no SDK and therefore no ``GripperConfig`` -- but it runs
    the *same* arithmetic, so it needs a stand-in that is shaped like one.

    The property that matters is not "the attributes exist" but "the helpers
    cannot tell the difference": a dry run that computes its target angle from a
    differently-shaped config would be advertising a number the real run never
    uses.  So most of these tests go through ``fraction_to_target_rad`` /
    ``rad_to_fraction`` rather than reading attributes.
    """

    def test_it_renames_the_file_keys_to_the_sdk_names(self):
        """The file says ``zero_position_rad``; the SDK's config says
        ``pos_closed_rad``.  Getting that mapping backwards would mirror the
        whole travel, which is exactly the kind of quiet error the dry run is
        supposed to surface rather than hide."""
        cfg = _common.calibration_config(dict(CALIB))
        assert cfg.pos_closed_rad == CALIB["zero_position_rad"]
        assert cfg.pos_open_rad == CALIB["max_position_rad"]
        assert cfg.rad_to_mm == pytest.approx(CALIB["rad_to_mm"])
        assert cfg.kp == CALIB["kp"]
        assert cfg.kd == CALIB["kd"]

    def test_the_helpers_compute_what_the_real_path_computes(self):
        """Same numbers as ``_config()``, which is what ``load_calibration``
        leaves behind on hardware: 0 % is closed, 100 % is open, and the two
        conversions invert each other."""
        dry = SimpleNamespace(config=_common.calibration_config(dict(CALIB)))
        real = _config()
        assert (_common.fraction_to_target_rad(dry, 0.0)
                == pytest.approx(_common.fraction_to_target_rad(real, 0.0)))
        assert (_common.fraction_to_target_rad(dry, 1.0)
                == pytest.approx(_common.fraction_to_target_rad(real, 1.0)))
        for fraction in (0.0, 0.25, 0.5, 1.0):
            target = _common.fraction_to_target_rad(dry, fraction)
            assert _common.rad_to_fraction(dry, target) == pytest.approx(fraction)

    def test_it_passes_the_same_self_consistency_check(self):
        dry = SimpleNamespace(config=_common.calibration_config(dict(CALIB)))
        _common.check_calibration(dry)          # must not raise

    def test_an_inconsistent_file_still_stops_the_dry_run(self):
        """The dry run is how a first-time user checks their calibration file,
        so it has to reject the same files the real path rejects."""
        bad = dict(CALIB, zero_position_rad=-1.731, max_position_rad=0.114)
        dry = SimpleNamespace(config=_common.calibration_config(bad))
        with pytest.raises(SystemExit, match="标定"):
            _common.check_calibration(dry)

    def test_a_file_without_kp_keeps_the_sdk_default(self):
        """``kp``/``kd`` are optional in the file, and ``load_calibration`` only
        sets the keys that are present -- so a missing one must not become 0.0,
        which would command a motor with no position term at all."""
        data = {k: v for k, v in CALIB.items() if k not in ("kp", "kd")}
        cfg = _common.calibration_config(data)
        assert cfg.kp == _common.NOMINAL_KP
        assert cfg.kd == _common.NOMINAL_KD

    def test_it_does_not_invent_the_keys_the_file_left_out(self):
        """``mst_id``/``can_id`` live in the file too, but they are checked
        against the command line before this point.  A default here would be a
        second, contradictory answer."""
        data = dict(CALIB, kp=5.0, kd=2.0)
        data.pop("can_id")
        data.pop("channel")
        cfg = _common.calibration_config(data)
        assert not hasattr(cfg, "can_id")
        assert not hasattr(cfg, "can_channel")
        assert cfg.mst_id == CALIB["mst_id"]

    def test_the_stroke_is_a_nominal_override_not_a_calibrated_one(self):
        """``max_stroke_mm`` is nowhere in the file: it is the millimetre scale
        the SDK names, so the caller says which one to assume."""
        assert (_common.calibration_config(dict(CALIB)).max_stroke_mm
                == _common.NOMINAL_STROKE_MM)
        assert (_common.calibration_config(dict(CALIB), max_stroke_mm=60.0)
                .max_stroke_mm == 60.0)


class TestChoosingCalibration:
    """Which calibration file this run uses is a *decision*, made every time.

    The SDK's default path is "wherever the last calibration was saved" and its
    ``load_calibration`` silently falls back to the shipped factory file when the
    path it was given cannot be read.  Neither is this gripper's geometry, so
    neither is an acceptable answer to "which angles should I command from?".
    """

    def test_an_explicit_path_is_used_without_asking(self, tmp_path, monkeypatch):
        wanted = _calib_file(tmp_path / "mine.json")
        _interactive(monkeypatch, False)      # 非交互也不该妨碍显式指定
        chosen = _common.choose_calibration_file(
            str(wanted), ask=lambda prompt: pytest.fail("不该提问"))
        assert chosen == wanted

    def test_the_candidates_leave_out_the_simulator_and_the_backups(
            self, tmp_path):
        real = _calib_file(tmp_path / "litegrip_calibration.json")
        _calib_file(tmp_path / "litegrip_calibration.sim.json")
        _calib_file(tmp_path / "litegrip_calibration.json.20260928.bak",
                    kp=100.0)
        assert _common.calibration_candidates([tmp_path]) == [real]

    def test_the_newest_candidate_comes_first(self, tmp_path):
        older = _calib_file(tmp_path / "a.json")
        newer = _calib_file(tmp_path / "b.json")
        os.utime(older, (1_600_000_000, 1_600_000_000))
        os.utime(newer, (1_700_000_000, 1_700_000_000))
        assert _common.calibration_candidates([tmp_path]) == [newer, older]

    def test_a_simulator_calibration_is_refused_even_when_named(self, tmp_path):
        """The studio keeps them apart on purpose; naming it must not override
        that.  Its scale is the simulated gripper's."""
        sim = _calib_file(tmp_path / "litegrip_calibration.sim.json")
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(str(sim))
        assert "仿真" in str(excinfo.value)

    def test_a_missing_file_is_refused(self, tmp_path):
        """The SDK would quietly load the factory calibration here; that is the
        failure this refusal exists for."""
        with pytest.raises(SystemExit, match="不存在"):
            _common.choose_calibration_file(str(tmp_path / "nope.json"))

    def test_a_file_without_the_sdks_keys_is_refused(self, tmp_path):
        odd = tmp_path / "other.json"
        odd.write_text(json.dumps({"hello": 1}), encoding="utf-8")
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(str(odd))
        message = str(excinfo.value)
        assert "zero_position_rad" in message

    def test_no_candidates_says_where_calibrations_come_from(self, monkeypatch):
        _interactive(monkeypatch, True)
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(None, candidates=[],
                                            ask=lambda prompt: pytest.fail(
                                                "没有候选还提问"))
        message = str(excinfo.value)
        assert "上位机" in message
        assert "--calib" in message

    def test_a_non_terminal_refuses_instead_of_picking_one(
            self, tmp_path, monkeypatch):
        """No tty means an operator is not there to choose -- and the answer is
        never "use whatever the SDK defaults to"."""
        real = _calib_file(tmp_path / "litegrip_calibration.json")
        _interactive(monkeypatch, False)
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(None, candidates=[real],
                                            ask=lambda prompt: pytest.fail(
                                                "非交互还提问"))
        message = str(excinfo.value)
        assert "--calib" in message
        assert str(real) in message, "没把候选列出来，操作员不知道该指哪个"

    def test_the_operator_picks_by_number(self, tmp_path, monkeypatch):
        first = _calib_file(tmp_path / "a.json")
        second = _calib_file(tmp_path / "b.json")
        _interactive(monkeypatch, True)
        printed: list[str] = []
        chosen = _common.choose_calibration_file(
            None, candidates=[first, second], ask=lambda prompt: "2",
            out=printed.append)
        assert chosen == second
        listing = "\n".join(printed)
        # The values are listed so the operator can recognise the file they just
        # calibrated with -- the SDK records no provenance at all.
        assert "rad_to_mm" in listing
        assert "closed +0.1140" in listing
        assert "mst_id 0x18" in listing, "ID 要按十六进制打，和命令行/日志一个写法"
        assert f"1) {first}" in listing

    def test_the_operator_can_type_a_path_instead(self, tmp_path, monkeypatch):
        """The host software's "save as" can put a calibration anywhere."""
        listed = _calib_file(tmp_path / "a.json")
        elsewhere = _calib_file(tmp_path / "somewhere" / "else.json")
        _interactive(monkeypatch, True)
        chosen = _common.choose_calibration_file(
            None, candidates=[listed], ask=lambda prompt: str(elsewhere),
            out=lambda text: None)
        assert chosen == elsewhere

    def test_nonsense_input_asks_again_instead_of_guessing(
            self, tmp_path, monkeypatch):
        real = _calib_file(tmp_path / "a.json")
        _interactive(monkeypatch, True)
        answers = iter(["banana", "1"])
        chosen = _common.choose_calibration_file(
            None, candidates=[real], ask=lambda prompt: next(answers),
            out=lambda text: None)
        assert chosen == real

    @pytest.mark.parametrize("answer", ["", "q", "quit"])
    def test_declining_refuses_rather_than_defaulting(
            self, tmp_path, monkeypatch, answer):
        real = _calib_file(tmp_path / "a.json")
        _interactive(monkeypatch, True)
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(
                None, candidates=[real], ask=lambda prompt: answer,
                out=lambda text: None)
        assert "标定" in str(excinfo.value)

    def test_end_of_input_is_not_a_choice(self, tmp_path, monkeypatch):
        real = _calib_file(tmp_path / "a.json")
        _interactive(monkeypatch, True)

        def eof(prompt):
            raise EOFError

        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(None, candidates=[real], ask=eof,
                                            out=lambda text: None)
        assert "--calib" in str(excinfo.value)


class TestCheckCalibrationMatchesArgs:
    """A calibration file names the motor it was measured on."""

    def test_a_matching_file_passes_and_says_nothing(self):
        assert _common.check_calibration_matches_args(
            CALIB, channel="can0", can_id=0x08, mst_id=0x18) == []

    def test_a_different_motor_id_is_refused(self):
        """The file that shipped with ``mst_id`` 18 (decimal, meant to be 0x18)
        bound the RX filter to 0x12: a master that transmits and hears nothing."""
        with pytest.raises(SystemExit) as excinfo:
            _common.check_calibration_matches_args(
                dict(CALIB, mst_id=18), can_id=0x08, mst_id=0x18)
        message = str(excinfo.value)
        assert "mst_id" in message
        assert "0x18" in message

    def test_a_different_can_id_is_refused(self):
        with pytest.raises(SystemExit, match="can_id"):
            _common.check_calibration_matches_args(
                dict(CALIB, can_id=0x0A), can_id=0x08, mst_id=0x18)

    def test_another_bus_is_only_a_note(self):
        """Moving the gripper to another CAN port is routine; refusing to run
        because the file says where it used to be plugged in would just block."""
        notes = _common.check_calibration_matches_args(
            CALIB, channel="can1", can_id=0x08, mst_id=0x18)
        assert notes and "can0" in notes[0]

    def test_a_file_that_names_no_motor_is_accepted(self):
        """Older files have no ``can_id``/``mst_id``; there is nothing to check."""
        assert _common.check_calibration_matches_args(
            {"zero_position_rad": 0.1}, can_id=0x08, mst_id=0x18) == []


class FakeCalibGrip:
    """The slice of ``LiteGrip`` that loading a calibration touches.

    ``apply=False`` is the SDK's silent factory fallback: it answers ``True``
    without having read the file, which is exactly what a caller cannot detect
    from the return value.
    """

    def __init__(self, apply: bool = True, takes_optional: bool = True) -> None:
        self.apply = apply
        self.takes_optional = takes_optional
        self.paths: list[str] = []
        self.config = SimpleNamespace(
            pos_closed_rad=0.0, pos_open_rad=1.14, rad_to_mm=105.26,
            max_stroke_mm=120.0, kp=100.0, kd=2.0, can_id=0x08, mst_id=0x18,
            can_channel="can0",
        )

    def load_calibration(self, path: str | None = None) -> bool:
        self.paths.append(str(path))
        if not self.apply:
            return True
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.config.pos_closed_rad = float(data["zero_position_rad"])
        self.config.pos_open_rad = float(data["max_position_rad"])
        self.config.rad_to_mm = float(data["rad_to_mm"])
        if self.takes_optional:
            for key, attr in (("kp", "kp"), ("kd", "kd"),
                              ("can_id", "can_id"), ("mst_id", "mst_id"),
                              ("channel", "can_channel")):
                if key in data:
                    setattr(self.config, attr, data[key])
        return True


class TestLoadingTheChosenCalibration:
    """Loading it is not enough -- the run has to *be* using that file.

    ``load_calibration`` returns ``True`` after silently substituting the factory
    calibration, so the caller compares what it asked for against what the config
    actually holds.  ``kp`` matters as much as the angles: it is the stiffness
    every hold frame is sent with.
    """

    def test_a_file_that_took_effect_passes(self, tmp_path):
        gripper = FakeCalibGrip()
        path = _calib_file(tmp_path / "c.json")
        data = _common.load_chosen_calibration(gripper, path)
        assert data["kp"] == 5.0
        assert gripper.paths == [str(path)]
        assert gripper.config.pos_closed_rad == pytest.approx(0.114)
        assert gripper.config.pos_open_rad == pytest.approx(-1.731)
        assert gripper.config.kp == pytest.approx(5.0)

    def test_a_silent_factory_fallback_is_caught(self, tmp_path):
        gripper = FakeCalibGrip(apply=False)
        path = _calib_file(tmp_path / "c.json")
        with pytest.raises(SystemExit) as excinfo:
            _common.load_chosen_calibration(gripper, path)
        message = str(excinfo.value)
        assert str(path) in message
        assert "出厂" in message

    def test_a_partly_applied_file_is_caught(self, tmp_path):
        """Angles copied but ``kp`` left at the SDK default is still a different
        gripper's stiffness than the one the operator calibrated."""
        gripper = FakeCalibGrip(takes_optional=False)
        path = _calib_file(tmp_path / "c.json")
        with pytest.raises(SystemExit) as excinfo:
            _common.load_chosen_calibration(gripper, path)
        assert "kp" in str(excinfo.value)

    def test_a_load_the_sdk_rejects_is_reported(self, tmp_path):
        class Refusing(FakeCalibGrip):
            def load_calibration(self, path=None) -> bool:
                return False

        with pytest.raises(SystemExit, match="载入标定失败"):
            _common.load_chosen_calibration(Refusing(),
                                           _calib_file(tmp_path / "c.json"))

    def test_an_inconsistent_file_still_hits_the_guard(self, tmp_path):
        """Whatever the file says, the angles have to make sense together."""
        path = _calib_file(tmp_path / "c.json", zero_position_rad=-1.0,
                           max_position_rad=1.0)
        with pytest.raises(SystemExit, match="标定"):
            _common.load_chosen_calibration(FakeCalibGrip(), path)


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

    def test_zero_travel_does_not_divide_by_zero(self):
        """The divisor is the travel, so a degenerate calibration is the case
        that has to be caught — ``max_stroke_mm`` never enters this function."""
        gripper = _config(pos_open_rad=0.114)     # travel == 0
        assert _common.rad_to_fraction(gripper, 0.0) == 0.0

    def test_the_nominal_stroke_does_not_enter_the_mapping(self):
        """Neither the SDK's default 120 nor a garbage value changes a reading.

        On the old millimetre route a ``max_stroke_mm`` of 0 was a division by
        zero and any other value scaled the answer.  Here the only two numbers
        that matter are the two calibrated angles, so the same pose reads the
        same however the nominal stroke is configured.
        """
        cfg = _config().config
        half_travel_rad = (cfg.pos_closed_rad + cfg.pos_open_rad) / 2.0
        for nominal in (0.0, 1.0, 120.0, 500.0):
            assert _common.rad_to_fraction(
                _config(max_stroke_mm=nominal), half_travel_rad
            ) == pytest.approx(0.5)

    def test_the_reading_the_hardware_actually_gave(self):
        """``get_state`` on can0 reported 29.85 mm on the SDK's mm scale.

        On a self-consistent file that is ``29.85 / 120 = 24.9 %`` of the
        travel, and that is what the window should render.
        """
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


def _traj_sdk(*absent: str) -> SimpleNamespace:
    """A stand-in for the *trajectory* SDK, lacking the named members.

    Built the same way as :func:`_sdk`, off ``TRAJECTORY_SDK_API``.  The two
    lists describe two different packages that happen to share a name, so the
    two helpers must not be interchangeable.
    """
    drop = set(absent)
    owners: dict = {"LiteGrip": {}, "Trajectory": {}}
    for path, _why in _common.TRAJECTORY_SDK_API:
        if path in drop:
            continue
        owner, _, attr = path.partition(".")
        owners[owner][attr] = True
    return SimpleNamespace(**{name: SimpleNamespace(**members)
                              for name, members in owners.items()})


class TestTrajectorySdkApiCheck:
    """03 refuses to run on the SDK that 04/05 use — loudly, at startup, naming
    the missing member and which checkout has it.

    There is no degraded path here: the recording loop and the replay loop are
    both the SDK's own background threads, timed to its own clock.  Hand-rolling
    either out of ``send_mit_frame`` would produce samples on a different beat
    than they were taken on, so the example stops instead.
    """

    def test_a_complete_sdk_passes(self):
        sdk = _traj_sdk()
        assert _common.missing_trajectory_sdk_api(sdk) == []
        _common.check_trajectory_sdk_api(sdk)       # must not raise

    def test_every_missing_member_is_named(self):
        sdk = _traj_sdk(*[path for path, _ in _common.TRAJECTORY_SDK_API])
        assert _common.missing_trajectory_sdk_api(sdk) == [
            path for path, _ in _common.TRAJECTORY_SDK_API]

    @pytest.mark.parametrize("absent",
                             [path for path, _ in _common.TRAJECTORY_SDK_API])
    def test_one_missing_member_is_reported_alone(self, absent):
        assert _common.missing_trajectory_sdk_api(_traj_sdk(absent)) == [absent]

    def test_it_says_which_checkout_to_point_at(self, monkeypatch):
        monkeypatch.delenv("LITEGRIP_TRAJ_SDK_DIR", raising=False)
        with pytest.raises(SystemExit) as excinfo:
            _common.check_trajectory_sdk_api(_traj_sdk("LiteGrip.record_start"))
        message = str(excinfo.value)
        assert "LiteGrip.record_start" in message
        assert "LITEGRIP_TRAJ_SDK_DIR" in message
        assert "litegrip-python" in message
        # ...and says which SDK the reader probably has, or "install the other
        # one" reads as "your install is broken".
        assert "refresh_status" in message

    def test_the_two_sdks_are_not_interchangeable(self):
        """Each check must fail on the other's package.  If either passed on
        both, the wrong-SDK failure would be silent again."""
        assert _common.missing_trajectory_sdk_api(_sdk()) != []
        assert _common.missing_sdk_api(_traj_sdk()) != []


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
        # ``None`` only means "not given on the command line": it is not a
        # request for the SDK's default calibration.  ``choose_calibration_file``
        # then asks, or refuses -- it never resolves ``None`` to a file.
        assert args.calib is None

    def test_the_calib_flag_is_documented_as_mandatory(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        help_text = parser.format_help()
        assert "--calib" in help_text
        assert "上位机" in help_text, "没说标定文件从哪来"

    def test_hardware_args_accept_hex_and_decimal(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args(["--can-id", "0x0A"]).can_id == 10
        assert parser.parse_args(["--can-id", "10"]).can_id == 10
        assert parser.parse_args(["--mst-id", "0x20"]).mst_id == 0x20

    def test_safety_banner_warns_about_real_motion(self):
        assert "真机" in _common.SAFETY_BANNER
        assert "Esc" in _common.SAFETY_BANNER


class TestTrajectorySdkDiscovery:
    """03 finds its SDK the same way 01-02 find theirs, on its own variable.

    Two variables, not one: ``LITEGRIP_SDK_DIR`` is the first thing
    :func:`_common.sdk_dir` looks at, so pointing it at the trajectory checkout
    would drag 04/05 over too — and they stop at startup there, because that
    package has no ``refresh_status``.
    """

    def test_its_own_env_var_wins(self, monkeypatch, tmp_path):
        monkeypatch.setenv("LITEGRIP_TRAJ_SDK_DIR", str(tmp_path))
        assert _common.trajectory_sdk_dir() == tmp_path

    def test_it_ignores_the_other_variable(self, monkeypatch, tmp_path):
        monkeypatch.setenv("LITEGRIP_SDK_DIR", str(tmp_path))
        monkeypatch.delenv("LITEGRIP_TRAJ_SDK_DIR", raising=False)
        assert _common.trajectory_sdk_dir() != tmp_path

    def test_the_other_variable_does_not_move_the_freshness_sdk(self, monkeypatch,
                                                               tmp_path):
        monkeypatch.setenv("LITEGRIP_TRAJ_SDK_DIR", str(tmp_path))
        monkeypatch.delenv("LITEGRIP_SDK_DIR", raising=False)
        assert _common.sdk_dir() != tmp_path

    def test_a_discovered_checkout_holds_the_package(self, monkeypatch):
        """Whatever comes back when nothing overrides it has to be usable: a
        directory without ``litegrip/__init__.py`` in it would send the import
        straight back to the other SDK."""
        monkeypatch.delenv("LITEGRIP_TRAJ_SDK_DIR", raising=False)
        found = _common.trajectory_sdk_dir()
        assert found is None or Path(found).is_dir()
        if found is not None:
            assert (Path(found) / "litegrip" / "__init__.py").is_file()


class TestLoadingTheOtherCheckout:
    """Two packages named ``litegrip`` on one machine, and 03 must get the right
    one.

    ``pip install -e`` registers a meta path finder whose priority sits above
    ``sys.path``, so ``sys.path.insert(0, trajectory_checkout)`` changes nothing
    — the import still resolves to the installed one.  Loading by directory is
    the only thing that works, which is why :func:`_common._load_package_from`
    exists rather than a ``sys.path`` tweak.
    """

    @pytest.fixture
    def clean_litegrip(self):
        """Restore ``sys.modules['litegrip*']`` however the test left it.

        Loading a throwaway package under that name replaces whatever the rest
        of the session imported, and this file shares the process with tests
        that use the real one.
        """
        def snapshot():
            return {name: module for name, module in sys.modules.items()
                    if name == "litegrip" or name.startswith("litegrip.")}

        def drop():
            for name in [n for n in sys.modules
                         if n == "litegrip" or n.startswith("litegrip.")]:
                del sys.modules[name]

        saved = snapshot()
        drop()
        try:
            yield
        finally:
            drop()
            sys.modules.update(saved)

    @pytest.fixture
    def throwaway(self, tmp_path) -> Path:
        package = tmp_path / "litegrip"
        package.mkdir()
        (package / "__init__.py").write_text(
            "MARKER = 'loaded from the directory'\n", encoding="utf-8")
        return tmp_path

    def test_it_loads_the_package_in_the_given_directory(self, throwaway,
                                                         clean_litegrip):
        module = _common._load_package_from(throwaway)
        assert module is not None
        assert module.MARKER == "loaded from the directory"
        assert sys.modules["litegrip"] is module, "加载完没挂到 sys.modules 上"

    def test_it_returns_none_when_there_is_no_package(self, tmp_path,
                                                      clean_litegrip):
        assert _common._load_package_from(tmp_path) is None

    def test_the_env_var_is_what_the_import_follows(self, monkeypatch, throwaway,
                                                    clean_litegrip):
        """End to end: the variable 03 documents is the one the loader obeys,
        even on a machine where the other SDK is installed and importable."""
        monkeypatch.setenv("LITEGRIP_TRAJ_SDK_DIR", str(throwaway))
        assert _common.import_trajectory_litegrip().MARKER == \
            "loaded from the directory"

    def test_a_broken_checkout_is_reported_rather_than_skipped(self, monkeypatch,
                                                               tmp_path,
                                                               clean_litegrip):
        """A checkout that fails to load must say so.  Falling through to the
        installed SDK would run 03 against the wrong package and blame it for
        the missing API."""
        package = tmp_path / "litegrip"
        package.mkdir()
        (package / "__init__.py").write_text("this is not python(\n",
                                             encoding="utf-8")
        monkeypatch.setenv("LITEGRIP_TRAJ_SDK_DIR", str(tmp_path))
        with pytest.raises(SystemExit) as excinfo:
            _common.import_trajectory_litegrip()
        assert str(tmp_path) in str(excinfo.value)


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
