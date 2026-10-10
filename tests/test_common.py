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

    Two tiers, and the order is the priority: ``--calib``, then the factory file
    shipped inside the SDK package.  There is no third tier: when neither works
    the run stops and says to pass ``--calib``.  Scanning ``~/.litegrip`` for
    candidates and asking an operator to pick one was removed on purpose --
    guessing which JSON belongs to which gripper is not something this example
    can do -- and the scan code survives only as the ``--list-calibrations``
    query, which prints paths and returns.

    The factory file is a usable default, not this gripper's calibration: it is
    the geometry of the bench fixture.  That is why the tier exists at all (a
    fresh machine runs without a calibration hunt) and why it must say out loud
    what it is.
    """

    def test_an_explicit_path_wins_over_the_factory_file(self, tmp_path):
        wanted = _calib_file(tmp_path / "mine.json")
        factory = _calib_file(tmp_path / "factory_calibration.json")
        chosen = _common.choose_calibration_file(
            str(wanted), factory=factory, out=lambda text: None)
        assert chosen == wanted

    def test_the_factory_file_is_the_default_and_says_what_it_is(
            self, tmp_path):
        """No ``--calib`` on a fresh machine: the run proceeds with the SDK's own
        file instead of hunting for the operator's.  It must still say the
        numbers are the bench fixture's, and how to point at another one."""
        factory = _calib_file(tmp_path / "factory_calibration.json")
        printed: list[str] = []
        chosen = _common.choose_calibration_file(
            None, factory=factory, out=printed.append)
        assert chosen == factory
        listing = "\n".join(printed)
        assert "出厂标定" in listing
        assert "--calib" in listing, "没说这台夹爪自己的标定怎么指"

    def test_an_unreadable_factory_file_refuses_and_names_the_flag(self, tmp_path):
        """The old third tier answered this with a candidate list.  Now it is a
        stop: an incomplete SDK install is not a reason to drive the gripper
        with a file nobody chose."""
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(
                None, factory=tmp_path / "gone.json", out=lambda text: None)
        message = str(excinfo.value)
        assert "--calib" in message
        assert "出厂标定" in message, "没说清默认那份为什么没用上"

    def test_a_corrupt_factory_file_refuses_the_same_way(self, tmp_path):
        """Parsing failure, not just a missing file -- the SDK package could
        ship a truncated one, and it is still not a default."""
        bad = tmp_path / "factory_calibration.json"
        bad.write_text("{ not json", encoding="utf-8")
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(None, factory=bad,
                                            out=lambda text: None)
        assert "--calib" in str(excinfo.value)

    def test_no_sdk_at_all_refuses_and_names_the_flag(self):
        """``factory=None`` is what a caller that cannot reach the SDK passes.
        It is the same stop, with the reason spelled out."""
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(None, out=lambda text: None)
        assert "--calib" in str(excinfo.value)

    def test_a_refusal_never_names_a_candidate(self, tmp_path, monkeypatch):
        """The one thing the removed third tier did -- printing candidates --
        must be gone from the refusal path.  ``calibration_candidates`` is
        stubbed to a real file: if anything still scanned, it would show up."""
        real = _calib_file(tmp_path / "litegrip_calibration.json")
        monkeypatch.setattr(_common, "calibration_candidates",
                            lambda *a, **k: [real])
        with pytest.raises(SystemExit) as excinfo:
            _common.choose_calibration_file(
                None, factory=tmp_path / "gone.json", out=lambda text: None)
        assert str(real) not in str(excinfo.value)

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

class TestListingCalibrations:
    """``--list-calibrations`` is a query, not a chooser: it prints and returns.

    The scan code had to stay for this -- ``--calib`` needs a path, and the host
    software's "save as" can put that file anywhere, so the tool that finds it is
    the only way to change calibration without opening a file manager.  What it
    must *not* do is pick anything, or touch the bus.
    """

    def _candidates(self, monkeypatch, paths):
        monkeypatch.setattr(_common, "calibration_candidates",
                            lambda *a, **k: list(paths))

    def test_it_lists_the_candidates_with_their_values(
            self, tmp_path, monkeypatch):
        real = _calib_file(tmp_path / "litegrip_calibration.json")
        self._candidates(monkeypatch, [real])
        printed: list[str] = []
        assert _common.list_calibrations(out=printed.append) == 0
        listing = "\n".join(printed)
        assert f"1) {real}" in listing
        # The values are listed so the operator can recognise the file they just
        # calibrated with -- the SDK records no provenance at all.
        assert "rad_to_mm" in listing
        assert "closed +0.1140" in listing
        assert "mst_id 0x18" in listing, "ID 要按十六进制打，和命令行/日志一个写法"
        assert "--calib" in listing, "没把列出来的路径怎么用说清楚"

    def test_no_candidates_is_still_a_successful_query(self, monkeypatch):
        """Nothing to list is not an error -- and the message still has to say
        what the run uses instead, or the empty list reads as "no calibration"."""
        self._candidates(monkeypatch, [])
        printed: list[str] = []
        assert _common.list_calibrations(out=printed.append) == 0
        listing = "\n".join(printed)
        assert "出厂标定" in listing
        assert "上位机" in listing, "没说标定文件是哪来的"

    def test_a_candidate_that_cannot_be_read_is_marked_not_omitted(
            self, tmp_path, monkeypatch):
        """A stale or hand-edited file in ``~/.litegrip`` must not turn the
        listing into a traceback -- it is exactly when the operator is hunting
        for the right file."""
        bad = tmp_path / "broken.json"
        bad.write_text("{ not json", encoding="utf-8")
        self._candidates(monkeypatch, [bad])
        printed: list[str] = []
        assert _common.list_calibrations(out=printed.append) == 0
        listing = "\n".join(printed)
        assert f"1) {bad}" in listing, "读不出的候选也得列出来"
        assert "读不出" in listing


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
    """Just the ``LiteGrip`` surface the ``_common`` helpers use."""

    def __init__(self, answering: bool = True) -> None:
        self.answering = answering
        self.polls: list[float] = []
        self.reads = 0
        self.frames: list[dict] = []

    def poll(self, timeout_s: float = 0.0) -> bool:
        self.polls.append(timeout_s)
        return self.answering

    def get_state(self, wait: bool = True):
        self.reads += 1
        return SimpleNamespace(position_rad=0.42)

    def send_mit_frame(self, q, kp, kd, dq=0.0, tau=0.0) -> bool:
        """Any control frame is recorded, so a read-only path can prove it sent
        none."""
        self.frames.append(dict(q=q, kp=kp, kd=kd, dq=dq, tau=tau))
        return True


class TestFreshState:
    """``fresh_state`` is the only sanctioned way to read the real position.

    It has exactly one gate: ``poll()``.  The SDK returns ``True`` only when a
    status frame **for our motor** was decoded during that call — it discards
    parameter-reply frames — so a ``True`` means the snapshot read immediately
    afterwards is the frame that just arrived.  Nothing else is checked, and
    nothing else has to be: the failure mode this path exists to stop is a
    cached ``MotorState._position = 0.0`` being commanded as if it were a
    measurement, and that cannot be what ``get_state`` returns right after a
    genuine frame.

    Do not add a second gate.  The previous version also consulted
    ``GripperState.has_data`` / ``is_stale`` / ``data_age_s``, which do not
    exist in the SDK these examples use (nexform-tech/litegrip-python).
    """

    def test_a_fresh_frame_returns_the_state(self):
        gripper = FakeGrip()
        state = _common.fresh_state(gripper, timeout_s=0.25)
        assert state is not None
        assert state.position_rad == 0.42
        assert gripper.polls == [0.25]

    def test_no_frame_returns_none_without_reading_the_cache(self):
        """A cache with nothing behind it must not be read at all — reading it
        is how ``0.0`` gets mistaken for a measurement."""
        gripper = FakeGrip(answering=False)
        assert _common.fresh_state(gripper) is None
        assert gripper.reads == 0, "等不到帧还去读了缓存"

    def test_the_poll_result_is_the_whole_verdict(self):
        """Whatever ``poll`` says is what comes back — no third opinion.

        A caller that wants to refuse a reading has to get ``None`` out of
        this, and the only thing that produces ``None`` is a failed poll.
        """
        assert _common.fresh_state(FakeGrip()) is not None
        assert _common.fresh_state(FakeGrip(answering=False)) is None

    def test_it_sends_no_control_frame_on_the_way(self):
        """Reading a position is not moving the motor: no MIT frame may go out
        while asking for one."""
        gripper = FakeGrip()
        _common.fresh_state(gripper)
        assert gripper.frames == []


def _sdk(*absent: str) -> SimpleNamespace:
    """A stand-in for the ``litegrip`` module, lacking the named members.

    Built fresh on every call out of exactly the members the check looks for:
    the check is a ``hasattr`` walk, so an absent one has to be genuinely
    absent, and deleting it off a shared class would leak into other tests.

    Two spellings come out of :data:`_common.REQUIRED_SDK_API`: ``Owner.attr``
    lands on a namespace named after the owner, and a bare name (``litegrip``'s
    module-level ``trajectory_dir``) lands on the module itself.
    """
    drop = set(absent)
    owners: dict[str, dict] = {}
    top: dict = {}
    for path, _why in _common.REQUIRED_SDK_API:
        if path in drop:
            continue
        owner, sep, attr = path.partition(".")
        if sep:
            owners.setdefault(owner, {})[attr] = True
        else:
            top[owner] = True
    return SimpleNamespace(**top,
                           **{name: SimpleNamespace(**members)
                              for name, members in owners.items()})


class TestSdkApiCheck:
    """The examples refuse to run on an SDK missing any public interface they
    call — loudly, at startup, naming the member and where to get one.

    All three hardware examples resolve the same checkout now, so there is one
    list.  The failure this stops is an ``AttributeError`` thrown from inside
    the frame loop, after the motor is already enabled.
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
        "LiteGrip.record_start",
        "LiteGrip.poll",
        "Trajectory.load",
        "trajectory_dir",
        "GripperState.position_rad",
    ])
    def test_one_missing_member_is_reported_alone(self, absent):
        assert _common.missing_sdk_api(_sdk(absent)) == [absent]

    def test_a_bare_module_level_name_is_checked(self):
        """``trajectory_dir`` has no dot in it, so the hasattr walk must not
        take it for an owner with an empty attribute — that would read as
        "always absent" and refuse every SDK."""
        assert "trajectory_dir" not in _common.missing_sdk_api(_sdk())
        assert "trajectory_dir" in _common.missing_sdk_api(_sdk("trajectory_dir"))

    def test_it_says_which_sdk_to_use(self):
        with pytest.raises(SystemExit) as excinfo:
            _common.check_sdk_api(_sdk("LiteGrip.record_start"))
        message = str(excinfo.value)
        assert "LiteGrip.record_start" in message
        assert "PyPI" in message, "没说明这个包不在 PyPI 上，用户会去 pip install"
        assert "LITEGRIP_SDK_DIR" in message
        assert "litegrip-python" in message
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


# ── CAN 接口：探测与拉起 ────────────────────────────────────────────────
#
# 样例里唯一会碰系统配置的一段，所以钉子要密。样本是 ``ip -details link show``
# 的**真输出**：前四份照抄上位机 litegrip-studio 的 selftest.py（从内核抓的原文，
# 连 tab 缩进和行尾空格都是 ip 自己的），后面几份是 2026-10-06 在这台机器上抓的。
IP_RAISED_CLASSIC = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)
IP_RAISED_FD = (
    "2: can0: <NOARP,UP,LOWER_UP> mtu 72 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
    "\t  dbitrate 2000000 dsample-point 0.800 \n"
    "\t  fd on fd-non-iso off\n"
)
IP_UNPLUGGED = 'Device "can0" does not exist.\n'
#: ``lo``：管理上 up，operstate 却是 UNKNOWN。所以只能看 ``<>`` 里的 UP，
#: 不能看后面那个 ``state UNKNOWN``。
IP_NOT_CAN = (
    "1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN "
    "mode DEFAULT group default qlen 1000\n"
    "    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00\n"
)
#: 本机 ``can0``，2026-10-06 抓的：健康，但 ``restart-ms 0``（bus-off 不自愈）。
IP_HERE_HEALTHY = (
    "66: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can state ERROR-ACTIVE restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
    "\t  tq 50 prop-seg 7 phase-seg1 7 phase-seg2 5 sjw 2\n"
)
#: 没配过就被 down 掉的接口。注意 ``<BERR-REPORTING>`` 夹在 ``can`` 和 ``state``
#: 中间——上位机那条 ``\bcan state (\S+)`` 在这里一个状态都读不到。
IP_DOWN = (
    "2: can0: <NOARP> mtu 16 qdisc noop state DOWN "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can <BERR-REPORTING> state STOPPED (berr-counter tx 0 rx 0) "
    "restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)
#: BUS-OFF：**标志位和比特率全都正常**，但一帧都发不出去。真机上撞到的就是这个
#: 形状——「Network is down」听起来像接口没起来，其实接口看着好好的。
IP_BUS_OFF = (
    "66: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP "
    "mode DEFAULT group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can state BUS-OFF restart-ms 0 \n"
    "\t  bitrate 1000000 sample-point 0.750 \n"
)


def _ip(argv, rc: int, text: str = ""):
    """一条给替身 ``subprocess.run`` 的规定答案。"""
    return tuple(argv), (rc, text)


def _ip_runner(*replies):
    """``subprocess.run`` 的替身：只回答准备好的命令。

    没准备到的命令直接报错——「多跑了一条命令」正是这些测试要拦住的事，让它变成
    一次失败而不是一次静默通过。调用记录留在 ``run.calls`` 里，是 ``(argv, kwargs)``。

    同一条命令写两遍（拉起前后各探测一次）按**顺序**依次回答，最后一个一直用下去。
    写成 ``dict`` 就会只剩最后一条——拉起前的探测直接读到「已经好了」，于是一条命令
    都不跑，测试还「通过」了。
    """
    answers: dict = {}
    for key, value in replies:
        answers.setdefault(key, []).append(value)
    calls = []

    def run(argv, **kwargs):
        calls.append((list(argv), kwargs))
        key = tuple(argv[1:] if argv and argv[0] == "sudo" else argv)
        if key not in answers:
            raise AssertionError(f"没准备这条命令：{argv}")
        queue = answers[key]
        rc, text = queue.pop(0) if len(queue) > 1 else queue[0]
        return SimpleNamespace(returncode=rc, stdout=text, stderr="")

    run.calls = calls
    return run


def _probe(channel="can0", text=IP_HERE_HEALTHY, rc=0):
    return _ip(["ip", "-details", "link", "show", channel], rc, text)


def _sudo(calls):
    """记录里的特权命令（argv 列表）。"""
    return [argv for argv, _ in calls if argv and argv[0] == "sudo"]


def _link_probe(text, channel="can0"):
    return _ip_runner(_probe(channel, text))


def _repair_ok(from_text=IP_DOWN):
    """从 ``from_text`` 这个状态开始、三条命令都成功的替身。"""
    return _ip_runner(
        _probe(text=from_text),
        _ip(["ip", "link", "set", "can0", "down"], 0),
        _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
             "1000000", "restart-ms", "100", "fd off"], 0),
        _ip(["ip", "link", "set", "can0", "up"], 0),
        _probe(text=IP_HERE_HEALTHY),
    )


class TestReadingTheCanLink:
    """``ip -details link show`` 的输出长什么样，只有 ``parse_can_link`` 知道——
    所以每一份真输出都在这里钉一遍。"""

    def test_a_healthy_classic_interface_is_ready(self):
        state = _common.parse_can_link(IP_HERE_HEALTHY)
        assert (state.exists, state.up, state.is_can) == (True, True, True)
        assert state.bitrate == 1_000_000
        assert (state.fd, state.can_state) == (False, "ERROR-ACTIVE")
        assert state.ready(1_000_000)

    def test_a_bitrate_mismatch_is_not_ready(self):
        assert not _common.parse_can_link(IP_HERE_HEALTHY).ready(500_000)

    def test_a_missing_interface_is_not_an_interface(self):
        state = _common.parse_can_link(IP_UNPLUGGED, 1)
        assert not state.exists
        assert not state.ready(1_000_000)
        assert state.describe() == "不存在"

    def test_a_nonzero_exit_is_not_an_interface_either(self):
        assert not _common.parse_can_link("", 2).exists

    def test_loopback_is_not_a_can_interface(self):
        """``lo`` 是唯一会把两个坑一次踩全的那种接口：``state UNKNOWN`` 不能读成
        「没 up」（那就成了子串判断），而它没有 ``link/can``，也就不是 CAN。"""
        state = _common.parse_can_link(IP_NOT_CAN)
        assert state.up, "把 operstate 的 UNKNOWN 当成「没起来」了"
        assert not state.is_can
        assert state.describe() == "不是 CAN 接口"
        assert not state.ready(1_000_000)

    def test_can_fd_is_read_as_the_nominal_bitrate(self):
        """FD 的 ``dbitrate 2000000`` 不是标称比特率，``bitrate 1000000`` 才是。"""
        state = _common.parse_can_link(IP_RAISED_FD)
        assert state.bitrate == 1_000_000
        assert state.fd
        assert not state.ready(1_000_000), "FD 不该被判成「已经对了」"

    def test_a_down_interface_reads_its_controller_state(self):
        state = _common.parse_can_link(IP_DOWN)
        assert not state.up
        assert state.is_can
        # ``can <BERR-REPORTING> state STOPPED``：上位机那条 ``\bcan state`` 正则
        # 在这里读不到任何状态，读不到就分不出 STOPPED 和 BUS-OFF。
        assert state.can_state == "STOPPED"
        assert not state.ready(1_000_000)

    def test_a_down_interface_that_still_has_a_carrier_is_not_up(self):
        """``<NOARP,LOWER_UP>``：管理上没起来，载波却在——``LOWER_UP`` 里也有 "UP"，
        所以这里不能做子串判断。判错了就会认为接口「已经好了」、什么都不做，然后第一
        帧发不出去。"""
        state = _common.parse_can_link(IP_DOWN.replace("<NOARP>", "<NOARP,LOWER_UP>"))
        assert not state.up
        assert not state.ready(1_000_000)

    def test_bus_off_is_the_state_that_lies(self):
        """**这次修复的关键一条。** BUS-OFF 的接口标志位是 ``UP,LOWER_UP``、比特率
        也是对的，光看标志位会判成「已经对了」、一条命令都不跑——然后第一帧发不出去，
        报出来的还是「夹爪可能未上电」。"""
        state = _common.parse_can_link(IP_BUS_OFF)
        assert state.up and state.bitrate == 1_000_000, "样本没构造对"
        assert state.deaf
        assert not state.ready(1_000_000)
        assert "BUS-OFF" in state.describe()

    def test_error_passive_is_reported_but_not_repaired(self):
        """总线边际时控制器会掉进 ERROR-PASSIVE，它自己会恢复：报告，但不改。"""
        text = IP_HERE_HEALTHY.replace("ERROR-ACTIVE", "ERROR-PASSIVE")
        state = _common.parse_can_link(text)
        assert state.ready(1_000_000)
        assert "ERROR-PASSIVE" in state.describe()

    def test_a_healthy_state_says_nothing_extra(self):
        assert _common.parse_can_link(IP_HERE_HEALTHY).describe() == (
            "已 up，经典 CAN，比特率 1000000")


class TestProbingTheCanLink:
    def test_it_asks_ip_and_nothing_else(self):
        probe = _link_probe(IP_HERE_HEALTHY)
        assert _common.probe_can_link("can0", run=probe).ready(1_000_000)
        assert [argv for argv, _ in probe.calls] == [
            ["ip", "-details", "link", "show", "can0"]]

    def test_it_pins_the_locale(self):
        """后面要按 ``does not exist`` 这种英文串分支，别让 locale 改掉它。"""
        probe = _link_probe(IP_HERE_HEALTHY)
        _common.probe_can_link("can0", run=probe)
        assert probe.calls[0][1]["env"]["LC_ALL"] == "C"

    def test_a_name_that_is_not_a_device_name_is_never_run(self):
        for bad in ("can0; rm -rf /", "", "$(whoami)", "can 0", "x" * 20):
            probe = _link_probe(IP_HERE_HEALTHY)
            assert _common.probe_can_link(bad, run=probe) is None
            assert probe.calls == [], f"{bad!r} 竟然进了命令"

    def test_a_probe_that_cannot_run_is_not_a_verdict(self):
        """没有 ``ip``、命令超时：返回 ``None``（没结论），不是「接口有病」。"""
        def missing(argv, **kwargs):
            raise FileNotFoundError("ip")
        assert _common.probe_can_link("can0", run=missing) is None


class TestBringingTheCanLinkUp:
    """照上位机的三条规则：便利不是闸门、只改真正不对的状态、失败时说清楚。"""

    def test_a_ready_interface_runs_no_command_at_all(self, monkeypatch):
        """「少改」的钉子：接口已经对了，就一条特权命令都不跑，**也不弹密码**。"""
        _interactive(monkeypatch, True)
        probe = _link_probe(IP_HERE_HEALTHY)
        assert _common.ensure_can_link("can0", run=probe) is True
        assert _sudo(probe.calls) == []
        assert len(probe.calls) == 1, "只该探测一次"

    def test_a_down_interface_goes_down_configure_up(self, monkeypatch):
        """三条命令、按这个顺序，多一条都没有。"""
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        assert _common.ensure_can_link("can0", run=runner) is True
        assert _sudo(runner.calls) == [
            ["sudo", "ip", "link", "set", "can0", "down"],
            ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate",
             "1000000", "restart-ms", "100", "fd off"],
            ["sudo", "ip", "link", "set", "can0", "up"],
        ]

    def test_it_never_goes_through_a_shell(self, monkeypatch):
        """命令是 list argv，接口名是**其中一个参数**——拼成一行再交给 shell，名字里
        有任何东西都会被解释。（``fd off`` 那个空格是 ``ip`` 自己的选项写法，必须连着
        写成一个参数。）"""
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        _common.ensure_can_link("can0", run=runner)
        for argv, kwargs in runner.calls:
            assert isinstance(argv, list), f"命令不是 list argv：{argv}"
            assert "shell" not in kwargs, f"{argv} 走了 shell"
            assert not any(" " in a for a in argv if a != "fd off"), argv

    def test_an_fd_interface_is_reported_and_left_alone(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_RAISED_FD)
        assert _common.ensure_can_link("can0", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "CAN FD" in "\n".join(lines)

    def test_a_non_can_interface_is_left_alone(self, monkeypatch):
        """``--channel lo`` 也不会被 down/up——那是参数打错了，不是接口不对。"""
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_NOT_CAN, "lo")
        assert _common.ensure_can_link("lo", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "不是 CAN 接口" in "\n".join(lines)

    def test_a_missing_device_never_reaches_a_privileged_command(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _ip_runner(_probe("can0", IP_UNPLUGGED, 1))
        assert _common.ensure_can_link("can0", run=probe, out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "读不到 can0" in "\n".join(lines)

    def test_a_restart_ms_rejection_retries_only_that_command(self, monkeypatch):
        """台架这块 gs_usb 克隆不认 ``restart-ms``。原样重试 configure 会把接口留在
        down（比原来更糟），所以只去掉这一项、只重试这一条，``up`` 照跑。"""
        _interactive(monkeypatch, True)
        runner = _ip_runner(
            _probe(text=IP_DOWN),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 1,
                "Error: argument \"restart-ms\" is wrong\n"),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "fd off"], 0),
            _ip(["ip", "link", "set", "can0", "up"], 0),
            _probe(text=IP_HERE_HEALTHY),
        )
        assert _common.ensure_can_link("can0", run=runner, out=lambda *_: None)
        sudo = _sudo(runner.calls)
        assert "restart-ms" not in sudo[-2], sudo[-2]
        assert sudo[-1] == ["sudo", "ip", "link", "set", "can0", "up"]

    def test_any_other_failure_is_not_retried(self, monkeypatch):
        """只有名字里有 restart 的失败才重试。别的失败一并重试，会把偶发错误变成
        静默降级——接口是起来了，可再也不自动从 bus-off 恢复。"""
        _interactive(monkeypatch, True)
        lines = []
        runner = _ip_runner(
            _probe(text=IP_DOWN),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 1,
                "RTNETLINK answers: Operation not supported\n"),
        )
        assert _common.ensure_can_link("can0", run=runner,
                                       out=lines.append) is False
        assert len(_sudo(runner.calls)) == 2, "失败之后还往下跑了"
        assert "sudo ip link set can0 up" in "\n".join(lines)

    def test_a_repair_that_did_not_work_says_so(self, monkeypatch):
        """三条命令都成功、复查却还是不对：不能说「已就绪」。"""
        _interactive(monkeypatch, True)
        lines = []
        runner = _ip_runner(
            _probe(text=IP_BUS_OFF),
            _ip(["ip", "link", "set", "can0", "down"], 0),
            _ip(["ip", "link", "set", "can0", "type", "can", "bitrate",
                 "1000000", "restart-ms", "100", "fd off"], 0),
            _ip(["ip", "link", "set", "can0", "up"], 0),
            _probe(text=IP_BUS_OFF),
        )
        assert _common.ensure_can_link("can0", run=runner, out=lines.append) is False
        assert "复查仍然不对" in "\n".join(lines)

    def test_bus_off_is_named_before_the_repair(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        runner = _repair_ok(from_text=IP_BUS_OFF)
        _common.ensure_can_link("can0", run=runner, out=lines.append)
        assert "BUS-OFF" in "\n".join(lines), "没告诉操作员这一帧都发不出去是为什么"

    def test_repair_false_only_probes(self, monkeypatch):
        """05 的 ``--status`` / 04 的 ``--passive`` 走这条：只看不动。替它们把接口
        改掉，恰好把 ``--status`` 要诊断的东西抹了。"""
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", repair=False, run=probe,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "sudo ip link set can0 down" in "\n".join(lines)

    def test_a_non_interactive_stdin_only_prints(self, monkeypatch):
        """CI / 管道里 sudo 要不到密码，会一直挂着。"""
        _interactive(monkeypatch, False)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", run=probe,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "非交互" in "\n".join(lines)

    def test_without_sudo_it_prints_the_command(self, monkeypatch):
        _interactive(monkeypatch, True)
        lines = []
        probe = _link_probe(IP_DOWN)
        assert _common.ensure_can_link("can0", run=probe, which=lambda _: None,
                                       out=lines.append) is False
        assert _sudo(probe.calls) == []
        assert "sudo ip link set can0 up" in "\n".join(lines)

    def test_the_printed_commands_are_the_ones_it_would_have_run(self, monkeypatch):
        """打印出来备用的命令，要和真跑的那三条一模一样——否则操作员照着敲，
        敲出来的是另一件事。"""
        hint = _common.manual_can_hint("can0")
        _interactive(monkeypatch, True)
        runner = _repair_ok()
        _common.ensure_can_link("can0", run=runner)
        for argv in _sudo(runner.calls):
            assert " ".join(argv) in hint, f"{argv} 不在打印出来的命令里"


class TestDiagnosingAFailedEnable:
    """真机上撞到的那个错：``使能失败: [Errno 100] Network is down`` 被译成
    「夹爪可能处于错误状态或未上电」。它不是夹爪的问题。"""

    def test_errno_100_is_the_host_link_not_the_gripper(self):
        probe = _link_probe(IP_DOWN)
        text = _common.can_link_failure(
            OSError(100, "Network is down"), "can0", run=probe)
        assert "不是夹爪" in text
        assert "sudo ip link set can0 down" in text
        assert "未 up" in text

    def test_a_bus_off_interface_is_named_in_the_diagnosis(self):
        probe = _link_probe(IP_BUS_OFF)
        text = _common.can_link_failure(
            OSError(100, "Network is down"), "can0", run=probe)
        assert "BUS-OFF" in text

    def test_other_errors_are_left_alone(self):
        """不是链路错就别抢话：``None`` 让调用方照原样报。"""
        for exc in (ValueError("boom"), OSError(13, "Permission denied"),
                    RuntimeError("使能超时")):
            assert _common.can_link_failure(exc, "can0") is None

    def test_the_connect_message_does_not_blame_the_link_state(self):
        """``connect()`` 只建 socket 和 bind，这两步在没 up 的接口上也成功——所以
        这里的检查项里不该出现「接口没起来」。"""
        text = _common.connect_failure_message("can0")
        assert "bind" in text
        assert "ip -details link show can0" in text

    def test_a_bad_link_is_named_before_the_gripper(self):
        probe = _link_probe(IP_BUS_OFF)
        text = _common.enable_failure_message("can0", run=probe)
        assert "先修链路" in text
        assert "新上电" not in text and "上电" not in text

    def test_a_good_link_keeps_the_gripper_wording(self):
        probe = _link_probe(IP_HERE_HEALTHY)
        text = _common.enable_failure_message("can0", run=probe)
        assert "夹爪可能处于错误状态或未上电" in text
        assert "已 up" in text, "没说清接口这时是什么状态"


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
        # ``None`` means "not given on the command line".  It is not a path:
        # ``choose_calibration_file`` turns it into the SDK's factory
        # calibration, and stops when even that cannot be read.
        assert args.calib is None
        assert args.list_calibrations is False
        assert args.no_can_setup is False

    def test_the_can_setup_can_be_turned_off(self):
        """逃生口：接口自己管。名字里带 ``no``，默认必须是「会探测」。"""
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args([]).no_can_setup is False
        assert parser.parse_args(["--no-can-setup"]).no_can_setup is True

    def test_the_calib_flag_documents_the_factory_default(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        help_text = parser.format_help()
        assert "--calib" in help_text
        assert "litegrip-studio" in help_text, "没说标定文件从哪来"
        assert "出厂标定" in help_text, "没说默认用哪份"
        # 找路径的唯一工具也得在 --help 里，否则只剩「--calib 指一份」这句话。
        assert "--list-calibrations" in help_text

    def test_hardware_args_accept_hex_and_decimal(self):
        parser = argparse.ArgumentParser()
        _common.add_hardware_args(parser)
        assert parser.parse_args(["--can-id", "0x0A"]).can_id == 10
        assert parser.parse_args(["--can-id", "10"]).can_id == 10
        assert parser.parse_args(["--mst-id", "0x20"]).mst_id == 0x20

    def test_safety_banner_warns_about_real_motion(self):
        assert "真机" in _common.SAFETY_BANNER
        assert "Esc" in _common.SAFETY_BANNER


class TestFactoryCalibrationPath:
    """The factory default has to resolve through the package, never through a
    path baked into this repository.

    A hard-coded ``/home/<someone>/...`` would work on the machine it was
    written on and nowhere else — a different laptop, a different virtualenv or
    a different checkout all move the file.  Resolving off ``litegrip.__file__``
    is what makes "the default is the SDK's own factory calibration" true
    everywhere.
    """

    def test_it_follows_the_package_directory(self, tmp_path):
        package = tmp_path / "site-packages" / "litegrip"
        package.mkdir(parents=True)
        (package / "__init__.py").write_text("", encoding="utf-8")
        sdk = SimpleNamespace(**{"__file__": str(package / "__init__.py")})
        found = _common.factory_calibration_path(sdk)
        assert found == package / "factory_calibration.json"
        assert tmp_path in found.parents, "跑到包目录外面去了"

    def test_the_real_sdk_ships_one(self):
        """Whatever SDK the examples would actually load has to carry the file
        the default points at -- otherwise every run without ``--calib`` stops,
        and the default is a lie."""
        try:
            litegrip = _common.import_litegrip()
        except SystemExit as exc:
            pytest.skip(f"这台机器上没有真机 SDK：{exc}")
        path = _common.factory_calibration_path(litegrip)
        assert path.is_file(), f"{path} 不存在"
        calib = _common.read_calibration_file(path)
        assert float(calib["rad_to_mm"]) > 0


class TestSdkDiscovery:
    """One checkout, one variable, for all three hardware examples.

    ``litegrip`` also exists as an editable install on this machine, from a
    different repository that has no trajectory API — and both report
    ``__version__ 2.2.0``, so the version number cannot tell them apart.  The
    sibling checkout therefore outranks the installed one: it is right there,
    its name says which repository it is, and the name the examples want is
    ``litegrip-python``.
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

    def test_env_var_wins(self, monkeypatch, tmp_path):
        monkeypatch.setenv("LITEGRIP_SDK_DIR", str(tmp_path))
        assert _common.sdk_dir() == tmp_path

    def test_discovers_a_real_sdk_if_one_is_around(self, monkeypatch):
        """When nothing overrides it, whatever comes back must actually exist."""
        monkeypatch.delenv("LITEGRIP_SDK_DIR", raising=False)
        found = _common.sdk_dir()
        assert found is None or Path(found).exists()

    def test_a_discovered_checkout_holds_the_package(self, monkeypatch):
        """Whatever comes back has to be usable: a directory without
        ``litegrip/__init__.py`` in it would send the import back to the
        installed package."""
        monkeypatch.delenv("LITEGRIP_SDK_DIR", raising=False)
        found = _common.sdk_dir()
        assert found is None or (Path(found) / "litegrip" / "__init__.py").is_file()

    def test_the_sibling_checkout_outranks_the_installed_package(self, monkeypatch,
                                                                tmp_path):
        """The order is the point: on this machine ``litegrip`` is installed
        editable from a repository that has no trajectory API, so falling back
        to "whatever imports" would point example 03 at the wrong package."""
        sibling = tmp_path / "litegrip-python"
        (sibling / "src" / "litegrip").mkdir(parents=True)
        (sibling / "src" / "litegrip" / "__init__.py").write_text(
            "", encoding="utf-8")
        monkeypatch.delenv("LITEGRIP_SDK_DIR", raising=False)
        monkeypatch.setattr(_common, "_REPO_ROOT", tmp_path / "repo")
        assert _common.sdk_dir() == sibling / "src"

    def test_it_loads_the_package_in_the_given_directory(self, throwaway,
                                                         clean_litegrip):
        """``pip install -e`` registers a meta path finder above ``sys.path``,
        so inserting the directory cannot win — the package has to be loaded
        explicitly, which is why :func:`_common._load_package_from` exists."""
        module = _common._load_package_from(throwaway)
        assert module is not None
        assert module.MARKER == "loaded from the directory"
        assert sys.modules["litegrip"] is module, "加载完没挂到 sys.modules 上"

    def test_it_returns_none_when_there_is_no_package(self, tmp_path,
                                                      clean_litegrip):
        assert _common._load_package_from(tmp_path) is None

    def test_the_env_var_is_what_the_import_follows(self, monkeypatch, throwaway,
                                                    clean_litegrip):
        """End to end: the variable the examples document is the one the loader
        obeys, even on a machine where the other SDK is installed and
        importable."""
        monkeypatch.setenv("LITEGRIP_SDK_DIR", str(throwaway))
        assert _common.import_litegrip().MARKER == "loaded from the directory"

    def test_a_broken_checkout_is_reported_rather_than_skipped(self, monkeypatch,
                                                               tmp_path,
                                                               clean_litegrip):
        """A checkout that fails to load must say so.  Falling through to the
        installed SDK would run the examples against the wrong package and
        blame it for the missing API."""
        package = tmp_path / "litegrip"
        package.mkdir()
        (package / "__init__.py").write_text("this is not python(\n",
                                             encoding="utf-8")
        monkeypatch.setenv("LITEGRIP_SDK_DIR", str(tmp_path))
        with pytest.raises(SystemExit) as excinfo:
            _common.import_litegrip()
        assert str(tmp_path) in str(excinfo.value)

    def test_bootstrap_makes_the_library_importable(self):
        _common.bootstrap_src()
        import litegrip_pybullet

        assert Path(litegrip_pybullet.__file__).is_file()
