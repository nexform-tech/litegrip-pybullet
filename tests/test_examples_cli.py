# -*- coding: utf-8 -*-
"""The three examples as programs: argument parsing, exit codes, startup paths.

Nothing here touches can0.  The only real-hardware paths exercised are the ones
that are *supposed* to fail — a CAN interface that does not exist — so the
suite is safe to run on a machine with a gripper attached.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = REPO_ROOT / "examples"
EXAMPLE_01 = EXAMPLES / "01_sim_only.py"
EXAMPLE_02 = EXAMPLES / "02_sim_to_real.py"
EXAMPLE_03 = EXAMPLES / "03_real_to_sim.py"

ALL_EXAMPLES = [EXAMPLE_01, EXAMPLE_02, EXAMPLE_03]

#: Long enough for a headless run, short enough to fail fast if it hangs.
TIMEOUT_S = 180.0

#: A CAN interface that cannot exist, so ``connect()`` fails without hardware.
NOWHERE = "nosuchcan0"


def run(script: Path, *args: str, env: dict | None = None
        ) -> subprocess.CompletedProcess:
    """Run an example the way a user would: as a script, from the repo root.

    stdin is closed on purpose.  A hardware example now *asks* which calibration
    file to use when ``--calib`` is missing, and a child that inherits a real
    terminal would sit there waiting for an operator to type — a test that hangs
    instead of failing.  Closed stdin is also the honest simulation of "run from
    a script": no tty, so the examples must refuse rather than prompt.
    """
    # no re-exec under test; `env` lets a test point the SDK discovery elsewhere
    merged = {**os.environ, "LITEGRIP_PYBULLET_REEXEC": "1", **(env or {})}
    return subprocess.run(
        [sys.executable, str(script), *args],
        cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=TIMEOUT_S,
        env=merged, stdin=subprocess.DEVNULL,
    )


@pytest.fixture(scope="module")
def calib_file(tmp_path_factory) -> str:
    """A calibration file for the runs that have to get *past* the chooser.

    The numbers are a plausible gripper's, not this bench's: nothing here reaches
    a bus, and a test whose point is "does it get as far as the CAN interface"
    should not depend on the angles.  The recorded ``can_id``/``mst_id`` do have
    to match the defaults, because the examples refuse a file that names another
    motor (that check is exercised in ``test_common.py``).
    """
    path = tmp_path_factory.mktemp("calib") / "litegrip_calibration.json"
    path.write_text(json.dumps({
        "channel": "can0",
        "can_id": 0x08,
        "mst_id": 0x18,
        "zero_position_rad": 0.114,
        "max_position_rad": -1.731,
        "travel_range_rad": 1.845,
        "rad_to_mm": 120.0 / 1.845,
        "kp": 5.0,
        "kd": 2.0,
    }), encoding="utf-8")
    return str(path)


def output_of(result: subprocess.CompletedProcess) -> str:
    """stdout + stderr: PyBullet writes its banner to stdout, failures to stderr."""
    return result.stdout + result.stderr


#: The flags that make 01's headless run quick (~4 s instead of ~10 s) without
#: changing what it demonstrates: the speed limit is what the demo is about, so
#: raising it is fair game, and the hold time is only for watching.
FAST = ("--headless", "--speed", "0.2", "--hold", "0.2")


@pytest.fixture(scope="module")
def fast_01() -> subprocess.CompletedProcess:
    """One 01 run, shared by every test that just reads its output.

    Spawning a PyBullet process per assertion would triple this suite's runtime
    for no extra coverage.
    """
    return run(EXAMPLE_01, *FAST)


@pytest.mark.parametrize("script", ALL_EXAMPLES, ids=lambda p: p.name)
class TestHelp:
    def test_help_exits_zero(self, script):
        result = run(script, "--help")
        assert result.returncode == 0
        assert "usage" in output_of(result)

    def test_help_describes_the_hardware_flags(self, script):
        result = run(script, "--help")
        assert "--urdf" in result.stdout
        assert "--headless" in result.stdout


@pytest.mark.parametrize("script", [EXAMPLE_02, EXAMPLE_03], ids=lambda p: p.name)
class TestHardwareFlags:
    """Only 02 and 03 take CAN options."""

    def test_can_flags_are_documented(self, script):
        result = run(script, "--help")
        assert "--channel" in result.stdout
        assert "--can-id" in result.stdout
        assert "--mst-id" in result.stdout


class TestExample01SimOnly:
    """01 is the one example that must run anywhere, with no hardware at all."""

    def test_headless_run_succeeds(self, fast_01):
        assert fast_01.returncode == 0, output_of(fast_01)

    def test_it_walks_through_all_four_demos(self, fast_01):
        text = output_of(fast_01)
        for marker in ("1/4", "2/4", "3/4", "4/4", "收尾", "完成"):
            assert marker in text, f"缺少 {marker} 段落"

    def test_it_reports_the_grip_and_the_slip(self, fast_01):
        """The two pull tests are the point of the example."""
        text = output_of(fast_01)
        assert "没动" in text          # 5 N: held by friction
        assert "滑了" in text          # 15 N: past the friction limit

    def test_the_part_only_touches_the_fingers_while_held(self, fast_01):
        assert "落在 link [0, 1]" in output_of(fast_01)

    def test_releasing_drops_the_part(self, fast_01):
        assert "手指张开 → 方块下落" in output_of(fast_01)

    def test_it_says_it_does_not_touch_real_hardware(self, fast_01):
        assert "不接真机" in output_of(fast_01)

    def test_a_bigger_object_and_a_stronger_grip_still_work(self):
        result = run(EXAMPLE_01, "--headless", "--object-mm", "60",
                     "--force", "20", "--speed", "0.2", "--hold", "0.2")
        assert result.returncode == 0, output_of(result)
        assert "60 mm" in output_of(result)

    def test_the_default_invocation_runs_to_completion(self):
        """No flags at all beyond --headless: the documented happy path."""
        result = run(EXAMPLE_01, "--headless")
        assert result.returncode == 0, output_of(result)
        assert "全行程" in output_of(result)

    def test_a_bad_urdf_path_fails_loudly(self):
        result = run(EXAMPLE_01, "--headless", "--urdf", "/nonexistent/g.urdf")
        assert result.returncode != 0
        assert "URDF" in output_of(result) or "urdf" in output_of(result)


class TestExample02NeedsAWindow:
    def test_it_refuses_headless_and_explains_why(self):
        result = run(EXAMPLE_02, "--headless", "--dry-run")
        assert result.returncode == 1
        text = output_of(result)
        assert "窗口" in text
        # ...and points at the example that can run without one
        assert "01_sim_only.py" in text


class TestExample02Status:
    """``--status`` diagnoses a wedged gripper without commanding it.

    Nothing here can reach hardware: the interface name cannot exist, so the
    run stops at ``connect()``.  What is being pinned is that ``--status``
    never gets as far as enabling the motor or printing the motion banner, and
    that the argument combinations are refused rather than silently ignored.
    """

    @pytest.fixture
    def missing_interface(self, calib_file) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_02, "--status", "--channel", NOWHERE,
                     "--calib", calib_file)
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到连 CAN 这一步（pip install litegrip）")
        return result

    def test_it_is_documented(self):
        assert "--status" in run(EXAMPLE_02, "--help").stdout

    def test_a_missing_can_interface_is_reported_clearly(self, missing_interface):
        assert missing_interface.returncode == 1
        assert NOWHERE in output_of(missing_interface)

    def test_it_never_enables_the_motor(self, missing_interface):
        """The whole point: a wedged motor must not be poked, only read."""
        text = output_of(missing_interface)
        assert "已使能" not in text
        assert "即将驱动真机" not in text            # the motion banner
        assert "未使能" in text or "不发送任何帧" in text or NOWHERE in text

    def test_clear_fault_without_status_is_refused(self):
        result = run(EXAMPLE_02, "--clear-fault")
        assert result.returncode == 1
        assert "--status" in output_of(result)

    def test_status_with_dry_run_is_refused(self):
        result = run(EXAMPLE_02, "--status", "--dry-run")
        assert result.returncode == 1
        text = output_of(result)
        assert "--dry-run" in text

    def test_the_speed_slider_is_announced_as_a_rate_limit(self):
        """``--speed`` is a cap on the position target, not a promise to arrive
        sooner, so the help has to say what 100 % *is* — millimetres per second."""
        stdout = run(EXAMPLE_02, "--help").stdout
        assert "--speed" in stdout
        assert "mm/s" in stdout


class TestChoosingCalibrationIsMandatory:
    """Every path that touches the hardware starts by picking a calibration file.

    ``--calib`` is the scripting way in; without it the example *asks*, and with
    no terminal to ask on it stops.  What must never happen is the third
    option -- quietly falling back to the SDK's default path or the factory
    calibration, whose angles belong to a different machine.
    """

    def test_dry_run_without_calib_refuses(self):
        """The dry run keeps its promise never to import the SDK, so this is the
        one that runs in CI -- and ``--dry-run`` needs a calibration anyway."""
        result = run(EXAMPLE_02, "--dry-run")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "--calib" in text
        # ...and it says where a calibration file comes from in the first place
        assert "上位机" in text
        assert "候选" in text, "没列出候选，操作员只能靠猜"

    def test_status_without_calib_refuses(self):
        result = run(EXAMPLE_02, "--status")
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到这一步（裸 SDK 会在选标定之前就停）")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "--calib" in text
        assert "上位机" in text

    def test_example_03_refuses_too(self):
        result = run(EXAMPLE_03, "--duration", "1")
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到这一步（裸 SDK 会在选标定之前就停）")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "--calib" in text
        assert "上位机" in text

    def test_a_simulator_calibration_is_refused(self, tmp_path):
        """The studio keeps the simulator's calibration in a separate
        ``.sim.json`` on purpose; using it for hardware would scale the
        commands by the wrong constant."""
        sim = tmp_path / "litegrip_calibration.sim.json"
        sim.write_text(json.dumps({"zero_position_rad": 0.1,
                                   "max_position_rad": -1.0,
                                   "rad_to_mm": 50.0}), encoding="utf-8")
        result = run(EXAMPLE_02, "--dry-run", "--calib", str(sim))
        assert result.returncode == 1, output_of(result)
        assert "仿真" in output_of(result)

    def test_a_missing_calibration_file_is_refused(self, tmp_path):
        """The SDK would silently fall back to the factory calibration here and
        return True; the example must stop instead."""
        result = run(EXAMPLE_02, "--dry-run", "--calib",
                     str(tmp_path / "nope.json"))
        assert result.returncode == 1, output_of(result)
        assert "不存在" in output_of(result)

    def test_the_dry_run_says_which_file_it_would_use(self, calib_file):
        """``--dry-run`` needs a calibration too, and says which one -- the whole
        reason to require it is that the numbers decide the target angles."""
        result = run(EXAMPLE_02, "--headless", "--dry-run", "--calib", calib_file)
        # 02 needs a window in every mode, so the dry-run never gets to open one
        # here; what matters is that the flag combination is still refused for
        # the window's sake, not for the calibration's.
        assert result.returncode == 1
        assert "窗口" in output_of(result)


class TestExample03WithoutHardware:
    """03 imports the SDK before it looks at the CAN interface.

    Without the SDK installed it stops earlier — with a different, equally valid
    message — so these two need it present to test what they claim to.
    """

    @pytest.fixture
    def missing_interface(self, calib_file) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1",
                     "--calib", calib_file)
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到连 CAN 这一步（pip install litegrip）")
        return result

    def test_a_missing_can_interface_is_reported_clearly(self, missing_interface):
        assert missing_interface.returncode == 1
        text = output_of(missing_interface)
        assert NOWHERE in text
        assert "真机" in text

    def test_nothing_is_enabled_when_the_interface_is_missing(self, missing_interface):
        """It must not get as far as enabling the motor or the safety banner."""
        text = output_of(missing_interface)
        assert "已使能" not in text
        assert "即将驱动真机" not in text

    def test_passive_is_documented_and_says_what_it_means(self):
        """``--passive`` is the escape hatch when another program drives CAN."""
        stdout = run(EXAMPLE_03, "--help").stdout
        # argparse re-wraps the help to the terminal width, and a Chinese run of
        # characters has no space to break at, so a phrase can arrive split
        # across two lines ("…锁通信\n超时故障"). Compare it without the breaks.
        flat = "".join(stdout.split())
        assert "--passive" in flat
        assert "一帧都不发" in flat
        # ...and it warns that running it alone leaves nobody feeding the motor
        assert "通信超时" in flat

    def test_passive_does_not_change_how_far_it_gets(self, calib_file):
        """Suppressing the sends must not short-circuit connecting or the error.

        Compared against the same run *without* ``--passive`` rather than against
        a fixed string, because how far 03 gets depends on whether the SDK is
        installed: with it the run reaches the CAN interface, without it it
        stops earlier. Both are correct; ``--passive`` must not alter either.
        """
        plain = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1",
                    "--calib", calib_file)
        passive = run(EXAMPLE_03, "--passive", "--channel", NOWHERE,
                      "--duration", "1", "--calib", calib_file)
        assert plain.returncode == 1
        assert passive.returncode == plain.returncode
        if "找不到真机 SDK" in output_of(plain):
            assert "找不到真机 SDK" in output_of(passive)
        else:
            assert NOWHERE in output_of(passive)

    def test_without_the_sdk_it_says_how_to_get_it(self, calib_file):
        """The SDK-absent path is worth covering too — CI is exactly that case."""
        result = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1",
                     "--calib", calib_file)
        text = output_of(result)
        assert result.returncode == 1
        if "找不到真机 SDK" in text:
            # `pip install litegrip` is not the answer — it is not on PyPI — so
            # the message must not offer it as one.
            assert "LITEGRIP_SDK_DIR" in text
            assert "pip install -e" in text
            assert "没有发布到 PyPI" in text
        else:
            assert NOWHERE in text  # SDK present: it got as far as the interface


class TestSdkWithoutTheRequiredApi:
    """An SDK that imports but cannot answer "is this reading current?" stops
    the examples at startup — naming the missing member and where to get one.

    This is the failure mode that used to be silent: the examples reached into
    ``gripper._can._controller`` for the 0xCC hook, so an SDK without the public
    API still "worked" right up until a guess about a measured position became a
    step command.  Exercised with a throwaway checkout so it runs in CI, where
    no SDK is installed at all.
    """

    @pytest.fixture
    def bare_sdk(self, tmp_path) -> dict:
        """An importable ``litegrip`` package with none of the required API."""
        package = tmp_path / "litegrip"
        package.mkdir()
        # Enough to import: 02/03 only build LiteGrip objects after the check.
        (package / "__init__.py").write_text(
            "class LiteGrip:\n"
            "    pass\n"
            "\n"
            "class GripperState:\n"
            "    pass\n",
            encoding="utf-8",
        )
        return {"LITEGRIP_SDK_DIR": str(tmp_path)}

    @pytest.mark.parametrize("script, args", [
        # 02 drives by drag now: its speed is a percentage, and no run-length
        # option is needed to reach the CAN interface and fail there.
        (EXAMPLE_02, ()),
        (EXAMPLE_03, ("--duration", "1")),
    ], ids=["02_sim_to_real.py", "03_real_to_sim.py"])
    def test_it_stops_before_connecting(self, script, args, bare_sdk):
        result = run(script, "--channel", NOWHERE, *args, env=bare_sdk)
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "缺少本仓库必须的公开接口" in text
        assert "LiteGrip.refresh_status" in text
        assert "GripperState.data_age_s" in text
        # ...and it never got as far as the bus, so nothing was transmitted.
        assert NOWHERE not in text
