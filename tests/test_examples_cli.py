# -*- coding: utf-8 -*-
"""The three examples as programs: argument parsing, exit codes, startup paths.

Nothing here touches can0.  The only real-hardware paths exercised are the ones
that are *supposed* to fail — a CAN interface that does not exist — so the
suite is safe to run on a machine with a gripper attached.
"""

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


def run(script: Path, *args: str) -> subprocess.CompletedProcess:
    """Run an example the way a user would: as a script, from the repo root."""
    env = {**os.environ, "LITEGRIP_PYBULLET_REEXEC": "1"}  # no re-exec under test
    return subprocess.run(
        [sys.executable, str(script), *args],
        cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=TIMEOUT_S,
        env=env,
    )


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
    def missing_interface(self) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_02, "--status", "--channel", NOWHERE)
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

    def test_a_future_duration_is_announced_as_a_rate_limit(self):
        """``--duration`` is a floor on speed, not a promise to go faster."""
        assert "mm/s" in run(EXAMPLE_02, "--help").stdout


class TestExample03WithoutHardware:
    """03 imports the SDK before it looks at the CAN interface.

    Without the SDK installed it stops earlier — with a different, equally valid
    message — so these two need it present to test what they claim to.
    """

    @pytest.fixture
    def missing_interface(self) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1")
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
        assert "--passive" in stdout
        assert "一帧都不发" in stdout
        # ...and it warns that running it alone leaves nobody feeding the motor
        assert "通信超时" in stdout

    def test_passive_does_not_change_how_far_it_gets(self):
        """Suppressing the sends must not short-circuit connecting or the error.

        Compared against the same run *without* ``--passive`` rather than against
        a fixed string, because how far 03 gets depends on whether the SDK is
        installed: with it the run reaches the CAN interface, without it it
        stops earlier. Both are correct; ``--passive`` must not alter either.
        """
        plain = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1")
        passive = run(EXAMPLE_03, "--passive", "--channel", NOWHERE,
                      "--duration", "1")
        assert plain.returncode == 1
        assert passive.returncode == plain.returncode
        if "找不到真机 SDK" in output_of(plain):
            assert "找不到真机 SDK" in output_of(passive)
        else:
            assert NOWHERE in output_of(passive)

    def test_without_the_sdk_it_says_how_to_get_it(self):
        """The SDK-absent path is worth covering too — CI is exactly that case."""
        result = run(EXAMPLE_03, "--channel", NOWHERE, "--duration", "1")
        text = output_of(result)
        assert result.returncode == 1
        if "找不到真机 SDK" in text:
            assert "pip install litegrip" in text
        else:
            assert NOWHERE in text  # SDK present: it got as far as the interface
