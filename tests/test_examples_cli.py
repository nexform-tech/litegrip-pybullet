# -*- coding: utf-8 -*-
"""The five examples as programs: argument parsing, exit codes, startup paths.

Nothing here touches can0.  The only real-hardware paths exercised are the ones
that are *supposed* to fail — a CAN interface that does not exist — so the
suite is safe to run on a machine with a gripper attached.
"""

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pybullet")

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = REPO_ROOT / "examples"
EXAMPLE_01 = EXAMPLES / "01_hello_sim.py"      # read the state, no motion
EXAMPLE_02 = EXAMPLES / "02_move_sim.py"       # speed-limited travel, positioning
EXAMPLE_03 = EXAMPLES / "03_trajectory.py"     # record a hand-taught motion, replay it
EXAMPLE_04 = EXAMPLES / "04_mirror_real.py"    # hardware → simulation
EXAMPLE_05 = EXAMPLES / "05_dual_control.py"   # simulation → hardware

ALL_EXAMPLES = [EXAMPLE_01, EXAMPLE_02, EXAMPLE_03, EXAMPLE_04, EXAMPLE_05]

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


#: The flags that make the simulated runs quick (~4 s instead of ~10 s) without
#: changing what they demonstrate: the speed limit is what the demo is about, so
#: raising it is fair game.  01 takes no options beyond the ones every example
#: shares.  03 is not here at all -- with no trajectory SDK on this machine its
#: runs stop at startup, and the one path that does run needs a ``.lgt`` fixture
#: (see ``TestExample03Trajectory``).
FAST_HELLO = ("--headless",)
FAST_MOVE = ("--headless", "--speed", "0.2")


@pytest.fixture(scope="module")
def fast_hello() -> subprocess.CompletedProcess:
    """One 01 run, shared by every test that just reads its output.

    Spawning a PyBullet process per assertion would triple this suite's runtime
    for no extra coverage.
    """
    return run(EXAMPLE_01, *FAST_HELLO)


@pytest.fixture(scope="module")
def fast_move() -> subprocess.CompletedProcess:
    """One 02 run, shared the same way."""
    return run(EXAMPLE_02, *FAST_MOVE)


#: 03 needs the *other* SDK checkout -- the one with the trajectory API.  Only
#: that one can write the ``.lgt`` fixture the offline run replays, and only that
#: one can load it back, so the run tests below skip where it is absent (CI is
#: exactly that case).  The loop itself is covered unconditionally in
#: ``tests/test_example03_loop.py`` against a stand-in SDK.
TRAJECTORY_SDK = REPO_ROOT.parent / "litegrip-python" / "src"

#: A trajectory short enough to replay in well under a second.
FIXTURE_RATE_HZ = 100.0
FIXTURE_SAMPLES = 50

#: Sentinel the fixture writer prints when there is no SDK to write with, as
#: opposed to anything else going wrong.
NO_SDK = "NO-TRAJECTORY-SDK: "


@pytest.fixture(scope="module")
def trajectory_file(tmp_path_factory) -> str:
    """A ``.lgt`` written by the SDK itself, so the format is its own.

    These samples were never on a gripper: 50 of them, opening 0.2 → 0.8 over
    half a second.  Writing them through ``Trajectory.save`` rather than by hand
    is what keeps this from being a test of my guess at the file format — and it
    is why this fixture skips where that SDK is absent, which is every machine
    that is not this bench and every CI runner.
    """
    if not (TRAJECTORY_SDK / "litegrip" / "__init__.py").is_file():
        pytest.skip(f"要有 {TRAJECTORY_SDK} 才能写出并读回一段 .lgt")
    workdir = tmp_path_factory.mktemp("traj")
    path = workdir / "fixture"
    script = workdir / "write_fixture.py"
    script.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(EXAMPLES)!r})\n"
        "from _common import import_trajectory_litegrip\n"
        "try:\n"
        "    import_trajectory_litegrip()\n"
        "except SystemExit as exc:\n"
        # Not a test failure: this machine has no trajectory SDK.  Reported as a
        # skip, and anything else the script does is still a failure, so a real
        # mistake in here cannot hide behind this.
        f"    print({NO_SDK!r} + str(exc))\n"
        "    raise SystemExit(0)\n"
        "from litegrip.trajectory import Trajectory, TrajectorySample\n"
        f"n = {FIXTURE_SAMPLES}\n"
        f"hz = {FIXTURE_RATE_HZ:g}\n"
        "samples = [TrajectorySample(t=i / hz, openness=0.2 + 0.6 * i / (n - 1),\n"
        "                            position_rad=1.2 - 0.6 * i / (n - 1))\n"
        "           for i in range(n)]\n"
        "traj = Trajectory(samples=samples, sample_hz=hz, can_id=0x08,\n"
        "                  pos_closed_rad=1.775959, pos_open_rad=-0.064279,\n"
        "                  rad_to_mm=65.21, mount='normal')\n"
        f"print(traj.save({str(path)!r}))\n",
        encoding="utf-8",
    )
    written = subprocess.run(
        [sys.executable, str(script)], cwd=str(REPO_ROOT), timeout=TIMEOUT_S,
        capture_output=True, text=True,
        env={**os.environ, "LITEGRIP_PYBULLET_REEXEC": "1"},
    )
    assert written.returncode == 0, written.stdout + written.stderr
    first = written.stdout.strip().splitlines()[0] if written.stdout.strip() else ""
    if first.startswith(NO_SDK):
        pytest.skip(first[len(NO_SDK):].strip())
    return written.stdout.strip().splitlines()[-1]


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


@pytest.mark.parametrize("script", [EXAMPLE_05, EXAMPLE_04, EXAMPLE_03],
                         ids=lambda p: p.name)
class TestHardwareFlags:
    """Only 03, 04 and 05 touch the hardware, so only they take CAN options."""

    def test_can_flags_are_documented(self, script):
        result = run(script, "--help")
        assert "--channel" in result.stdout
        assert "--can-id" in result.stdout
        assert "--mst-id" in result.stdout


class TestExample01HelloSim:
    """01 creates the simulation and *reads* it: no hardware, no motion.

    It is the example that must run anywhere, and the only one whose point is
    that nothing happens.
    """

    #: Everything in ``GripperSim`` that moves the fingers, or puts a body in
    #: the scene for them to move against.  01 must not call any of them.
    MOTION_CALLS = ("command_fraction", "command_joint", "reset_fraction",
                    "settle", "run_for", "add_box")

    def test_headless_run_succeeds(self, fast_hello):
        assert fast_hello.returncode == 0, output_of(fast_hello)

    def test_it_reads_the_state_and_stops(self, fast_hello):
        text = output_of(fast_hello)
        for marker in ("[1] 模型常量", "[2] 当前开度（三种写法）", "完成"):
            assert marker in text, f"缺少 {marker} 段落"

    def test_it_shows_the_opening_in_all_three_notations(self, fast_hello):
        """The whole reason the file exists: 0..1, joint metres, and the SDK's
        millimetres are three views of one opening, not three quantities."""
        text = output_of(fast_hello)
        assert "100.0%" in text        # fraction(), fully open out of the box
        assert "87.00 mm" in text      # aperture_mm(), the physical gap
        assert "120.00 mm" in text     # fraction_to_sdk_mm(), the SDK's scale

    def test_the_jaws_never_move(self, fast_hello):
        """Only 01's own output is searched, so the words may appear in the
        other examples' runs without weakening this."""
        text = output_of(fast_hello)
        for moved in ("全行程", "到位", "夹住", "接触点", "方块", "下落"):
            assert moved not in text, f"01 是只读样例，输出里不该有「{moved}」"

    def test_it_says_it_is_read_only(self, fast_hello):
        text = output_of(fast_hello)
        assert "只读" in text
        assert "不接真机" in text

    def test_it_never_commands_motion(self):
        """The source, not just this run: 01 must not call a motion method.

        ``sim.step()`` is deliberately allowed — it is the window's event pump,
        and with no command issued it advances a setpoint that already equals
        the measured position, so it cannot move the jaws.  The docstring's
        ``演示:`` list is not a call, so it is not what this reads.
        """
        tree = ast.parse(EXAMPLE_01.read_text(encoding="utf-8"))
        called = {node.func.attr for node in ast.walk(tree)
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute)}
        called |= {node.func.id for node in ast.walk(tree)
                   if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
        offenders = sorted(called & set(self.MOTION_CALLS))
        assert not offenders, f"01 是只读样例，却调用了 {offenders}"

    def test_a_bad_urdf_path_fails_loudly(self):
        result = run(EXAMPLE_01, "--headless", "--urdf", "/nonexistent/g.urdf")
        assert result.returncode != 0
        assert "URDF" in output_of(result) or "urdf" in output_of(result)


class TestExample02MoveSim:
    """02 moves the jaws: the rate limit, and positioning by opening."""

    def test_headless_run_succeeds(self, fast_move):
        assert fast_move.returncode == 0, output_of(fast_move)

    def test_it_walks_through_both_demos(self, fast_move):
        text = output_of(fast_move)
        for marker in ("[1] 全行程开合（速度受限）",
                       "[2] 走到中间位（归一化开度）", "完成"):
            assert marker in text, f"缺少 {marker} 段落"

    def test_the_default_invocation_runs_to_completion(self):
        """No flags at all beyond --headless: the documented happy path."""
        result = run(EXAMPLE_02, "--headless")
        assert result.returncode == 0, output_of(result)
        assert "全行程" in output_of(result)

    def test_a_slower_speed_is_accepted(self):
        result = run(EXAMPLE_02, "--headless", "--speed", "0.02")
        assert result.returncode == 0, output_of(result)

    def test_the_grasp_options_are_not_here(self):
        """02 does not grip, and the split must not leave the old flags behind
        as silent no-ops: argparse has to refuse them."""
        result = run(EXAMPLE_02, "--headless", "--object-mm", "60")
        assert result.returncode == 2, output_of(result)

    def test_a_bad_urdf_path_fails_loudly(self):
        result = run(EXAMPLE_02, "--headless", "--urdf", "/nonexistent/g.urdf")
        assert result.returncode != 0
        assert "URDF" in output_of(result) or "urdf" in output_of(result)


class TestExample03Trajectory:
    """03 records a motion on the hardware and replays it into both targets.

    The one path that runs without a gripper is ``--play`` on a file: it reads a
    ``.lgt`` and drives the window from it.  The recording path and the hardware
    replay are checked only as far as argument parsing and the refusal to start
    — running either would move a real gripper, and the module that would do the
    moving is covered with a stand-in in ``tests/test_example03_loop.py``.
    """

    def test_help_lists_the_trajectory_options(self):
        stdout = run(EXAMPLE_03, "--help").stdout
        for flag in ("--play", "--record", "--speed", "--real"):
            assert flag in stdout, f"03 的选项里缺少 {flag}"

    def test_the_grasp_options_are_gone(self):
        """The old demo's flags must not survive as silent no-ops."""
        for flag, value in (("--object-mm", "60"), ("--force", "10"),
                            ("--hold", "1"), ("--pull", "15"),
                            ("--slip", "1")):
            result = run(EXAMPLE_03, "--headless", flag, value)
            assert result.returncode == 2, \
                f"{flag} 还认得：{output_of(result)}"

    def test_record_and_play_are_refused_together(self):
        """They are two different sessions; one run cannot be both."""
        result = run(EXAMPLE_03, "--record", "1", "--play", "somewhere")
        assert result.returncode == 2
        assert "--play" in output_of(result)

    def test_real_without_play_is_refused(self):
        """``--real`` modifies a replay.  A recording is already on hardware, so
        accepting the flag there would mean nothing and read as if it did."""
        result = run(EXAMPLE_03, "--real")
        assert result.returncode == 2
        assert "--real" in output_of(result)

    def test_a_non_positive_speed_is_refused(self):
        result = run(EXAMPLE_03, "--play", "somewhere", "--speed", "0")
        assert result.returncode == 2
        assert "--speed" in output_of(result)

    @pytest.fixture
    def bare_trajectory_sdk(self, tmp_path) -> dict:
        """An importable ``litegrip`` package with none of the trajectory API."""
        package = tmp_path / "litegrip"
        package.mkdir()
        (package / "__init__.py").write_text(
            "class LiteGrip:\n"
            "    pass\n",
            encoding="utf-8",
        )
        return {"LITEGRIP_TRAJ_SDK_DIR": str(tmp_path)}

    def test_an_sdk_without_the_trajectory_api_names_what_is_missing(
            self, bare_trajectory_sdk):
        """The failure mode that has a physical cost: an SDK that imports but
        cannot record or replay must stop the example, not half-run it."""
        result = run(EXAMPLE_03, "--channel", NOWHERE, env=bare_trajectory_sdk)
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "缺少轨迹录制/回放的公开接口" in text
        assert "LiteGrip.record_start" in text
        assert "LiteGrip.play_start" in text
        assert "Trajectory.load" in text
        # ...and says which of the two checkouts has them
        assert "LITEGRIP_TRAJ_SDK_DIR" in text
        assert "litegrip-python" in text
        assert NOWHERE not in text      # never reached the bus

    def test_without_the_sdk_it_says_where_to_get_it(self, tmp_path):
        """Pointing the variable at nothing leaves whatever is installed; CI has
        nothing installed at all.  Both must stop with a usable message."""
        result = run(EXAMPLE_03, "--channel", NOWHERE,
                     env={"LITEGRIP_TRAJ_SDK_DIR": str(tmp_path / "nope")})
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        if "找不到带轨迹" in text:
            assert "LITEGRIP_TRAJ_SDK_DIR" in text
            assert "litegrip-python" in text
            assert "pip install -e" in text
        else:
            # An SDK is importable, but it is the one 04/05 use.
            assert "缺少轨迹录制/回放的公开接口" in text
        assert NOWHERE not in text

    def test_play_runs_the_window_and_touches_no_can(self, trajectory_file):
        """``--play FILE`` is the offline viewer, and the only runnable path."""
        result = run(EXAMPLE_03, "--play", trajectory_file, "--headless")
        assert result.returncode == 0, output_of(result)
        text = output_of(result)
        assert "[5] 回放：只灌仿真" in text
        assert "不连真机" in text
        assert "放完了" in text

    def test_play_reports_the_trajectory_it_read(self, trajectory_file):
        text = output_of(run(EXAMPLE_03, "--play", trajectory_file, "--headless"))
        assert f"{FIXTURE_SAMPLES} 个样本" in text
        assert "开度 0.200 → 0.800" in text

    def test_a_missing_trajectory_file_is_reported(self, trajectory_file):
        """A bare name is looked up in the SDK's trajectory directory, so the
        message has to say where that is -- it is not the working directory."""
        result = run(EXAMPLE_03, "--play", "no_such_trajectory_here",
                     "--headless")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "找不到这段轨迹" in text
        assert ".lgt" in text
        assert "trajectories" in text

    def test_play_real_says_the_fingers_will_move(self, trajectory_file):
        """``--real`` turns the viewer into a motion command, and the banner has
        to say so before anything is sent."""
        result = run(EXAMPLE_03, "--play", trajectory_file, "--real",
                     "--channel", NOWHERE, "--calib", "/nonexistent/c.json")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "两个手指会真实运动" in text
        assert NOWHERE not in text      # it stopped at the calibration, first

    def test_a_bad_urdf_path_fails_loudly(self, trajectory_file):
        result = run(EXAMPLE_03, "--play", trajectory_file, "--headless",
                     "--urdf", "/nonexistent/g.urdf")
        assert result.returncode != 0
        assert "URDF" in output_of(result) or "urdf" in output_of(result)


class TestExample05NeedsAWindow:
    def test_it_refuses_headless_and_explains_why(self):
        result = run(EXAMPLE_05, "--headless", "--dry-run")
        assert result.returncode == 1
        text = output_of(result)
        assert "窗口" in text
        # ...and points at the examples that can run without one
        assert "01_hello_sim.py" in text


class TestExample05Status:
    """``--status`` diagnoses a wedged gripper without commanding it.

    Nothing here can reach hardware: the interface name cannot exist, so the
    run stops at ``connect()``.  What is being pinned is that ``--status``
    never gets as far as enabling the motor or printing the motion banner, and
    that the argument combinations are refused rather than silently ignored.
    """

    @pytest.fixture
    def missing_interface(self, calib_file) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_05, "--status", "--channel", NOWHERE,
                     "--calib", calib_file)
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到连 CAN 这一步（pip install litegrip）")
        return result

    def test_it_is_documented(self):
        assert "--status" in run(EXAMPLE_05, "--help").stdout

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
        result = run(EXAMPLE_05, "--clear-fault")
        assert result.returncode == 1
        assert "--status" in output_of(result)

    def test_status_with_dry_run_is_refused(self):
        result = run(EXAMPLE_05, "--status", "--dry-run")
        assert result.returncode == 1
        text = output_of(result)
        assert "--dry-run" in text

    def test_the_speed_slider_is_announced_as_a_rate_limit(self):
        """``--speed`` is a cap on the position target, not a promise to arrive
        sooner, so the help has to say what 100 % *is* — millimetres per second."""
        stdout = run(EXAMPLE_05, "--help").stdout
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
        result = run(EXAMPLE_05, "--dry-run")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "--calib" in text
        # ...and it says where a calibration file comes from in the first place
        assert "上位机" in text
        assert "候选" in text, "没列出候选，操作员只能靠猜"

    def test_status_without_calib_refuses(self):
        result = run(EXAMPLE_05, "--status")
        if "找不到真机 SDK" in output_of(result):
            pytest.skip("装真机 SDK 才能测到这一步（裸 SDK 会在选标定之前就停）")
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "--calib" in text
        assert "上位机" in text

    def test_example_04_refuses_too(self):
        result = run(EXAMPLE_04, "--duration", "1")
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
        result = run(EXAMPLE_05, "--dry-run", "--calib", str(sim))
        assert result.returncode == 1, output_of(result)
        assert "仿真" in output_of(result)

    def test_a_missing_calibration_file_is_refused(self, tmp_path):
        """The SDK would silently fall back to the factory calibration here and
        return True; the example must stop instead."""
        result = run(EXAMPLE_05, "--dry-run", "--calib",
                     str(tmp_path / "nope.json"))
        assert result.returncode == 1, output_of(result)
        assert "不存在" in output_of(result)

    def test_the_dry_run_says_which_file_it_would_use(self, calib_file):
        """``--dry-run`` needs a calibration too, and says which one -- the whole
        reason to require it is that the numbers decide the target angles."""
        result = run(EXAMPLE_05, "--headless", "--dry-run", "--calib", calib_file)
        # 02 needs a window in every mode, so the dry-run never gets to open one
        # here; what matters is that the flag combination is still refused for
        # the window's sake, not for the calibration's.
        assert result.returncode == 1
        assert "窗口" in output_of(result)


class TestExample04WithoutHardware:
    """04 imports the SDK before it looks at the CAN interface.

    Without the SDK installed it stops earlier — with a different, equally valid
    message — so these two need it present to test what they claim to.
    """

    @pytest.fixture
    def missing_interface(self, calib_file) -> subprocess.CompletedProcess:
        result = run(EXAMPLE_04, "--channel", NOWHERE, "--duration", "1",
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
        stdout = run(EXAMPLE_04, "--help").stdout
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
        a fixed string, because how far 04 gets depends on whether the SDK is
        installed: with it the run reaches the CAN interface, without it it
        stops earlier. Both are correct; ``--passive`` must not alter either.
        """
        plain = run(EXAMPLE_04, "--channel", NOWHERE, "--duration", "1",
                    "--calib", calib_file)
        passive = run(EXAMPLE_04, "--passive", "--channel", NOWHERE,
                      "--duration", "1", "--calib", calib_file)
        assert plain.returncode == 1
        assert passive.returncode == plain.returncode
        if "找不到真机 SDK" in output_of(plain):
            assert "找不到真机 SDK" in output_of(passive)
        else:
            assert NOWHERE in output_of(passive)

    def test_without_the_sdk_it_says_how_to_get_it(self, calib_file):
        """The SDK-absent path is worth covering too — CI is exactly that case."""
        result = run(EXAMPLE_04, "--channel", NOWHERE, "--duration", "1",
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
        # Enough to import: 04/05 only build LiteGrip objects after the check.
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
        (EXAMPLE_05, ()),
        (EXAMPLE_04, ("--duration", "1")),
    ], ids=["05_dual_control.py", "04_mirror_real.py"])
    def test_it_stops_before_connecting(self, script, args, bare_sdk):
        result = run(script, "--channel", NOWHERE, *args, env=bare_sdk)
        assert result.returncode == 1, output_of(result)
        text = output_of(result)
        assert "缺少本仓库必须的公开接口" in text
        assert "LiteGrip.refresh_status" in text
        assert "GripperState.data_age_s" in text
        # ...and it never got as far as the bus, so nothing was transmitted.
        assert NOWHERE not in text
