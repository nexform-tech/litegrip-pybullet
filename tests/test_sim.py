# -*- coding: utf-8 -*-
"""``GripperSim``: speed-limited motion, force caps, and grasping."""

import pybullet as p
import pytest

from litegrip_pybullet import (
    APERTURE_CLOSED_MM,
    APERTURE_OPEN_MM,
    DEFAULT_VELOCITY_M_S,
    N_FINGERS,
    STROKE_M,
    GripperSim,
    pressed,
)

#: Full-stroke time at the default speed limit [s].
FULL_STROKE_S = STROKE_M / DEFAULT_VELOCITY_M_S

#: Tolerance for "did it get there", in fraction units: settle()'s own 1 µm
#: tolerance works out to ~2.3e-5 of the stroke, so this leaves headroom.
STEP_TOLERANCE = 1e-4


@pytest.fixture
def sim():
    """Headless, not wall-clock paced: the tests run as fast as they can."""
    gripper = GripperSim(real_time=False)
    try:
        yield gripper
    finally:
        gripper.disconnect()


def close_in(sim: GripperSim, box: int, mass: float = 0.05) -> None:
    """Grab ``box`` in mid-air: hold it steady (mass 0) while the jaws close.

    Without this the part free-falls ~24 mm onto the gripper's base during the
    ~1 s the jaws take to close, and then it is resting on the base rather than
    being held by the fingers — which makes any friction test meaningless.
    """
    p.changeDynamics(box, -1, mass=0.0, physicsClientId=sim.client_id)
    sim.command_fraction(0.0, force_n=10.0)
    sim.settle(timeout_s=FULL_STROKE_S + 0.3)
    p.changeDynamics(box, -1, mass=mass, physicsClientId=sim.client_id)
    sim.run_for(0.05)


class TestConstruction:
    def test_starts_fully_open(self, sim):
        assert sim.fraction() == pytest.approx(1.0, abs=1e-9)
        assert sim.aperture_mm() == pytest.approx(APERTURE_OPEN_MM, abs=1e-6)

    def test_two_finger_joints(self, sim):
        assert len(sim.finger_joints) == N_FINGERS
        assert len(sim.joint_values()) == N_FINGERS

    def test_connected(self, sim):
        assert sim.connected()
        assert sim.client_id >= 0

    def test_headless_has_no_window(self, sim):
        assert sim.gui is False
        assert sim.keyboard_events() == {}
        sim.status_text("ignored")  # must not raise
        sim.focus_camera()           # must not raise

    def test_urdf_path_is_the_rewritten_one(self, sim):
        assert sim.urdf_path.endswith(".urdf")
        assert "package://" not in open(sim.urdf_path, encoding="utf-8").read()

    def test_disconnect_closes_the_connection(self):
        gripper = GripperSim(real_time=False)
        gripper.disconnect()
        assert not gripper.connected()


#: How far past the commanded ramp the servo may lag before the jaws truly
#: arrive.  The ramp sets the *speed*; the position servo then trails it by a
#: few tens of milliseconds (first-order, no overshoot).
SERVO_LAG_S = 0.25


class TestSpeedLimit:
    """The jaws move at hardware speed, not at whatever PyBullet feels like."""

    def test_full_stroke_takes_about_a_second(self, sim):
        sim.command_fraction(0.0)
        spent, reached = sim.settle(timeout_s=5.0)
        assert reached
        assert FULL_STROKE_S <= spent <= FULL_STROKE_S + SERVO_LAG_S

    def test_half_stroke_takes_about_half_a_second(self, sim):
        sim.command_fraction(0.0)
        sim.settle(timeout_s=5.0)
        sim.command_fraction(0.5)
        spent, reached = sim.settle(timeout_s=5.0)
        assert reached
        half = FULL_STROKE_S / 2.0
        assert half <= spent <= half + SERVO_LAG_S

    def test_speed_actually_scales_the_time(self, sim):
        """Quarter speed, half the stroke → four times as long."""
        slow = STROKE_M / 4.0
        sim.command_fraction(0.0, velocity_m_s=slow)
        sim.settle(timeout_s=10.0)
        sim.command_fraction(0.5, velocity_m_s=slow)
        spent, reached = sim.settle(timeout_s=10.0)
        assert reached
        assert FULL_STROKE_S * 2.0 <= spent <= FULL_STROKE_S * 2.0 + SERVO_LAG_S

    def test_it_does_not_snap_across_the_stroke(self, sim):
        """The failure this guards against: no rate limit at all (~80 ms)."""
        sim.command_fraction(0.0)
        spent, _ = sim.settle(timeout_s=5.0)
        assert spent > 0.5, f"全行程只用了 {spent:.3f} s，速度限制没生效"

    def test_settle_reports_unreached_when_it_cannot_get_there(self, sim):
        sim.command_fraction(0.0, velocity_m_s=STROKE_M / 50.0)
        spent, reached = sim.settle(timeout_s=1.0)
        assert reached is False
        assert spent == pytest.approx(1.0, abs=0.05)
        # it did move, just not all the way
        assert sim.fraction() < 1.0


class TestPositioning:
    @pytest.mark.parametrize("fraction", [0.0, 0.25, 0.5, 0.75, 1.0])
    def test_command_fraction_reaches_the_target(self, sim, fraction):
        sim.command_fraction(fraction)
        sim.settle(timeout_s=5.0)
        assert sim.fraction() == pytest.approx(fraction, abs=STEP_TOLERANCE)

    @pytest.mark.parametrize("fraction", [0.0, 0.3, 1.0])
    def test_aperture_follows_the_opening(self, sim, fraction):
        sim.command_fraction(fraction)
        sim.settle(timeout_s=5.0)
        assert sim.aperture_mm() == pytest.approx(
            APERTURE_CLOSED_MM
            + fraction * (APERTURE_OPEN_MM - APERTURE_CLOSED_MM),
            abs=0.05,
        )

    def test_both_fingers_carry_the_same_value(self, sim):
        """Equal joint values on opposite axes = a symmetric jaw."""
        sim.command_fraction(0.4)
        sim.settle(timeout_s=5.0)
        left, right = sim.joint_values()
        assert left == pytest.approx(right, abs=1e-6)

    def test_fraction_is_clamped(self, sim):
        sim.command_fraction(5.0)
        sim.settle(timeout_s=5.0)
        assert sim.fraction() == pytest.approx(1.0, abs=STEP_TOLERANCE)
        sim.command_fraction(-5.0)
        sim.settle(timeout_s=5.0)
        assert sim.fraction() == pytest.approx(0.0, abs=STEP_TOLERANCE)

    def test_measured_fraction_never_leaves_the_unit_interval(self, sim):
        for fraction in (-1.0, 0.5, 2.0):
            sim.command_fraction(fraction)
            sim.settle(timeout_s=5.0)
            assert 0.0 <= sim.fraction() <= 1.0


class TestReset:
    """``reset_fraction`` is the kinematic mirror used by example 03."""

    @pytest.mark.parametrize("fraction", [0.0, 0.13, 0.5, 0.99, 1.0])
    def test_teleports_exactly(self, sim, fraction):
        sim.reset_fraction(fraction)
        assert sim.fraction() == pytest.approx(fraction, abs=1e-9)

    def test_does_not_drift_afterwards(self, sim):
        """The motor is re-armed at the teleport position."""
        sim.reset_fraction(0.25)
        sim.run_for(0.5)
        assert sim.fraction() == pytest.approx(0.25, abs=1e-6)

    def test_a_later_command_still_works(self, sim):
        sim.reset_fraction(0.25)
        sim.command_fraction(0.75)
        sim.settle(timeout_s=5.0)
        assert sim.fraction() == pytest.approx(0.75, abs=STEP_TOLERANCE)

    def test_target_follows_the_teleport(self, sim):
        joint = sim.reset_fraction(0.0)
        assert joint == pytest.approx(STROKE_M, abs=1e-12)
        assert sim.target_joint == pytest.approx(STROKE_M, abs=1e-12)


class TestForceAndGrasping:
    def test_free_motion_uses_far_less_than_the_cap(self, sim):
        sim.command_fraction(0.0, force_n=10.0)
        sim.settle(timeout_s=5.0)
        assert sim.finger_force_n() < 1.0

    def test_grasp_stalls_at_the_force_cap(self, sim):
        center = sim.grasp_center()
        box = sim.add_box([0.020, 0.015, 0.015], center)
        close_in(sim, box)
        assert sim.finger_force_n() == pytest.approx(10.0, abs=0.5)
        assert sim.contacts(box)

    def test_settle_reports_unreached_when_blocked(self, sim):
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        p.changeDynamics(box, -1, mass=0.0, physicsClientId=sim.client_id)
        sim.command_fraction(0.0, force_n=10.0)
        _, reached = sim.settle(timeout_s=FULL_STROKE_S + 0.3)
        assert reached is False

    def test_only_the_fingers_touch_a_mid_air_part(self, sim):
        """The part clears the base, so the grip is real friction, not a ledge."""
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        close_in(sim, box)
        touching = {contact[3] for contact in sim.contacts(box)}
        assert touching, "手指没碰到工件"
        assert touching <= set(sim.finger_joints), (
            f"工件碰到了非手指的 link：{touching}"
        )

    def test_the_part_stays_up(self, sim):
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        close_in(sim, box)
        before = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        sim.run_for(0.5)
        after = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        assert after == pytest.approx(before, abs=2e-3)

    def test_a_small_pull_is_held(self, sim):
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        close_in(sim, box)
        before = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        for _ in range(250):
            p.applyExternalForce(box, -1, [0.0, 0.0, -5.0], [0.0, 0.0, 0.0],
                                 p.WORLD_FRAME, physicsClientId=sim.client_id)
            sim.step()
        after = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        assert after == pytest.approx(before, abs=2e-3)

    def test_a_big_pull_slips(self, sim):
        """Friction along the jaw faces has a limit; 40 N is past it."""
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        close_in(sim, box)
        before = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        for _ in range(250):
            p.applyExternalForce(box, -1, [0.0, 0.0, -40.0], [0.0, 0.0, 0.0],
                                 p.WORLD_FRAME, physicsClientId=sim.client_id)
            sim.step()
        after = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        assert after < before - 0.005, "40 N 都没拉动，摩擦模型可能不对"

    def test_a_gap_releases_the_part(self, sim):
        box = sim.add_box([0.020, 0.015, 0.015], sim.grasp_center())
        close_in(sim, box)
        before = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        sim.command_fraction(1.0, force_n=10.0)
        sim.settle(timeout_s=FULL_STROKE_S + 0.3)
        sim.run_for(0.5)
        after = p.getBasePositionAndOrientation(
            box, physicsClientId=sim.client_id)[0][2]
        assert after < before - 0.005, "松爪后工件没掉下来"


class TestStepping:
    def test_sim_time_advances_with_the_time_step(self, sim):
        before = sim.sim_time
        sim.step(10)
        assert sim.sim_time == pytest.approx(before + 10 * sim.time_step)

    def test_run_for_advances_sim_time(self, sim):
        before = sim.sim_time
        sim.run_for(0.2)
        assert sim.sim_time == pytest.approx(before + 0.2, abs=sim.time_step)

    def test_grabbing_center_sits_between_the_jaws(self, sim):
        center = sim.grasp_center()
        assert center[0] == pytest.approx(0.0, abs=1e-3)   # symmetric in X
        left, right = sim.link_aabb(sim.finger_joints[0]), sim.link_aabb(
            sim.finger_joints[1])
        assert min(left[0][2], right[0][2]) < center[2] < max(left[1][2],
                                                              right[1][2])


class TestPressed:
    def test_empty_events(self):
        assert not pressed({}, (ord("q"),))

    def test_triggered_key(self):
        assert pressed({ord("q"): p.KEY_WAS_TRIGGERED}, (ord("q"),))
        assert pressed({27: p.KEY_WAS_TRIGGERED}, (27, ord("q")))

    def test_released_key_is_not_pressed(self):
        assert not pressed({ord("q"): p.KEY_WAS_RELEASED}, (ord("q"),))
