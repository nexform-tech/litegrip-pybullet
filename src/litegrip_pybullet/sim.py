"""PyBullet model of the LiteGrip gripper.

:class:`GripperSim` loads the two-finger model, drives the prismatic finger
joints, and reports the opening as a normalised *fraction* (``0.0`` closed,
``1.0`` open) — the quantity this library shares with the real SDK, see
:mod:`litegrip_pybullet.model`.

The fingers are position-controlled with a **force cap** and a **speed limit**:
PyBullet drags them towards the commanded opening, and the command itself walks
towards the target at :data:`DEFAULT_VELOCITY_M_S`, so the jaws move at the
hardware's speed and stall against an object exactly like the real ones stall
against a grasped part.  The ramp is explicit rather than delegated to PyBullet
because ``targetVelocity`` is not a rate limit there — see
:meth:`GripperSim._apply_motor`.  It is also how the SDK's ``move_at_speed``
drives the real motor: a linear ramp, streamed frame by frame.

The ramp sets the *commanded* position; the position servo then trails it by a
few tens of milliseconds (first-order, without overshoot), so a full stroke
takes ~1.0 s of ramp **plus** ~0.13 s to actually arrive — see
:meth:`GripperSim.settle`, whose reported time is the total.  That lag is a
property of the servo, not of the speed limit, and it shrinks neither with
force nor with distance.

The force cap is given in newtons and is used as the joint's maximum output
force — the same simplification the URDF itself makes, where a rotary motor
driving a rack is modelled as a prismatic joint with ``effort`` in newtons.

Typical use::

    sim = GripperSim(gui=True)
    sim.command_fraction(0.5, force_n=10.0)   # close halfway, 10 N cap
    sim.settle()
    print(sim.fraction(), sim.aperture_mm(), sim.finger_force_n())
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence, Tuple

from .model import (
    N_FINGERS,
    STROKE_M,
    clamp_fraction,
    fraction_to_joint,
    joint_to_aperture_mm,
    joint_to_fraction,
)
from .urdf import resolve_urdf

try:
    import pybullet as p
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise ImportError("litegrip_pybullet 需要 pybullet，请先 pip install pybullet") from exc

__all__ = [
    "GripperSim",
    "DEFAULT_TIME_STEP",
    "DEFAULT_MAX_FORCE_N",
    "DEFAULT_SETTLE_TOLERANCE_M",
    "QUIT_KEYS",
    "CONFIRM_KEYS",
]

#: Physics step.  1/500 s tracks the ~85 mm/s stroke smoothly and still settles
#: exactly at the end of the move.
DEFAULT_TIME_STEP = 1.0 / 500.0

#: Default force cap [N] for simulated jaws — the URDF's own ``effort``.
DEFAULT_MAX_FORCE_N = 10.0

#: Default jaw speed [m/s]: one finger crosses its full travel in a second, i.e.
#: a jaw closing speed of 85.5 mm/s for the two fingers together — the hardware's
#: rated 85 mm/s.
DEFAULT_VELOCITY_M_S = STROKE_M

#: PyBullet keyboard codes (GLFW) — Esc/Q quit, Enter/Space confirms.
QUIT_KEYS = (27, ord("q"), ord("Q"))
CONFIRM_KEYS = (p.B3G_RETURN, p.B3G_SPACE)

#: Default ``settle`` tolerance [m].  Tight on purpose: the ramp moves the
#: setpoint by up to ``velocity × time_step`` (85 µm) per step, so a tolerance
#: anywhere near that would let ``settle`` report success while the jaws are
#: still short of the target.  Once the setpoint stops, the servo converges to
#: the target exactly, so a micron is cheap and unambiguous.
DEFAULT_SETTLE_TOLERANCE_M = 1e-6


def _dec(value) -> str:
    return value.decode() if isinstance(value, bytes) else str(value)


class GripperSim:
    """A PyBullet LiteGrip: two fingers, one opening fraction.

    Args:
        urdf_path: Description to load; ``None`` uses :func:`resolve_urdf`'s
            search order (bundled copy first).
        gui: Open a PyBullet window instead of running headless.
        client_id: Attach to an existing PyBullet connection instead of opening
            one (the caller then owns it, and ``gui`` is ignored).
        time_step: Physics step in seconds.
        gravity: Gravity vector; ``(0, 0, 0)`` for a pure kinematic mirror.
        real_time: Pace :meth:`step` to wall-clock time.  Leave ``True`` for
            windowed demos, ``False`` for headless runs that want to go fast.
        max_force_n: Initial force cap [N].
        velocity_m_s: Initial jaw speed [m/s].
        base_position, base_orientation: Where to mount the gripper.
    """

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        *,
        gui: bool = False,
        client_id: Optional[int] = None,
        time_step: float = DEFAULT_TIME_STEP,
        gravity: Sequence[float] = (0.0, 0.0, -9.81),
        real_time: bool = True,
        max_force_n: float = DEFAULT_MAX_FORCE_N,
        velocity_m_s: float = DEFAULT_VELOCITY_M_S,
        base_position: Sequence[float] = (0.0, 0.0, 0.0),
        base_orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> None:
        if client_id is None:
            self._cid = p.connect(p.GUI if gui else p.DIRECT)
            self._owns_connection = True
            self._gui = bool(gui)
        else:
            self._cid = client_id
            self._owns_connection = False
            info = p.getConnectionInfo(physicsClientId=self._cid)
            self._gui = info.get("connectionMethod") == p.GUI_CONNECTION
        if self._cid < 0:  # pragma: no cover - no display available
            raise RuntimeError(
                "无法连接 PyBullet（GUI 模式需要可用的显示；可加 --headless）"
            )

        self._time_step = float(time_step)
        self._real_time = bool(real_time)
        self._sim_time = 0.0
        self._max_force_n = float(max_force_n)
        self._velocity_m_s = max(float(velocity_m_s), 1e-6)
        self._text_id: Optional[int] = None

        p.setGravity(*gravity, physicsClientId=self._cid)
        p.setTimeStep(self._time_step, physicsClientId=self._cid)
        p.setRealTimeSimulation(0, physicsClientId=self._cid)

        path = resolve_urdf(urdf_path)
        self._urdf_path = path
        self._body = p.loadURDF(
            path,
            basePosition=list(base_position),
            baseOrientation=list(base_orientation),
            useFixedBase=True,
            physicsClientId=self._cid,
        )
        self._joints = self._collect_finger_joints()
        self._target = self._mean_joint()
        self._setpoint = self._target
        self._apply_motor()

    # ── Construction helpers ────────────────────────────────────────────────

    def _collect_finger_joints(self) -> List[int]:
        """The prismatic finger joints, in URDF order."""
        joints = []
        for jid in range(p.getNumJoints(self._body, physicsClientId=self._cid)):
            if p.getJointInfo(self._body, jid, physicsClientId=self._cid)[2] == \
                    p.JOINT_PRISMATIC:
                joints.append(jid)
        if len(joints) != N_FINGERS:
            raise RuntimeError(
                f"URDF 里有 {len(joints)} 个 prismatic 关节，期望 {N_FINGERS} 个"
                f"（{self._urdf_path}）"
            )
        # PyBullet gives every loaded joint a default position motor that locks
        # it; zero it out so the joint only moves when we command it.
        p.setJointMotorControlArray(
            self._body, joints, p.VELOCITY_CONTROL,
            forces=[0.0] * len(joints), physicsClientId=self._cid,
        )
        return joints

    # ── Properties ──────────────────────────────────────────────────────────

    @property
    def client_id(self) -> int:
        """PyBullet physics client id."""
        return self._cid

    @property
    def body_id(self) -> int:
        """PyBullet body id of the gripper."""
        return self._body

    @property
    def finger_joints(self) -> Tuple[int, ...]:
        """Joint indices of the two fingers (URDF order: right, left)."""
        return tuple(self._joints)

    @property
    def gui(self) -> bool:
        """True when there is a window to draw in (:meth:`status_text` & co)."""
        return self._gui

    @property
    def urdf_path(self) -> str:
        """Path of the rewritten URDF actually loaded."""
        return self._urdf_path

    @property
    def time_step(self) -> float:
        """Physics step in seconds."""
        return self._time_step

    @property
    def sim_time(self) -> float:
        """Simulated seconds elapsed since construction."""
        return self._sim_time

    @property
    def max_force_n(self) -> float:
        """Current force cap [N]."""
        return self._max_force_n

    @property
    def target_joint(self) -> float:
        """Commanded finger travel [m]."""
        return self._target

    def connected(self) -> bool:
        """True while the PyBullet connection (and window, if any) is alive."""
        try:
            return bool(p.isConnected(physicsClientId=self._cid))
        except Exception:  # pragma: no cover - connection torn down
            return False

    # ── State ───────────────────────────────────────────────────────────────

    def joint_values(self) -> Tuple[float, ...]:
        """Measured finger travels [m], one per finger."""
        return tuple(
            p.getJointState(self._body, jid, physicsClientId=self._cid)[0]
            for jid in self._joints
        )

    def _mean_joint(self) -> float:
        values = self.joint_values()
        return sum(values) / len(values)

    def fraction(self) -> float:
        """Measured opening as a fraction (``0.0`` closed, ``1.0`` open)."""
        return joint_to_fraction(self._mean_joint())

    def aperture_mm(self) -> float:
        """Measured jaw gap [mm], from the mesh geometry."""
        return joint_to_aperture_mm(self._mean_joint())

    def finger_force_n(self) -> float:
        """Largest finger output force [N] currently applied by the motors.

        For a prismatic joint PyBullet reports the applied motor force here, so
        this is what the sim is pushing with — the cap when the jaws are blocked
        by an object, less when they move freely.
        """
        return max(
            abs(p.getJointState(self._body, jid, physicsClientId=self._cid)[3])
            for jid in self._joints
        )

    def link_aabb(self, link: int):
        """Axis-aligned bounds of a link, as ``(min_xyz, max_xyz)``.

        Note that PyBullet inflates these bounds by ~3 mm per side relative to
        the mesh, so do not read jaw positions off them — use
        :meth:`aperture_mm` for that.  Midpoints are unaffected.
        """
        return p.getAABB(self._body, link, physicsClientId=self._cid)

    def grasp_center(self) -> Tuple[float, float, float]:
        """Centre of the volume between the jaws — where to put a grasped part.

        Derived from the finger bounding boxes, so it follows the model instead
        of hard-coded offsets.
        """
        boxes = [self.link_aabb(jid) for jid in self._joints]
        return tuple(
            sum(bound[axis] for box in boxes for bound in box) / (2 * len(boxes))
            for axis in range(3)
        )

    # ── Commands ────────────────────────────────────────────────────────────

    def command_fraction(
        self,
        fraction: float,
        *,
        force_n: Optional[float] = None,
        velocity_m_s: Optional[float] = None,
    ) -> float:
        """Command an opening (``0.0`` closed … ``1.0`` open).

        Args:
            fraction: Target opening; clamped to ``[0, 1]``.
            force_n: Force cap for this and later commands [N].
            velocity_m_s: Jaw speed limit [m/s].

        Returns:
            The commanded finger travel [m].
        """
        return self.command_joint(
            fraction_to_joint(fraction), force_n=force_n, velocity_m_s=velocity_m_s
        )

    def command_joint(
        self,
        joint: float,
        *,
        force_n: Optional[float] = None,
        velocity_m_s: Optional[float] = None,
    ) -> float:
        """Command a raw finger travel in metres, clamped to the URDF limits."""
        self._target = max(0.0, min(STROKE_M, float(joint)))
        if force_n is not None:
            self._max_force_n = max(0.0, float(force_n))
        if velocity_m_s is not None:
            self._velocity_m_s = max(float(velocity_m_s), 1e-6)
        return self._target

    def reset_fraction(self, fraction: float) -> float:
        """Teleport the jaws to ``fraction`` (kinematic, no physics).

        Used to mirror a real gripper: the sim is not asked to reproduce the
        motion, only to show where the hardware is.  The motor is re-armed at
        the new position so the joints do not drift afterwards.
        """
        joint = fraction_to_joint(fraction)
        for jid in self._joints:
            p.resetJointState(
                self._body, jid, targetValue=joint, targetVelocity=0.0,
                physicsClientId=self._cid,
            )
        self._target = joint
        self._setpoint = joint
        self._apply_motor()
        return joint

    def _advance_setpoint(self) -> None:
        """Walk the commanded position towards the target at the speed limit.

        This is what makes the simulated jaws move at hardware speed instead of
        snapping to the target: the move becomes a speed-limited ramp, exactly
        like the linear ramp the SDK's ``move_at_speed`` streams to the motor.
        """
        limit = self._velocity_m_s * self._time_step
        delta = self._target - self._setpoint
        if abs(delta) <= limit:
            self._setpoint = self._target
        else:
            self._setpoint += limit if delta > 0.0 else -limit

    def _apply_motor(self) -> None:
        """Point the position servo at the current setpoint.

        ``targetVelocity`` is left at zero on purpose.  PyBullet does not use it
        as a rate limit: a non-zero value biases the servo, which then settles
        with a steady-state error of ``0.02 × targetVelocity`` — 855 µm, i.e. 2 %
        of the stroke, for the speeds used here, and with the *motor force at
        zero*, so the jaws sit visibly short of the commanded opening and stay
        there.  The ramp in :meth:`_advance_setpoint` is what limits the speed;
        this servo only has to track a target that never moves more than one
        step away from where the jaws already are.
        """
        p.setJointMotorControlArray(
            self._body,
            self._joints,
            p.POSITION_CONTROL,
            targetPositions=[self._setpoint] * len(self._joints),
            targetVelocities=[0.0] * len(self._joints),
            forces=[self._max_force_n] * len(self._joints),
            physicsClientId=self._cid,
        )

    # ── Stepping ────────────────────────────────────────────────────────────

    def step(self, n: int = 1) -> bool:
        """Advance the simulation ``n`` steps.

        Returns:
            ``False`` if the connection is gone (window closed) — loops should
            stop rather than raise.
        """
        for _ in range(max(1, int(n))):
            if not self.connected():
                return False
            if self._setpoint != self._target:
                self._advance_setpoint()
                self._apply_motor()
            started = time.monotonic()
            p.stepSimulation(physicsClientId=self._cid)
            self._sim_time += self._time_step
            if self._real_time:
                spent = time.monotonic() - started
                if spent < self._time_step:
                    time.sleep(self._time_step - spent)
        return True

    def run_for(self, seconds: float) -> bool:
        """Step until ``seconds`` of simulated time have passed."""
        steps = max(1, int(round(float(seconds) / self._time_step)))
        return self.step(steps)

    def settle(
        self,
        *,
        tolerance_m: float = DEFAULT_SETTLE_TOLERANCE_M,
        timeout_s: float = 3.0,
    ) -> Tuple[float, bool]:
        """Step until the fingers reach their commanded target.

        Args:
            tolerance_m: Acceptable distance to the target [m].
            timeout_s: Give up after this much *simulated* time.

        Returns:
            ``(simulated seconds spent, reached)``.  ``reached`` is ``False``
            when the jaws stalled — which is the expected outcome when they
            close on an object, not an error.  The seconds are the *total*:
            the speed-limited ramp plus the servo's own settling.
        """
        started = self._sim_time
        while True:
            if not self.step():
                return self._sim_time - started, False
            error = max(abs(j - self._target) for j in self.joint_values())
            if error <= tolerance_m:
                return self._sim_time - started, True
            if self._sim_time - started >= timeout_s:
                return self._sim_time - started, False

    # ── Scene helpers ───────────────────────────────────────────────────────

    def add_box(
        self,
        half_extents: Sequence[float],
        position: Sequence[float],
        *,
        mass: float = 0.05,
        friction: float = 1.0,
        color: Sequence[float] = (0.85, 0.35, 0.25, 1.0),
    ) -> int:
        """Add a graspable box and return its body id.

        Friction matters here: a grasped box is held by friction between the jaw
        faces and the part, so it defaults to a high coefficient.
        """
        half = list(half_extents)
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half,
                                       physicsClientId=self._cid)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                     rgbaColor=list(color),
                                     physicsClientId=self._cid)
        body = p.createMultiBody(
            baseMass=mass, baseCollisionShapeIndex=shape,
            baseVisualShapeIndex=visual, basePosition=list(position),
            physicsClientId=self._cid,
        )
        p.changeDynamics(body, -1, lateralFriction=friction,
                         spinningFriction=0.01, rollingFriction=0.01,
                         physicsClientId=self._cid)
        return body

    def contacts(self, other_body: int) -> List:
        """Contact points between the gripper and ``other_body``."""
        return p.getContactPoints(self._body, other_body, physicsClientId=self._cid)

    # ── Window helpers (no-ops when headless) ───────────────────────────────

    def status_text(self, text: str) -> None:
        """Show/replace a status line in the window (headless: no-op)."""
        if not self._gui or not self.connected():
            return
        anchor = [0.0, 0.0, 0.15]
        kwargs = dict(textColorRGB=[1, 1, 0], textSize=1.2,
                      physicsClientId=self._cid)
        if self._text_id is None:
            self._text_id = p.addUserDebugText(text, anchor, **kwargs)
        else:
            p.addUserDebugText(text, anchor, replaceItemUniqueId=self._text_id,
                               **kwargs)

    def focus_camera(
        self,
        *,
        distance: float = 0.5,
        yaw: float = 35.0,
        pitch: float = -20.0,
        target: Sequence[float] = (0.0, 0.0, 0.05),
    ) -> None:
        """Frame the gripper (headless: no-op)."""
        if not self._gui:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch,
            cameraTargetPosition=list(target), physicsClientId=self._cid,
        )

    def keyboard_events(self) -> Dict[int, int]:
        """Key events from the window this frame, or ``{}`` when headless."""
        if not self._gui or not self.connected():
            return {}
        try:
            return dict(p.getKeyboardEvents(physicsClientId=self._cid))
        except Exception:  # pragma: no cover - window closed mid-frame
            return {}

    def disconnect(self) -> None:
        """Close the PyBullet connection if this object opened it."""
        if self._owns_connection and self.connected():
            try:
                p.disconnect(physicsClientId=self._cid)
            except Exception:  # pragma: no cover
                pass


def pressed(events: Dict[int, int], keys: Sequence[int]) -> bool:
    """True if any of ``keys`` was pressed in a :meth:`GripperSim.keyboard_events`.

    ``keys`` are PyBullet/GLFW key codes, e.g. :data:`QUIT_KEYS`.
    """
    return any(events.get(key, 0) & p.KEY_WAS_TRIGGERED for key in keys)
