"""LiteGrip PyBullet simulation environment.

A simulation of the LiteGrip two-finger gripper that speaks the same *opening*
as the real one, so the same program can drive either — or both at once, which
is what the examples do:

* :class:`~litegrip_pybullet.sim.GripperSim` — the PyBullet gripper.
* :mod:`~litegrip_pybullet.model` — the fraction / joint value / aperture / SDK
  millimetre conversions; the single place where units are reconciled.
* :mod:`~litegrip_pybullet.urdf` — finds the description (bundled copy of
  ``litegrip-urdf``, or ``$LITEGRIP_URDF_DIR``) and rewrites it for PyBullet.

Example::

    from litegrip_pybullet import GripperSim

    sim = GripperSim(gui=True)
    sim.command_fraction(0.0, force_n=10.0)   # close with a 10 N cap
    sim.settle()
    print(f"aperture = {sim.aperture_mm():.1f} mm")
"""

from .model import (
    APERTURE_CLOSED_MM,
    APERTURE_OPEN_MM,
    DEFAULT_MAX_STROKE_MM,
    FINGER_FACE_AT_ZERO_M,
    MAX_GRIP_FORCE_N,
    N_FINGERS,
    STROKE_M,
    clamp_fraction,
    fraction_to_aperture_mm,
    fraction_to_joint,
    fraction_to_sdk_mm,
    joint_to_aperture_mm,
    joint_to_fraction,
    sdk_mm_to_fraction,
)
from .sim import (
    CONFIRM_KEYS,
    DEFAULT_MAX_FORCE_N,
    DEFAULT_TIME_STEP,
    DEFAULT_VELOCITY_M_S,
    QUIT_KEYS,
    GripperSim,
    pressed,
)
from .urdf import UrdfError, bundle_dir, default_urdf, normalize_urdf, resolve_urdf

__all__ = [
    "APERTURE_CLOSED_MM",
    "APERTURE_OPEN_MM",
    "CONFIRM_KEYS",
    "DEFAULT_MAX_FORCE_N",
    "DEFAULT_MAX_STROKE_MM",
    "DEFAULT_TIME_STEP",
    "DEFAULT_VELOCITY_M_S",
    "FINGER_FACE_AT_ZERO_M",
    "GripperSim",
    "MAX_GRIP_FORCE_N",
    "N_FINGERS",
    "QUIT_KEYS",
    "STROKE_M",
    "UrdfError",
    "bundle_dir",
    "clamp_fraction",
    "default_urdf",
    "fraction_to_aperture_mm",
    "fraction_to_joint",
    "fraction_to_sdk_mm",
    "joint_to_aperture_mm",
    "joint_to_fraction",
    "normalize_urdf",
    "pressed",
    "resolve_urdf",
    "sdk_mm_to_fraction",
]
