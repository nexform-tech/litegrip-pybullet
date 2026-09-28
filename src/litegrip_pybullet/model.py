"""LiteGrip gripper geometry and unit mapping.

Three different numbers describe the same jaw opening, and mixing them up is
the classic source of off-by-a-lot bugs in gripper code:

* **joint value** — the URDF prismatic finger travel, in metres.  ``0.0`` is
  fully open, :data:`STROKE_M` is fully closed; both fingers carry the *same*
  value because their prismatic axes point in opposite directions (see the
  ``litegrip-urdf`` README — equal signs move the jaws symmetrically).
* **aperture** — the physical gap between the two jaw faces, in millimetres,
  derived from the meshes: 87 mm fully open, 1.5 mm at the calibrated closed
  position.
* **SDK position** — :attr:`litegrip.models.GripperState.position_mm`, what the
  SDK reports and accepts.  It is scaled by the per-unit calibration
  (``rad_to_mm`` / ``pos_closed_rad`` / ``pos_open_rad``) and always spans
  ``0 .. GripperConfig.max_stroke_mm`` (nominal 120 mm), whatever the physical
  travel of the unit in hand happens to be.

The SDK's millimetres are therefore **not** the physical jaw gap, and this
library never assumes they are.  Every conversion goes through one normalised
**fraction** — ``0.0`` fully closed, ``1.0`` fully open — which is the only
quantity both sides agree on::

    joint    = STROKE_M * (1 - fraction)
    sdk_mm   = fraction * max_stroke_mm

The fraction is also what the real-time programs exchange: ``02_sim_to_real``
takes the fraction from the simulated jaws, ``03_real_to_sim`` derives it from
``get_state().position_mm``.
"""

from __future__ import annotations

# ── Geometry of the modelled hardware (litegrip-urdf) ────────────────────────

#: Per-finger prismatic travel [m], i.e. the URDF joint ``upper`` limit.  This
#: is the *measured* single-finger travel of the reference unit (85.452 mm of
#: total jaw travel), and it is what the URDF itself uses by default.
STROKE_M = 0.042726

#: Only the prismatic joints move; the URDF declares exactly this many.
N_FINGERS = 2

#: Inner face of each jaw at ``joint = 0`` [m].  From the finger mesh bounds:
#: ``gripper_slider_link{1,2}.STL`` spans x = [-0.00095, +0.0235] about a joint
#: origin at x = ∓0.067, so the faces sit at x = ±0.0435 and the jaw gap is
#: ``2 × 0.0435 = 87 mm`` — the model opening quoted by the ``litegrip-urdf``
#: README.  (PyBullet's ``getAABB`` reports ±0.0405 for the same links: it
#: inflates the bounds by ~3 mm per side, so do not read the gap off an AABB.)
FINGER_FACE_AT_ZERO_M = 0.0435

#: Jaw gap with the fingers fully open [mm].
APERTURE_OPEN_MM = FINGER_FACE_AT_ZERO_M * 2.0 * 1000.0  # 87.0

#: Jaw gap at ``joint = STROKE_M`` [mm] — the URDF's zero position as seen by
#: the mesh geometry.  The calibrated closed position of a real unit leaves a
#: ~1.5 mm gap there, which is why this is not 0: the jaws meet mechanically
#: before the meshes would overlap.
APERTURE_CLOSED_MM = (FINGER_FACE_AT_ZERO_M - STROKE_M) * 2.0 * 1000.0  # 1.548

# ── SDK-side nominal values ──────────────────────────────────────────────────

#: Default ``max_stroke_mm`` of ``litegrip.models.GripperConfig``: the SDK's
#: full-open reading, in its own calibrated millimetres.
DEFAULT_MAX_STROKE_MM = 120.0

#: Rated maximum gripping force of the LiteGrip [N]; used to bound ``--force``.
MAX_GRIP_FORCE_N = 40.0


def clamp_fraction(fraction: float) -> float:
    """Clamp a fraction into ``[0.0, 1.0]``."""
    return max(0.0, min(1.0, float(fraction)))


def fraction_to_joint(fraction: float) -> float:
    """Normalised opening → URDF prismatic joint value [m]."""
    return (1.0 - clamp_fraction(fraction)) * STROKE_M


def joint_to_fraction(joint: float) -> float:
    """URDF prismatic joint value [m] → normalised opening."""
    return clamp_fraction(1.0 - float(joint) / STROKE_M)


def joint_to_aperture_mm(joint: float) -> float:
    """URDF prismatic joint value [m] → jaw gap [mm], clamped at 0.

    Purely kinematic: it reproduces the mesh geometry, and says nothing about
    the contact geometry PyBullet uses (see :data:`FINGER_FACE_AT_ZERO_M`).
    """
    return max(0.0, (FINGER_FACE_AT_ZERO_M - float(joint)) * 2.0 * 1000.0)


def fraction_to_aperture_mm(fraction: float) -> float:
    """Normalised opening → jaw gap [mm]."""
    return joint_to_aperture_mm(fraction_to_joint(fraction))


def sdk_mm_to_fraction(
    position_mm: float, max_stroke_mm: float = DEFAULT_MAX_STROKE_MM
) -> float:
    """SDK position [mm] → normalised opening."""
    if max_stroke_mm <= 0.0:
        raise ValueError(f"max_stroke_mm 必须为正数，收到 {max_stroke_mm!r}")
    return clamp_fraction(float(position_mm) / float(max_stroke_mm))


def fraction_to_sdk_mm(
    fraction: float, max_stroke_mm: float = DEFAULT_MAX_STROKE_MM
) -> float:
    """Normalised opening → SDK position [mm]."""
    return clamp_fraction(fraction) * float(max_stroke_mm)
