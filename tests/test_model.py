# -*- coding: utf-8 -*-
"""Unit mapping: fraction ↔ joint value ↔ aperture ↔ SDK millimetres."""

import pytest

from litegrip_pybullet import (
    APERTURE_CLOSED_MM,
    APERTURE_OPEN_MM,
    DEFAULT_MAX_STROKE_MM,
    FINGER_FACE_AT_ZERO_M,
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




class TestGeometry:
    def test_stroke_matches_the_upstream_urdf(self):
        """The bundled URDF's own ``stroke`` default is what the model is built on.

        If upstream changes the travel, this fails loudly instead of silently
        making every conversion in this library wrong.
        """
        import re

        from litegrip_pybullet import default_urdf

        text = default_urdf().read_text(encoding="utf-8")
        match = re.search(r'<xacro:arg\s+name="stroke"\s+default="([^"]+)"', text)
        assert match is not None, "URDF 里找不到 stroke 的 xacro:arg 声明"
        assert float(match.group(1)) == pytest.approx(STROKE_M, abs=1e-12)

    def test_open_aperture_is_87_mm(self):
        assert APERTURE_OPEN_MM == pytest.approx(87.0, abs=1e-6)

    def test_closed_aperture_is_the_mesh_derived_gap(self):
        assert APERTURE_CLOSED_MM == pytest.approx(
            (FINGER_FACE_AT_ZERO_M - STROKE_M) * 2000.0, abs=1e-9
        )
        # Not zero: the jaws meet mechanically before the meshes overlap.
        assert 0.0 < APERTURE_CLOSED_MM < 2.0

    def test_two_fingers(self):
        assert N_FINGERS == 2


class TestFractionJoint:
    @pytest.mark.parametrize("fraction", [0.0, 0.25, 0.5, 0.75, 1.0])
    def test_round_trip(self, fraction):
        assert joint_to_fraction(fraction_to_joint(fraction)) == pytest.approx(
            fraction, abs=1e-12
        )

    def test_open_is_zero_travel(self):
        """Fraction 1.0 (open) is joint 0; fraction 0.0 (closed) is full stroke."""
        assert fraction_to_joint(1.0) == pytest.approx(0.0, abs=1e-15)
        assert fraction_to_joint(0.0) == pytest.approx(STROKE_M, abs=1e-15)

    @pytest.mark.parametrize("bad,expected", [(-1.0, 0.0), (1.5, 1.0), (-0.3, 0.0)])
    def test_fraction_is_clamped(self, bad, expected):
        assert clamp_fraction(bad) == expected
        assert joint_to_fraction(fraction_to_joint(bad)) == pytest.approx(expected)

    def test_joint_outside_stroke_is_clamped(self):
        assert joint_to_fraction(-0.01) == 1.0
        assert joint_to_fraction(STROKE_M * 2) == 0.0


class TestAperture:
    def test_ends_match_the_mesh_geometry(self):
        assert fraction_to_aperture_mm(1.0) == pytest.approx(APERTURE_OPEN_MM)
        assert fraction_to_aperture_mm(0.0) == pytest.approx(APERTURE_CLOSED_MM)

    def test_midpoint_is_linear(self):
        assert fraction_to_aperture_mm(0.5) == pytest.approx(
            (APERTURE_OPEN_MM + APERTURE_CLOSED_MM) / 2.0
        )

    def test_never_negative(self):
        assert joint_to_aperture_mm(STROKE_M * 10) == 0.0

    def test_fraction_to_aperture_agrees_with_two_steps(self):
        for fraction in (0.0, 0.1, 0.37, 0.9, 1.0):
            assert fraction_to_aperture_mm(fraction) == pytest.approx(
                joint_to_aperture_mm(fraction_to_joint(fraction))
            )


class TestSdkMillimetres:
    """SDK millimetres are a calibrated scale, not the physical jaw gap."""

    def test_full_scale_is_max_stroke(self):
        assert fraction_to_sdk_mm(1.0) == pytest.approx(DEFAULT_MAX_STROKE_MM)
        assert fraction_to_sdk_mm(0.0) == pytest.approx(0.0)
        assert fraction_to_sdk_mm(0.5) == pytest.approx(60.0)

    @pytest.mark.parametrize("mm", [0.0, 29.85, 60.0, 120.0])
    def test_round_trip(self, mm):
        assert fraction_to_sdk_mm(sdk_mm_to_fraction(mm)) == pytest.approx(mm)

    def test_custom_stroke(self):
        assert fraction_to_sdk_mm(0.5, max_stroke_mm=200.0) == pytest.approx(100.0)
        assert sdk_mm_to_fraction(100.0, max_stroke_mm=200.0) == pytest.approx(0.5)

    def test_out_of_range_is_clamped(self):
        assert sdk_mm_to_fraction(-5.0) == 0.0
        assert sdk_mm_to_fraction(500.0) == 1.0

    def test_zero_stroke_is_a_programming_error(self):
        with pytest.raises(ValueError):
            sdk_mm_to_fraction(10.0, max_stroke_mm=0.0)
        with pytest.raises(ValueError):
            sdk_mm_to_fraction(10.0, max_stroke_mm=-1.0)

    def test_sdk_mm_is_not_the_physical_aperture(self):
        """The whole reason the fraction exists: the two scales differ."""
        assert fraction_to_sdk_mm(1.0) == pytest.approx(120.0)
        assert fraction_to_aperture_mm(1.0) == pytest.approx(87.0)
        assert fraction_to_sdk_mm(1.0) != pytest.approx(
            fraction_to_aperture_mm(1.0), abs=1.0
        )
