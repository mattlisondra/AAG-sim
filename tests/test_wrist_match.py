import numpy as np
import pytest

from aag_yam_sim.wrist_match import (
    D405_COLOR_NOMINAL,
    D435I_COLOR_NOMINAL,
    PinholeIntrinsics,
    apply_match_plan,
    compute_match_plan,
    rotate_intrinsics_180,
)


def test_nominal_plan_matches_d405_projection_at_working_plane():
    plan = compute_match_plan(
        D405_COLOR_NOMINAL,
        D435I_COLOR_NOMINAL,
        reference_distance_mm=116.9619168789568,
    )

    assert plan.actual_distance_scale == pytest.approx(1.4254033611)
    assert plan.actual_distance_mm == pytest.approx(166.7179094)
    assert (plan.crop_left, plan.crop_top, plan.crop_width, plan.crop_height) == (
        28,
        0,
        584,
        360,
    )
    assert plan.effective.fx / plan.actual_distance_scale == pytest.approx(
        D405_COLOR_NOMINAL.fx, rel=2e-3
    )
    assert plan.effective.fy / plan.actual_distance_scale == pytest.approx(
        D405_COLOR_NOMINAL.fy, rel=2e-3
    )


def test_plan_rejects_distance_that_would_need_missing_pixels():
    with pytest.raises(ValueError, match="too close"):
        compute_match_plan(
            D405_COLOR_NOMINAL,
            D435I_COLOR_NOMINAL,
            reference_distance_mm=117.0,
            actual_distance_mm=150.0,
        )


def test_crop_maps_source_principal_point_to_reference_principal_point():
    reference = PinholeIntrinsics(640, 360, 355.0, 325.0, 315.0, 177.0)
    source = PinholeIntrinsics(640, 360, 463.0, 463.0, 323.0, 181.0)
    plan = compute_match_plan(reference, source, reference_distance_mm=117.0)
    assert plan.effective.cx == pytest.approx(reference.cx, abs=1.1)
    # The limiting vertical axis uses the full source height, so an off-center
    # factory principal point is reported rather than hidden with synthetic
    # border pixels.  Correct the remaining offset through physical aiming.
    assert plan.crop_top == 0
    assert plan.effective.cy == pytest.approx(source.cy)


def test_numpy_transform_preserves_contract_and_rotation():
    source = PinholeIntrinsics(8, 4, 8.0, 8.0, 4.0, 2.0)
    reference = PinholeIntrinsics(8, 4, 4.0, 4.0, 4.0, 2.0)
    plan = compute_match_plan(reference, source, reference_distance_mm=10.0)
    frame = np.arange(8 * 4 * 3, dtype=np.uint8).reshape(4, 8, 3)
    result = apply_match_plan(frame, plan, rotate_180=True, backend="numpy")
    assert result.shape == (4, 8, 3)
    assert result.dtype == np.uint8


def test_rotate_intrinsics_uses_discrete_pixel_coordinates():
    original = PinholeIntrinsics(640, 360, 463.0, 464.0, 322.25, 178.75)
    rotated = rotate_intrinsics_180(original)
    assert rotated.fx == original.fx
    assert rotated.fy == original.fy
    assert rotated.cx == pytest.approx(316.75)
    assert rotated.cy == pytest.approx(180.25)
