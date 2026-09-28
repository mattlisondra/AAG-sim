import numpy as np
import pytest

from aag_yam_sim.camera_profiles import available_profiles, intrinsic_from_hfov, load_profile
from aag_yam_sim.contracts import YAM_CONTRACT


def test_named_profiles_are_valid_and_complete():
    paths = available_profiles()
    assert {path.stem for path in paths} >= {
        "molmoact2-reference",
        "d435i-all-nominal",
        "d435i-wrist-raw-rigid-optimized",
    }
    for path in paths:
        profile = load_profile(path.stem)
        assert tuple(profile.cameras) == YAM_CONTRACT.camera_keys
        for camera in profile.cameras.values():
            assert (camera.width, camera.height) == (640, 360)
            assert camera.intrinsic.shape == (3, 3)


def test_reference_intrinsics_match_upstream_formula():
    profile = load_profile("molmoact2-reference")
    assert np.allclose(profile.cameras["top_cam"].intrinsic, intrinsic_from_hfov(640, 360, 69.4))
    assert np.allclose(profile.cameras["left_cam"].intrinsic, intrinsic_from_hfov(640, 360, 87.0))
    assert np.allclose(profile.cameras["right_cam"].intrinsic, intrinsic_from_hfov(640, 360, 87.0))


def test_physical_d405_nominal_profile_is_not_upstream_square_pixel_approximation():
    profile = load_profile("d405-wrist-physical-nominal")
    intrinsic = profile.cameras["left_cam"].intrinsic

    assert intrinsic[0, 0] == np.float32(355.3960047453417)
    assert intrinsic[1, 1] == np.float32(324.72859594885625)
    assert intrinsic[0, 0] != intrinsic[1, 1]


def test_raw_rigid_optimized_profile_keeps_distinct_mirrored_wrist_poses():
    profile = load_profile("d435i-wrist-raw-rigid-optimized")
    left = profile.cameras["left_cam"]
    right = profile.cameras["right_cam"]

    assert left.position_m[0] == pytest.approx(-right.position_m[0], abs=2e-9)
    assert left.position_m[1:] == pytest.approx(right.position_m[1:], abs=2e-9)
    assert left.intrinsic[0, 0] == right.intrinsic[0, 0]
