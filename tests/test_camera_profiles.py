import numpy as np

from aag_yam_sim.camera_profiles import available_profiles, intrinsic_from_hfov, load_profile
from aag_yam_sim.contracts import YAM_CONTRACT


def test_named_profiles_are_valid_and_complete():
    paths = available_profiles()
    assert {path.stem for path in paths} >= {"molmoact2-reference", "d435i-all-nominal"}
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
