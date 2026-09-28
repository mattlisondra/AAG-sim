import numpy as np
import pytest

from aag_yam_sim.camera_profiles import load_profile
from aag_yam_sim.sim_camera_compare import (
    MountParameters,
    image_metrics,
    mount_pose_from_parameters,
    task_alignment_metrics,
)


def test_default_mount_pose_matches_raw_candidate_profile():
    reference = load_profile("molmoact2-reference").cameras["left_cam"]
    candidate = load_profile("d435i-wrist-cad-raw-nominal").cameras["left_cam"]
    position, quaternion, details = mount_pose_from_parameters(reference, MountParameters())

    assert position == pytest.approx(candidate.position_m, abs=1e-9)
    assert quaternion == pytest.approx(candidate.quaternion_wxyz, abs=1e-9)
    assert details["optical_setback_mm"] == pytest.approx(46.6633868261)
    assert details["total_pitch_deg"] == pytest.approx(2.3122907863)


def test_pitch_trim_changes_orientation_but_not_position():
    reference = load_profile("molmoact2-reference").cameras["left_cam"]
    base_position, base_quaternion, _ = mount_pose_from_parameters(reference, MountParameters())
    position, quaternion, details = mount_pose_from_parameters(
        reference,
        MountParameters(pitch_trim_deg=1.25),
    )

    assert position == pytest.approx(base_position)
    assert not np.allclose(quaternion, base_quaternion)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)
    assert details["total_pitch_deg"] == pytest.approx(3.5622907863)


def test_image_metrics_are_exact_for_identical_images():
    image = np.arange(8 * 9 * 3, dtype=np.uint8).reshape(8, 9, 3)
    metrics = image_metrics(image, image.copy())

    assert metrics["score"] == 0.0
    assert metrics["mae"] == 0.0
    assert metrics["rmse"] == 0.0
    assert metrics["psnr_db"] == float("inf")
    assert metrics["gray_ncc"] == pytest.approx(1.0)


def test_image_metrics_penalize_a_shifted_image():
    image = np.zeros((24, 32, 3), dtype=np.uint8)
    image[6:18, 10:22] = 255
    shifted = np.roll(image, 3, axis=1)

    metrics = image_metrics(image, shifted)
    assert metrics["score"] > 0
    assert metrics["mae"] > 0
    assert metrics["edge_mae"] > 0
    assert metrics["gray_ncc"] < 1


def test_task_alignment_metrics_expose_shifted_gripper_geometry():
    rgb = np.zeros((32, 48, 3), dtype=np.uint8)
    segmentation = np.zeros((32, 48), dtype=np.int16)
    segmentation[18:31, 20:28] = 10
    segmentation[8:16, 32:44] = 25

    exact = task_alignment_metrics(
        rgb,
        rgb,
        segmentation,
        segmentation,
        finger_ids=(10,),
        task_ids=(25,),
    )
    shifted = task_alignment_metrics(
        rgb,
        rgb,
        segmentation,
        np.roll(segmentation, 5, axis=1),
        finger_ids=(10,),
        task_ids=(25,),
    )

    assert exact["score"] == 0.0
    assert exact["finger_iou"] == 1.0
    assert exact["task_iou"] == 1.0
    assert shifted["score"] > exact["score"]
    assert shifted["finger_iou"] < exact["finger_iou"]
    assert shifted["task_iou"] < exact["task_iou"]
