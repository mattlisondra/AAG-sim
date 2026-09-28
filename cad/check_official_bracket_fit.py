#!/usr/bin/env python3
"""Check the generated adapters against the published YAM D405 bracket STL.

The reference STL is intentionally not redistributed here: MakerWorld marks it
CC BY-NC-SA 4.0.  Pass a locally downloaded copy to this script.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

try:
    import numpy as np
    import trimesh
except ImportError as exc:  # pragma: no cover - CAD-only environment
    raise SystemExit(
        "numpy, trimesh, and manifold3d are required; see the command in cad/README.md"
    ) from exc


EXPECTED_REFERENCE_SHA256 = "c912eb55577ce157fb8b2cc11cb7baf350383baba564007fec4ce0a7b8ac0de2"

# Local frame recovered from the published bracket's planar D405 contact face.
# X runs across the camera, Y is image-up, and Z points out toward the scene.
BRACKET_PLANE_NORMAL_BACK = np.array([0.90630779, 0.0, 0.42261826])
LOCAL_X_IN_BRACKET = np.array([0.0, -1.0, 0.0])
LOCAL_Y_IN_BRACKET = np.array([-0.42261826, 0.0, 0.90630779])
CONTACT_PLANE_OFFSET = 13.722
INTERFACE_SEED_POINT = np.array([-10.0, -17.0, 56.947])

# Same physical-envelope assumptions as d435i_yam_wrist_adapter.py.
D435I_BODY = np.array([90.0, 25.0, 25.05])
D435I_BODY_CENTER_OFFSET = 23.5
D405_BODY_DEPTH = 23.0
D435I_BODY_DEPTH = 25.05
D435I_COLOR_HFOV_DEG = 69.4
D435I_COLOR_VFOV_DEG = 42.5
DEFAULT_REFERENCE_DISTANCE = 116.9619168789568
DEFAULT_MATCH_DISTANCE = 166.71790944273448
DEFAULT_VIEW_DOWN_ANGLE_DEG = 20.307327540589718
DEFAULT_CAMERA_MOUNT_IMAGE_UP = 24.0


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def adapter_to_bracket_transform() -> np.ndarray:
    normal_out = -BRACKET_PLANE_NORMAL_BACK
    origin = INTERFACE_SEED_POINT + BRACKET_PLANE_NORMAL_BACK * (
        CONTACT_PLANE_OFFSET - BRACKET_PLANE_NORMAL_BACK @ INTERFACE_SEED_POINT
    )
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack([LOCAL_X_IN_BRACKET, LOCAL_Y_IN_BRACKET, normal_out])
    transform[:3, 3] = origin
    return transform


def collision_volume(first: trimesh.Trimesh, second: trimesh.Trimesh) -> float:
    result = trimesh.boolean.intersection([first, second], engine="manifold")
    if result is None or result.is_empty:
        return 0.0
    return float(result.volume)


def camera_envelope(
    side: str,
    *,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float = 0.0,
    camera_lateral_mm: float = 0.0,
    camera_yaw_deg: float = 0.0,
    clearance_mm: float = 0.0,
) -> trimesh.Trimesh:
    if clearance_mm < 0.0:
        raise ValueError("clearance_mm must be non-negative")
    sign = -1.0 if side == "left" else 1.0
    extra_ray = match_distance_mm - reference_distance_mm
    view_down_angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(view_down_angle)
    ideal_image_up = extra_ray * math.sin(view_down_angle)
    rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    camera = trimesh.creation.box(extents=D435I_BODY + 2.0 * clearance_mm)
    camera.apply_translation(
        [
            sign * (D435I_BODY_CENTER_OFFSET + camera_lateral_mm),
            camera_mount_image_up_mm,
            rear_z + D435I_BODY[2] / 2.0,
        ]
    )

    aim = math.atan2(
        camera_mount_image_up_mm - ideal_image_up,
        match_distance_mm,
    ) + math.radians(camera_pitch_trim_deg)
    pitch_pivot = [0.0, camera_mount_image_up_mm, rear_z]
    camera.apply_transform(
        trimesh.transformations.rotation_matrix(aim, [1.0, 0.0, 0.0], pitch_pivot)
    )
    yaw_pivot = [sign * camera_lateral_mm, camera_mount_image_up_mm, rear_z]
    camera.apply_transform(
        trimesh.transformations.rotation_matrix(
            math.radians(sign * camera_yaw_deg),
            [0.0, 1.0, 0.0],
            yaw_pivot,
        )
    )
    return camera


def camera_optical_transform(
    side: str,
    *,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float = 0.0,
    camera_lateral_mm: float = 0.0,
    camera_yaw_deg: float = 0.0,
    lens_recess_mm: float = 0.0,
) -> np.ndarray:
    """Return the D435i RGB camera-to-adapter transform.

    Camera coordinates use +X image-right, +Y image-up, and +Z forward.  A
    positive ``lens_recess_mm`` moves the assumed optical center behind the
    housing front plane, making the near-field visibility check conservative.
    """
    if side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    if lens_recess_mm < 0.0 or lens_recess_mm >= D435I_BODY_DEPTH:
        raise ValueError("lens_recess_mm must be in [0, D435I body depth)")
    sign = -1.0 if side == "left" else 1.0
    extra_ray = match_distance_mm - reference_distance_mm
    view_down_angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(view_down_angle)
    ideal_image_up = extra_ray * math.sin(view_down_angle)
    rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    aim = math.atan2(
        camera_mount_image_up_mm - ideal_image_up,
        match_distance_mm,
    ) + math.radians(camera_pitch_trim_deg)

    pitch_pivot = [0.0, camera_mount_image_up_mm, rear_z]
    pitch = trimesh.transformations.rotation_matrix(aim, [1.0, 0.0, 0.0], pitch_pivot)
    yaw_pivot = [sign * camera_lateral_mm, camera_mount_image_up_mm, rear_z]
    yaw = trimesh.transformations.rotation_matrix(
        math.radians(sign * camera_yaw_deg),
        [0.0, 1.0, 0.0],
        yaw_pivot,
    )
    posed = yaw @ pitch
    optical_center = np.array(
        [
            sign * camera_lateral_mm,
            camera_mount_image_up_mm,
            rear_z + D435I_BODY_DEPTH - lens_recess_mm,
            1.0,
        ]
    )
    transform = np.eye(4)
    transform[:3, :3] = posed[:3, :3]
    transform[:3, 3] = (posed @ optical_center)[:3]
    return transform


def camera_view_frustum(
    side: str,
    *,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float = 0.0,
    camera_lateral_mm: float = 0.0,
    camera_yaw_deg: float = 0.0,
    lens_recess_mm: float = 0.0,
    angular_margin_deg: float = 0.0,
    near_mm: float = 0.25,
    far_mm: float = 250.0,
) -> trimesh.Trimesh:
    """Build a rectangular D435i RGB visibility volume in adapter coordinates."""
    if angular_margin_deg < 0.0:
        raise ValueError("angular_margin_deg must be non-negative")
    if not 0.0 < near_mm < far_mm:
        raise ValueError("frustum distances must satisfy 0 < near < far")
    half_h = math.radians(D435I_COLOR_HFOV_DEG / 2.0 + angular_margin_deg)
    half_v = math.radians(D435I_COLOR_VFOV_DEG / 2.0 + angular_margin_deg)
    if half_h >= math.pi / 2.0 or half_v >= math.pi / 2.0:
        raise ValueError("angular margin makes the frustum invalid")

    def corners(distance: float) -> list[list[float]]:
        x = distance * math.tan(half_h)
        y = distance * math.tan(half_v)
        return [
            [-x, -y, distance],
            [x, -y, distance],
            [x, y, distance],
            [-x, y, distance],
        ]

    vertices = np.asarray(corners(near_mm) + corners(far_mm), dtype=float)
    faces = np.asarray(
        [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
        ],
        dtype=np.int64,
    )
    frustum = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    frustum.apply_transform(
        camera_optical_transform(
            side,
            reference_distance_mm=reference_distance_mm,
            match_distance_mm=match_distance_mm,
            view_down_angle_deg=view_down_angle_deg,
            camera_mount_image_up_mm=camera_mount_image_up_mm,
            camera_pitch_trim_deg=camera_pitch_trim_deg,
            camera_lateral_mm=camera_lateral_mm,
            camera_yaw_deg=camera_yaw_deg,
            lens_recess_mm=lens_recess_mm,
        )
    )
    return frustum


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bracket_stl", type=Path, help="locally downloaded MakerWorld STL")
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=Path(__file__).parent / "generated",
    )
    parser.add_argument(
        "--reference-distance-mm",
        type=float,
        default=DEFAULT_REFERENCE_DISTANCE,
    )
    parser.add_argument("--match-distance-mm", type=float, default=DEFAULT_MATCH_DISTANCE)
    parser.add_argument(
        "--view-down-angle-deg",
        type=float,
        default=DEFAULT_VIEW_DOWN_ANGLE_DEG,
    )
    parser.add_argument(
        "--camera-mount-image-up-mm",
        type=float,
        default=DEFAULT_CAMERA_MOUNT_IMAGE_UP,
    )
    parser.add_argument("--camera-pitch-trim-deg", type=float, default=0.0)
    parser.add_argument("--camera-lateral-mm", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)
    parser.add_argument("--tolerance-mm3", type=float, default=0.01)
    parser.add_argument(
        "--allow-reference-mismatch",
        action="store_true",
        help="continue despite an unknown STL hash; alignment may be invalid",
    )
    args = parser.parse_args()

    actual_sha = file_sha256(args.bracket_stl)
    print(f"reference SHA-256: {actual_sha}")
    if actual_sha != EXPECTED_REFERENCE_SHA256:
        message = "reference differs from the STL used to design this adapter"
        if not args.allow_reference_mismatch:
            raise SystemExit(f"ERROR: {message}")
        print(f"WARNING: {message}; alignment may be invalid")

    bracket = trimesh.load_mesh(args.bracket_stl, process=True)
    if not bracket.is_watertight:
        raise SystemExit("reference bracket is not watertight; boolean result is unreliable")

    transform = adapter_to_bracket_transform()
    failed = False
    for side in ("left", "right"):
        adapter_path = args.generated_dir / f"yam_d435i_{side}.stl"
        adapter = trimesh.load_mesh(adapter_path, process=True)
        adapter.apply_transform(transform)
        plastic_collision = collision_volume(bracket, adapter)

        camera = camera_envelope(
            side,
            reference_distance_mm=args.reference_distance_mm,
            match_distance_mm=args.match_distance_mm,
            view_down_angle_deg=args.view_down_angle_deg,
            camera_mount_image_up_mm=args.camera_mount_image_up_mm,
            camera_pitch_trim_deg=args.camera_pitch_trim_deg,
            camera_lateral_mm=args.camera_lateral_mm,
            camera_yaw_deg=args.camera_yaw_deg,
        )
        camera.apply_transform(transform)
        bracket_camera_collision = collision_volume(bracket, camera)
        carrier_camera_collision = collision_volume(adapter, camera)

        print(
            f"{side}: adapter={plastic_collision:.6f} mm^3, "
            f"bracket-to-D435i={bracket_camera_collision:.6f} mm^3, "
            f"carrier-to-D435i={carrier_camera_collision:.6f} mm^3"
        )
        failed |= (
            max(plastic_collision, bracket_camera_collision, carrier_camera_collision)
            > args.tolerance_mm3
        )

    if failed:
        print(f"FAIL: collision exceeds {args.tolerance_mm3:.6f} mm^3 tolerance")
        return 1
    print(f"PASS: no collision above {args.tolerance_mm3:.6f} mm^3 tolerance")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
