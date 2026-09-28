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


EXPECTED_REFERENCE_SHA256 = (
    "c912eb55577ce157fb8b2cc11cb7baf350383baba564007fec4ce0a7b8ac0de2"
)

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
    transform[:3, :3] = np.column_stack(
        [LOCAL_X_IN_BRACKET, LOCAL_Y_IN_BRACKET, normal_out]
    )
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
) -> trimesh.Trimesh:
    sign = -1.0 if side == "left" else 1.0
    extra_ray = match_distance_mm - reference_distance_mm
    view_down_angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(view_down_angle)
    ideal_image_up = extra_ray * math.sin(view_down_angle)
    rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    camera = trimesh.creation.box(extents=D435I_BODY)
    camera.apply_translation(
        [
            sign * D435I_BODY_CENTER_OFFSET,
            camera_mount_image_up_mm,
            rear_z + D435I_BODY[2] / 2.0,
        ]
    )

    aim = math.atan2(
        camera_mount_image_up_mm - ideal_image_up,
        match_distance_mm,
    )
    rotation = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(aim), -math.sin(aim)],
            [0.0, math.sin(aim), math.cos(aim)],
        ]
    )
    pivot = np.array([0.0, camera_mount_image_up_mm, rear_z])
    camera.vertices = (camera.vertices - pivot) @ rotation.T + pivot
    return camera


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
        )
        camera.apply_transform(transform)
        camera_collision = collision_volume(bracket, camera)

        print(
            f"{side}: adapter={plastic_collision:.6f} mm^3, "
            f"D435i envelope={camera_collision:.6f} mm^3"
        )
        failed |= max(plastic_collision, camera_collision) > args.tolerance_mm3

    if failed:
        print(f"FAIL: collision exceeds {args.tolerance_mm3:.6f} mm^3 tolerance")
        return 1
    print(f"PASS: no collision above {args.tolerance_mm3:.6f} mm^3 tolerance")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
