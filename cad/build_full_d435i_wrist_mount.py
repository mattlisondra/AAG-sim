#!/usr/bin/env python3
"""Fuse the D435i carrier to the official YAM bracket as one printable mount.

The output directly incorporates the MakerWorld bracket mesh and is therefore
kept in the separately attributed CC BY-NC-SA 4.0 derived-output directory.
"""

from __future__ import annotations

import argparse
import math
import tempfile
from pathlib import Path

try:
    import numpy as np
    import trimesh
    from cadquery import exporters
except ImportError as exc:  # pragma: no cover - CAD-only environment
    raise SystemExit(
        "cadquery, numpy, trimesh, and manifold3d are required; see cad/README.md"
    ) from exc

from check_official_bracket_fit import (
    DEFAULT_CAMERA_MOUNT_IMAGE_UP,
    DEFAULT_MATCH_DISTANCE,
    DEFAULT_REFERENCE_DISTANCE,
    DEFAULT_VIEW_DOWN_ANGLE_DEG,
    EXPECTED_REFERENCE_SHA256,
    adapter_to_bracket_transform,
    camera_envelope,
    collision_volume,
    file_sha256,
)
from d435i_yam_wrist_adapter import (
    D405_BODY_DEPTH,
    D405_COLOR_AXIS_FROM_MOUNT,
    D435I_BODY_DEPTH,
    D435I_COLOR_AXIS_FROM_MOUNT,
    D435I_M3_PITCH,
    setback_mount,
)

ARM_HOLE_YZ = ((-46.0, 5.0), (-6.0, 5.0))


def through_hole_probe(
    *,
    radius_mm: float,
    height_mm: float,
    center: tuple[float, float, float],
    axis: str,
) -> trimesh.Trimesh:
    probe = trimesh.creation.cylinder(radius=radius_mm, height=height_mm, sections=48)
    if axis == "x":
        probe.apply_transform(
            trimesh.transformations.rotation_matrix(math.pi / 2.0, [0.0, 1.0, 0.0])
        )
    elif axis != "z":
        raise ValueError("axis must be 'x' or 'z'")
    probe.apply_translation(center)
    return probe


def verify_arm_holes(full_mount: trimesh.Trimesh) -> list[float]:
    results = []
    for y, z in ARM_HOLE_YZ:
        probe = through_hole_probe(
            radius_mm=1.45,
            height_mm=60.0,
            center=(0.0, y, z),
            axis="x",
        )
        results.append(collision_volume(full_mount, probe))
    return results


def verify_camera_holes(
    full_mount: trimesh.Trimesh,
    *,
    side: str,
    transform: np.ndarray,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float,
) -> list[float]:
    extra_ray = match_distance_mm - reference_distance_mm
    view_down_angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(view_down_angle)
    ideal_image_up = extra_ray * math.sin(view_down_angle)
    camera_rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    aim = math.atan2(
        camera_mount_image_up_mm - ideal_image_up,
        match_distance_mm,
    ) + math.radians(camera_pitch_trim_deg)
    sign = -1.0 if side == "left" else 1.0
    body_center_x = sign * (D435I_COLOR_AXIS_FROM_MOUNT - D405_COLOR_AXIS_FROM_MOUNT)
    hole_xs = (
        body_center_x - D435I_M3_PITCH / 2.0,
        body_center_x + D435I_M3_PITCH / 2.0,
    )
    pivot = [0.0, camera_mount_image_up_mm, camera_rear_z]
    carrier_rotation = trimesh.transformations.rotation_matrix(aim, [1.0, 0.0, 0.0], pivot)

    results = []
    for x in hole_xs:
        probe = through_hole_probe(
            radius_mm=1.45,
            height_mm=9.0,
            center=(x, camera_mount_image_up_mm, camera_rear_z - 3.0),
            axis="z",
        )
        probe.apply_transform(carrier_rotation)
        probe.apply_transform(transform)
        results.append(collision_volume(full_mount, probe))
    return results


def build_one(
    *,
    bracket: trimesh.Trimesh,
    side: str,
    output_dir: Path,
    fusion_overlap_mm: float,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float,
) -> None:
    sign = -1 if side == "left" else 1
    carrier = setback_mount(
        outboard_sign=sign,
        reference_distance_mm=reference_distance_mm,
        match_distance_mm=match_distance_mm,
        view_down_angle_deg=view_down_angle_deg,
        camera_mount_image_up_mm=camera_mount_image_up_mm,
        include_d405_insert_pockets=False,
        base_overlap_mm=fusion_overlap_mm,
        camera_pitch_trim_deg=camera_pitch_trim_deg,
    )

    transform = adapter_to_bracket_transform()
    with tempfile.TemporaryDirectory(prefix="aag-yam-full-mount-") as temporary:
        carrier_path = Path(temporary) / f"{side}-carrier.stl"
        exporters.export(carrier, str(carrier_path), tolerance=0.04, angularTolerance=0.1)
        carrier_mesh = trimesh.load_mesh(carrier_path, process=True)
    carrier_mesh.apply_transform(transform)

    fusion_volume = collision_volume(bracket, carrier_mesh)
    if fusion_volume <= 0.01:
        raise RuntimeError(f"{side}: carrier does not overlap bracket for a solid fusion")

    full_mount = trimesh.boolean.union([bracket, carrier_mesh], engine="manifold")
    if full_mount is None or full_mount.is_empty:
        raise RuntimeError(f"{side}: mesh union failed")
    components = full_mount.split(only_watertight=False)
    if not full_mount.is_watertight or len(components) != 1:
        raise RuntimeError(
            f"{side}: expected one watertight component, got "
            f"watertight={full_mount.is_watertight}, components={len(components)}"
        )

    arm_hole_collisions = verify_arm_holes(full_mount)
    camera_hole_collisions = verify_camera_holes(
        full_mount,
        side=side,
        transform=transform,
        reference_distance_mm=reference_distance_mm,
        match_distance_mm=match_distance_mm,
        view_down_angle_deg=view_down_angle_deg,
        camera_mount_image_up_mm=camera_mount_image_up_mm,
        camera_pitch_trim_deg=camera_pitch_trim_deg,
    )
    camera = camera_envelope(
        side,
        reference_distance_mm=reference_distance_mm,
        match_distance_mm=match_distance_mm,
        view_down_angle_deg=view_down_angle_deg,
        camera_mount_image_up_mm=camera_mount_image_up_mm,
        camera_pitch_trim_deg=camera_pitch_trim_deg,
    )
    camera.apply_transform(transform)
    camera_collision = collision_volume(full_mount, camera)

    all_clearance = arm_hole_collisions + camera_hole_collisions + [camera_collision]
    if max(all_clearance) > 0.01:
        raise RuntimeError(
            f"{side}: blocked through-hole or camera-envelope overlap: {all_clearance}"
        )

    output_path = output_dir / f"yam_d435i_full_wrist_mount_{side}.stl"
    full_mount.export(output_path)
    print(
        f"{side}: fusion={fusion_volume:.3f} mm^3, volume={full_mount.volume:.3f} mm^3, "
        f"faces={len(full_mount.faces)}, arm_holes={arm_hole_collisions}, "
        f"camera_holes={camera_hole_collisions}, camera={camera_collision:.6f} mm^3"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bracket_stl", type=Path, help="locally downloaded MakerWorld STL")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).parent / "generated" / "official-bracket-derived",
    )
    parser.add_argument("--fusion-overlap-mm", type=float, default=0.5)
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
    args = parser.parse_args()

    actual_sha = file_sha256(args.bracket_stl)
    if actual_sha != EXPECTED_REFERENCE_SHA256:
        raise SystemExit(
            "reference STL hash differs from the geometry used for alignment; refusing to fuse"
        )
    bracket = trimesh.load_mesh(args.bracket_stl, process=True)
    if not bracket.is_watertight:
        raise SystemExit("reference bracket must be watertight")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    common = {
        "bracket": bracket,
        "output_dir": args.output_dir,
        "fusion_overlap_mm": args.fusion_overlap_mm,
        "reference_distance_mm": args.reference_distance_mm,
        "match_distance_mm": args.match_distance_mm,
        "view_down_angle_deg": args.view_down_angle_deg,
        "camera_mount_image_up_mm": args.camera_mount_image_up_mm,
        "camera_pitch_trim_deg": args.camera_pitch_trim_deg,
    }
    build_one(side="left", **common)
    build_one(side="right", **common)
    print(f"Exported full replacement mounts to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
