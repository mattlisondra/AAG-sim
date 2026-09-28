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
    camera_view_frustum,
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


def rectangular_beam(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness_mm: float,
) -> trimesh.Trimesh:
    """Create a square-section beam between two adapter-frame points."""
    start_point = np.asarray(start, dtype=float)
    end_point = np.asarray(end, dtype=float)
    delta = end_point - start_point
    length = float(np.linalg.norm(delta))
    if length <= 0.0 or thickness_mm <= 0.0:
        raise ValueError("beam length and thickness must be positive")
    beam = trimesh.creation.box(extents=[thickness_mm, thickness_mm, length])
    beam.apply_transform(trimesh.geometry.align_vectors([0.0, 0.0, 1.0], delta))
    beam.apply_translation((start_point + end_point) / 2.0)
    return beam


def view_clearance_braces(
    side: str,
    *,
    thickness_mm: float,
    lower_y: float = -46.0,
    lower_z: float = -12.0,
) -> list[trimesh.Trimesh]:
    """Return an intentionally stout outboard truss routed below the RGB view."""
    if side == "left":
        support_x = (44.0, 56.0)
        source_x = 18.0
        outer_x = 62.0
    elif side == "right":
        support_x = (-42.0, -30.0)
        # The source bracket is asymmetric and carries much more material on
        # +X at the below-view anchor height. Reach across that solid web for
        # a broad fusion patch before turning outboard toward the two rails.
        source_x = 18.0
        outer_x = -48.0
    else:
        raise ValueError("side must be 'left' or 'right'")

    carrier_y = 8.0
    carrier_z = -68.0
    return [
        rectangular_beam(
            (source_x, lower_y, lower_z),
            (outer_x, lower_y, lower_z),
            thickness_mm,
        ),
        *[
            rectangular_beam(
                (x, lower_y, lower_z),
                (x, carrier_y, carrier_z),
                thickness_mm,
            )
            for x in support_x
        ],
    ]


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
    camera_lateral_mm: float,
    camera_yaw_deg: float,
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
    optical_axis_x = sign * camera_lateral_mm
    body_center_x = sign * (
        D435I_COLOR_AXIS_FROM_MOUNT - D405_COLOR_AXIS_FROM_MOUNT + camera_lateral_mm
    )
    hole_xs = (
        body_center_x - D435I_M3_PITCH / 2.0,
        body_center_x + D435I_M3_PITCH / 2.0,
    )
    pivot = [0.0, camera_mount_image_up_mm, camera_rear_z]
    carrier_rotation = trimesh.transformations.rotation_matrix(aim, [1.0, 0.0, 0.0], pivot)
    yaw_rotation = trimesh.transformations.rotation_matrix(
        math.radians(sign * camera_yaw_deg),
        [0.0, 1.0, 0.0],
        [optical_axis_x, camera_mount_image_up_mm, camera_rear_z],
    )

    results = []
    for x in hole_xs:
        probe = through_hole_probe(
            radius_mm=1.45,
            height_mm=9.0,
            center=(x, camera_mount_image_up_mm, camera_rear_z - 3.0),
            axis="z",
        )
        probe.apply_transform(carrier_rotation)
        probe.apply_transform(yaw_rotation)
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
    camera_lateral_mm: float,
    camera_yaw_deg: float,
    camera_clearance_relief_mm: float,
    view_clearance_margin_deg: float,
    lens_recess_mm: float,
    view_brace_thickness_mm: float,
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
        camera_lateral_mm=camera_lateral_mm,
        camera_yaw_deg=camera_yaw_deg,
    )

    transform = adapter_to_bracket_transform()
    with tempfile.TemporaryDirectory(prefix="aag-yam-full-mount-") as temporary:
        carrier_path = Path(temporary) / f"{side}-carrier.stl"
        exporters.export(carrier, str(carrier_path), tolerance=0.04, angularTolerance=0.1)
        carrier_mesh = trimesh.load_mesh(carrier_path, process=True)
    carrier_mesh.apply_transform(transform)

    working_bracket = bracket
    removed_volume = 0.0
    if camera_clearance_relief_mm > 0.0:
        relief = camera_envelope(
            side,
            reference_distance_mm=reference_distance_mm,
            match_distance_mm=match_distance_mm,
            view_down_angle_deg=view_down_angle_deg,
            camera_mount_image_up_mm=camera_mount_image_up_mm,
            camera_pitch_trim_deg=camera_pitch_trim_deg,
            camera_lateral_mm=camera_lateral_mm,
            camera_yaw_deg=camera_yaw_deg,
            clearance_mm=camera_clearance_relief_mm,
        )
        relief.apply_transform(transform)
        working_bracket = trimesh.boolean.difference([bracket, relief], engine="manifold")
        if working_bracket is None or working_bracket.is_empty:
            raise RuntimeError(f"{side}: camera-clearance relief removed the bracket")
        if (
            not working_bracket.is_watertight
            or len(working_bracket.split(only_watertight=False)) != 1
        ):
            raise RuntimeError(f"{side}: camera-clearance relief split the bracket")
        removed_volume = float(bracket.volume - working_bracket.volume)

    fusion_volume = collision_volume(working_bracket, carrier_mesh)
    if fusion_volume <= 0.01:
        raise RuntimeError(f"{side}: carrier does not overlap bracket for a solid fusion")

    full_mount = trimesh.boolean.union([working_bracket, carrier_mesh], engine="manifold")
    if full_mount is None or full_mount.is_empty:
        raise RuntimeError(f"{side}: mesh union failed")
    view_intrusion_removed = 0.0
    view_brace_volume = 0.0
    view_clearance: trimesh.Trimesh | None = None
    if view_clearance_margin_deg > 0.0:
        view_clearance = camera_view_frustum(
            side,
            reference_distance_mm=reference_distance_mm,
            match_distance_mm=match_distance_mm,
            view_down_angle_deg=view_down_angle_deg,
            camera_mount_image_up_mm=camera_mount_image_up_mm,
            camera_pitch_trim_deg=camera_pitch_trim_deg,
            camera_lateral_mm=camera_lateral_mm,
            camera_yaw_deg=camera_yaw_deg,
            lens_recess_mm=lens_recess_mm,
            angular_margin_deg=view_clearance_margin_deg,
        )
        view_clearance.apply_transform(transform)
        cleared_mount = trimesh.boolean.difference([full_mount, view_clearance], engine="manifold")
        if cleared_mount is None or cleared_mount.is_empty:
            raise RuntimeError(f"{side}: RGB view clearance removed the mount")
        view_intrusion_removed = float(full_mount.volume - cleared_mount.volume)

        cleared_components = sorted(
            cleared_mount.split(only_watertight=False),
            key=lambda component: component.volume,
            reverse=True,
        )
        if len(cleared_components) < 2:
            raise RuntimeError(
                f"{side}: expected the view cut to leave two functional bodies, "
                f"got {len(cleared_components)}"
            )
        # The optical-tunnel cut can leave small detached scraps of the former
        # central support. They are intentionally omitted from the print; V2
        # reconnects the two functional bodies with a new outboard truss.
        discarded_view_scraps = float(sum(component.volume for component in cleared_components[2:]))
        cleared_components = cleared_components[:2]
        cleared_mount = trimesh.util.concatenate(cleared_components)

        routes: list[
            tuple[
                float,
                float,
                float,
                list[trimesh.Trimesh],
                list[list[float]],
            ]
        ] = []
        for lower_y, lower_z in (
            (-46.0, -12.0),
            (-52.0, -12.0),
            (-40.0, -12.0),
            (-46.0, -20.0),
            (-52.0, -20.0),
            (-40.0, -20.0),
            (-34.0, -20.0),
            (-52.0, -4.0),
            (-46.0, -4.0),
            (-40.0, -4.0),
            (-34.0, -4.0),
        ):
            candidate = view_clearance_braces(
                side,
                thickness_mm=view_brace_thickness_mm,
                lower_y=lower_y,
                lower_z=lower_z,
            )
            for brace in candidate:
                brace.apply_transform(transform)
            contacts = [
                [collision_volume(brace, component) for component in cleared_components]
                for brace in candidate
            ]
            view_contacts = [collision_volume(brace, view_clearance) for brace in candidate]
            touches_first = any(row[0] > 0.1 for row in contacts)
            touches_second = any(row[1] > 0.1 for row in contacts)
            if touches_first and touches_second and max(view_contacts) <= 0.01:
                contact_first = sum(row[0] for row in contacts)
                contact_second = sum(row[1] for row in contacts)
                routes.append(
                    (
                        min(contact_first, contact_second),
                        lower_y,
                        lower_z,
                        candidate,
                        contacts,
                    )
                )
        if not routes:
            raise RuntimeError(f"{side}: no view-safe brace route connects both mount bodies")
        _, lower_y, lower_z, braces, brace_contacts = max(
            routes,
            key=lambda route: route[0],
        )
        print(
            f"{side}: view-brace route y={lower_y:.1f}, z={lower_z:.1f}, "
            f"contacts={brace_contacts}, discarded_scraps={discarded_view_scraps:.3f} mm^3"
        )
        view_brace_volume = float(sum(brace.volume for brace in braces))
        full_mount = trimesh.boolean.union([cleared_mount, *braces], engine="manifold")
        if full_mount is None or full_mount.is_empty:
            raise RuntimeError(f"{side}: failed to union view-clearance braces")
        # Trim numerical or fillet-edge intrusion after union so the stated
        # angular margin is a hard final-mesh guarantee.
        full_mount = trimesh.boolean.difference([full_mount, view_clearance], engine="manifold")
        if full_mount is None or full_mount.is_empty:
            raise RuntimeError(f"{side}: final RGB view trim removed the mount")

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
        camera_lateral_mm=camera_lateral_mm,
        camera_yaw_deg=camera_yaw_deg,
    )
    camera = camera_envelope(
        side,
        reference_distance_mm=reference_distance_mm,
        match_distance_mm=match_distance_mm,
        view_down_angle_deg=view_down_angle_deg,
        camera_mount_image_up_mm=camera_mount_image_up_mm,
        camera_pitch_trim_deg=camera_pitch_trim_deg,
        camera_lateral_mm=camera_lateral_mm,
        camera_yaw_deg=camera_yaw_deg,
    )
    camera.apply_transform(transform)
    camera_collision = collision_volume(full_mount, camera)

    nominal_view = camera_view_frustum(
        side,
        reference_distance_mm=reference_distance_mm,
        match_distance_mm=match_distance_mm,
        view_down_angle_deg=view_down_angle_deg,
        camera_mount_image_up_mm=camera_mount_image_up_mm,
        camera_pitch_trim_deg=camera_pitch_trim_deg,
        camera_lateral_mm=camera_lateral_mm,
        camera_yaw_deg=camera_yaw_deg,
        lens_recess_mm=lens_recess_mm,
        angular_margin_deg=0.0,
    )
    nominal_view.apply_transform(transform)
    view_collision = collision_volume(full_mount, nominal_view)
    margin_view_collision = (
        collision_volume(full_mount, view_clearance)
        if view_clearance is not None
        else view_collision
    )

    all_clearance = arm_hole_collisions + camera_hole_collisions + [camera_collision]
    if view_clearance_margin_deg > 0.0:
        all_clearance.extend([view_collision, margin_view_collision])
    if max(all_clearance) > 0.01:
        raise RuntimeError(
            f"{side}: blocked through-hole or camera-envelope overlap: {all_clearance}"
        )

    output_path = output_dir / f"yam_d435i_full_wrist_mount_{side}.stl"
    full_mount.export(output_path)
    print(
        f"{side}: fusion={fusion_volume:.3f} mm^3, volume={full_mount.volume:.3f} mm^3, "
        f"relief_removed={removed_volume:.3f} mm^3, "
        f"view_removed={view_intrusion_removed:.3f} mm^3, "
        f"view_braces={view_brace_volume:.3f} mm^3, "
        f"faces={len(full_mount.faces)}, arm_holes={arm_hole_collisions}, "
        f"camera_holes={camera_hole_collisions}, camera={camera_collision:.6f} mm^3, "
        f"nominal_view={view_collision:.6f} mm^3, "
        f"margin_view={margin_view_collision:.6f} mm^3"
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
    parser.add_argument("--camera-lateral-mm", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)
    parser.add_argument(
        "--camera-clearance-relief-mm",
        type=float,
        default=0.0,
        help="experimental clearance subtracted from the replaceable source bracket",
    )
    parser.add_argument(
        "--view-clearance-margin-deg",
        type=float,
        default=0.0,
        help="clear the RGB frustum plus this angular margin at every image edge",
    )
    parser.add_argument(
        "--lens-recess-mm",
        type=float,
        default=3.0,
        help="conservative RGB optical-center recess behind the housing front plane",
    )
    parser.add_argument("--view-brace-thickness-mm", type=float, default=12.0)
    args = parser.parse_args()
    if args.camera_clearance_relief_mm < 0.0:
        raise SystemExit("--camera-clearance-relief-mm must be non-negative")
    if args.view_clearance_margin_deg < 0.0:
        raise SystemExit("--view-clearance-margin-deg must be non-negative")
    if args.view_brace_thickness_mm <= 0.0:
        raise SystemExit("--view-brace-thickness-mm must be positive")

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
        "camera_lateral_mm": args.camera_lateral_mm,
        "camera_yaw_deg": args.camera_yaw_deg,
        "camera_clearance_relief_mm": args.camera_clearance_relief_mm,
        "view_clearance_margin_deg": args.view_clearance_margin_deg,
        "lens_recess_mm": args.lens_recess_mm,
        "view_brace_thickness_mm": args.view_brace_thickness_mm,
    }
    build_one(side="left", **common)
    build_one(side="right", **common)
    print(f"Exported full replacement mounts to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
