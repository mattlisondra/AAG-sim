#!/usr/bin/env python3
"""Parametric D405-bracket to D435i wrist-camera adapter for the I2RT YAM.

Coordinate convention used by the model:

* X: lateral across the gripper/camera image;
* Y: toward the top of the wrist image (roughly back toward the wrist);
* Z: forward along the nominal camera optical direction, toward the work area.

The existing D405 bracket contact plane is Z=0.  The D435i carrier is moved
behind that plane and toward +Y so its RGB optical center is farther from the
nominal grasp point while retaining the reference bearing.

This is research hardware, not an I2RT/Ai2 production drawing.  The camera-side
dimensions come from the RealSense mechanical drawings.  The support routing
uses the envelope measured from i2rt robotics' published YAM D405 bracket STL;
clearance must still be verified on the user's exact gripper before motion.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

try:
    import cadquery as cq
    from cadquery import exporters
except ImportError as exc:  # pragma: no cover - exercised only in CAD environment
    raise SystemExit(
        "cadquery is required to generate the parts; run with `uv run --with cadquery`"
    ) from exc


# Authoritative camera-interface dimensions, millimetres.
D405_M3_PITCH = 20.0
D435I_M3_PITCH = 45.0
D435I_BODY_WIDTH = 90.0
D435I_BODY_HEIGHT = 25.0
D435I_BODY_DEPTH = 25.05
D405_BODY_DEPTH = 23.0
D405_COLOR_AXIS_FROM_MOUNT = 9.0
D435I_COLOR_AXIS_FROM_MOUNT = 32.5

# Nominal view match derived from 84x58 degree D405 and 69.4x42.5 degree
# D435i color FOVs plus the upstream camera-to-grasp geometry.
DEFAULT_REFERENCE_DISTANCE = 116.9619168789568
DEFAULT_MATCH_DISTANCE = 166.71790944273448
DEFAULT_VIEW_DOWN_ANGLE_DEG = 20.307327540589718
# Raising the camera slightly beyond the pinhole-optimal 17.27 mm clears the
# published YAM bracket and D435i housing envelopes.  A small downward pitch
# correction keeps the nominal grasp-point bearing unchanged.
DEFAULT_CAMERA_MOUNT_IMAGE_UP = 24.0


def rounded_plate(width: float, depth: float, thickness: float, radius: float):
    return (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def cylinders(points, diameter: float, z0: float, height: float):
    result = None
    for x, y in points:
        cylinder = (
            cq.Workplane("XY")
            .workplane(offset=z0)
            .center(x, y)
            .circle(diameter / 2.0)
            .extrude(height)
        )
        result = cylinder if result is None else result.union(cylinder)
    return result


def pitch_coupon(pitch_mm: float, *, notches: int):
    """Make a single-interface overlay gauge with tactile ID notches.

    These are deliberately separate parts: the old combined four-hole plate
    made it too easy to confuse which pair belonged to which interface.
    """
    width = pitch_mm + 16.0
    plate = rounded_plate(width, 20.0, 4.0, 3.0)
    holes = [(-pitch_mm / 2.0, 0.0), (pitch_mm / 2.0, 0.0)]
    plate = plate.cut(cylinders(holes, 3.4, -0.5, 5.0))
    for index in range(notches):
        notch_x = -width / 2.0 + 5.0 + index * 5.0
        notch = (
            cq.Workplane("XY")
            .center(notch_x, 10.0)
            .circle(1.5)
            .extrude(5.0)
            .translate((0.0, 0.0, -0.5))
        )
        plate = plate.cut(notch)
    return plate


def setback_mount(
    *,
    outboard_sign: int,
    reference_distance_mm: float = DEFAULT_REFERENCE_DISTANCE,
    match_distance_mm: float = DEFAULT_MATCH_DISTANCE,
    view_down_angle_deg: float = DEFAULT_VIEW_DOWN_ANGLE_DEG,
    camera_mount_image_up_mm: float = DEFAULT_CAMERA_MOUNT_IMAGE_UP,
    insert_diameter_mm: float = 4.6,
    include_d405_insert_pockets: bool = True,
    base_overlap_mm: float = 0.0,
    camera_pitch_trim_deg: float = 0.0,
):
    """Build one mirrored adapter.

    ``outboard_sign`` selects which side the long D435i body occupies.  Use
    +1 and -1 for opposite wrists; determine the physical left/right assignment
    with a stationary fit check because gripper and camera yaw conventions vary.
    """
    if outboard_sign not in (-1, 1):
        raise ValueError("outboard_sign must be -1 or +1")
    if match_distance_mm <= reference_distance_mm:
        raise ValueError("match distance must be farther than the D405 reference distance")
    if base_overlap_mm < 0.0:
        raise ValueError("base_overlap_mm must be non-negative")

    extra_ray = match_distance_mm - reference_distance_mm
    angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(angle)
    ideal_image_up_shift = extra_ray * math.sin(angle)
    if camera_mount_image_up_mm < ideal_image_up_shift:
        raise ValueError(
            "camera_mount_image_up_mm must not be below the pinhole-optimal "
            f"{ideal_image_up_shift:.2f} mm"
        )
    geometric_aim_correction_deg = math.degrees(
        math.atan2(camera_mount_image_up_mm - ideal_image_up_shift, match_distance_mm)
    )
    aim_correction_deg = geometric_aim_correction_deg + camera_pitch_trim_deg

    # Match the D405 color axis rather than the body center.  The D405 color
    # origin is 9 mm from its mount reference; D435i is 32.5 mm.  Mirroring the
    # camera therefore places its body/mount midpoint 23.5 mm outboard.
    body_center_x = outboard_sign * (
        D435I_COLOR_AXIS_FROM_MOUNT - D405_COLOR_AXIS_FROM_MOUNT
    )
    camera_holes = [
        (body_center_x - D435I_M3_PITCH / 2.0, camera_mount_image_up_mm),
        (body_center_x + D435I_M3_PITCH / 2.0, camera_mount_image_up_mm),
    ]

    # The compact base replaces the D405 at its confirmed 20 mm camera
    # interface.  A second crossbar sits outward of the old camera face and
    # sends the structural rails around, rather than through, the official YAM
    # bracket envelope (local X approximately -21..35 mm).
    base_width = 44.0
    base_depth = 28.0
    base_thickness = 6.0
    base = rounded_plate(
        base_width,
        base_depth,
        base_thickness + base_overlap_mm,
        4.0,
    ).translate((0.0, 0.0, -base_overlap_mm))
    if include_d405_insert_pockets:
        insert_points = [(-D405_M3_PITCH / 2.0, 0.0), (D405_M3_PITCH / 2.0, 0.0)]
        # Blind pockets open on the existing-bracket contact face (Z=0).
        base = base.cut(cylinders(insert_points, insert_diameter_mm, -0.1, 5.3))
    # Put both rails on the side opposite the D435i's long outboard housing.
    # This clears both the asymmetric YAM bracket (-21..35 mm locally) and the
    # 90 mm camera body.  Running one rail on each side would pass a rail
    # through the camera housing after the 49.8 mm optical setback.
    support_x = (44.0, 56.0) if outboard_sign == -1 else (-42.0, -30.0)
    outrigger_min_x = min(-base_width / 2.0, support_x[0] - 4.0)
    outrigger_max_x = max(base_width / 2.0, support_x[1] + 4.0)
    outrigger_center_x = (outrigger_min_x + outrigger_max_x) / 2.0
    outrigger_width = outrigger_max_x - outrigger_min_x
    outrigger_y = 5.0
    outrigger = rounded_plate(outrigger_width, 10.0, 6.0, 3.0).translate(
        (outrigger_center_x, outrigger_y, base_thickness)
    )

    # Account for the 2.05 mm difference in housing depth so the optical/front
    # planes, rather than the rear faces, receive the requested displacement.
    camera_rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    boss_height = 1.0
    carrier_thickness = 5.0
    carrier_front_z = camera_rear_z - boss_height
    carrier_back_z = carrier_front_z - carrier_thickness
    carrier_width = 86.0
    carrier_depth = 14.0  # narrow bar avoids covering the D435i rear vents
    carrier_center_x = body_center_x
    carrier_main = (
        rounded_plate(carrier_width, carrier_depth, carrier_thickness, 3.0)
        .translate((carrier_center_x, camera_mount_image_up_mm, carrier_back_z))
    )
    # A narrow side tie reaches both rails while overlapping the main carrier
    # by 10 mm.  It stays behind, rather than across, the D435i housing.
    carrier_min_x = carrier_center_x - carrier_width / 2.0
    carrier_max_x = carrier_center_x + carrier_width / 2.0
    if outboard_sign == -1:
        tie_min_x = carrier_max_x - 10.0
        tie_max_x = support_x[1] + 4.0
    else:
        tie_min_x = support_x[0] - 4.0
        tie_max_x = carrier_min_x + 10.0
    tie_width = tie_max_x - tie_min_x
    tie_center_x = (tie_min_x + tie_max_x) / 2.0
    carrier_tie = rounded_plate(tie_width, 7.0, carrier_thickness, 2.5).translate(
        (tie_center_x, camera_mount_image_up_mm, carrier_back_z)
    )
    bosses = cylinders(camera_holes, 10.0, carrier_front_z, boss_height)
    carrier = carrier_main.union(carrier_tie).union(bosses)
    camera_clearance = cylinders(camera_holes, 3.4, carrier_back_z - 0.5, 7.0)
    carrier = carrier.cut(camera_clearance)

    # The 6.73 mm extra rise required for housing clearance would move the
    # pinch point down in the image.  Rotate the camera down around its screw
    # row to retain the nominal bearing.
    carrier = carrier.rotate(
        (0.0, camera_mount_image_up_mm, camera_rear_z),
        (1.0, camera_mount_image_up_mm, camera_rear_z),
        aim_correction_deg,
    )

    # Two diagonal rails run outside the measured official bracket envelope.
    # The front outrigger and rear carrier tie provide the cross-bracing; a
    # mid-span tie would pass through the original bracket.
    base_anchor_y = outrigger_y
    base_anchor_z = base_thickness + 3.0
    carrier_center_z_unrotated = carrier_back_z + carrier_thickness / 2.0
    pivot_delta_z = carrier_center_z_unrotated - camera_rear_z
    aim_rad = math.radians(aim_correction_deg)
    carrier_anchor_y = camera_mount_image_up_mm - pivot_delta_z * math.sin(aim_rad)
    carrier_anchor_z = camera_rear_z + pivot_delta_z * math.cos(aim_rad)
    dy = carrier_anchor_y - base_anchor_y
    dz = carrier_anchor_z - base_anchor_z
    beam_length = math.hypot(dy, dz)
    beam_angle_deg = math.degrees(math.atan2(dz, dy))
    mid_y = (base_anchor_y + carrier_anchor_y) / 2.0
    mid_z = (base_anchor_z + carrier_anchor_z) / 2.0

    result = base.union(outrigger).union(carrier)
    for x in support_x:
        beam = (
            cq.Workplane("XY")
            .box(7.0, beam_length + 3.0, 7.0)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), beam_angle_deg)
            .translate((x, mid_y, mid_z))
        )
        result = result.union(beam)
    return result


def export_part(part, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    stl_path = stem.with_suffix(".stl")
    step_path = stem.with_suffix(".step")
    exporters.export(part, str(stl_path), tolerance=0.04, angularTolerance=0.1)
    exporters.export(part, str(step_path))

    # OpenCascade's STEP writer emits trailing spaces on many entity lines.
    # Normalize its ASCII output so generated artifacts pass git diff --check.
    normalized = "\n".join(line.rstrip() for line in step_path.read_text().splitlines()) + "\n"
    step_path.write_text(normalized)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "generated")
    parser.add_argument("--reference-distance-mm", type=float, default=DEFAULT_REFERENCE_DISTANCE)
    parser.add_argument("--match-distance-mm", type=float, default=DEFAULT_MATCH_DISTANCE)
    parser.add_argument("--view-down-angle-deg", type=float, default=DEFAULT_VIEW_DOWN_ANGLE_DEG)
    parser.add_argument(
        "--camera-mount-image-up-mm",
        type=float,
        default=DEFAULT_CAMERA_MOUNT_IMAGE_UP,
    )
    parser.add_argument("--camera-pitch-trim-deg", type=float, default=0.0)
    args = parser.parse_args()

    export_part(
        pitch_coupon(D405_M3_PITCH, notches=1),
        args.output_dir / "yam_d405_camera_interface_20mm_coupon",
    )
    export_part(
        pitch_coupon(D435I_M3_PITCH, notches=2),
        args.output_dir / "d435i_camera_interface_45mm_coupon",
    )
    common = {
        "reference_distance_mm": args.reference_distance_mm,
        "match_distance_mm": args.match_distance_mm,
        "view_down_angle_deg": args.view_down_angle_deg,
        "camera_mount_image_up_mm": args.camera_mount_image_up_mm,
        "camera_pitch_trim_deg": args.camera_pitch_trim_deg,
    }
    export_part(setback_mount(outboard_sign=-1, **common), args.output_dir / "yam_d435i_left")
    export_part(setback_mount(outboard_sign=1, **common), args.output_dir / "yam_d435i_right")
    print(f"Exported STL and STEP files to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
