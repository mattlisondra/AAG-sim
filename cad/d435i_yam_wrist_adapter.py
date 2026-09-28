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
dimensions come from the RealSense mechanical drawings; clearance behind the
I2RT D405 bracket must be verified on the user's exact gripper before motion.
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


def fit_coupon():
    """Small gauge for checking the confirmed 20 mm and 45 mm hole pitches."""
    plate = rounded_plate(76.0, 24.0, 4.0, 3.0).translate((0.0, 0.0, 0.0))
    d405 = [(-D405_M3_PITCH / 2.0, 0.0), (D405_M3_PITCH / 2.0, 0.0)]
    d435 = [(-D435I_M3_PITCH / 2.0, 6.0), (D435I_M3_PITCH / 2.0, 6.0)]
    plate = plate.cut(cylinders(d405, 3.4, -0.5, 5.0))
    plate = plate.cut(cylinders(d435, 3.4, -0.5, 5.0))
    return plate


def setback_mount(
    *,
    outboard_sign: int,
    reference_distance_mm: float = DEFAULT_REFERENCE_DISTANCE,
    match_distance_mm: float = DEFAULT_MATCH_DISTANCE,
    view_down_angle_deg: float = DEFAULT_VIEW_DOWN_ANGLE_DEG,
    insert_diameter_mm: float = 4.6,
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

    extra_ray = match_distance_mm - reference_distance_mm
    angle = math.radians(view_down_angle_deg)
    optical_shift = extra_ray * math.cos(angle)
    image_up_shift = extra_ray * math.sin(angle)

    # Match the D405 color axis rather than the body center.  The D405 color
    # origin is 9 mm from its mount reference; D435i is 32.5 mm.  Mirroring the
    # camera therefore places its body/mount midpoint 23.5 mm outboard.
    body_center_x = outboard_sign * (
        D435I_COLOR_AXIS_FROM_MOUNT - D405_COLOR_AXIS_FROM_MOUNT
    )
    camera_holes = [
        (body_center_x - D435I_M3_PITCH / 2.0, image_up_shift),
        (body_center_x + D435I_M3_PITCH / 2.0, image_up_shift),
    ]

    base_width = 48.0
    base_depth = 34.0
    base_thickness = 6.0
    base = rounded_plate(base_width, base_depth, base_thickness, 4.0)
    insert_points = [(-D405_M3_PITCH / 2.0, 0.0), (D405_M3_PITCH / 2.0, 0.0)]
    # Blind pockets open on the existing-bracket contact face (Z=0).
    base = base.cut(cylinders(insert_points, insert_diameter_mm, -0.1, 5.3))

    # Account for the 2.05 mm difference in housing depth so the optical/front
    # planes, rather than the rear faces, receive the requested displacement.
    camera_rear_z = (D405_BODY_DEPTH - D435I_BODY_DEPTH) - optical_shift
    boss_height = 1.0
    carrier_thickness = 5.0
    carrier_front_z = camera_rear_z - boss_height
    carrier_back_z = carrier_front_z - carrier_thickness
    carrier_width = 86.0
    carrier_depth = 14.0  # narrow bar avoids covering the D435i rear vents
    carrier_center_x = outboard_sign * 19.0
    carrier = (
        rounded_plate(carrier_width, carrier_depth, carrier_thickness, 3.0)
        .translate((carrier_center_x, image_up_shift, carrier_back_z))
    )
    bosses = cylinders(camera_holes, 10.0, carrier_front_z, boss_height)
    carrier = carrier.union(bosses)
    camera_clearance = cylinders(camera_holes, 3.4, carrier_back_z - 0.5, 7.0)
    carrier = carrier.cut(camera_clearance)

    # Two diagonal beams and a cross-tie make the optical setback stiff in pitch
    # and roll.  They are intentionally clear of the central camera screw row.
    base_anchor_y = 0.0
    base_anchor_z = base_thickness / 2.0
    carrier_center_z = carrier_back_z + carrier_thickness / 2.0
    dy = image_up_shift - base_anchor_y
    dz = carrier_center_z - base_anchor_z
    beam_length = math.hypot(dy, dz)
    beam_angle_deg = math.degrees(math.atan2(dz, dy))
    mid_y = (base_anchor_y + image_up_shift) / 2.0
    mid_z = (base_anchor_z + carrier_center_z) / 2.0

    result = base.union(carrier)
    for x in (-18.0, 18.0):
        beam = (
            cq.Workplane("XY")
            .box(7.0, beam_length + 3.0, 7.0)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), beam_angle_deg)
            .translate((x, mid_y, mid_z))
        )
        result = result.union(beam)
    cross_tie = (
        cq.Workplane("XY")
        .box(43.0, 7.0, 7.0)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), beam_angle_deg)
        .translate((0.0, mid_y, mid_z))
    )
    return result.union(cross_tie)


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
    args = parser.parse_args()

    export_part(fit_coupon(), args.output_dir / "camera_hole_pitch_fit_coupon")
    common = {
        "reference_distance_mm": args.reference_distance_mm,
        "match_distance_mm": args.match_distance_mm,
        "view_down_angle_deg": args.view_down_angle_deg,
    }
    export_part(setback_mount(outboard_sign=-1, **common), args.output_dir / "yam_d435i_left")
    export_part(setback_mount(outboard_sign=1, **common), args.output_dir / "yam_d435i_right")
    print(f"Exported STL and STEP files to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
