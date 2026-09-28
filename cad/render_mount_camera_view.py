#!/usr/bin/env python3
"""Render a full wrist mount from the D435i RGB optical center.

This is a mount-occlusion audit, not a scene renderer.  Any dark pixel in the
output is printed mount geometry that can enter the nominal color image.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import numpy as np
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy
except ImportError as exc:  # pragma: no cover - CAD-only environment
    raise SystemExit("numpy, vtk, trimesh, and manifold3d are required; see cad/README.md") from exc

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_official_bracket_fit import (  # noqa: E402
    D435I_COLOR_VFOV_DEG,
    DEFAULT_REFERENCE_DISTANCE,
    DEFAULT_VIEW_DOWN_ANGLE_DEG,
    adapter_to_bracket_transform,
    camera_optical_transform,
)


def render_mount_view(
    stl: Path,
    output: Path,
    *,
    side: str,
    reference_distance_mm: float,
    match_distance_mm: float,
    view_down_angle_deg: float,
    camera_mount_image_up_mm: float,
    camera_pitch_trim_deg: float,
    camera_lateral_mm: float,
    camera_yaw_deg: float,
    lens_recess_mm: float,
) -> float:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(stl))
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.0, 0.0, 0.0)
    actor.GetProperty().SetAmbient(1.0)
    actor.GetProperty().SetDiffuse(0.0)
    actor.GetProperty().SetSpecular(0.0)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1.0, 1.0, 1.0)
    renderer.AddActor(actor)

    camera_to_adapter = camera_optical_transform(
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
    camera_to_bracket = adapter_to_bracket_transform() @ camera_to_adapter
    origin = camera_to_bracket[:3, 3]
    forward = camera_to_bracket[:3, :3] @ np.array([0.0, 0.0, 1.0])
    image_up = camera_to_bracket[:3, :3] @ np.array([0.0, 1.0, 0.0])
    camera = renderer.GetActiveCamera()
    camera.SetPosition(*origin)
    camera.SetFocalPoint(*(origin + forward * 100.0))
    camera.SetViewUp(*image_up)
    camera.SetViewAngle(D435I_COLOR_VFOV_DEG)
    camera.SetClippingRange(0.25, 250.0)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetMultiSamples(0)
    window.SetSize(640, 360)
    window.AddRenderer(renderer)
    window.Render()

    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()

    pixels = vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(360, 640, 3)
    visible = np.any(pixels < 250, axis=2)
    return float(np.mean(visible))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mount_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--reference-distance-mm", type=float, default=DEFAULT_REFERENCE_DISTANCE)
    parser.add_argument("--match-distance-mm", type=float, required=True)
    parser.add_argument("--view-down-angle-deg", type=float, default=DEFAULT_VIEW_DOWN_ANGLE_DEG)
    parser.add_argument("--camera-mount-image-up-mm", type=float, required=True)
    parser.add_argument("--camera-pitch-trim-deg", type=float, required=True)
    parser.add_argument("--camera-lateral-mm", type=float, required=True)
    parser.add_argument("--camera-yaw-deg", type=float, required=True)
    parser.add_argument("--lens-recess-mm", type=float, default=3.0)
    args = parser.parse_args()

    output_dir = args.output_dir or args.mount_dir
    failed = False
    for side in ("left", "right"):
        stl = args.mount_dir / f"yam_d435i_full_wrist_mount_{side}.stl"
        output = output_dir / f"d435i_rgb_mount_mask_{side}.png"
        coverage = render_mount_view(
            stl,
            output,
            side=side,
            reference_distance_mm=args.reference_distance_mm,
            match_distance_mm=args.match_distance_mm,
            view_down_angle_deg=args.view_down_angle_deg,
            camera_mount_image_up_mm=args.camera_mount_image_up_mm,
            camera_pitch_trim_deg=args.camera_pitch_trim_deg,
            camera_lateral_mm=args.camera_lateral_mm,
            camera_yaw_deg=args.camera_yaw_deg,
            lens_recess_mm=args.lens_recess_mm,
        )
        print(f"{side}: mount covers {coverage * 100.0:.3f}% of nominal RGB pixels -> {output}")
        failed |= coverage > 0.0
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
