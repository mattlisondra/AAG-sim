#!/usr/bin/env python3
"""Render generated STL files and report basic mesh integrity with VTK."""

from __future__ import annotations

import argparse
from pathlib import Path

import vtk


def render(stl: Path, output: Path) -> tuple[int, int, tuple[float, ...] | None]:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(stl))
    reader.Update()

    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.Update()

    edges = vtk.vtkFeatureEdges()
    edges.SetInputConnection(clean.GetOutputPort())
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff()
    edges.ManifoldEdgesOff()
    edges.Update()

    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(clean.GetOutputPort())
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.14, 0.42, 0.68)
    actor.GetProperty().SetSpecular(0.25)
    actor.GetProperty().SetSpecularPower(20.0)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.96, 0.97, 0.98)
    renderer.AddActor(actor)
    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.Azimuth(38.0)
    camera.Elevation(24.0)
    camera.Zoom(1.15)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1200, 900)
    window.AddRenderer(renderer)
    window.Render()

    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetScale(1)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()

    mesh = clean.GetOutput()
    bad_edges = edges.GetOutput()
    bad_bounds = bad_edges.GetBounds() if bad_edges.GetNumberOfCells() else None
    return mesh.GetNumberOfCells(), bad_edges.GetNumberOfCells(), bad_bounds


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "directory",
        type=Path,
        nargs="?",
        default=Path(__file__).parent / "generated",
    )
    args = parser.parse_args()
    failed = False
    for stl in sorted(args.directory.glob("*.stl")):
        triangles, bad_edges, bad_bounds = render(stl, stl.with_suffix(".png"))
        print(
            f"{stl.name}: triangles={triangles}, boundary/non-manifold edges={bad_edges}, "
            f"bad-edge bounds={bad_bounds}"
        )
        failed |= bad_edges != 0
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
