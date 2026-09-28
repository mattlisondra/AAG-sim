# MolmoAct2 raw-RGB view-clear V2 mounts

These are the recommended one-piece D435i replacement mounts. They retain the
V1 raw-RGB, rigid-pose-only camera placement, but remove printed material from
the D435i RGB field of view and reconnect the mount with a thick outboard
truss. The policy input remains the untouched 640 × 360 RGB frame: no crop,
resize, warp, reprojection, filling, or inpainting.

V1 matched camera and scene geometry in simulation, but that simulator did not
render the printed bracket. A separate first-person STL audit later found that
V1 could cover 42.3% of the left image and 64.1% of the right image. V2 fixes
that physical-geometry issue while preserving the optimized optical pose.

## Pose and optical tunnel

```text
D435i RGB lens-to-working-plane distance  188.717909 mm
camera mount image-up                       8.625000 mm
added pitch trim                            5.437500 deg
mirrored lateral optical-axis offset        9.000000 mm
mirrored yaw                                3.375000 deg
D435i nominal RGB field of view             69.4 deg H x 42.5 deg V
assumed lens recess                          3.000000 mm
clearance margin at every image edge         3.200000 deg
cleared field of view                       75.8 deg H x 48.9 deg V
```

The optical tunnel is wider than the nominal RGB cone by 3.2° at each edge.
The final meshes have zero reported nominal-cone intersection and less than
`0.00007 mm³` expanded-cone intersection (Boolean numerical tolerance). The
640 × 360 first-person STL audit reports 0.000% mount pixels for both sides.
These are geometric checks based on the documented assumptions, not
measurements of an individual camera.

## Structural geometry and printing

The view cut divides the old center support, so V2 uses one 12 × 12 mm lower
tie and two 12 × 12 mm diagonal outboard rails. The builder tests several
routes, rejects any that enter the expanded view cone, and chooses the route
with the strongest minimum fusion contact to the two functional mount bodies.
For these files it selected `y = -34 mm`, `z = -4 mm` in the adapter frame.

The meshes are deliberately bulky. Their solid volumes are 59.60 cm³ left and
54.82 cm³ right, equivalent to approximately 75.7 g and 69.6 g at 1.27 g/cm³
solid PETG before slicer infill. The D435i itself is approximately 75 g.

For a first fit-check, use PETG or PA-CF, 0.2 mm layers, at least five walls,
six top/bottom layers, and 40–60% gyroid infill. Orient each part so the two
long diagonal rails print as continuously as practical; inspect the sliced
preview and enable support under unsupported portions of the diagonal rails,
lower tie, and camera carrier. The exact support placement depends on the
printer, nozzle, layer height, and chosen orientation.

The contact patches and rails are much more substantial than V1, but no FEA,
fatigue, vibration, impact, or destructive load test has been performed. Treat
the files as experimental fit-check parts. First load-test each print with a
dummy mass and the robot unpowered, then verify fasteners, cable strain relief,
payload/inertia, and the full robot collision envelope.

## Rebuild

```bash
uv run --with cadquery --with trimesh --with manifold3d \
  --with scipy --with networkx \
  python cad/build_full_d435i_wrist_mount.py \
  '/absolute/path/to/camera+bracket(for+D405)+-+camera+bracket(for+D405).stl' \
  --output-dir cad/generated/official-bracket-derived/molmoact2-raw-rigid-view-clear-v2 \
  --match-distance-mm 188.717909 \
  --camera-mount-image-up-mm 8.625 \
  --camera-pitch-trim-deg 5.4375 \
  --camera-lateral-mm 9.0 \
  --camera-yaw-deg 3.375 \
  --view-clearance-margin-deg 3.2 \
  --lens-recess-mm 3.0 \
  --view-brace-thickness-mm 12.0
```

Render the actual STL from the assumed RGB optical center:

```bash
uv run --with vtk --with trimesh --with manifold3d \
  python cad/render_mount_camera_view.py \
  cad/generated/official-bracket-derived/molmoact2-raw-rigid-view-clear-v2 \
  --match-distance-mm 188.717909 \
  --camera-mount-image-up-mm 8.625 \
  --camera-pitch-trim-deg 5.4375 \
  --camera-lateral-mm 9.0 \
  --camera-yaw-deg 3.375 \
  --lens-recess-mm 3.0
```

`parameters.json` records the mesh checks, support-contact volumes, hashes, and
the unchanged simulation image-match metrics.
