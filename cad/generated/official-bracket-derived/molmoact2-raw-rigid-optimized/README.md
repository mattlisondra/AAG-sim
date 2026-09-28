# MolmoAct2 raw-RGB rigid-pose-optimized mounts

These one-piece replacement mounts position D435i wrist cameras to approximate
the pinned MolmoAct2 D405 simulator view using **physical translation and
rotation only**. The D435i image is passed through untouched: no crop, resize,
warp, reprojection, hole filling, or inpainting is part of this result.

## Parameters

```text
D435i RGB lens-to-working-plane distance  188.717909 mm
camera mount image-up                       8.625000 mm
added pitch trim                            5.437500 deg
mirrored lateral optical-axis offset        9.000000 mm
mirrored yaw                                3.375000 deg
camera-envelope relief                      0.000000 mm
```

The simulated local optical poses, metrics, mesh checks, and hashes are in
`parameters.json`.

## Rebuild

```bash
uv run --with cadquery --with trimesh --with manifold3d \
  --with scipy --with networkx \
  python cad/build_full_d435i_wrist_mount.py \
  '/absolute/path/to/camera+bracket(for+D405)+-+camera+bracket(for+D405).stl' \
  --output-dir cad/generated/official-bracket-derived/molmoact2-raw-rigid-optimized \
  --match-distance-mm 188.717909 \
  --camera-mount-image-up-mm 8.625 \
  --camera-pitch-trim-deg 5.4375 \
  --camera-lateral-mm 9.0 \
  --camera-yaw-deg 3.375
```

Both outputs are one-component watertight meshes. The original arm passages
and both D435i passages are unobstructed, and the nominal camera-envelope
intersection is below `0.00014 mm³` without cutting relief from the source
bracket.

The mount is an experimental research fit-check candidate, not a structural
or robot-safety certification. Check print tolerance, fastener engagement,
cable routing, payload/inertia, and the complete bimanual collision envelope
with power removed before enabling motion.
