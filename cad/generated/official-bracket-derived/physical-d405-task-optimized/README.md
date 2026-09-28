# Experimental physical-D405 task-optimized mounts

These one-piece replacement mounts correspond to the task-aware ManiSkill
alignment in `docs/SIM_CAMERA_COMPARISON.md`. They target the nominal physical
D405 84° × 58° projection and prioritize the visible gripper/task geometry.
They do not claim pixel identity at every scene depth.

## Parameters

```text
D435i RGB lens-to-working-plane distance  166.717909 mm
camera mount image-up                      17.625000 mm
added pitch trim                            6.000000 deg
mirrored lateral optical-axis offset        3.750000 mm
mirrored yaw                                1.500000 deg
camera-envelope relief                      0.750000 mm
```

The fixed RGB wrapper crop is `(left=28, top=0, width=584, height=360)`, resized
to 640 × 360. The exact simulated local poses and metrics are recorded in
`parameters.json`.

## Rebuild

```bash
uv run --with cadquery --with trimesh --with manifold3d \
  --with scipy --with networkx \
  python cad/build_full_d435i_wrist_mount.py \
  '/absolute/path/to/camera+bracket(for+D405)+-+camera+bracket(for+D405).stl' \
  --output-dir cad/generated/official-bracket-derived/physical-d405-task-optimized \
  --match-distance-mm 166.717909 \
  --camera-mount-image-up-mm 17.625 \
  --camera-pitch-trim-deg 6.0 \
  --camera-lateral-mm 3.75 \
  --camera-yaw-deg 1.5 \
  --camera-clearance-relief-mm 0.75
```

The validated build is one watertight component per side with unobstructed arm
and camera passages. Exact D435i-envelope overlap was below 0.00003 mm³. The
left source bracket needed no relief; the asymmetric right variant removed
296.756 mm³, about 1.27% of the source-bracket volume.

This is experimental research hardware. Mesh checks do not validate material
strength, printer tolerances, screw-head access, cable clearance, wrist payload,
or collision with the complete robot. Start with a stationary, unpowered fit
check and follow `docs/D435I_WRIST_ADAPTER.md`.
