# Compare D405 and D435i wrist views in ManiSkill

This workflow renders the upstream MolmoAct2 D405 wrist-camera approximation
and a raw D435i model from the same deterministic ManiSkill scene. It then
applies the same crop/resize operation used by the hardware camera adapter and
saves paired RGB images, overlays, amplified differences, and numeric metrics.

It does **not** load a MolmoAct2 checkpoint, start the inference server, or move
the simulated robot. Only ManiSkill/SAPIEN rendering uses the GPU.

## One-time setup

From the repository root:

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
uv sync --project third_party/molmoact2
uv run --project third_party/molmoact2 \
  python -m mani_skill.utils.download_asset ycb -y
```

The last command installs the YCB assets in ManiSkill's user data directory.

## Save one paired comparison

```bash
bash scripts/compare_wrist_cameras.sh \
  --seed 42 \
  --shader-pack minimal
```

The default comparison is:

- environment: `BimanualYAMPutEverythingInBox-v1`;
- reference: pinned upstream `molmoact2-reference` camera pose and 87°
  square-pixel wrist-camera approximation;
- candidate: nominal 640 × 360 D435i RGB intrinsics at the CAD-derived mount
  pose;
- postprocessing: centered crop calculated from both pinhole models, followed
  by bilinear resize to 640 × 360.

Each run creates a timestamped directory under `outputs/camera_compare/` with:

```text
report.json
baseline/seed_42/left_cam/
  reference_d405.png
  candidate_d435i_raw.png
  candidate_d435i_matched.png
  overlay_50_50.png
  difference_x4.png
  comparison.png
```

The same files are produced for `right_cam`. Use `--camera left_cam` to render
only one wrist. The `comparison.png` montage is usually the fastest visual
check: compare the gripper silhouette, box/object edges, horizon, and table
texture rather than relying on one metric.

## Optimize the simulated mount

Use several scene seeds so one object placement cannot dominate the result:

```bash
bash scripts/compare_wrist_cameras.sh \
  --seed 42 --seed 43 --seed 44 \
  --optimize \
  --optimization-passes 4 \
  --shader-pack minimal
```

The optimizer performs a deterministic coordinate search over:

- RGB optical-center distance to the nominal grasp plane;
- camera screw-row displacement in the image-up direction;
- pitch trim added to the geometry-derived aiming correction.

It minimizes `0.75 × normalized RGB MAE + 0.25 × edge MAE` across both wrists
and all requested seeds. Lower score, MAE, RMSE, and edge MAE are better;
higher PSNR and gray NCC are better. The optimized images are saved beside the
baseline, and `report.json` contains the full search history, poses, crop
plans, and a reproducible CAD command.

On the currently pinned simulator, the three-seed run above produced:

| Result | Distance | Image-up | Pitch trim | Score | PSNR |
|---|---:|---:|---:|---:|---:|
| Conservative CAD baseline | 166.718 mm | 24.0 mm | 0° | 0.05400 | 20.34 dB |
| Sim-optimized | 172.718 mm | 19.5 mm | −2.875° | 0.04342 | 22.38 dB |

This is a 19.6% reduction in the aggregate comparison score. The result was
also passed through the full-mount builder against the supplied official
bracket STL: both arm passages and both camera passages remained open, and the
camera-envelope intersections stayed below the builder's 0.01 mm³ tolerance.

To reproduce that candidate in temporary output without replacing the checked
in conservative meshes:

```bash
candidate_dir="$(mktemp -d /tmp/aag-d435i-sim-XXXXXX)"
uv run --with cadquery --with trimesh --with manifold3d \
  --with scipy --with networkx \
  python cad/build_full_d435i_wrist_mount.py \
  '/home/asblab8/Desktop/camera+bracket(for+D405)+-+camera+bracket(for+D405).stl' \
  --output-dir "$candidate_dir" \
  --match-distance-mm 172.717909 \
  --camera-mount-image-up-mm 19.5 \
  --camera-pitch-trim-deg -2.875
```

The builder refuses a source STL whose hash differs from the geometry used for
alignment, fuses one watertight part per wrist, and reruns the hole and housing
clearance probes.

## What this experiment can and cannot establish

The simulation comparison isolates first-order pose, projection, crop, and
parallax effects. It is useful for rejecting obviously poor mount geometry
before printing. It is not sufficient evidence to print the optimized variant
as the final hardware revision:

- the upstream simulated D405 uses an 87° square-pixel approximation, whereas
  the physical D405 nominal color FOV is 84° × 58°;
- real intrinsics and distortion vary by device and stream profile;
- exposure, white balance, sensor response, cable clearance, printed
  tolerances, and robot self-occlusion are not simulated;
- moving the camera center causes depth-dependent parallax that no single 2-D
  crop can eliminate.

For the real robot, first query both D435i devices with
`scripts/query_realsense_intrinsics.py`, then use
`scripts/run_d435i_camera_server.py` for the fixed crop/resize. Compare
stationary physical D405 and D435i frames at multiple gripper openings and
object depths before enabling motion. See
[D435i wrist cameras that approximate the MolmoAct2 D405 view](D435I_WRIST_ADAPTER.md)
for the hardware acceptance sequence.
