# Compare D405 and raw D435i wrist views in ManiSkill

This workflow renders the pinned MolmoAct2 D405 reference and a D435i candidate
from the same deterministic ManiSkill scene. The default comparison uses the
**untouched D435i RGB frame**. It does not crop, resize, warp, reproject, fill
holes, inpaint, load MolmoAct2, or start an inference server.

Only the candidate camera translation and rotation change. The resulting pose
is shared with the printable-mount generator.

## Reference calibration

The default `molmoact2-reference` profile exactly reproduces the pinned
MolmoAct2 simulator: 640 × 360, 87° horizontal FOV, and equal `fx`/`fy`.

Do not derive a supposedly exact 640 × 360 D405 intrinsic matrix by converting
the datasheet's horizontal and vertical FOV limits independently. D405 color
intrinsics are factory calibrated per stream and the RealSense SDK performs a
D405-specific ISP crop/scale calculation. Independent FOV conversion was the
source of the vertically squashed reference shown in the earlier comparison.
For real-camera work, replace the simulation reference with measured stream
intrinsics from the actual recording D405 when they are available.

## One-time setup

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
uv sync --project third_party/molmoact2
uv run --project third_party/molmoact2 \
  python -m mani_skill.utils.download_asset ycb -y
```

## Render and optimize

Save one raw paired comparison:

```bash
bash scripts/compare_wrist_cameras.sh \
  --comparison-mode raw --seed 42 --shader-pack minimal
```

Search one shared mirrored mount over both wrists and several scenes:

```bash
bash scripts/compare_wrist_cameras.sh \
  --comparison-mode raw \
  --seed 42 --seed 43 --seed 44 \
  --optimize --optimization-passes 5 \
  --shader-pack minimal
```

Each run writes `report.json` plus, for every seed and wrist:

```text
reference_d405.png
candidate_d435i_raw.png
candidate_d435i_matched.png  # identical to raw in raw mode
overlay_50_50.png
difference_x4.png
edge_overlay_red_cyan.png
finger_mask_overlay_red_cyan.png
task_mask_overlay_red_cyan.png
comparison.png
```

Red/cyan overlays show D405-only/D435i-only edges or masks; white is overlap.
The objective penalizes the worse of finger and task-object alignment so an
excellent gripper match cannot hide a poor scene-object match. It also retains
full-frame and foreground RGB/edge terms.

`--comparison-mode crop` exists only for reproducing the older single-plane
crop/resize experiment. It is not used for the recommended raw-RGB mount.

## Current pose-only result

Across seeds 42–44 and both wrists, the final raw-RGB candidate is:

| Parameter | Value |
|---|---:|
| D435i RGB lens-to-working-plane distance | 188.717909 mm |
| Camera mount image-up displacement | 8.625 mm |
| Added pitch trim | 5.4375° |
| Mirrored lateral optical-axis displacement | 9.0 mm |
| Mirrored yaw | 3.375° |
| Image transform | none |

| Metric | Initial mount | Optimized mount |
|---|---:|---:|
| Balanced score (lower is better) | 0.49960 | **0.13073** |
| Finger IoU | 27.3% | **87.4%** |
| Task-object IoU | 66.7% | **82.9%** |
| Full-frame RGB MAE | 6.22% | **5.08%** |

![Raw pose-only D405/D435i comparison](assets/camera-comparison/molmoact2-raw-rigid-optimized-seed42-left.png)

The simulated optical poses relative to each `link_6` are:

```text
left  p = [ 0.009000001, 0.131117452, 0.006032483 ] m
left  q = [ 0.595586837,-0.340354096,-0.361017165,-0.631745214 ] wxyz
right p = [-0.008999999, 0.131117452, 0.006032482 ] m
right q = [ 0.631745206,-0.361017160,-0.340354101,-0.595586846 ] wxyz
```

The full report and exact mesh parameters are recorded in
[`parameters.json`](../cad/generated/official-bracket-derived/molmoact2-raw-rigid-optimized/parameters.json).

## Build the corresponding mounts

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

The checked-in meshes are watertight single components, preserve both original
arm passages and both D435i passages, and need no camera-envelope relief cut.
See the [mount README](../cad/generated/official-bracket-derived/molmoact2-raw-rigid-optimized/README.md).

## Why pose-only cannot be pixel-identical at all depths

The D435i RGB lens is narrower than the D405 reference. Translation can match
apparent scale at one depth and rotation can match bearing, but relocating the
optical center changes parallax. The gripper, target object, table, and distant
background occupy different depths, so no single rigid pose can make all of
them pixel-identical simultaneously.

The reported pose is therefore a measured compromise, not an exact optical
equivalence claim. It deliberately leaves residual differences visible. Real
hardware adds per-unit intrinsics, lens distortion, exposure, color response,
mount tolerance, and hand-eye error; validate stationary paired frames before
robot motion.
