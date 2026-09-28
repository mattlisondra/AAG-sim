# AAG × MolmoAct2 Bimanual YAM Sim

A small, reproducible bridge between an existing
[MolmoAct2-BimanualYAM](https://huggingface.co/allenai/MolmoAct2-BimanualYAM)
inference server and the official
[MolmoAct2 ManiSkill evaluator](https://github.com/allenai/molmoact2/tree/main/sim_eval).

This repository is deliberately an integration layer, not a fork of the model.
It pins the official source, validates the server/camera/action contract, adds a
camera-profile flag, and records a five-scene AAG service-manipulation benchmark
specification.

> Status: the official `BimanualYAMPutEverythingInBox-v1` task is runnable now.
> The five AAG scenes are benchmark specifications for the next environment
> implementation phase; they are not silently mapped onto the wrong objects in
> the official one-box task.

## What is known (and what is not)

The upstream YAM server currently expects this exact contract:

| Item | Value |
|---|---|
| Checkpoint | `allenai/MolmoAct2-BimanualYAM` |
| Endpoint | `POST /act` (default port `8202`) |
| Camera keys/order | `top_cam`, `left_cam`, `right_cam` |
| State | 14 floats: left 6 joints + gripper, right 6 joints + gripper |
| Control | absolute joint-position actions, 14 floats per step |
| Normalization tag | `yam_dual_molmoact2` |
| Training/sim image size | 640 × 360 RGB |

The model receives RGB pixels, language, and robot state. Camera calibration is
**not** included in the `/act` payload. Intrinsics and extrinsics still matter:
they determine the pixels and viewpoint presented to the policy.

The official simulator currently approximates the reference rig as:

- overhead/front role: Intel RealSense D435i, nominal 69.4° horizontal FOV;
- wrist roles: Intel RealSense D405, nominal 87.0° horizontal FOV;
- fixed poses embedded in the upstream ManiSkill `BimanualYAM` agent.

Those FOV figures and simulator poses are not a substitute for the factory
intrinsics and hand–eye calibration of a physical rig. RealSense intrinsics are
stream-profile/device specific. See [Camera calibration](docs/CALIBRATION.md).

## Zero-shot expectations

The fine-tuned checkpoint is the correct starting point for this embodiment,
and the official repository exposes a zero-shot ManiSkill evaluation. That does
not make arbitrary multi-stage cleanup reliable zero-shot. Expect the best first
results on large, familiar, forgiving pick-and-place objects with limited
occlusion. Compound instructions should be scored both end-to-end and by
subtask; if long-horizon performance is weak, an AAG task planner can issue one
grounded subtask at a time while keeping MolmoAct2 as the motor policy.

## Repository layout

```text
cad/                             parametric D435i wrist adapter + STL/STEP
configs/cameras/                 camera models and simulated mount poses
configs/hardware/                physical D435i camera-server settings
configs/benchmarks/              five AAG scene specifications
docs/                            architecture, benchmark, calibration notes
scripts/                         simulation and hardware launch helpers
src/aag_yam_sim/                 validation and evaluation CLI
tests/                           dependency-light contract tests
third_party/molmoact2/           pinned official repository (git submodule)
```

## Quick start

### 1. Clone with the official implementation

```bash
git clone --recurse-submodules <this-repository-url>
cd AAG-sim
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

The upstream repository uses Git LFS for some test artifacts. If an unavailable
artifact blocks checkout, source-only use is sufficient here:

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive
```

### 2. Install this lightweight bridge

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e '.[dev]'
```

For simulation, also install the upstream environment exactly as documented by
MolmoAct2 (the upstream project currently uses Python 3.11/3.12 and `uv`):

```bash
cd third_party/molmoact2
uv sync
uv run python sim_eval/scripts/download_assets.py
cd ../..
```

The simulator must run in that upstream environment because it contains Torch,
ManiSkill, and SAPIEN. `scripts/run_sim.sh` selects it automatically and
overlays this bridge package; the lightweight root environment remains useful
for `doctor`, camera-profile inspection, and benchmark inspection.

### 3. Start or reuse the inference server

On the GPU server, from the upstream repository:

```bash
uv run python examples/yam/host_server_yam.py \
  --host 0.0.0.0 --port 8202 --dtype bfloat16
```

Check the integration before moving anything:

```bash
aag-yam doctor \
  --server-url http://127.0.0.1:8202/act \
  --camera-profile molmoact2-reference
```

### 4. Run the official zero-shot simulation smoke test

```bash
bash scripts/run_sim.sh \
  --server-url http://127.0.0.1:8202/act \
  --camera-profile molmoact2-reference \
  --env-id BimanualYAMPutEverythingInBox-v1 \
  --instruction 'put everything into the box' \
  --episodes 10
```

Results, videos, and first-frame camera images are written under `outputs/` by
the upstream evaluator.

## Camera-profile CLI flag

Available profiles:

```bash
aag-yam camera-profiles
```

- `molmoact2-reference`: reproduces the camera matrices and mount poses in the
  pinned upstream simulator.
- `d405-wrist-physical-nominal`: keeps the upstream wrist poses but projects
  datasheet-derived D405 84° × 58° limits. It is a diagnostic approximation,
  not a measured stream calibration and not the default comparison reference.
- `d435i-all-nominal`: replaces both wrist camera intrinsics with a nominal
  D435i FOV while retaining the reference mounts. This is a domain-shift test,
  not a calibrated physical-rig profile.
- `d435i-wrist-raw-rigid-optimized`: uses raw nominal D435i intrinsics and the
  current pose-only optimum, with no image transform.
- `d435i-wrist-d405-match-nominal`: preserves the older single-plane
  crop/resize experiment for comparison; it is not the recommended raw path.
- `d435i-all-calibrated.example`: copy this file, insert measured intrinsics and
  hand–eye extrinsics, remove the `.example` suffix, and pass its path to
  `--camera-profile`.

Example:

```bash
bash scripts/run_sim.sh \
  --server-url http://gpu-host:8202/act \
  --camera-profile d435i-all-nominal \
  --episodes 3
```

## D435i wrist-camera adapter

The recommended D435i experiment feeds untouched 640 × 360 RGB to MolmoAct2
and approximates the pinned D405 view using only physical camera translation
and rotation. It performs no crop, resize, warp, reprojection, hole filling, or
inpainting.

The repository includes:

- one-piece left/right replacement mounts that retain the official bracket's
  through-holes for attachment to the YAM wrists and add complete D435i screw
  passages;
- a parametric carrier generator, reproducible mesh-fusion/checking script,
  and separate interface gauges;
- `scripts/query_realsense_intrinsics.py` for the two physical serial numbers;
- `scripts/run_d435i_camera_server.py`, which preserves the existing YAM ZMQ
  client/server protocol and defaults to raw RGB pass-through;
- a raw, pose-only ManiSkill comparison and optimizer.

You can compare the pinned MolmoAct2 reference and raw D435i candidate views
headlessly, without loading the policy or starting its server:

```bash
bash scripts/compare_wrist_cameras.sh --seed 42
```

For a multi-scene mount search:

```bash
bash scripts/compare_wrist_cameras.sh \
  --comparison-mode raw \
  --seed 42 --seed 43 --seed 44 \
  --optimize --optimization-passes 5
```

Across both wrists and seeds 42–44, the pose-only search reached 87.4% mean
finger IoU and 82.9% task-object IoU while reducing full-frame RGB MAE. The
corresponding printable pair is under
[`cad/generated/official-bracket-derived/molmoact2-raw-rigid-optimized`](cad/generated/official-bracket-derived/molmoact2-raw-rigid-optimized/README.md).
It remains a simulation-derived fit-check candidate; use measured intrinsics
and stationary real-camera pairs before robot motion.

Read [Headless D405/D435i simulation comparison](docs/SIM_CAMERA_COMPARISON.md)
for the saved RGB outputs, metrics, current sim-derived result, and its limits.

Read [D435i wrist adapter and view matching](docs/D435I_WRIST_ADAPTER.md) before
printing. The preferred full mount directly incorporates the supplied official
i2rt YAM D405 bracket geometry, retains its original 40 mm-pitch arm passages,
and is validated as one watertight component with two open D435i passages. The
unmodified reference STL is not redistributed. The derived full-mount meshes
are clearly isolated under the source model's CC BY-NC-SA 4.0 license. Print
the small 45 mm gauge and perform the documented stationary physical
fit/collision check first.

## Five-scene AAG benchmark

Inspect the complete machine-readable specification:

```bash
aag-yam scenarios
aag-yam scenario dining-table-cleanup
```

| Difficulty | Scene | Broad command | Actions |
|---:|---|---|---:|
| 1 | Dining-table cleanup | “Clean up the dining table.” | 3 |
| 2 | Kitchen dish sorting | “Put away the dishes.” | 3 |
| 3 | Bedside assistance | “Clear some space on the bedside table.” | 3 |
| 4 | Living-room tidying | “Tidy up the living room.” | 3 |
| 5 | Cafeteria/service station | “Organize the serving area.” | 3 groups |

See [Benchmark design](docs/BENCHMARK.md) for success metrics and the staged
implementation plan.

## Safety and scope

- Start in simulation, then shadow mode (predict/log without executing), then
  low-speed single-subtask hardware trials.
- Enforce joint, velocity, workspace, torque/contact, and gripper limits outside
  the learned policy.
- Keep a physical e-stop and a human supervisor in reach.
- Never assume a simulator camera profile is a physical calibration.
- Treat zero-shot as a hypothesis to measure, not a deployment guarantee.

## Upstream provenance

The execution engine and checkpoint are maintained by Ai2. This bridge targets
the pinned submodule commit; run `aag-yam doctor` after updating it because
camera conventions, action adapters, or endpoint schemas may change.

MolmoAct2 is released under Apache-2.0. Code in this integration repository is
also Apache-2.0 licensed. The STL-derived files under
`cad/generated/official-bracket-derived` are a documented exception licensed
CC BY-NC-SA 4.0 to match their source model.
