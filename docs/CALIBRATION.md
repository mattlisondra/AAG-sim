# Camera calibration and profile use

## Why calibration is separate from the model request

MolmoAct2-BimanualYAM consumes RGB images, language, and a 14-value robot state.
It does not consume a camera matrix or camera pose in the `/act` payload. The
simulator still needs both to render observations that resemble the training
setup, and physical deployment still needs stable mounting and view geometry.

## Reference simulator profile

`molmoact2-reference` reproduces the current upstream code:

- 640 × 360 for all policy cameras;
- 69.4° nominal horizontal FOV for the top D435i role;
- 87.0° nominal horizontal FOV for the two D405 wrist roles;
- the same SAPIEN mount transforms and quaternion convention (`wxyz`).

The upstream intrinsic helper assumes square pixels and sets `fy = fx`. That is
a simulator approximation, not the exact factory calibration of a particular
serial number and stream mode.

## D435i wrist experiment

`d435i-all-nominal` changes the simulated wrist FOV to the same nominal value as
the D435i top camera and keeps the reference wrist mounts. Use it for an A/B
domain-shift experiment:

```bash
bash scripts/run_sim.sh --server-url HOST:8202 \
  --camera-profile molmoact2-reference --episodes 10

bash scripts/run_sim.sh --server-url HOST:8202 \
  --camera-profile d435i-all-nominal --episodes 10
```

Keep seeds, object layouts, instructions, action-chunk length, and policy
checkpoint identical. Compare success, per-subtask completion, collisions,
interventions, and episode length.

## Building a calibrated profile

1. Select the exact RealSense color stream used by the policy (initially
   640 × 360 at 30 Hz to match the reference pipeline).
2. Query each device's color intrinsics for that active stream profile. Record
   `fx`, `fy`, `ppx`, and `ppy` as:

   ```text
   [[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]]
   ```

3. Estimate the top-camera-to-base transform and each wrist-camera-to-link-6
   transform. Record position in metres and rotation as a normalized `wxyz`
   quaternion in the SAPIEN camera convention.
4. Copy `configs/cameras/d435i-all-calibrated.example.json`, replace every
   placeholder value, and rename it (for example `lab-d435i-rig.json`).
5. Inspect the three first-frame images saved by an evaluation. Confirm that
   left/right roles are not swapped, RGB is not BGR, the image is not inverted,
   the table horizon is plausible, and both grippers occupy expected regions.
6. Validate against measured fiducials or known scene points before treating
   it as calibrated.

## Physical camera capture

The upstream YAM hardware example opens RealSense color and depth streams at
640 × 360, 30 Hz, converts BGR to RGB, and maps the physical front camera to
the server's `top_cam` key. Camera serial-number assignment matters; never rely
on USB enumeration order.

## What the CLI flag changes

For simulation, `--camera-profile` replaces the three ManiSkill sensor camera
matrices and mount poses in-process before an environment is created. It does
not modify checkpoint weights, normalization statistics, robot kinematics, or
the HTTP request schema.
