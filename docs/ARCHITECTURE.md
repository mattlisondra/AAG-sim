# Architecture

```text
                 language instruction
                         |
ManiSkill scene -> top/left/right RGB + 14-D state
                         |
                         v
               lightweight HTTP client
                         |
              POST /act (json_numpy)
                         |
                         v
 GPU host: MolmoAct2-BimanualYAM server
                         |
                  action chunk (N, 14)
                         |
                         v
 state/action adapter -> absolute joint-position controller
```

## Responsibility boundary

This repository owns:

- the camera-profile schema and runtime simulator override;
- server/upstream contract checks;
- the AAG benchmark scene and instruction specification;
- a stable CLI around the upstream evaluator.

The pinned MolmoAct2 submodule owns:

- model loading and `predict_action`;
- `json_numpy` `/act` server semantics;
- the YAM MJCF, ManiSkill robot, state/action adapters, and official task;
- video and result generation.

No model code or robot assets are copied here. This keeps upstream provenance
clear and makes divergence visible as a failed contract check.

## Action convention

The policy checkpoint uses 14 values ordered as:

```text
[left_joint_1..6, left_gripper, right_joint_1..6, right_gripper]
```

The upstream simulator converts between its 16-position MJCF representation
(two finger joints per gripper) and the policy's 14-value representation. The
policy outputs absolute joint targets, not Cartesian end-effector deltas.

## Language-conditioning conditions

For each AAG scene, evaluate three conditions separately:

1. **Resolved compound:** the explicit multi-subtask instruction in the spec.
2. **Resolved atomic:** one subtask at a time, resetting or continuing the same
   episode as defined in the result metadata.
3. **Broad/AAG:** a planner resolves the broad instruction into grounded
   subtasks before policy execution.

The model server should receive resolved text in conditions 1 and 2. Condition
3 measures planner plus policy and must not be compared directly with pure
motor-policy results without reporting both layers.
