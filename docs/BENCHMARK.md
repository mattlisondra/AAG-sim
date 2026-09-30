# AAG service-manipulation benchmark

The benchmark increases semantic and physical difficulty while keeping early
motor actions forgiving. The source of truth is
`configs/benchmarks/aag_service_scenes.json`.

## Rollout protocol

For each scene, camera profile, and instruction condition:

1. Run at least 10 deterministic scene seeds during development and reserve a
   separate seed set for final reporting.
2. Log the broad instruction, resolved instruction, ordered subtasks, object
   poses, robot state, camera profile hash/name, upstream commit, checkpoint,
   action-chunk size, and termination reason.
3. Report end-to-end success and per-subtask success. A three-step episode that
   finishes two steps is not an end-to-end success.
4. Report safety outcomes independently: collision, joint-limit clamp,
   workspace clamp, intervention, timeout, or invalid action.
5. Save policy-view frames as well as the human-render video.

## Recommended progression

| Stage | Test | Purpose |
|---:|---|---|
| 0 | Upstream two-object box task | Validate installation and wire contract |
| 1 | One object, one destination, resolved language | Validate grasp/control |
| 2 | Three atomic subtasks in one scene | Validate scene persistence |
| 3 | One resolved compound instruction | Validate long-horizon policy behavior |
| 4 | Broad instruction resolved by AAG | Validate planner + policy |
| 5 | Camera-profile A/B and clutter sweep | Measure domain shift and robustness |

## Core metrics

- end-to-end success rate: every required final-state predicate is true at
  termination;
- atomic partial-task score: completed object placements divided by required
  object placements, so the cafeteria task has six atomic placements rather
  than only three language clauses;
- clause completion rate and ordered-prefix completion rate, reported
  separately from atomic partial credit;
- grounded-object and destination accuracy;
- steps and wall-clock time to completion;
- inference latency and control stalls;
- collision/intervention/limit-clamp rate;
- performance versus clutter level and camera profile.

Do not infer partial credit from reward alone. The evaluator should expose one
named boolean per atomic placement, plus the first step at which it became true.
For a compound command, report both final-state completion and a latched
ever-completed diagnostic; the end-to-end result must still fail if a later
action undoes a required final arrangement.

## Simulator implementation status

The official upstream `BimanualYAMPutEverythingInBox-v1` environment remains
the Stage 0 closed-loop smoke test. The five service scenes now have declarative
layouts, ManiSkill-native YCB objects where suitable, procedural collision
geometry for task-specific props, registered `Preview-v0` environments, and a
headless three-camera renderer. They do not yet have task predicates, rollout
termination, randomized spawn distributions, or meaningful rewards.

Render all current layouts without loading the policy:

```bash
bash scripts/render_service_scenes.sh \
  --camera-profile d435i-wrist-raw-rigid-optimized \
  --seed 42
```

The layout source of truth is
`configs/benchmarks/aag_service_scene_layouts.json`; language and required
subtasks remain in `configs/benchmarks/aag_service_scenes.json`. This separation
keeps scene geometry independently editable while allowing tests to ensure that
the IDs and registered preview environments stay aligned.

## Evaluation implementation TODO

1. Add deterministic reset distributions with collision-free rejection and log
   every sampled object pose.
2. Stabilize physical properties for all movable props: scale, mass, center of
   mass, friction, restitution, and grasp-clearance envelopes.
3. Implement object-level relations:
   - `in`: the object support proxy lies inside the target's interior XY bounds
     and below its rim, with low terminal velocity;
   - `on`: the object footprint overlaps the target support region, its bottom
     is near the support height, and it is settled;
   - `beside`: both objects share a support surface, are outside one another,
     and their XY separation lies in a declared interval.
4. Expand group phrases such as “both bottles” and “the cups” into atomic
   object predicates before scoring.
5. Return named `subtask_complete`, `subtask_first_step`, ordered-prefix, and
   final all-complete values in `info`; terminate successfully only when all
   final predicates hold for a short stability window.
6. Extend the current evaluator to save complete policy-view videos, state and
   action traces, inference latency, safety events, and the predicate timeline.
7. Validate one resolved atomic instruction at a time, then a resolved compound
   instruction, then connect the broad AAG instruction resolver.
8. Run matched-seed D405-reference versus D435i-candidate A/B trials and report
   confidence intervals rather than a single success percentage.

Until those items are complete, an image that looks plausible is a layout
approval artifact—not evidence that MolmoAct2 can complete the task.
