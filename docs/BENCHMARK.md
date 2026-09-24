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

- end-to-end success rate;
- ordered and unordered subtask completion rate;
- grounded-object and destination accuracy;
- steps and wall-clock time to completion;
- inference latency and control stalls;
- collision/intervention/limit-clamp rate;
- performance versus clutter level and camera profile.

## Simulator implementation status

The official upstream `BimanualYAMPutEverythingInBox-v1` environment is the
Stage 0 smoke test. The five service scenes are currently specifications. They
need object assets, spawn distributions, collision geometry, and scene-specific
success predicates before their numbers are meaningful. Keeping this explicit
prevents an instruction from being evaluated against a scene that does not
contain the referenced objects.

The recommended next implementation is one parameterized ManiSkill base class
with declarative object/target definitions, plus five registered environment
IDs. Each relation needs its own geometric predicate (`in`, `on`, `beside`) and
each subtask needs an independently logged completion bit.
