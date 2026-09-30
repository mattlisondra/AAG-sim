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

Do not infer partial credit from reward alone. The evaluator exposes one named
boolean per atomic placement, plus the first step at which it became true. For
a compound command, it reports both current final-state completion and latched
ordered progress; end-to-end success still requires all final predicates to be
true simultaneously.

This follows RoboLab's separation between final-state termination and subtask
progress: successful episodes score 1.0, while failed episodes retain their
fractional progress. See [RoboLab subtask scoring](https://github.com/NVlabs/RoboLab/blob/main/docs/subtask.md)
and [RoboLab result analysis](https://github.com/NVlabs/RoboLab/blob/main/docs/analysis.md).

## Simulator implementation status

The official upstream `BimanualYAMPutEverythingInBox-v1` environment remains
the Stage 0 closed-loop smoke test. The five service scenes have declarative
layouts, ManiSkill-native YCB objects where suitable, procedural collision
geometry for task-specific props, one-step `Preview-v0` environments, and
closed-loop `-v1` environments. Dynamic objects receive deterministic
seed-dependent XY/yaw jitter and explicit friction.

The `-v1` tasks implement:

- ordered stages with atomic object predicates;
- `in`, `on`, and distance-bounded `beside` relations, including optional
  common-support containment;
- latched ordered partial progress plus current final-state checks;
- automatic ManiSkill success termination;
- per-episode and aggregate SR/partial-score reporting;
- named completion flags and first-completion steps in `results.json`.

Render all current layouts without loading the policy:

```bash
bash scripts/render_service_scenes.sh \
  --camera-profile d435i-wrist-raw-rigid-optimized \
  --seed 42
```

Run the complete five-task D435i evaluation with the policy server already
listening on port 8202:

```bash
bash scripts/run_service_eval.sh \
  --server-url http://127.0.0.1:8202/act \
  --camera-profile d435i-wrist-raw-rigid-optimized \
  --episodes 10 \
  --max-episode-steps 4500 \
  --seed 42 \
  --output-dir outputs/d435i-service-10episodes
```

The layout source of truth is
`configs/benchmarks/aag_service_scene_layouts.json`; language and required
subtasks remain in `configs/benchmarks/aag_service_scenes.json`. This separation
keeps scene geometry independently editable while allowing tests to ensure that
the IDs and registered preview environments stay aligned.

## Evaluation implementation TODO

1. Add collision-rejection sampling for wider spawn distributions; the current
   bounded jitter is deliberately conservative.
2. Record full policy-view videos, state/action traces, inference latency,
   safety events, and per-step predicate timelines. Current MP4s use the human
   observer view and save the initial policy frames separately.
3. Validate one resolved atomic instruction at a time, then the resolved
   compound instructions, then connect the broad AAG instruction resolver.
4. Run matched-seed D405-reference versus D435i-candidate A/B trials and report
   confidence intervals rather than a single success percentage.
5. Tune object mass, friction, grasp clearance, and relation thresholds from
   rollout failures without changing the benchmark after looking at final-test
   outcomes.

The environment/scoring machinery is validated by forcing each declared stage
into its goal pose and passing it through the real episode loop. This verifies
partial-score progression, full score, and success termination; it is not a
claim that the learned policy achieves those states.
