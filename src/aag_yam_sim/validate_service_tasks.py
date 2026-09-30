"""Force each task into its declared goal state and verify scoring/termination."""

from __future__ import annotations

import argparse
import json

import gymnasium as gym
import numpy as np
import torch
from mani_skill.utils.structs.pose import Pose

from .camera_profiles import install_upstream_camera_profile, load_profile
from .evaluation import run_episode
from .service_scene_spec import service_scene_layouts


class _ZeroClient:
    def __init__(self, action_shape: tuple[int, ...]):
        self.action_shape = action_shape

    def infer(self, obs: dict, instruction: str) -> np.ndarray:
        return np.zeros(self.action_shape, dtype=np.float32)


def _force_declared_goals(env, predicates: list[dict]) -> None:
    task = env.unwrapped
    for predicate in predicates:
        actor = task.actors[predicate["object"]]
        target = task.actors[predicate["target"]]
        offset = torch.tensor(predicate["goal_offset_xy"], dtype=torch.float32, device=task.device)
        position = target.pose.p.clone()
        position[:, :2] += offset
        bottom_offset = -float(task.object_bounds[predicate["object"]][0, 2])
        support_name = predicate.get("support")
        if predicate["relation"] in {"in", "on"}:
            target_spec = next(
                item for item in task.layout["objects"] if item["name"] == predicate["target"]
            )
            position[:, 2] = target.pose.p[:, 2] + float(target_spec["wall"]) + bottom_offset
        elif support_name:
            support = task.actors[support_name]
            support_spec = next(
                item for item in task.layout["objects"] if item["name"] == support_name
            )
            position[:, 2] = support.pose.p[:, 2] + float(support_spec["wall"]) + bottom_offset
        else:
            position[:, 2] = bottom_offset
        quaternion = torch.zeros((task.num_envs, 4), dtype=torch.float32, device=task.device)
        quaternion[:, 0] = 1.0
        actor.set_pose(Pose.create_from_pq(position, quaternion))
        actor.set_linear_velocity(torch.zeros((task.num_envs, 3), device=task.device))
        actor.set_angular_velocity(torch.zeros((task.num_envs, 3), device=task.device))


def validate_task(env_id: str, seed: int) -> dict:
    env = gym.make(
        env_id,
        obs_mode="state",
        control_mode="pd_joint_pos",
        reward_mode="dense",
        max_episode_steps=100,
        sim_config={"sim_freq": 150, "control_freq": 30},
    )
    try:
        obs, _ = env.reset(seed=seed)
        initial = env.unwrapped.evaluate()
        if bool(initial["success"].item()):
            raise AssertionError(f"{env_id} begins in a successful state")
        if float(initial["partial_task_score"].item()) != 0.0:
            raise AssertionError(f"{env_id} begins with non-zero partial task score")

        partial_score_trace = []
        completed = 0
        info = initial
        for stage in env.unwrapped.layout["stages"]:
            _force_declared_goals(env, stage["predicates"])
            info = env.unwrapped.evaluate()
            completed += len(stage["predicates"])
            expected_score = completed / len(env.unwrapped.predicates)
            actual_score = float(info["partial_task_score"].item())
            partial_score_trace.append(actual_score)
            if not np.isclose(actual_score, expected_score):
                raise AssertionError(
                    f"{env_id} partial score {actual_score} != expected {expected_score}"
                )
        if not bool(info["success"].item()):
            raise AssertionError(f"{env_id} declared goal poses do not satisfy success")
        if float(info["partial_task_score"].item()) != 1.0:
            raise AssertionError(f"{env_id} goal state did not receive full partial score")

        rollout = run_episode(
            env,
            _ZeroClient(env.action_space.shape),
            obs,
            instruction="forced-goal validation",
            max_steps=1,
        )
        if not rollout["success"] or rollout["termination_reason"] != "success":
            raise AssertionError(
                f"{env_id} did not terminate after success; "
                f"rollout={rollout['termination_reason']}, "
                f"subtasks={rollout['subtask_current']}"
            )
        return {
            "env_id": env_id,
            "success": rollout["success"],
            "partial_task_score": rollout["partial_task_score"],
            "completed_subtasks": rollout["completed_subtasks"],
            "total_subtasks": rollout["total_subtasks"],
            "partial_score_trace": partial_score_trace,
        }
    finally:
        env.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-profile", default="d435i-wrist-raw-rigid-optimized")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    install_upstream_camera_profile(load_profile(args.camera_profile))
    from . import service_scenes  # noqa: F401

    results = [validate_task(layout["env_id"], args.seed) for layout in service_scene_layouts()]
    print(json.dumps({"tasks": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
