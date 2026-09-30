"""Closed-loop evaluator with RoboLab-style subtask progress reporting."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import torch
from sim_eval.inference.client import MolmoActClientBase
from sim_eval.run_eval import (
    DEFAULT_LANGUAGE_INSTRUCTIONS,
    EvalConfig,
    _capture_frame,
    _extract_input_frames,
    _save_image,
    _save_video,
)
from tqdm import tqdm

from .benchmark import scenario_by_env_id

logger = logging.getLogger(__name__)


def _scalar(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().reshape(-1)
        return float(value[0].item()) if len(value) else default
    array = np.asarray(value).reshape(-1)
    return float(array[0]) if len(array) else default


def _boolean(value: Any) -> bool:
    return bool(_scalar(value, 0.0))


def _instruction_for(env_id: str, override: str | None) -> str:
    if override:
        return override
    try:
        return str(scenario_by_env_id(env_id)["resolved_instruction"])
    except KeyError:
        return DEFAULT_LANGUAGE_INSTRUCTIONS.get(env_id, env_id)


def _subtask_values(info: dict[str, Any], prefix: str) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for key, value in info.items():
        if not key.startswith(prefix):
            continue
        name = key.removeprefix(prefix)
        if prefix == "subtask_first_step/":
            values[name] = int(_scalar(value, -1.0))
        else:
            values[name] = _boolean(value)
    return values


def run_episode(
    env: gym.Env,
    client: MolmoActClientBase,
    obs: dict,
    instruction: str,
    max_steps: int,
) -> dict[str, Any]:
    total_reward = 0.0
    success = False
    max_score = 0.0
    completed_subtasks = 0
    total_subtasks = 0
    frames: list[np.ndarray] = []
    input_frames = _extract_input_frames(obs) if isinstance(obs, dict) else {}
    final_info: dict[str, Any] = {}
    termination_reason = "max_steps"

    frame = _capture_frame(env)
    if frame is not None:
        frames.append(frame)

    _step_index = 0
    for _step_index in range(max_steps):
        action = client.infer(obs, instruction)
        obs, reward, terminated, truncated, info = env.step(action)
        final_info = info
        total_reward += _scalar(reward)

        step_success = _boolean(info.get("success", False))
        success |= step_success
        score = info.get("partial_task_score")
        if score is None and "n_in_box" in info and "n_total" in info:
            score = _scalar(info["n_in_box"]) / max(_scalar(info["n_total"]), 1.0)
        max_score = max(max_score, _scalar(score, 1.0 if step_success else 0.0))
        completed_value = info.get("completed_subtasks", info.get("n_in_box"))
        total_value = info.get("total_subtasks", info.get("n_total"))
        completed_subtasks = max(
            completed_subtasks,
            int(_scalar(completed_value, round(max_score))),
        )
        total_subtasks = max(total_subtasks, int(_scalar(total_value, 0.0)))

        frame = _capture_frame(env)
        if frame is not None:
            frames.append(frame)

        terminated_value = _boolean(terminated)
        truncated_value = _boolean(truncated)
        if terminated_value or truncated_value:
            termination_reason = "success" if step_success else "truncated"
            break

    if success:
        max_score = 1.0
    return {
        "total_reward": total_reward,
        "success": success,
        "partial_task_score": max_score,
        "completed_subtasks": completed_subtasks,
        "total_subtasks": total_subtasks,
        "steps": _step_index + 1,
        "termination_reason": termination_reason,
        "subtask_complete": _subtask_values(final_info, "subtask_complete/"),
        "subtask_current": _subtask_values(final_info, "subtask_current/"),
        "subtask_first_step": _subtask_values(final_info, "subtask_first_step/"),
        "frames": frames,
        "input_frames": input_frames,
    }


def evaluate_task(env_id: str, client: MolmoActClientBase, config: EvalConfig) -> dict[str, Any]:
    instruction = _instruction_for(env_id, config.language_instruction)
    logger.info("─" * 50)
    logger.info("Task: %s  |  instruction: %s", env_id, instruction)

    env = gym.make(
        env_id,
        obs_mode="rgb",
        control_mode="pd_joint_pos",
        render_mode="rgb_array",
        max_episode_steps=config.max_episode_steps,
        reward_mode="none",
        sensor_configs={"shader_pack": config.shader_pack},
        sim_config={"sim_freq": config.sim_freq, "control_freq": config.control_freq},
    )
    out_dir = Path(config.output_dir)
    episodes: list[dict[str, Any]] = []
    successes: list[bool] = []
    scores: list[float] = []
    rewards: list[float] = []
    steps_list: list[int] = []

    try:
        progress = tqdm(range(config.n_episodes), desc=env_id, leave=False)
        for episode in progress:
            episode_seed = config.seed + episode
            obs, _ = env.reset(seed=episode_seed)
            torch.manual_seed(episode_seed)
            np.random.seed(episode_seed)

            if episode == 0:
                sensors = list((obs.get("sensor_data") or {}).keys())
                logger.info(
                    "Cameras in obs: %s  |  server expects: %s",
                    sensors,
                    list(client.schema.camera_keys),
                )

            result = run_episode(env, client, obs, instruction, config.max_episode_steps)
            client.reset()
            episode_result = {
                "episode": episode,
                "seed": episode_seed,
                "instruction": instruction,
                "success": result["success"],
                "score": result["partial_task_score"],
                "partial_task_score": result["partial_task_score"],
                "completed_subtasks": result["completed_subtasks"],
                "total_subtasks": result["total_subtasks"],
                "subtask_complete": result["subtask_complete"],
                "subtask_current": result["subtask_current"],
                "subtask_first_step": result["subtask_first_step"],
                "reward": float(result["total_reward"]),
                "steps": result["steps"],
                "termination_reason": result["termination_reason"],
            }
            episodes.append(episode_result)
            successes.append(result["success"])
            scores.append(result["partial_task_score"])
            rewards.append(result["total_reward"])
            steps_list.append(result["steps"])

            if config.save_video and episode < config.max_videos and result["frames"]:
                _save_video(
                    result["frames"],
                    out_dir / "videos" / env_id / f"ep{episode:03d}.mp4",
                    fps=config.control_freq,
                )
            if episode < config.max_videos and result["input_frames"]:
                for camera, frame in result["input_frames"].items():
                    _save_image(
                        frame,
                        out_dir / "frames" / env_id / f"ep{episode:03d}_{camera}.png",
                    )
            progress.set_postfix(
                success=f"{np.mean(successes) * 100:.0f}%",
                score=f"{np.mean(scores):.2f}",
            )
    finally:
        env.close()

    failed_scores = [score for score, success in zip(scores, successes, strict=True) if not success]
    summary = {
        "env_id": env_id,
        "instruction": instruction,
        "success_rate": float(np.mean(successes)),
        "partial_task_score": float(np.mean(scores)),
        "failed_partial_task_score": float(np.mean(failed_scores)) if failed_scores else None,
        "avg_reward": float(np.mean(rewards)),
        "avg_steps": float(np.mean(steps_list)),
        "episodes": episodes,
    }
    logger.info(
        "%s  success=%.0f%%  partial_score=%.3f  avg_steps=%.0f",
        env_id,
        summary["success_rate"] * 100,
        summary["partial_task_score"],
        summary["avg_steps"],
    )
    return summary
