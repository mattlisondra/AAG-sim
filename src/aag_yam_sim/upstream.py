"""Validated integration with the pinned upstream simulation evaluator."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from .camera_profiles import CameraProfile, install_upstream_camera_profile

REQUIRED_UPSTREAM_FILES = (
    "sim_eval/run_eval.py",
    "sim_eval/robots/bimanual_yam.py",
    "sim_eval/inference/client.py",
    "examples/yam/host_server_yam.py",
)


def validate_upstream(path: Path) -> list[str]:
    return [name for name in REQUIRED_UPSTREAM_FILES if not (path / name).is_file()]


def run_official_eval(
    *,
    upstream_dir: Path,
    server_url: str,
    profile: CameraProfile,
    env_ids: list[str],
    instruction: str | None,
    episodes: int,
    max_episode_steps: int,
    n_action_steps: int | None,
    seed: int,
    output_dir: Path,
    save_video: bool,
) -> dict[str, Any]:
    """Run upstream's evaluator with a process-local camera profile override."""
    missing = validate_upstream(upstream_dir)
    if missing:
        raise FileNotFoundError(f"invalid MolmoAct2 checkout; missing: {', '.join(missing)}")
    if str(upstream_dir) not in sys.path:
        sys.path.insert(0, str(upstream_dir))

    install_upstream_camera_profile(profile)

    from sim_eval.inference.client import YAMClient
    from sim_eval.run_eval import EvalConfig, _evaluate_task

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = output_dir / timestamp
    config = EvalConfig(
        policy_type="remote-yam",
        remote_url=server_url,
        env_id=env_ids,
        language_instruction=instruction,
        n_episodes=episodes,
        max_episode_steps=max_episode_steps,
        n_action_steps=n_action_steps,
        seed=seed,
        output_dir=str(run_dir),
        save_video=save_video,
    )
    client = YAMClient(url=server_url, n_action_steps=n_action_steps)
    task_results: dict[str, Any] = {}
    success_rates: list[float] = []
    rewards: list[float] = []
    for env_id in env_ids:
        result = _evaluate_task(env_id, client, config)
        task_results[env_id] = result
        success_rates.append(float(result["success_rate"]))
        rewards.append(float(result["avg_reward"]))

    report = {
        "integration": {
            "camera_profile": profile.name,
            "camera_profile_source": str(profile.source),
            "camera_calibration": profile.calibration,
            "upstream_dir": str(upstream_dir),
            "server_url": server_url,
        },
        "tasks": task_results,
        "overall": {
            "mean_success_rate": float(np.mean(success_rates)),
            "mean_reward": float(np.mean(rewards)),
            "num_tasks": len(task_results),
        },
    }
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return {"output_dir": str(run_dir), **report}
