"""Load the machine-readable AAG benchmark specification."""

from __future__ import annotations

import json
from typing import Any

from .paths import BENCHMARK_PATH


def load_benchmark() -> dict[str, Any]:
    return json.loads(BENCHMARK_PATH.read_text(encoding="utf-8"))


def scenarios() -> list[dict[str, Any]]:
    return list(load_benchmark()["scenarios"])


def scenario_by_id(scenario_id: str) -> dict[str, Any]:
    for item in scenarios():
        if item["id"] == scenario_id:
            return item
    known = ", ".join(item["id"] for item in scenarios())
    raise KeyError(f"unknown scenario {scenario_id!r}; known: {known}")


def scenario_by_env_id(env_id: str) -> dict[str, Any]:
    for item in scenarios():
        if env_id in {item["env_id"], item["preview_env_id"]}:
            return item
    known = ", ".join(
        env_id for item in scenarios() for env_id in (item["env_id"], item["preview_env_id"])
    )
    raise KeyError(f"unknown scenario environment {env_id!r}; known: {known}")
