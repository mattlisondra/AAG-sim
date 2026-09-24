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
