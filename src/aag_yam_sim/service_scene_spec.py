"""Load the declarative ManiSkill preview layouts for AAG service scenes."""

from __future__ import annotations

import json
from typing import Any

from .paths import SERVICE_SCENE_LAYOUTS_PATH


def load_service_scene_layouts() -> dict[str, Any]:
    return json.loads(SERVICE_SCENE_LAYOUTS_PATH.read_text(encoding="utf-8"))


def service_scene_layouts() -> list[dict[str, Any]]:
    return list(load_service_scene_layouts()["scenes"])


def service_scene_layout(scene_id: str) -> dict[str, Any]:
    for scene in service_scene_layouts():
        if scene["id"] == scene_id:
            return scene
    known = ", ".join(scene["id"] for scene in service_scene_layouts())
    raise KeyError(f"unknown service scene {scene_id!r}; known: {known}")


def service_scene_by_env_id(env_id: str) -> dict[str, Any]:
    for scene in service_scene_layouts():
        if scene["env_id"] == env_id:
            return scene
    known = ", ".join(scene["env_id"] for scene in service_scene_layouts())
    raise KeyError(f"unknown service-scene environment {env_id!r}; known: {known}")
