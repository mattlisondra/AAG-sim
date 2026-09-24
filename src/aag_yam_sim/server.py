"""Read-only diagnostics for an existing MolmoAct2 YAM inference server."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import requests

from .contracts import YAM_CONTRACT, normalize_server_url


@dataclass(frozen=True)
class ServerCheck:
    url: str
    response: dict[str, Any]
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def check_server(value: str, timeout: float = 5.0) -> ServerCheck:
    url = normalize_server_url(value)
    response = requests.get(url, timeout=timeout)
    response.raise_for_status()
    payload = response.json()
    errors: list[str] = []
    expected = {
        "status": "ok",
        "repo_id": YAM_CONTRACT.repo_id,
        "norm_tag": YAM_CONTRACT.norm_tag,
        "num_cameras": len(YAM_CONTRACT.camera_keys),
        "state_dim": YAM_CONTRACT.state_dim,
    }
    for key, value in expected.items():
        if payload.get(key) != value:
            errors.append(f"{key}: expected {value!r}, got {payload.get(key)!r}")
    return ServerCheck(url=url, response=payload, errors=tuple(errors))
