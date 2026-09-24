"""Authoritative MolmoAct2-BimanualYAM wire-contract constants."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class YAMContract:
    repo_id: str = "allenai/MolmoAct2-BimanualYAM"
    norm_tag: str = "yam_dual_molmoact2"
    camera_keys: tuple[str, str, str] = ("top_cam", "left_cam", "right_cam")
    state_dim: int = 14
    action_dim: int = 14
    default_port: int = 8202
    image_width: int = 640
    image_height: int = 360


YAM_CONTRACT = YAMContract()


def normalize_server_url(value: str) -> str:
    """Normalize a host, host:port, or URL into the YAM ``/act`` endpoint."""
    url = value.strip().rstrip("/")
    if not url:
        raise ValueError("server URL cannot be empty")
    if "://" not in url:
        url = f"http://{url}"
    if not url.endswith("/act"):
        url = f"{url}/act"
    return url
