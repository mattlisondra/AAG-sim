"""Load and validate simulator camera profiles.

Profiles describe virtual pinhole cameras. They do not claim to calibrate a
physical camera merely because a device family name appears in the profile.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import YAM_CONTRACT
from .paths import CAMERA_CONFIG_DIR

CAMERA_ORDER = YAM_CONTRACT.camera_keys


@dataclass(frozen=True)
class CameraSpec:
    uid: str
    model: str
    width: int
    height: int
    intrinsic: np.ndarray
    mount: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraProfile:
    name: str
    calibration: str
    description: str
    source: Path
    cameras: dict[str, CameraSpec]


def intrinsic_from_hfov(width: int, height: int, hfov_deg: float) -> np.ndarray:
    """Match the square-pixel projection used by upstream BimanualYAM."""
    if not 1.0 < hfov_deg < 179.0:
        raise ValueError(f"hfov_deg must be in (1, 179), got {hfov_deg}")
    fx = (width / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    return np.array(
        [[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )


def _matrix(camera: dict[str, Any], uid: str) -> np.ndarray:
    raw = camera.get("intrinsic")
    if raw is None:
        hfov = camera.get("nominal_hfov_deg")
        if hfov is None:
            raise ValueError(f"camera {uid!r} needs 'intrinsic' or 'nominal_hfov_deg'")
        raw = intrinsic_from_hfov(int(camera["width"]), int(camera["height"]), float(hfov))
    matrix = np.asarray(raw, dtype=np.float32)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(f"camera {uid!r} intrinsic must be a finite 3x3 matrix")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
        raise ValueError(f"camera {uid!r} intrinsic is not a valid pinhole matrix")
    return matrix


def resolve_profile(value: str | Path) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    named = CAMERA_CONFIG_DIR / f"{value}.json"
    if named.is_file():
        return named
    options = ", ".join(path.stem for path in available_profiles())
    raise FileNotFoundError(f"camera profile {value!r} not found; available: {options}")


def load_profile(value: str | Path) -> CameraProfile:
    path = resolve_profile(value)
    data = json.loads(path.read_text(encoding="utf-8"))
    camera_data = data.get("cameras", {})
    missing = [key for key in CAMERA_ORDER if key not in camera_data]
    extra = sorted(set(camera_data) - set(CAMERA_ORDER))
    if missing or extra:
        raise ValueError(f"camera keys must be {CAMERA_ORDER}; missing={missing}, extra={extra}")

    cameras: dict[str, CameraSpec] = {}
    for uid in CAMERA_ORDER:
        raw = camera_data[uid]
        position = tuple(float(x) for x in raw["position_m"])
        quaternion = tuple(float(x) for x in raw["quaternion_wxyz"])
        if len(position) != 3 or len(quaternion) != 4:
            raise ValueError(f"camera {uid!r} needs 3-D position and wxyz quaternion")
        norm = float(np.linalg.norm(quaternion))
        if not np.isclose(norm, 1.0, atol=1e-4):
            raise ValueError(f"camera {uid!r} quaternion norm is {norm:.6f}, expected 1")
        width, height = int(raw["width"]), int(raw["height"])
        if width <= 0 or height <= 0:
            raise ValueError(f"camera {uid!r} resolution must be positive")
        cameras[uid] = CameraSpec(
            uid=uid,
            model=str(raw["model"]),
            width=width,
            height=height,
            intrinsic=_matrix(raw, uid),
            mount=str(raw["mount"]),
            position_m=position,
            quaternion_wxyz=quaternion,
        )

    return CameraProfile(
        name=str(data["name"]),
        calibration=str(data["calibration"]),
        description=str(data.get("description", "")),
        source=path,
        cameras=cameras,
    )


def available_profiles() -> list[Path]:
    return sorted(
        path for path in CAMERA_CONFIG_DIR.glob("*.json") if not path.name.endswith(".example.json")
    )


def install_upstream_camera_profile(profile: CameraProfile):
    """Patch the imported upstream YAM agent's sensor property for this process.

    Importing is intentionally delayed so dependency-light commands and tests do
    not require ManiSkill/SAPIEN.
    """
    import sapien
    from mani_skill.sensors.camera import CameraConfig
    from sim_eval.robots.bimanual_yam import BimanualYAM

    def sensor_configs(robot):
        result = []
        for uid in CAMERA_ORDER:
            spec = profile.cameras[uid]
            if spec.mount not in robot.robot.links_map:
                raise KeyError(
                    f"camera {uid!r} mount {spec.mount!r} is not a YAM link; "
                    f"available={sorted(robot.robot.links_map)}"
                )
            result.append(
                CameraConfig(
                    uid=uid,
                    pose=sapien.Pose(p=spec.position_m, q=spec.quaternion_wxyz),
                    width=spec.width,
                    height=spec.height,
                    intrinsic=spec.intrinsic,
                    near=0.01,
                    far=100,
                    mount=robot.robot.links_map[spec.mount],
                )
            )
        return result

    BimanualYAM._sensor_configs = property(sensor_configs)
    return BimanualYAM
