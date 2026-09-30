"""Render human and policy-camera previews for the AAG service scenes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import gymnasium as gym
import imageio.v2 as imageio
import mani_skill.envs  # noqa: F401
import numpy as np
from PIL import Image, ImageDraw

from .benchmark import scenario_by_id
from .camera_profiles import install_upstream_camera_profile, load_profile
from .paths import REPO_ROOT
from .service_scene_spec import service_scene_layout, service_scene_layouts


def _as_uint8(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    array = np.asarray(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.dtype != np.uint8:
        array = array * 255.0 if array.max(initial=0.0) <= 1.0 else array
        array = np.clip(array, 0, 255).astype(np.uint8)
    return array


def _sensor_frames(obs: dict[str, Any]) -> dict[str, np.ndarray]:
    result = {}
    for camera, sensor_data in (obs.get("sensor_data") or {}).items():
        image = sensor_data.get("rgb") if isinstance(sensor_data, dict) else sensor_data
        if image is not None:
            result[camera] = _as_uint8(image)
    return result


def _overview(
    output: Path,
    *,
    scene_name: str,
    instruction: str,
    panels: list[tuple[str, np.ndarray]],
) -> None:
    panel_width, panel_height = 640, 360
    label_height, header_height, footer_height = 26, 42, 54
    canvas = Image.new(
        "RGB",
        (panel_width * 2, header_height + (panel_height + label_height) * 2 + footer_height),
        (18, 18, 20),
    )
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 12), scene_name, fill=(245, 245, 245))
    for index, (title, frame) in enumerate(panels):
        column, row = index % 2, index // 2
        x = column * panel_width
        y = header_height + row * (panel_height + label_height)
        draw.rectangle((x, y, x + panel_width, y + label_height), fill=(28, 28, 32))
        draw.text((x + 10, y + 7), title, fill=(235, 235, 235))
        panel = Image.fromarray(frame).resize((panel_width, panel_height), Image.Resampling.LANCZOS)
        canvas.paste(panel, (x, y + label_height))
    footer_y = header_height + (panel_height + label_height) * 2
    draw.rectangle((0, footer_y, canvas.width, canvas.height), fill=(28, 28, 32))
    draw.text((12, footer_y + 10), f"Resolved command: {instruction}", fill=(235, 235, 235))
    canvas.save(output)


def render_scene(
    *,
    scene_id: str,
    camera_profile: str,
    output_dir: Path,
    seed: int,
    shader_pack: str,
) -> dict[str, Any]:
    layout = service_scene_layout(scene_id)
    benchmark = scenario_by_id(scene_id)
    scene_dir = output_dir / scene_id
    scene_dir.mkdir(parents=True, exist_ok=True)

    env = gym.make(
        layout["preview_env_id"],
        obs_mode="rgb",
        control_mode="pd_joint_pos",
        render_mode="rgb_array",
        max_episode_steps=1,
        reward_mode="none",
        sensor_configs={"shader_pack": shader_pack},
        sim_config={"sim_freq": 150, "control_freq": 30},
    )
    try:
        obs, _ = env.reset(seed=seed)
        human = _as_uint8(env.unwrapped.render_rgb_array())
        frames = _sensor_frames(obs)
    finally:
        env.close()

    imageio.imwrite(scene_dir / "human.png", human)
    for camera in ("top_cam", "left_cam", "right_cam"):
        imageio.imwrite(scene_dir / f"{camera}.png", frames[camera])
    _overview(
        scene_dir / "overview.png",
        scene_name=benchmark["scene"],
        instruction=benchmark["resolved_instruction"],
        panels=[
            ("Human overview", human),
            ("Policy top_cam", frames["top_cam"]),
            ("Policy left_cam", frames["left_cam"]),
            ("Policy right_cam", frames["right_cam"]),
        ],
    )

    manifest = {
        "scene_id": scene_id,
        "env_id": layout["env_id"],
        "preview_env_id": layout["preview_env_id"],
        "camera_profile": camera_profile,
        "seed": seed,
        "status": "visual-preview",
        "broad_instruction": benchmark["broad_instruction"],
        "resolved_instruction": benchmark["resolved_instruction"],
        "objects": [
            {
                "name": spec["name"],
                "label": spec["label"],
                "kind": spec["kind"],
                "asset_id": spec.get("asset_id"),
                "scale": spec.get("scale", 1.0),
                "position_xy": spec["position_xy"],
            }
            for spec in layout["objects"]
        ],
        "warning": (
            "This manifest is a visual preview. Use env_id for closed-loop evaluation; "
            "preview_env_id is limited to one step."
        ),
    }
    (scene_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        action="append",
        choices=[scene["id"] for scene in service_scene_layouts()],
        default=[],
        help="scene to render; repeat or omit to render all five",
    )
    parser.add_argument("--camera-profile", default="d435i-wrist-raw-rigid-optimized")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "outputs" / "service_scene_previews",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--shader-pack", default="minimal")
    args = parser.parse_args()

    profile = load_profile(args.camera_profile)
    install_upstream_camera_profile(profile)
    from . import service_scenes  # noqa: F401  # register environments after profile patch

    selected = args.scene or [scene["id"] for scene in service_scene_layouts()]
    root = args.output_dir.resolve() / profile.name
    summaries = [
        render_scene(
            scene_id=scene_id,
            camera_profile=profile.name,
            output_dir=root,
            seed=args.seed,
            shader_pack=args.shader_pack,
        )
        for scene_id in selected
    ]
    print(json.dumps({"output_dir": str(root), "scenes": summaries}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
