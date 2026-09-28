"""Paired ManiSkill comparison of D405-reference and D435i wrist views.

This module renders both camera models from an identical simulation state.  It
does not construct, download, or call a MolmoAct2 policy.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from .camera_profiles import (
    CameraProfile,
    CameraSpec,
    install_upstream_camera_profile,
    load_profile,
)
from .paths import DEFAULT_UPSTREAM_DIR, REPO_ROOT
from .wrist_match import PinholeIntrinsics, WristMatchPlan, apply_match_plan, compute_match_plan

WRIST_CAMERAS = ("left_cam", "right_cam")
OPTIMIZATION_STEPS = {
    "match_distance_mm": 4.0,
    "camera_mount_image_up_mm": 3.0,
    "pitch_trim_deg": 2.0,
    "mirrored_lateral_mm": 3.0,
    "mirrored_yaw_deg": 1.0,
    "crop_offset_x_px": 4.0,
    "crop_offset_y_px": 4.0,
}
DEFAULT_REFERENCE_DISTANCE_MM = 116.9619168789568
DEFAULT_MATCH_DISTANCE_MM = 166.71790944273448
DEFAULT_VIEW_DOWN_ANGLE_DEG = 20.307327540589718
DEFAULT_CAMERA_MOUNT_IMAGE_UP_MM = 24.0


@dataclass(frozen=True)
class MountParameters:
    """Physical parameters shared by the CAD generator and simulated mount."""

    match_distance_mm: float = DEFAULT_MATCH_DISTANCE_MM
    camera_mount_image_up_mm: float = DEFAULT_CAMERA_MOUNT_IMAGE_UP_MM
    pitch_trim_deg: float = 0.0
    mirrored_lateral_mm: float = 0.0
    mirrored_yaw_deg: float = 0.0
    crop_offset_x_px: float = 0.0
    crop_offset_y_px: float = 0.0


def quaternion_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(first, dtype=float)
    bw, bx, by, bz = np.asarray(second, dtype=float)
    return np.asarray(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quaternion_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, dtype=float)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise ValueError("zero-length quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def mount_pose_from_parameters(
    reference_spec: CameraSpec,
    parameters: MountParameters,
    *,
    reference_distance_mm: float = DEFAULT_REFERENCE_DISTANCE_MM,
    view_down_angle_deg: float = DEFAULT_VIEW_DOWN_ANGLE_DEG,
    mirror_sign: int = 0,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Return CAD-equivalent optical pose relative to the wrist link."""
    if parameters.match_distance_mm <= reference_distance_mm:
        raise ValueError("match distance must exceed the reference distance")
    extra_ray = parameters.match_distance_mm - reference_distance_mm
    down_angle = math.radians(view_down_angle_deg)
    optical_setback = extra_ray * math.cos(down_angle)
    ideal_image_up = extra_ray * math.sin(down_angle)
    if parameters.camera_mount_image_up_mm < ideal_image_up:
        raise ValueError(
            "camera mount is below the crop-feasible ray: "
            f"need at least {ideal_image_up:.3f} mm image-up"
        )

    reference_q = np.asarray(reference_spec.quaternion_wxyz, dtype=float)
    reference_rotation = quaternion_matrix(reference_q)
    forward = reference_rotation[:, 0]
    image_up = reference_rotation[:, 2]
    image_right = -reference_rotation[:, 1]
    position = (
        np.asarray(reference_spec.position_m, dtype=float)
        - optical_setback / 1000.0 * forward
        + parameters.camera_mount_image_up_mm / 1000.0 * image_up
        + mirror_sign * parameters.mirrored_lateral_mm / 1000.0 * image_right
    )

    geometric_pitch = math.atan2(
        parameters.camera_mount_image_up_mm - ideal_image_up,
        parameters.match_distance_mm,
    )
    total_pitch = geometric_pitch + math.radians(parameters.pitch_trim_deg)
    pitch_quaternion = np.asarray(
        [math.cos(total_pitch / 2.0), 0.0, math.sin(total_pitch / 2.0), 0.0]
    )
    yaw = mirror_sign * math.radians(parameters.mirrored_yaw_deg)
    yaw_quaternion = np.asarray([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
    quaternion = quaternion_multiply(
        quaternion_multiply(reference_q, pitch_quaternion),
        yaw_quaternion,
    )
    quaternion /= np.linalg.norm(quaternion)
    details = {
        "optical_setback_mm": float(optical_setback),
        "ideal_image_up_mm": float(ideal_image_up),
        "geometric_pitch_deg": float(math.degrees(geometric_pitch)),
        "total_pitch_deg": float(math.degrees(total_pitch)),
        "mirrored_lateral_mm": float(mirror_sign * parameters.mirrored_lateral_mm),
        "mirrored_yaw_deg": float(mirror_sign * parameters.mirrored_yaw_deg),
    }
    return position, quaternion, details


def image_metrics(reference: np.ndarray, candidate: np.ndarray) -> dict[str, float]:
    """Return deterministic RGB and edge-alignment metrics; lower score is better."""
    ref = np.asarray(reference, dtype=np.float32) / 255.0
    cand = np.asarray(candidate, dtype=np.float32) / 255.0
    if ref.shape != cand.shape or ref.ndim != 3 or ref.shape[2] != 3:
        raise ValueError(f"expected equal HxWx3 images, got {ref.shape} and {cand.shape}")
    difference = ref - cand
    mae = float(np.mean(np.abs(difference)))
    rmse = float(np.sqrt(np.mean(difference * difference)))
    psnr = float("inf") if rmse == 0 else float(20.0 * math.log10(1.0 / rmse))

    weights = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    ref_gray = ref @ weights
    cand_gray = cand @ weights
    ref_gradients = (np.diff(ref_gray, axis=0), np.diff(ref_gray, axis=1))
    cand_gradients = (np.diff(cand_gray, axis=0), np.diff(cand_gray, axis=1))
    edge_mae = float(
        0.5
        * (
            np.mean(np.abs(ref_gradients[0] - cand_gradients[0]))
            + np.mean(np.abs(ref_gradients[1] - cand_gradients[1]))
        )
    )

    ref_centered = ref_gray - np.mean(ref_gray)
    cand_centered = cand_gray - np.mean(cand_gray)
    denominator = float(np.linalg.norm(ref_centered) * np.linalg.norm(cand_centered))
    ncc = 0.0 if denominator == 0 else float(np.sum(ref_centered * cand_centered) / denominator)
    score = 0.75 * mae + 0.25 * edge_mae
    return {
        "score": float(score),
        "mae": mae,
        "rmse": rmse,
        "psnr_db": psnr,
        "edge_mae": edge_mae,
        "gray_ncc": ncc,
    }


def _binary_dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """Small dependency-free square dilation used only to weight nearby RGB edges."""
    source = np.asarray(mask, dtype=bool)
    padded = np.pad(source, radius, mode="constant")
    result = np.zeros_like(source)
    for dy in range(2 * radius + 1):
        for dx in range(2 * radius + 1):
            result |= padded[dy : dy + source.shape[0], dx : dx + source.shape[1]]
    return result


def _mask_iou(reference: np.ndarray, candidate: np.ndarray) -> float:
    intersection = int(np.count_nonzero(reference & candidate))
    union = int(np.count_nonzero(reference | candidate))
    return 1.0 if union == 0 else intersection / union


def task_alignment_metrics(
    reference_rgb: np.ndarray,
    candidate_rgb: np.ndarray,
    reference_segmentation: np.ndarray,
    candidate_segmentation: np.ndarray,
    *,
    finger_ids: tuple[int, ...],
    task_ids: tuple[int, ...],
) -> dict[str, float]:
    """Measure the geometry the manipulation policy actually needs to retain.

    Global RGB error is dominated by the table texture.  This objective gives
    explicit priority to the finger silhouettes and task actors, then compares
    RGB/edges in a dilated neighborhood around those foreground regions.
    """
    reference_fingers = np.isin(reference_segmentation, finger_ids)
    candidate_fingers = np.isin(candidate_segmentation, finger_ids)
    reference_task = np.isin(reference_segmentation, task_ids)
    candidate_task = np.isin(candidate_segmentation, task_ids)
    finger_iou = _mask_iou(reference_fingers, candidate_fingers)
    task_iou = _mask_iou(reference_task, candidate_task)

    focus = _binary_dilate(
        reference_fingers | candidate_fingers | reference_task | candidate_task,
        radius=8,
    )
    weights = np.full(focus.shape, 0.02, dtype=np.float32)
    weights[focus] = 1.0
    weights[_binary_dilate(reference_task | candidate_task, radius=4)] = 2.0
    weights[_binary_dilate(reference_fingers | candidate_fingers, radius=4)] = 3.0

    ref = np.asarray(reference_rgb, dtype=np.float32) / 255.0
    cand = np.asarray(candidate_rgb, dtype=np.float32) / 255.0
    absolute = np.mean(np.abs(ref - cand), axis=2)
    weighted_mae = float(np.sum(absolute * weights) / np.sum(weights))

    gray_weights = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    ref_gray = ref @ gray_weights
    cand_gray = cand @ gray_weights
    ref_edge = np.hypot(
        np.pad(np.diff(ref_gray, axis=1), ((0, 0), (0, 1))),
        np.pad(np.diff(ref_gray, axis=0), ((0, 1), (0, 0))),
    )
    cand_edge = np.hypot(
        np.pad(np.diff(cand_gray, axis=1), ((0, 0), (0, 1))),
        np.pad(np.diff(cand_gray, axis=0), ((0, 1), (0, 0))),
    )
    weighted_edge_mae = float(np.sum(np.abs(ref_edge - cand_edge) * weights) / np.sum(weights))

    finger_loss = 1.0 - finger_iou
    task_loss = 1.0 - task_iou
    score = (
        0.50 * finger_loss
        + 0.30 * task_loss
        + 0.15 * weighted_mae
        + 0.05 * weighted_edge_mae
    )
    return {
        "score": float(score),
        "finger_iou": float(finger_iou),
        "task_iou": float(task_iou),
        "focus_rgb_mae": weighted_mae,
        "focus_edge_mae": weighted_edge_mae,
    }


def _as_uint8(image: Any) -> np.ndarray:
    try:
        import torch

        if isinstance(image, torch.Tensor):
            image = image.detach().cpu().numpy()
    except ImportError:  # pragma: no cover - only the simulation runtime uses torch
        pass
    array = np.asarray(image)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.dtype != np.uint8:
        array = np.clip(array * 255.0 if array.max() <= 1 else array, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def _as_numpy(image: Any) -> np.ndarray:
    try:
        import torch

        if isinstance(image, torch.Tensor):
            image = image.detach().cpu().numpy()
    except ImportError:  # pragma: no cover - only the simulation runtime uses torch
        pass
    array = np.asarray(image)
    if array.ndim >= 3 and array.shape[0] == 1:
        array = array[0]
    return np.ascontiguousarray(array)


def _offset_match_plan(
    plan: WristMatchPlan,
    offset_x_px: float,
    offset_y_px: float,
) -> WristMatchPlan:
    """Shift an otherwise calibrated crop while keeping it inside the raw frame."""
    left = min(
        max(plan.crop_left + int(round(offset_x_px)), 0),
        plan.source.width - plan.crop_width,
    )
    top = min(
        max(plan.crop_top + int(round(offset_y_px)), 0),
        plan.source.height - plan.crop_height,
    )
    scale_x = plan.output_width / plan.crop_width
    scale_y = plan.output_height / plan.crop_height
    effective = PinholeIntrinsics(
        plan.output_width,
        plan.output_height,
        plan.source.fx * scale_x,
        plan.source.fy * scale_y,
        (plan.source.cx - left) * scale_x,
        (plan.source.cy - top) * scale_y,
    )
    return replace(plan, crop_left=left, crop_top=top, effective=effective)


def _apply_match_plan_nearest(frame: np.ndarray, plan: WristMatchPlan) -> np.ndarray:
    """Apply the crop to label images without interpolating actor IDs."""
    array = np.asarray(frame)
    y0, x0 = plan.crop_top, plan.crop_left
    cropped = array[y0 : y0 + plan.crop_height, x0 : x0 + plan.crop_width]
    x = np.rint(np.linspace(0, plan.crop_width - 1, plan.output_width)).astype(np.int64)
    y = np.rint(np.linspace(0, plan.crop_height - 1, plan.output_height)).astype(np.int64)
    return np.ascontiguousarray(cropped[y[:, None], x[None, :]])


def _edge_overlay(reference: np.ndarray, candidate: np.ndarray) -> np.ndarray:
    weights = np.asarray([0.299, 0.587, 0.114], dtype=np.float32)
    ref = np.asarray(reference, dtype=np.float32) / 255.0 @ weights
    cand = np.asarray(candidate, dtype=np.float32) / 255.0 @ weights

    def edges(gray: np.ndarray) -> np.ndarray:
        magnitude = np.hypot(
            np.pad(np.diff(gray, axis=1), ((0, 0), (0, 1))),
            np.pad(np.diff(gray, axis=0), ((0, 1), (0, 0))),
        )
        return np.clip(magnitude * 5.0, 0.0, 1.0)

    ref_edge = edges(ref)
    cand_edge = edges(cand)
    return np.rint(
        np.stack([ref_edge, cand_edge, cand_edge], axis=2) * 255.0
    ).astype(np.uint8)


def _segmentation_overlay(
    reference: np.ndarray,
    candidate: np.ndarray,
    ids: tuple[int, ...],
) -> np.ndarray:
    ref = np.isin(reference, ids)
    cand = np.isin(candidate, ids)
    return np.stack([ref, cand, cand], axis=2).astype(np.uint8) * 255


def _save_image(path: Path, image: np.ndarray) -> None:
    import imageio.v3 as iio

    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, _as_uint8(image))


def _save_montage(path: Path, panels: list[tuple[str, np.ndarray]]) -> None:
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.load_default()
    width = sum(panel.shape[1] for _, panel in panels)
    height = max(panel.shape[0] for _, panel in panels)
    canvas = Image.new("RGB", (width, height + 24), color=(20, 20, 20))
    draw = ImageDraw.Draw(canvas)
    x = 0
    for label, panel in panels:
        image = Image.fromarray(_as_uint8(panel), mode="RGB")
        canvas.paste(image, (x, 24))
        draw.text((x + 6, 6), label, fill=(255, 255, 255), font=font)
        x += panel.shape[1]
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)


def _profile_intrinsics(spec: CameraSpec) -> PinholeIntrinsics:
    return PinholeIntrinsics.from_matrix(spec.width, spec.height, spec.intrinsic)


class ManiSkillCameraComparison:
    def __init__(
        self,
        *,
        upstream_dir: Path,
        env_id: str,
        reference_profile: CameraProfile,
        candidate_profile: CameraProfile,
        cameras: tuple[str, ...],
        seeds: tuple[int, ...],
        shader_pack: str,
        reference_distance_mm: float,
    ) -> None:
        missing_asset = Path.home() / ".maniskill/data/assets/mani_skill2_ycb/info_pick_v0.json"
        if not missing_asset.is_file():
            raise FileNotFoundError(
                "ManiSkill YCB assets are missing. Run: uv run --project "
                "third_party/molmoact2 python -m mani_skill.utils.download_asset ycb -y"
            )
        if str(upstream_dir) not in sys.path:
            sys.path.insert(0, str(upstream_dir))
        install_upstream_camera_profile(reference_profile)
        import gymnasium as gym

        importlib.import_module("sim_eval.tasks")
        self.env = gym.make(
            env_id,
            obs_mode="rgb+segmentation",
            control_mode="pd_joint_pos",
            render_mode="rgb_array",
            reward_mode="none",
            sensor_configs={"shader_pack": shader_pack},
            reconfiguration_freq=0,
            num_envs=1,
        )
        self.reference_profile = reference_profile
        self.candidate_profile = candidate_profile
        self.cameras = cameras
        self.seeds = seeds
        self.reference_distance_mm = reference_distance_mm
        links = self.env.unwrapped.agent.robot.links_map
        self.finger_ids = {
            "left_cam": tuple(
                int(links[name].per_scene_id[0].item())
                for name in ("left_link_left_finger", "left_link_right_finger")
            ),
            "right_cam": tuple(
                int(links[name].per_scene_id[0].item())
                for name in ("right_link_left_finger", "right_link_right_finger")
            ),
        }
        self.task_ids = tuple(
            int(actor.per_scene_id[0].item())
            for name, actor in self.env.unwrapped.scene.actors.items()
            if name not in {"table-workspace", "ground"}
        )
        self.reference_frames = self._capture_references()

    def close(self) -> None:
        self.env.close()

    def _set_camera(self, uid: str, spec: CameraSpec, position: np.ndarray, quaternion: np.ndarray):
        import sapien

        camera = self.env.unwrapped._sensors[uid].camera
        camera.set_local_pose(sapien.Pose(p=position, q=quaternion))
        intrinsic = spec.intrinsic
        camera.set_perspective_parameters(
            camera.get_near(),
            camera.get_far(),
            float(intrinsic[0, 0]),
            float(intrinsic[1, 1]),
            float(intrinsic[0, 2]),
            float(intrinsic[1, 2]),
            0.0,
        )
        camera._cached_intrinsic_matrix = None
        camera._cached_model_matrix = None
        camera._cached_extrinsic_matrix = None

    def _capture(self) -> dict[str, dict[str, np.ndarray]]:
        # get_sensor_images() is a visualization helper that colorizes label
        # images.  Use the raw sensor path so segmentation remains actor IDs.
        images = self.env.unwrapped._get_obs_sensor_data()
        return {
            uid: {
                "rgb": _as_uint8(images[uid]["rgb"]),
                "segmentation": _as_numpy(images[uid]["segmentation"])[..., 0],
            }
            for uid in self.cameras
        }

    def _capture_references(self) -> dict[tuple[int, str], dict[str, np.ndarray]]:
        result: dict[tuple[int, str], dict[str, np.ndarray]] = {}
        for seed in self.seeds:
            self.env.reset(seed=seed)
            for uid in self.cameras:
                spec = self.reference_profile.cameras[uid]
                self._set_camera(
                    uid,
                    spec,
                    np.asarray(spec.position_m),
                    np.asarray(spec.quaternion_wxyz),
                )
            frames = self._capture()
            for uid, frame in frames.items():
                result[(seed, uid)] = frame
        return result

    def evaluate(
        self,
        parameters: MountParameters,
        *,
        save_dir: Path | None = None,
    ) -> dict[str, Any]:
        per_view: list[dict[str, Any]] = []
        plans: dict[str, WristMatchPlan] = {}
        pose_details: dict[str, dict[str, Any]] = {}
        for uid in self.cameras:
            reference_spec = self.reference_profile.cameras[uid]
            candidate_spec = self.candidate_profile.cameras[uid]
            position, quaternion, details = mount_pose_from_parameters(
                reference_spec,
                parameters,
                reference_distance_mm=self.reference_distance_mm,
                mirror_sign=-1 if uid == "left_cam" else 1,
            )
            plan = compute_match_plan(
                _profile_intrinsics(reference_spec),
                _profile_intrinsics(candidate_spec),
                reference_distance_mm=self.reference_distance_mm,
                actual_distance_mm=parameters.match_distance_mm,
            )
            plan = _offset_match_plan(
                plan,
                parameters.crop_offset_x_px,
                parameters.crop_offset_y_px,
            )
            plans[uid] = plan
            pose_details[uid] = {
                **details,
                "position_m": position.tolist(),
                "quaternion_wxyz": quaternion.tolist(),
            }

        for seed in self.seeds:
            self.env.reset(seed=seed)
            for uid in self.cameras:
                spec = self.candidate_profile.cameras[uid]
                pose = pose_details[uid]
                self._set_camera(
                    uid,
                    spec,
                    np.asarray(pose["position_m"]),
                    np.asarray(pose["quaternion_wxyz"]),
                )
            raw_frames = self._capture()
            for uid in self.cameras:
                reference = self.reference_frames[(seed, uid)]
                raw = raw_frames[uid]
                matched = apply_match_plan(raw["rgb"], plans[uid], backend="numpy")
                matched_segmentation = _apply_match_plan_nearest(
                    raw["segmentation"],
                    plans[uid],
                )
                global_metrics = image_metrics(reference["rgb"], matched)
                focus_metrics = task_alignment_metrics(
                    reference["rgb"],
                    matched,
                    reference["segmentation"],
                    matched_segmentation,
                    finger_ids=self.finger_ids[uid],
                    task_ids=self.task_ids,
                )
                per_view.append(
                    {
                        "seed": seed,
                        "camera": uid,
                        **focus_metrics,
                        "global_rgb_score": global_metrics["score"],
                        "global_rgb_mae": global_metrics["mae"],
                        "global_rgb_rmse": global_metrics["rmse"],
                        "global_psnr_db": global_metrics["psnr_db"],
                        "global_edge_mae": global_metrics["edge_mae"],
                        "global_gray_ncc": global_metrics["gray_ncc"],
                    }
                )
                if save_dir is not None:
                    view_dir = save_dir / f"seed_{seed}" / uid
                    overlay = np.rint(
                        0.5 * reference["rgb"].astype(np.float32)
                        + 0.5 * matched.astype(np.float32)
                    ).astype(np.uint8)
                    difference = np.clip(
                        np.abs(reference["rgb"].astype(np.int16) - matched.astype(np.int16))
                        * 4,
                        0,
                        255,
                    ).astype(np.uint8)
                    _save_image(view_dir / "reference_d405.png", reference["rgb"])
                    _save_image(view_dir / "candidate_d435i_raw.png", raw["rgb"])
                    _save_image(view_dir / "candidate_d435i_matched.png", matched)
                    _save_image(view_dir / "overlay_50_50.png", overlay)
                    _save_image(view_dir / "difference_x4.png", difference)
                    _save_image(
                        view_dir / "edge_overlay_red_cyan.png",
                        _edge_overlay(reference["rgb"], matched),
                    )
                    _save_image(
                        view_dir / "finger_mask_overlay_red_cyan.png",
                        _segmentation_overlay(
                            reference["segmentation"],
                            matched_segmentation,
                            self.finger_ids[uid],
                        ),
                    )
                    _save_image(
                        view_dir / "task_mask_overlay_red_cyan.png",
                        _segmentation_overlay(
                            reference["segmentation"],
                            matched_segmentation,
                            self.task_ids,
                        ),
                    )
                    _save_montage(
                        view_dir / "comparison.png",
                        [
                            ("D405 reference", reference["rgb"]),
                            ("D435i raw", raw["rgb"]),
                            ("D435i crop/resize", matched),
                            ("red/cyan edge overlay", _edge_overlay(reference["rgb"], matched)),
                        ],
                    )

        aggregate: dict[str, float] = {}
        for key in (
            "score",
            "finger_iou",
            "task_iou",
            "focus_rgb_mae",
            "focus_edge_mae",
            "global_rgb_score",
            "global_rgb_mae",
            "global_rgb_rmse",
            "global_psnr_db",
            "global_edge_mae",
            "global_gray_ncc",
        ):
            aggregate[key] = float(np.mean([item[key] for item in per_view]))
        return {
            "parameters": asdict(parameters),
            "aggregate": aggregate,
            "per_view": per_view,
            "poses": pose_details,
            "match_plans": {uid: plan.to_dict() for uid, plan in plans.items()},
        }


def optimize_mount(
    comparison: ManiSkillCameraComparison,
    initial: MountParameters,
    *,
    passes: int,
    fields: tuple[str, ...] = tuple(OPTIMIZATION_STEPS),
) -> tuple[MountParameters, dict[str, Any], list[dict[str, Any]]]:
    cache: dict[tuple[float, ...], dict[str, Any]] = {}
    history: list[dict[str, Any]] = []

    def evaluate(parameters: MountParameters) -> dict[str, Any] | None:
        key = (
            round(parameters.match_distance_mm, 6),
            round(parameters.camera_mount_image_up_mm, 6),
            round(parameters.pitch_trim_deg, 6),
            round(parameters.mirrored_lateral_mm, 6),
            round(parameters.mirrored_yaw_deg, 6),
            round(parameters.crop_offset_x_px, 6),
            round(parameters.crop_offset_y_px, 6),
        )
        if key in cache:
            return cache[key]
        try:
            report = comparison.evaluate(parameters)
        except ValueError:
            return None
        cache[key] = report
        return report

    current = initial
    best_report = evaluate(current)
    if best_report is None:
        raise ValueError("initial mount parameters are not crop-feasible")
    dimensions = tuple((field, OPTIMIZATION_STEPS[field]) for field in fields)
    for pass_index in range(passes):
        for field, initial_step in dimensions:
            step = initial_step / (2**pass_index)
            candidates: list[tuple[MountParameters, dict[str, Any]]] = []
            center = getattr(current, field)
            for multiplier in (-2, -1, 0, 1, 2):
                candidate = replace(current, **{field: center + multiplier * step})
                report = evaluate(candidate)
                if report is not None:
                    candidates.append((candidate, report))
            if not candidates:
                continue
            current, best_report = min(
                candidates,
                # Rounded score avoids drifting along numerically identical
                # crop-clamping plateaus; prefer the smallest explicit trim.
                key=lambda item: (
                    round(item[1]["aggregate"]["score"], 12),
                    abs(getattr(item[0], field)),
                ),
            )
            history.append(
                {
                    "pass": pass_index + 1,
                    "dimension": field,
                    "step": step,
                    "selected": asdict(current),
                    "score": best_report["aggregate"]["score"],
                }
            )
    return current, best_report, history


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render paired D405/D435i ManiSkill wrist views without a policy server"
    )
    parser.add_argument("--upstream-dir", type=Path, default=DEFAULT_UPSTREAM_DIR)
    parser.add_argument("--env-id", default="BimanualYAMPutEverythingInBox-v1")
    parser.add_argument("--reference-profile", default="d405-wrist-physical-nominal")
    parser.add_argument("--candidate-profile", default="d435i-wrist-cad-raw-nominal")
    parser.add_argument("--camera", action="append", choices=WRIST_CAMERAS, default=[])
    parser.add_argument("--seed", action="append", type=int, default=[])
    parser.add_argument("--shader-pack", default="minimal")
    parser.add_argument(
        "--reference-distance-mm",
        type=float,
        default=DEFAULT_REFERENCE_DISTANCE_MM,
    )
    parser.add_argument("--match-distance-mm", type=float, default=DEFAULT_MATCH_DISTANCE_MM)
    parser.add_argument(
        "--camera-mount-image-up-mm",
        type=float,
        default=DEFAULT_CAMERA_MOUNT_IMAGE_UP_MM,
    )
    parser.add_argument("--pitch-trim-deg", type=float, default=0.0)
    parser.add_argument("--mirrored-lateral-mm", type=float, default=0.0)
    parser.add_argument("--mirrored-yaw-deg", type=float, default=0.0)
    parser.add_argument("--crop-offset-x-px", type=float, default=0.0)
    parser.add_argument("--crop-offset-y-px", type=float, default=0.0)
    parser.add_argument("--optimize", action="store_true")
    parser.add_argument("--optimization-passes", type=int, default=2)
    parser.add_argument(
        "--optimize-field",
        action="append",
        choices=tuple(OPTIMIZATION_STEPS),
        default=[],
        help="parameter to search; repeat to lock all unspecified parameters",
    )
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "outputs/camera_compare")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.optimization_passes < 1:
        raise SystemExit("--optimization-passes must be positive")
    cameras = tuple(args.camera or WRIST_CAMERAS)
    seeds = tuple(args.seed or [42])
    parameters = MountParameters(
        match_distance_mm=args.match_distance_mm,
        camera_mount_image_up_mm=args.camera_mount_image_up_mm,
        pitch_trim_deg=args.pitch_trim_deg,
        mirrored_lateral_mm=args.mirrored_lateral_mm,
        mirrored_yaw_deg=args.mirrored_yaw_deg,
        crop_offset_x_px=args.crop_offset_x_px,
        crop_offset_y_px=args.crop_offset_y_px,
    )
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_dir.resolve() / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)

    reference_profile = load_profile(args.reference_profile)
    candidate_profile = load_profile(args.candidate_profile)
    comparison = ManiSkillCameraComparison(
        upstream_dir=args.upstream_dir.resolve(),
        env_id=args.env_id,
        reference_profile=reference_profile,
        candidate_profile=candidate_profile,
        cameras=cameras,
        seeds=seeds,
        shader_pack=args.shader_pack,
        reference_distance_mm=args.reference_distance_mm,
    )
    try:
        baseline = comparison.evaluate(parameters, save_dir=run_dir / "baseline")
        best_parameters = parameters
        best = baseline
        history: list[dict[str, Any]] = []
        if args.optimize:
            best_parameters, _, history = optimize_mount(
                comparison,
                parameters,
                passes=args.optimization_passes,
                fields=tuple(args.optimize_field or OPTIMIZATION_STEPS),
            )
            best = comparison.evaluate(best_parameters, save_dir=run_dir / "optimized")
    finally:
        comparison.close()

    cad_command = (
        "uv run --with cadquery --with trimesh --with manifold3d --with scipy "
        "--with networkx python cad/build_full_d435i_wrist_mount.py "
        "'/absolute/path/to/the-bracket.stl' "
        f"--match-distance-mm {best_parameters.match_distance_mm:.6f} "
        f"--camera-mount-image-up-mm {best_parameters.camera_mount_image_up_mm:.6f} "
        f"--camera-pitch-trim-deg {best_parameters.pitch_trim_deg:.6f} "
        f"--camera-lateral-mm {best_parameters.mirrored_lateral_mm:.6f} "
        f"--camera-yaw-deg {best_parameters.mirrored_yaw_deg:.6f}"
    )
    report = {
        "output_dir": str(run_dir),
        "env_id": args.env_id,
        "reference_profile": args.reference_profile,
        "candidate_profile": args.candidate_profile,
        "cameras": cameras,
        "seeds": seeds,
        "shader_pack": args.shader_pack,
        "baseline": baseline,
        "optimized": best if args.optimize else None,
        "optimization_history": history,
        "recommended_cad_command": cad_command,
        "notes": [
            "The primary score explicitly weights finger/task segmentation overlap; "
            "lower score and higher finger/task IoU are better.",
            "Global RGB metrics are retained as secondary diagnostics so background "
            "texture cannot hide manipulation-geometry errors.",
            f"Reference calibration: {reference_profile.calibration}. This remains a "
            "nominal profile unless loaded from measured device intrinsics.",
            "This compares ideal pinhole renders; real D405/D435i color, distortion, "
            "and exposure still differ.",
            "Regenerate CAD and rerun physical collision checks before adopting "
            "optimized dimensions.",
        ],
    }
    (run_dir / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
