"""Evaluable ManiSkill environments for the five AAG service scenes."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import sapien
import sapien.physx as physx
import torch
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.building import actors
from mani_skill.utils.registration import register_env
from mani_skill.utils.scene_builder.table import TableSceneBuilder
from mani_skill.utils.structs.pose import Pose
from sim_eval.robots.bimanual_yam import BimanualYAM

from .service_scene_spec import service_scene_layout


def _z_rotation(degrees: float) -> list[float]:
    angle = math.radians(degrees) / 2.0
    return [math.cos(angle), 0.0, 0.0, math.sin(angle)]


def _vertical_cylinder_pose(z: float = 0.0) -> sapien.Pose:
    # SAPIEN cylinders use local +X as their long axis; rotate it onto +Z.
    return sapien.Pose(p=[0.0, 0.0, z], q=[math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0])


def _render_material(color: list[float]) -> sapien.render.RenderMaterial:
    return sapien.render.RenderMaterial(base_color=color)


def _scale_mesh_builder(builder: sapien.ActorBuilder, factor: float) -> None:
    """Uniformly scale mesh-backed visual and collision records before build."""
    for record in builder.visual_records:
        record.scale = np.asarray(record.scale, dtype=np.float32) * factor
    for record in builder.collision_records:
        if record.type in {"convex_mesh", "nonconvex_mesh", "multiple_convex_meshes"}:
            record.scale = np.asarray(record.scale, dtype=np.float32) * factor


def _add_box(
    builder: sapien.ActorBuilder,
    *,
    half_size: list[float],
    color: list[float],
) -> None:
    builder.add_box_visual(half_size=half_size, material=_render_material(color))
    builder.add_box_collision(half_size=half_size)


def _add_disc(
    builder: sapien.ActorBuilder,
    *,
    radius: float,
    half_thickness: float,
    color: list[float],
) -> None:
    pose = _vertical_cylinder_pose()
    builder.add_cylinder_visual(
        pose=pose,
        radius=radius,
        half_length=half_thickness,
        material=_render_material(color),
    )
    builder.add_cylinder_collision(pose=pose, radius=radius, half_length=half_thickness)


def _add_bottle(
    builder: sapien.ActorBuilder,
    *,
    body_radius: float,
    body_height: float,
    color: list[float],
    cap_color: list[float],
) -> None:
    body_half = body_height / 2.0
    body_pose = _vertical_cylinder_pose(body_half)
    builder.add_cylinder_visual(
        pose=body_pose,
        radius=body_radius,
        half_length=body_half,
        material=_render_material(color),
    )
    builder.add_cylinder_collision(
        pose=body_pose,
        radius=body_radius,
        half_length=body_half,
    )
    cap_half = 0.009
    cap_pose = _vertical_cylinder_pose(body_height + cap_half)
    builder.add_cylinder_visual(
        pose=cap_pose,
        radius=body_radius * 0.58,
        half_length=cap_half,
        material=_render_material(cap_color),
    )
    builder.add_cylinder_collision(
        pose=cap_pose,
        radius=body_radius * 0.58,
        half_length=cap_half,
    )


def _add_open_container(
    builder: sapien.ActorBuilder,
    *,
    inner_half: list[float],
    height: float,
    wall: float,
    color: list[float],
    rack: bool = False,
) -> None:
    inner_x, inner_y = inner_half
    outer_x, outer_y = inner_x + wall, inner_y + wall
    material = _render_material(color)
    floor_pose = sapien.Pose(p=[0.0, 0.0, wall / 2.0])
    floor_half = [outer_x, outer_y, wall / 2.0]
    builder.add_box_visual(pose=floor_pose, half_size=floor_half, material=material)
    builder.add_box_collision(pose=floor_pose, half_size=floor_half)

    wall_z = wall + height / 2.0
    for sign in (-1.0, 1.0):
        x_pose = sapien.Pose(p=[sign * (inner_x + wall / 2.0), 0.0, wall_z])
        x_half = [wall / 2.0, outer_y, height / 2.0]
        builder.add_box_visual(pose=x_pose, half_size=x_half, material=material)
        builder.add_box_collision(pose=x_pose, half_size=x_half)

        y_pose = sapien.Pose(p=[0.0, sign * (inner_y + wall / 2.0), wall_z])
        y_half = [inner_x, wall / 2.0, height / 2.0]
        builder.add_box_visual(pose=y_pose, half_size=y_half, material=material)
        builder.add_box_collision(pose=y_pose, half_size=y_half)

    if rack:
        divider_height = max(0.025, height * 0.7)
        for offset in (-inner_y / 2.0, 0.0, inner_y / 2.0):
            pose = sapien.Pose(p=[0.0, offset, wall + divider_height / 2.0])
            half = [inner_x * 0.82, wall / 2.0, divider_height / 2.0]
            builder.add_box_visual(pose=pose, half_size=half, material=material)
            builder.add_box_collision(pose=pose, half_size=half)


class _AAGServiceSceneEnv(BaseEnv):
    """Shared YAM table, declarative props, and ordered subtask scoring."""

    SUPPORTED_ROBOTS = ["bimanual_yam"]
    agent: BimanualYAM
    SCENE_ID = ""

    def __init__(self, *args, **kwargs):
        self.layout = service_scene_layout(self.SCENE_ID)
        self.dynamic_objects: list[tuple[Any, dict[str, Any]]] = []
        self.actors: dict[str, Any] = {}
        self.object_bounds: dict[str, np.ndarray] = {}
        self.predicates = [
            predicate for stage in self.layout["stages"] for predicate in stage["predicates"]
        ]
        self.stage_predicate_indices: list[list[int]] = []
        cursor = 0
        for stage in self.layout["stages"]:
            indices = list(range(cursor, cursor + len(stage["predicates"])))
            self.stage_predicate_indices.append(indices)
            cursor += len(indices)
        super().__init__(
            *args,
            robot_uids="bimanual_yam",
            reconfiguration_freq=1,
            **kwargs,
        )

    @property
    def _default_sensor_configs(self):
        pose = sapien_utils.look_at(eye=[0.15, 0.0, 0.65], target=[-0.30, 0.0, 0.04])
        return [CameraConfig("base_camera", pose, 640, 480, np.pi / 2, 0.01, 100)]

    @property
    def _default_human_render_camera_configs(self):
        pose = sapien_utils.look_at(eye=[0.35, 0.48, 0.72], target=[-0.30, 0.0, 0.04])
        return CameraConfig(
            "render_camera",
            pose=pose,
            width=1280,
            height=720,
            fov=1.10,
            near=0.01,
            far=100,
            shader_pack="minimal",
        )

    def _build_procedural(self, spec: dict[str, Any]):
        builder = self.scene.create_actor_builder()
        kind = spec["kind"]
        if kind == "box":
            _add_box(
                builder,
                half_size=spec["half_size"],
                color=spec["color"],
            )
        elif kind == "disc":
            _add_disc(
                builder,
                radius=float(spec["radius"]),
                half_thickness=float(spec["half_thickness"]),
                color=spec["color"],
            )
        elif kind == "bottle":
            _add_bottle(
                builder,
                body_radius=float(spec["body_radius"]),
                body_height=float(spec["body_height"]),
                color=spec["color"],
                cap_color=spec["cap_color"],
            )
        elif kind in {"open_container", "rack"}:
            _add_open_container(
                builder,
                inner_half=spec["inner_half"],
                height=float(spec["height"]),
                wall=float(spec["wall"]),
                color=spec["color"],
                rack=kind == "rack",
            )
        else:  # pragma: no cover - validated by dependency-light tests
            raise ValueError(f"unknown procedural object kind {kind!r}")
        return builder

    def _load_scene(self, options: dict):
        self.table_scene = TableSceneBuilder(self, robot_init_qpos_noise=0.0)
        self.table_scene.build()

        self.dynamic_objects = []
        self.actors = {}
        for spec in self.layout["objects"]:
            if spec["kind"] == "ycb":
                builder = actors.get_actor_builder(self.scene, id=f"ycb:{spec['asset_id']}")
                _scale_mesh_builder(builder, float(spec.get("scale", 1.0)))
            else:
                builder = self._build_procedural(spec)

            x, y = spec["position_xy"]
            is_static = bool(spec.get("static", False))
            static_z = float(spec.get("initial_z", 0.0))
            if is_static and "initial_z" not in spec:
                if spec["kind"] == "box":
                    static_z = float(spec["half_size"][2])
                elif spec["kind"] == "disc":
                    static_z = float(spec["half_thickness"])
            builder.initial_pose = sapien.Pose(
                # Keep dynamic actors at z=0 during mesh-bound inspection;
                # _after_reconfigure then recovers the local bottom offset.
                p=[x, y, static_z if is_static else 0.0],
                q=_z_rotation(float(spec.get("yaw_deg", 0.0))),
            )
            actor = (
                builder.build_static(name=spec["name"])
                if is_static
                else builder.build(name=spec["name"])
            )
            self.actors[spec["name"]] = actor
            if not is_static:
                for body in actor._bodies:
                    for collision_shape in body.get_collision_shapes():
                        collision_shape.physical_material = physx.PhysxMaterial(
                            static_friction=1.0,
                            dynamic_friction=1.0,
                            restitution=0.0,
                        )
                self.dynamic_objects.append((actor, spec))

    def _after_reconfigure(self, options: dict):
        self.object_bounds = {}
        for actor, spec in self.dynamic_objects:
            collision_mesh = actor.get_first_collision_mesh(to_world_frame=False)
            if collision_mesh is None:
                raise RuntimeError(f"object {spec['name']!r} has no collision mesh")
            self.object_bounds[spec["name"]] = collision_mesh.bounding_box.bounds.copy()

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        with torch.device(self.device):
            batch_size = len(env_idx)
            self.table_scene.initialize(env_idx)
            self.agent.robot.set_pose(sapien.Pose(p=[-0.65, 0.0, 0.01]))
            self.agent.robot.set_qpos(self.agent.keyframes["home"].qpos)

            for actor, spec in self.dynamic_objects:
                x, y = spec["position_xy"]
                spawn_noise = float(spec.get("spawn_noise_xy", 0.012))
                xy_noise = (torch.rand((batch_size, 2)) * 2.0 - 1.0) * spawn_noise
                bottom_offset = -float(self.object_bounds[spec["name"]][0, 2])
                xyz = torch.tensor([x, y, bottom_offset], dtype=torch.float32).repeat(batch_size, 1)
                xyz[:, :2] += xy_noise
                yaw_noise = (torch.rand((batch_size,)) * 2.0 - 1.0) * float(
                    spec.get("spawn_yaw_noise_deg", 8.0)
                )
                yaw = torch.deg2rad(yaw_noise + float(spec.get("yaw_deg", 0.0))) / 2.0
                quaternion = torch.zeros((batch_size, 4), dtype=torch.float32)
                quaternion[:, 0] = torch.cos(yaw)
                quaternion[:, 3] = torch.sin(yaw)
                actor.set_pose(Pose.create_from_pq(xyz, quaternion))

            total = len(self.predicates)
            self._subtask_latched = torch.zeros(
                (self.num_envs, total), dtype=torch.bool, device=self.device
            )
            self._subtask_first_step = torch.full(
                (self.num_envs, total), -1, dtype=torch.int32, device=self.device
            )

    def _inside_container(self, object_name: str, target_name: str, *, on: bool) -> torch.Tensor:
        obj = self.actors[object_name]
        target = self.actors[target_name]
        target_spec = next(item for item in self.layout["objects"] if item["name"] == target_name)
        delta = obj.pose.p[:, :2] - target.pose.p[:, :2]
        margin = 0.012
        inner_x, inner_y = (float(value) for value in target_spec["inner_half"])
        inside_xy = (torch.abs(delta[:, 0]) < inner_x - margin) & (
            torch.abs(delta[:, 1]) < inner_y - margin
        )
        floor_z = target.pose.p[:, 2] + float(target_spec["wall"])
        vertical_limit = 0.10 if on else float(target_spec["height"]) + 0.07
        z_ok = (obj.pose.p[:, 2] > floor_z - 0.03) & (obj.pose.p[:, 2] < floor_z + vertical_limit)
        return inside_xy & z_ok

    def _beside(self, predicate: dict[str, Any]) -> torch.Tensor:
        obj = self.actors[predicate["object"]]
        target = self.actors[predicate["target"]]
        distance = torch.linalg.norm(obj.pose.p[:, :2] - target.pose.p[:, :2], dim=1)
        result = (distance >= float(predicate["min_distance"])) & (
            distance <= float(predicate["max_distance"])
        )
        object_bottom = obj.pose.p[:, 2] + float(self.object_bounds[predicate["object"]][0, 2])
        support_name = predicate.get("support")
        if support_name:
            support = self.actors[support_name]
            support_spec = next(
                item for item in self.layout["objects"] if item["name"] == support_name
            )
            inner = torch.tensor(
                support_spec["inner_half"], dtype=torch.float32, device=self.device
            )
            for actor in (obj, target):
                delta = torch.abs(actor.pose.p[:, :2] - support.pose.p[:, :2])
                result &= (delta < inner - 0.012).all(dim=1)
            support_floor = support.pose.p[:, 2] + float(support_spec["wall"])
            result &= torch.abs(object_bottom - support_floor) < 0.04
        else:
            result &= torch.abs(object_bottom) < 0.04
        return result

    def _predicate_value(self, predicate: dict[str, Any]) -> torch.Tensor:
        relation = predicate["relation"]
        if relation == "in":
            return self._inside_container(predicate["object"], predicate["target"], on=False)
        if relation == "on":
            return self._inside_container(predicate["object"], predicate["target"], on=True)
        if relation == "beside":
            return self._beside(predicate)
        raise ValueError(f"unknown task relation {relation!r}")

    def evaluate(self):
        current = torch.stack(
            [self._predicate_value(predicate) for predicate in self.predicates], dim=1
        )
        previous = self._subtask_latched.clone()
        prior_stages_complete = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        for indices in self.stage_predicate_indices:
            stage_indices = torch.tensor(indices, dtype=torch.long, device=self.device)
            enabled = prior_stages_complete
            self._subtask_latched[:, stage_indices] |= current[:, stage_indices] & enabled[:, None]
            prior_stages_complete &= previous[:, stage_indices].all(dim=1)

        newly_completed = self._subtask_latched & ~previous
        elapsed = self._elapsed_steps[:, None].expand_as(self._subtask_first_step)
        self._subtask_first_step = torch.where(
            newly_completed & (self._subtask_first_step < 0),
            elapsed,
            self._subtask_first_step,
        )
        completed = self._subtask_latched.float().sum(dim=1)
        total = len(self.predicates)
        score = completed / float(total)
        final_state_complete = current.all(dim=1)
        ordered_complete = self._subtask_latched.all(dim=1)
        info: dict[str, Any] = {
            "success": final_state_complete & ordered_complete,
            "partial_task_score": score,
            "completed_subtasks": completed,
            "total_subtasks": torch.full_like(completed, total),
            "final_state_complete": final_state_complete,
        }
        for index, predicate in enumerate(self.predicates):
            predicate_id = predicate["id"]
            info[f"subtask_complete/{predicate_id}"] = self._subtask_latched[:, index]
            info[f"subtask_current/{predicate_id}"] = current[:, index]
            info[f"subtask_first_step/{predicate_id}"] = self._subtask_first_step[:, index]
        return info

    def compute_dense_reward(self, obs, action, info):
        return info["partial_task_score"]

    def compute_normalized_dense_reward(self, obs, action, info):
        return self.compute_dense_reward(obs, action, info)


@register_env("AAGDiningTableCleanupPreview-v0", max_episode_steps=1)
class AAGDiningTableCleanupPreviewEnv(_AAGServiceSceneEnv):
    SCENE_ID = "dining-table-cleanup"


@register_env("AAGKitchenDishSortingPreview-v0", max_episode_steps=1)
class AAGKitchenDishSortingPreviewEnv(_AAGServiceSceneEnv):
    SCENE_ID = "kitchen-dish-sorting"


@register_env("AAGBedsideAssistancePreview-v0", max_episode_steps=1)
class AAGBedsideAssistancePreviewEnv(_AAGServiceSceneEnv):
    SCENE_ID = "bedside-assistance"


@register_env("AAGLivingRoomTidyingPreview-v0", max_episode_steps=1)
class AAGLivingRoomTidyingPreviewEnv(_AAGServiceSceneEnv):
    SCENE_ID = "living-room-tidying"


@register_env("AAGCafeteriaServiceStationPreview-v0", max_episode_steps=1)
class AAGCafeteriaServiceStationPreviewEnv(_AAGServiceSceneEnv):
    SCENE_ID = "cafeteria-service-station"


@register_env("AAGDiningTableCleanup-v1", max_episode_steps=4500)
class AAGDiningTableCleanupEnv(_AAGServiceSceneEnv):
    SCENE_ID = "dining-table-cleanup"


@register_env("AAGKitchenDishSorting-v1", max_episode_steps=4500)
class AAGKitchenDishSortingEnv(_AAGServiceSceneEnv):
    SCENE_ID = "kitchen-dish-sorting"


@register_env("AAGBedsideAssistance-v1", max_episode_steps=4500)
class AAGBedsideAssistanceEnv(_AAGServiceSceneEnv):
    SCENE_ID = "bedside-assistance"


@register_env("AAGLivingRoomTidying-v1", max_episode_steps=4500)
class AAGLivingRoomTidyingEnv(_AAGServiceSceneEnv):
    SCENE_ID = "living-room-tidying"


@register_env("AAGCafeteriaServiceStation-v1", max_episode_steps=4500)
class AAGCafeteriaServiceStationEnv(_AAGServiceSceneEnv):
    SCENE_ID = "cafeteria-service-station"
