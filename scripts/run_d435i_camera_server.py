#!/usr/bin/env python3
"""Run the existing YAM ZMQ camera server with D435i wrist-view matching.

This script imports the camera implementation and wire protocol from the pinned
YAM checkout.  Only the configured wrist RGB arrays are crop/resized; camera
names, timestamps, and the REP/PUB protocol remain unchanged.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
from pathlib import Path
from typing import Any

from aag_yam_sim.wrist_match import (
    PinholeIntrinsics,
    apply_match_plan,
    compute_match_plan,
    rotate_intrinsics_180,
)

LOGGER = logging.getLogger("d435i_yam_camera_server")


class MatchedCamera:
    """CameraDriver-compatible RGB transform around the existing RealSense reader."""

    def __init__(self, camera: Any, plan: Any, *, rotate_180: bool) -> None:
        self._camera = camera
        self._plan = plan
        self._rotate_180 = rotate_180

    @property
    def _latest_frame_timestamp(self):
        return self._camera._latest_frame_timestamp

    @property
    def _stop_event(self):
        return self._camera._stop_event

    def read(self):
        image, depth = self._camera.read()
        image = apply_match_plan(
            image,
            self._plan,
            rotate_180=self._rotate_180,
            backend="opencv",
        )
        return image, depth


def _source_intrinsics(camera: Any) -> PinholeIntrinsics:
    rs = camera._rs
    profile = camera._pipeline.get_active_profile()
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream.get_intrinsics()
    return PinholeIntrinsics(stream.width(), stream.height(), intr.fx, intr.fy, intr.ppx, intr.ppy)


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--yam-config", type=Path, required=True)
    parser.add_argument(
        "--match-config",
        type=Path,
        default=repo_root / "configs/hardware/d435i-wrist-match.example.json",
    )
    parser.add_argument(
        "--gello-root",
        type=Path,
        default=repo_root / "third_party/molmoact2/YAM/gello_software",
    )
    parser.add_argument("--rep-endpoint", default="tcp://127.0.0.1:5555")
    parser.add_argument("--pub-endpoint", default="tcp://127.0.0.1:5556")
    parser.add_argument("--pub-period-sec", type=float, default=1.0 / 30.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    sys.path.insert(0, str(args.gello_root.resolve()))
    try:
        from gello.cameras.camera_server import CameraServer
        from gello.cameras.realsense_camera import RealSenseCamera
        from omegaconf import OmegaConf
    except ImportError as exc:
        raise SystemExit(
            "Run this in the existing YAM robot environment with pyrealsense2, "
            "opencv, pyzmq, and omegaconf installed"
        ) from exc

    match = json.loads(args.match_config.read_text(encoding="utf-8"))
    ref = match["reference"]
    reference = PinholeIntrinsics(
        int(ref["width"]),
        int(ref["height"]),
        float(ref["fx"]),
        float(ref["fy"]),
        float(ref["cx"]),
        float(ref["cy"]),
    )
    reference_distance = float(ref["working_distance_mm"])
    actual_distance = float(match["actual_working_distance_mm"])

    yam = OmegaConf.to_container(OmegaConf.load(args.yam_config), resolve=True)
    camera_specs = yam["sensors"]["cameras"]
    cameras = {}
    for name, spec in camera_specs.items():
        serial = str(spec["device_id"])
        LOGGER.info("Opening %s (%s)", name, serial)
        raw = RealSenseCamera(serial)
        wrist_spec = match["wrist_cameras"].get(name, {})
        if wrist_spec.get("enabled", False):
            source = _source_intrinsics(raw)
            rotate_180 = bool(wrist_spec.get("rotate_180", False))
            if rotate_180:
                source = rotate_intrinsics_180(source)
            plan = compute_match_plan(
                reference,
                source,
                reference_distance_mm=reference_distance,
                actual_distance_mm=float(
                    wrist_spec.get("actual_working_distance_mm", actual_distance)
                ),
            )
            LOGGER.info(
                "%s D405 match: source fx/fy=%.3f/%.3f, crop=(%d,%d %dx%d), "
                "distance=%.2f mm",
                name,
                source.fx,
                source.fy,
                plan.crop_left,
                plan.crop_top,
                plan.crop_width,
                plan.crop_height,
                plan.actual_distance_mm,
            )
            cameras[name] = MatchedCamera(
                raw,
                plan,
                rotate_180=rotate_180,
            )
        else:
            cameras[name] = raw

    server = CameraServer(
        cameras=cameras,
        rep_endpoint=args.rep_endpoint,
        pub_endpoint=(args.pub_endpoint or None),
        pub_period_sec=args.pub_period_sec,
    )

    def stop(signum, _frame):
        LOGGER.info("Signal %d received", signum)
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    server.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
