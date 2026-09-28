#!/usr/bin/env python3
"""Print exact 640x360 color intrinsics for connected RealSense cameras."""

from __future__ import annotations

import argparse
import json


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", help="query one serial; default queries every device")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit("pyrealsense2 is required in the robot camera environment") from exc

    context = rs.context()
    serials = [device.get_info(rs.camera_info.serial_number) for device in context.query_devices()]
    if not serials:
        raise SystemExit("No RealSense devices found")
    if args.serial:
        if args.serial not in serials:
            raise SystemExit(f"serial {args.serial!r} not found; connected={serials}")
        serials = [args.serial]

    output = []
    for serial in serials:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
        profile = pipeline.start(config)
        try:
            stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = stream.get_intrinsics()
            device = profile.get_device()
            output.append(
                {
                    "name": device.get_info(rs.camera_info.name),
                    "serial": serial,
                    "stream": {"width": args.width, "height": args.height, "fps": args.fps},
                    "intrinsic": {
                        "fx": intr.fx,
                        "fy": intr.fy,
                        "cx": intr.ppx,
                        "cy": intr.ppy,
                        "distortion_model": str(intr.model),
                        "coefficients": list(intr.coeffs),
                    },
                }
            )
        finally:
            pipeline.stop()

    print(json.dumps(output, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
