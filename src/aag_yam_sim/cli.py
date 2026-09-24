"""Command-line entry point for AAG × MolmoAct2 YAM evaluation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .benchmark import scenario_by_id, scenarios
from .camera_profiles import available_profiles, load_profile
from .contracts import normalize_server_url
from .paths import DEFAULT_UPSTREAM_DIR, REPO_ROOT
from .server import check_server
from .upstream import run_official_eval, validate_upstream


def _profile_summary(name: str) -> dict:
    profile = load_profile(name)
    return {
        "name": profile.name,
        "calibration": profile.calibration,
        "description": profile.description,
        "source": str(profile.source),
        "cameras": {
            key: {
                "model": camera.model,
                "resolution": [camera.width, camera.height],
                "mount": camera.mount,
                "intrinsic": camera.intrinsic.tolist(),
                "position_m": camera.position_m,
                "quaternion_wxyz": camera.quaternion_wxyz,
            }
            for key, camera in profile.cameras.items()
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aag-yam")
    commands = parser.add_subparsers(dest="command", required=True)

    doctor = commands.add_parser("doctor", help="validate upstream, camera, and server contracts")
    doctor.add_argument("--server-url", required=True)
    doctor.add_argument("--camera-profile", default="molmoact2-reference")
    doctor.add_argument("--upstream-dir", type=Path, default=DEFAULT_UPSTREAM_DIR)
    doctor.add_argument("--timeout", type=float, default=5.0)

    commands.add_parser("camera-profiles", help="list simulator camera profiles")

    show_profile = commands.add_parser("camera-profile", help="show a resolved camera profile")
    show_profile.add_argument("name")

    commands.add_parser("scenarios", help="list AAG benchmark scenarios")

    show_scenario = commands.add_parser("scenario", help="show one benchmark scenario")
    show_scenario.add_argument("scenario_id")

    evaluate = commands.add_parser("eval-official", help="run upstream ManiSkill YAM evaluation")
    evaluate.add_argument("--server-url", required=True)
    evaluate.add_argument("--camera-profile", default="molmoact2-reference")
    evaluate.add_argument("--upstream-dir", type=Path, default=DEFAULT_UPSTREAM_DIR)
    evaluate.add_argument(
        "--env-id", action="append", default=[], help="repeat for multiple upstream env IDs"
    )
    evaluate.add_argument("--instruction")
    evaluate.add_argument("--episodes", type=int, default=10)
    evaluate.add_argument("--max-episode-steps", type=int, default=800)
    evaluate.add_argument("--n-action-steps", type=int)
    evaluate.add_argument("--seed", type=int, default=42)
    evaluate.add_argument("--output-dir", type=Path, default=REPO_ROOT / "outputs")
    evaluate.add_argument("--no-video", action="store_true")
    return parser


def _print_json(value) -> None:
    print(json.dumps(value, indent=2))


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)

    if args.command == "camera-profiles":
        _print_json([_profile_summary(path.stem) for path in available_profiles()])
        return 0
    if args.command == "camera-profile":
        _print_json(_profile_summary(args.name))
        return 0
    if args.command == "scenarios":
        _print_json(
            [
                {
                    "id": item["id"],
                    "difficulty": item["difficulty"],
                    "scene": item["scene"],
                    "broad_instruction": item["broad_instruction"],
                    "status": item["status"],
                }
                for item in scenarios()
            ]
        )
        return 0
    if args.command == "scenario":
        try:
            _print_json(scenario_by_id(args.scenario_id))
        except KeyError as exc:
            raise SystemExit(str(exc)) from exc
        return 0
    if args.command == "doctor":
        report = {
            "camera_profile": _profile_summary(args.camera_profile),
            "upstream": {
                "path": str(args.upstream_dir.resolve()),
                "missing": validate_upstream(args.upstream_dir),
            },
        }
        try:
            check = check_server(args.server_url, timeout=args.timeout)
            report["server"] = {
                "url": check.url,
                "ok": check.ok,
                "errors": check.errors,
                "response": check.response,
            }
        except Exception as exc:  # diagnostics should return a complete report
            report["server"] = {"url": normalize_server_url(args.server_url), "error": str(exc)}
        report["ok"] = not report["upstream"]["missing"] and report["server"].get("ok", False)
        _print_json(report)
        return 0 if report["ok"] else 1
    if args.command == "eval-official":
        if args.episodes < 1 or args.max_episode_steps < 1:
            raise SystemExit("--episodes and --max-episode-steps must be positive")
        env_ids = args.env_id or ["BimanualYAMPutEverythingInBox-v1"]
        report = run_official_eval(
            upstream_dir=args.upstream_dir.resolve(),
            server_url=normalize_server_url(args.server_url),
            profile=load_profile(args.camera_profile),
            env_ids=env_ids,
            instruction=args.instruction,
            episodes=args.episodes,
            max_episode_steps=args.max_episode_steps,
            n_action_steps=args.n_action_steps,
            seed=args.seed,
            output_dir=args.output_dir.resolve(),
            save_video=not args.no_video,
        )
        _print_json(report)
        return 0
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
