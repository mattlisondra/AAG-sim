"""Repository resource paths."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CAMERA_CONFIG_DIR = REPO_ROOT / "configs" / "cameras"
BENCHMARK_PATH = REPO_ROOT / "configs" / "benchmarks" / "aag_service_scenes.json"
DEFAULT_UPSTREAM_DIR = REPO_ROOT / "third_party" / "molmoact2"
