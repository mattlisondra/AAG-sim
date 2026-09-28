#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
upstream_dir="${repo_root}/third_party/molmoact2"

if [[ ! -f "${upstream_dir}/sim_eval/robots/bimanual_yam.py" ]]; then
  echo "MolmoAct2 submodule is missing. Run:" >&2
  echo "  GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive" >&2
  exit 2
fi

if [[ ! -f "${HOME}/.maniskill/data/assets/mani_skill2_ycb/info_pick_v0.json" ]]; then
  echo "ManiSkill YCB assets are missing. Install them once with:" >&2
  echo "  uv run --project third_party/molmoact2 python -m mani_skill.utils.download_asset ycb -y" >&2
  exit 2
fi

export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec uv run \
  --project "${upstream_dir}" \
  python -m aag_yam_sim.sim_camera_compare \
  --upstream-dir "${upstream_dir}" \
  "$@"
