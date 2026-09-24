#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
upstream_dir="${repo_root}/third_party/molmoact2"

if [[ ! -f "${upstream_dir}/sim_eval/run_eval.py" ]]; then
  echo "MolmoAct2 submodule is missing. Run:" >&2
  echo "  GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive" >&2
  exit 2
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 2
fi

# Use the upstream environment because it owns ManiSkill, SAPIEN, Torch, and
# the exact dependency versions expected by sim_eval. Overlay this bridge as an
# editable package so the aag-yam command is available in that same process.
exec uv run \
  --project "${upstream_dir}" \
  --with-editable "${repo_root}" \
  aag-yam eval-official \
  --upstream-dir "${upstream_dir}" \
  "$@"
