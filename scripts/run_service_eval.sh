#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

exec bash "${script_dir}/run_sim.sh" \
  --env-id AAGDiningTableCleanup-v1 \
  --env-id AAGKitchenDishSorting-v1 \
  --env-id AAGBedsideAssistance-v1 \
  --env-id AAGLivingRoomTidying-v1 \
  --env-id AAGCafeteriaServiceStation-v1 \
  "$@"
