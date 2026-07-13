#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/source_visual_localization_env.sh"

ros2 launch warehouse_visual_localization nav2_visual_localization.launch.py "$@"
