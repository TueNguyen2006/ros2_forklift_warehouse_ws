#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

python3 kinematics_dynamics_demo.py "$@"
python3 slam_localization_demo.py "$@"
python3 slip_friction_demo.py "$@"
python3 velocity_profile_demo.py "$@"
python3 cbf_safety_demo.py "$@"
python3 controller_tracking_demo.py "$@"

