#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/../.."

MPLBACKEND=Agg python3 tests/interactive/kinematics_dynamics_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/cbf_safety_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/controller_tracking_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/slip_friction_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/velocity_profile_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/localization_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/canonical_constraints_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/curvature_speed_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/corridor_footprint_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/rollover_safety_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/human_ssm_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/cbf_filter_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/docking_geometry_lab.py --smoke
MPLBACKEND=Agg python3 tests/interactive/visual_servo_lab.py --smoke
