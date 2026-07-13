set -e
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /opt/ros/humble/setup.bash
source /home/tuenguyen/ros2_forklift_warehouse_artifacts/install/setup.bash
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x rviz2 || true
LOG=/tmp/rear_sim_noise_check.log
: > "$LOG"
timeout 35s ros2 launch warehouse_visual_localization rear_steer_sim_only.launch.py gui:=false rviz:=false headless:=true >"$LOG" 2>&1 || true
python3 - <<'PY'
from pathlib import Path
patterns = ['Get noise index not valid', '[ERROR]', '[Err]', 'Traceback', 'Segmentation', 'NaN', 'nan', 'WARN']
for i, line in enumerate(Path('/tmp/rear_sim_noise_check.log').read_text(errors='ignore').splitlines(), 1):
    if any(p in line for p in patterns):
        print(f'{i}:{line}')
PY
