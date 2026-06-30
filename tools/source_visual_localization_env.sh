#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${HOME}/ros2_forklift_warehouse_artifacts}"
INSTALL_BASE="${INSTALL_BASE:-${ARTIFACT_ROOT}/install}"
LOCAL_RTABMAP_PREFIX="${LOCAL_RTABMAP_PREFIX:-${HOME}/ros2_local_overlay/opt/ros/humble}"

# ROS / colcon setup scripts are not nounset-safe, so temporarily relax `set -u`.
_visual_env_restore_nounset=0
if [[ $- == *u* ]]; then
  _visual_env_restore_nounset=1
  set +u
fi

source /opt/ros/humble/setup.bash
source "${INSTALL_BASE}/local_setup.bash"

if [[ ${_visual_env_restore_nounset} -eq 1 ]]; then
  set -u
fi
unset _visual_env_restore_nounset

if [[ -d "${LOCAL_RTABMAP_PREFIX}/share/ament_index/resource_index/packages" ]]; then
  export AMENT_PREFIX_PATH="${LOCAL_RTABMAP_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${LOCAL_RTABMAP_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export COLCON_PREFIX_PATH="${LOCAL_RTABMAP_PREFIX}:${COLCON_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${LOCAL_RTABMAP_PREFIX}/lib:${LOCAL_RTABMAP_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PATH="${LOCAL_RTABMAP_PREFIX}/bin:${PATH}"
  export PYTHONPATH="${LOCAL_RTABMAP_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi

export VISUAL_LOCALIZATION_WORKSPACE_DIR="${WORKSPACE_DIR}"

if [[ -z "${DISPLAY:-}" && -S /mnt/wslg/.X11-unix/X0 ]]; then
  export DISPLAY=":0"
fi

if [[ -z "${WAYLAND_DISPLAY:-}" && -S /mnt/wslg/runtime-dir/wayland-0 ]]; then
  export WAYLAND_DISPLAY="wayland-0"
fi

if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
  if [[ -d "/mnt/wslg/run/user/$(id -u)" ]]; then
    export XDG_RUNTIME_DIR="/mnt/wslg/run/user/$(id -u)"
  elif [[ -d /mnt/wslg/runtime-dir ]]; then
    export XDG_RUNTIME_DIR="/mnt/wslg/runtime-dir"
  fi
fi

export VISUAL_LOCALIZATION_FORCE_SOFTWARE_RENDERING="${VISUAL_LOCALIZATION_FORCE_SOFTWARE_RENDERING:-false}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"
export QT_OPENGL="${QT_OPENGL:-desktop}"
export OGRE_RTT_MODE="${OGRE_RTT_MODE:-Copy}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

if [[ ! -e /mnt/shared_memory && -d /mnt/wslg ]]; then
  cat >&2 <<'EOF'
[visual_localization_env] WARNING: /mnt/shared_memory is missing.
[visual_localization_env] WSLg will likely fall back to COPY MODE (blank/frozen Gazebo or broken thumbnails).
[visual_localization_env] This is an environment/runtime issue, not a ROS launch issue.
[visual_localization_env] Recommended fix from Windows PowerShell:
[visual_localization_env]   wsl --shutdown
[visual_localization_env] Then reopen Ubuntu-22.04 and rerun this script.
EOF
fi
