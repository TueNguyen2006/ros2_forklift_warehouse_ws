from pathlib import Path
from setuptools import setup


package_name = "warehouse_visual_localization"


def collect_data_files():
    data_files = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ]

    for folder in ["launch", "config", "maps"]:
        for path in Path(folder).rglob("*"):
            if path.is_file():
                if "__pycache__" in path.parts or path.suffix == ".pyc":
                    continue
                destination = f"share/{package_name}/{path.parent.as_posix()}"
                data_files.append((destination, [str(path)]))

    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=collect_data_files(),
    install_requires=["setuptools"],
    scripts=[
        "scripts/tf_debug_node.py",
        "scripts/odom_evaluator.py",
        "scripts/wait_for_map_tf.py",
        "scripts/startup_motion_probe.py",
        "scripts/pose_source_monitor.py",
        "scripts/gazebo_goal_bridge.py",
        "scripts/scan_retimestamp.py",
        "scripts/twist_watchdog.py",
        "scripts/planar_motion_guard.py",
        "scripts/visual_route_runner.py",
    ],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Visual odometry and localization bringup for warehouse navigation.",
    license="Apache-2.0",
)
