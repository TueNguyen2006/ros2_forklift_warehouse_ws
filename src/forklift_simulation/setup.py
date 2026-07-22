from pathlib import Path
from setuptools import setup


package_name = "forklift_simulation"


def collect_data_files():
    data_files = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ]

    for folder in ["launch", "worlds", "models"]:
        for path in Path(folder).rglob("*"):
            if path.is_file():
                destination = f"share/{package_name}/{path.parent.as_posix()}"
                data_files.append((destination, [str(path)]))

    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=collect_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Gazebo Classic simulation assets and launch helpers for forklift navigation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ensure_world_map = forklift_simulation.world_map_generator:main",
        ],
    },
)
