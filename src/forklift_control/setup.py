from pathlib import Path
from setuptools import find_packages
from setuptools import setup


package_name = "forklift_control"


def collect_data_files():
    data_files = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ]

    for folder in ["config", "launch"]:
        for path in Path(folder).rglob("*"):
            if path.is_file():
                destination = f"share/{package_name}/{path.parent.as_posix()}"
                data_files.append((destination, [str(path)]))

    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=collect_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="Ackermann/rear-steer command adapters and wheel odometry for forklift simulation.",
    license="Apache-2.0",
    test_suite="test",
    entry_points={
        "console_scripts": [
            "ackermann_command_adapter = forklift_control.ackermann_command_adapter:main",
            "wheel_odometry = forklift_control.wheel_odometry:main",
            "ground_truth_odom = forklift_control.ground_truth_odom:main",
        ],
    },
)
