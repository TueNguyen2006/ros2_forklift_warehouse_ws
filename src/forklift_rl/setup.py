from pathlib import Path
from setuptools import find_packages
from setuptools import setup


package_name = "forklift_rl"


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
    description="Gymnasium-compatible RL environment scaffolding for forklift physics simulation.",
    license="Apache-2.0",
    test_suite="test",
)
