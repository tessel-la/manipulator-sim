from glob import glob
from setuptools import find_packages, setup

package_name = "manipulator_actions"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config/sequences", glob("config/sequences/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Riccardo",
    maintainer_email="riccardo@example.com",
    description="Reusable ROS2 action server and CLI for MoveIt Servo manipulator control.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "action_server = manipulator_actions.action_server:main",
            "manipulator_cli = manipulator_actions.cli:main",
        ],
    },
)
