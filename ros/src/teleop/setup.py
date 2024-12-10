from setuptools import find_packages, setup
from glob import glob
import os

package_name = "teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            ["teleop_config.yaml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            ["teleop_launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eduardo-barreto",
    maintainer_email="eduardopontobarreto@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop = src.main:main",
        ],
    },
)
