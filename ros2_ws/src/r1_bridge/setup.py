from glob import glob

from setuptools import setup

package_name = "r1_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aisl",
    maintainer_email="kms62197724@gmail.com",
    description="R1 LowState (unitree_hg) -> JointState / Imu / TF for RViz",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "lowstate_bridge = r1_bridge.lowstate_bridge:main",
        ],
    },
)
