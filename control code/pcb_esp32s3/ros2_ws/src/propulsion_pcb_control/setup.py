from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = "propulsion_pcb_control"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tomatol",
    maintainer_email="tomatol@localhost",
    description="ROS 2 joystick-to-serial controller for the catheter propulsion module.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "propulsion_node = propulsion_pcb_control.propulsion_node:main",
            "simulated_joy_node = propulsion_pcb_control.simulated_joy_node:main",
        ],
    },
)
