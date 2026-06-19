import os
from glob import glob

from setuptools import find_packages, setup

# HH_260617: Install Python controllers as ROS 2 console scripts under lib/camrod_parking.
package_name = "camrod_parking"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob(os.path.join("config", "*.yaml"))),
        (f"share/{package_name}/launch", glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="hong",
    maintainer_email="hwanhong57@gmail.com",
    description="CAMROD parking and campsite maneuver controllers.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "site_maneuver_node = camrod_parking.site_maneuver_node:main",
            "drop_zone_parking_node = camrod_parking.drop_zone_parking_node:main",
        ],
    },
)
