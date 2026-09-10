import os
from glob import glob

from setuptools import find_packages, setup


package_name = "camrod_ui_tester"


def files_under(source: str, destination: str):
    entries = []
    for root, _, files in os.walk(source):
        if not files:
            continue
        relative = os.path.relpath(root, source)
        target = destination if relative == "." else os.path.join(destination, relative)
        entries.append((target, [os.path.join(root, name) for name in files]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        *files_under(
            "assets/control_panel",
            f"share/{package_name}/assets/control_panel",
        ),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn[standard]", "PyYAML"],
    zip_safe=True,
    maintainer="CAMROD",
    maintainer_email="hwanhong57@gmail.com",
    description="Interactive ROS 2 runtime simulator for standalone CAMROD UI testing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ui_simulator = camrod_ui_tester.simulator_node:main",
            "ranger_parameter_stub = camrod_ui_tester.ranger_parameter_stub:main",
        ],
    },
)
