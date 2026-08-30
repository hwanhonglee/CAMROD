"""Setup for the CAMROD-to-CARLA adapter package."""

from glob import glob
import os

from setuptools import setup


PACKAGE_NAME = "camrod_carla_adapter"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    package_dir={"": "src"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "config"),
            glob("config/*.yaml")
            + glob("config/*.json")
            + glob("config/*.xml")
            + glob("config/*.osm"),
        ),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="hwanhonglee",
    maintainer_email="hwanhong57@gmail.com",
    description=(
        "Fail-closed CAMROD Twist and CARLA 4WS/feedback integration"
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "twist_to_4ws = "
            "camrod_carla_adapter.twist_to_4ws_node:main",
            "carla_feedback_bridge = "
            "camrod_carla_adapter.feedback_bridge_node:main",
            "carla_sensor_relay = "
            "camrod_carla_adapter.sensor_relay_node:main",
            "carla_lidar_filter = "
            "camrod_carla_adapter.lidar_filter_node:main",
            "carla_radar_relay = "
            "camrod_carla_adapter.radar_relay_node:main",
            "carla_sensor_source_audit = "
            "camrod_carla_adapter.sensor_source_audit_node:main",
            "carla_platform_heartbeat = "
            "camrod_carla_adapter.carla_platform_heartbeat_node:main",
            "carla_charging_contact_emulator = "
            "camrod_carla_adapter.charging_contact_emulator_node:main",
            "carla_step_pacer = "
            "camrod_carla_adapter.carla_step_pacer_node:main",
        ],
    },
)
