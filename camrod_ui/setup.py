import os
# HJ_260601: Added ui_guest_node entry point and guest_frontend assets.
from setuptools import find_packages, setup

package_name = "camrod_ui"


def _collect_data_files(src_dir: str, dst_base: str):
    collected = []
    if not os.path.isdir(src_dir):
        return collected
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        rel_root = os.path.relpath(root, src_dir)
        dst = dst_base if rel_root == "." else os.path.join(dst_base, rel_root)
        collected.append((dst, [os.path.join(root, f) for f in files]))
    return collected


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(where="runtime/python"),
    package_dir={"": "runtime/python"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"runtime/resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Launch files install to share/camrod_ui/launch/ for bringup compatibility.
        (f"share/{package_name}/launch", [
            "camrod_ui_robot/launch/ui.launch.py",
            "camrod_ui_guest/launch/ui_guest.launch.py",
        ]),
        *_collect_data_files("camrod_ui_robot/assets/frontend/build", f"share/{package_name}/camrod_ui_robot/assets/frontend/build"),
        *_collect_data_files("camrod_ui_guest/assets/guest_frontend", f"share/{package_name}/camrod_ui_guest/assets/guest_frontend"),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn[standard]"],
    # HH_260730 - Let `colcon test` discover the pure readiness/mission policy
    # regression suite instead of relying on an out-of-band direct invocation.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="hong",
    maintainer_email="hwanhong57@gmail.com",
    description="UI backend package for CAMROD runtime control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ui_backend_node = camrod_ui.ui_backend_node:main",
            "ui_guest_node = camrod_ui.ui_guest_node:main",
            # HH_260727 - Native lightweight shell for the operator web UI.
            "camrod_ui_window = camrod_ui.operator_ui_window:main",
        ],
    },
)
