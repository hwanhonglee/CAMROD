import os
from pathlib import Path
import runpy
# HJ_260601: Added ui_guest_node entry point and guest_frontend assets.
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop
from setuptools.command.install import install
from distutils.errors import DistutilsSetupError

package_name = "camrod_ui"


def _verify_frontend_for_install():
    # HH_261006 - Fail actual build/install commands when a direct colcon call
    # would package a missing model or absent React build. Keep metadata-only
    # discovery (including colcon list on a fresh clone) side-effect free.
    root = Path(__file__).resolve().parent
    frontend = root / "camrod_ui_robot/assets/frontend"
    verifier = runpy.run_path(str(root / "scripts/verify_frontend_assets.py"))
    try:
        verifier["verify_assets"](frontend, frontend / "build")
    except (OSError, ValueError) as exc:
        raise DistutilsSetupError(
            f"CAMROD UI assets are incomplete: {exc}. "
            "Build using ./colcon_build.sh --packages-select camrod_ui first."
        ) from exc


class VerifiedBuildPy(build_py):
    def run(self):
        _verify_frontend_for_install()
        super().run()


class VerifiedDevelop(develop):
    def run(self):
        _verify_frontend_for_install()
        super().run()


class VerifiedInstall(install):
    def run(self):
        _verify_frontend_for_install()
        super().run()


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
    cmdclass={"build_py": VerifiedBuildPy, "develop": VerifiedDevelop, "install": VerifiedInstall},
    packages=find_packages(where="runtime/python"),
    package_dir={"": "runtime/python"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"runtime/resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Launch files install to share/camrod_ui/launch/ for bringup compatibility.
        (f"share/{package_name}/launch", [
            "camrod_ui_robot/launch/ui.launch.py",
            "camrod_ui_robot/launch/mission_recorder.launch.py",
            "camrod_ui_guest/launch/ui_guest.launch.py",
        ]),
        *_collect_data_files("camrod_ui_robot/assets/frontend/build", f"share/{package_name}/camrod_ui_robot/assets/frontend/build"),
        *_collect_data_files("camrod_ui_guest/assets/guest_frontend", f"share/{package_name}/camrod_ui_guest/assets/guest_frontend"),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn[standard]"],
    zip_safe=True,
    maintainer="hong",
    maintainer_email="hwanhong57@gmail.com",
    description="UI backend package for CAMROD runtime control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ui_backend_node = camrod_ui.ui_backend_node:main",
            # HH_261002 - Passive per-mission decoded platform/CAN recorder.
            "mission_recorder_node = camrod_ui.mission_recorder_node:main",
            "ui_guest_node = camrod_ui.ui_guest_node:main",
            # HH_260727 - Native lightweight shell for the operator web UI.
            "camrod_ui_window = camrod_ui.operator_ui_window:main",
        ],
    },
)
