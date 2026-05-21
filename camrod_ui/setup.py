import os
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
        (f"share/{package_name}/launch", ["launch/ui.launch.py"]),
        *_collect_data_files("runtime/assets/frontend/build", f"share/{package_name}/assets/frontend/build"),
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
        ],
    },
)
