#!/usr/bin/env python3
"""Entry point for camrod_bringup.

Thin wrapper that loads _bringup_impl.py at runtime so YAML edits in the
source tree apply immediately without rebuilding the package.
"""

import importlib.util
import os


def _load_impl_module():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    impl_path = os.path.join(launch_dir, '_bringup_impl.py')
    spec = importlib.util.spec_from_file_location('camrod_bringup_launch_impl', impl_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Failed to load bringup implementation: {impl_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    return _load_impl_module().generate_launch_description()
