# Copyright (c) 2026 CAMROD Project
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_params():
    """Prefer the bringup-owned live config, with a standalone fallback."""
    config_root = os.environ.get('CAMROD_CONFIG_ROOT', '').strip()
    if config_root:
        candidate = Path(config_root) / 'snapshot' / 'camrod_topics.params.yaml'
        if candidate.is_file():
            return str(candidate)

    # Walk up from this launch file (symlink-install resolves back into the
    # source tree) until a sibling camrod_bringup package is found, so the
    # lookup survives however deep this package sits in the workspace.
    for ancestor in Path(__file__).resolve().parents:
        source_candidate = (
            ancestor
            / 'camrod_bringup'
            / 'config'
            / 'snapshot'
            / 'camrod_topics.params.yaml'
        )
        if source_candidate.is_file():
            return str(source_candidate)

    try:
        candidate = (
            Path(get_package_share_directory('camrod_bringup'))
            / 'config'
            / 'snapshot'
            / 'camrod_topics.params.yaml'
        )
        if candidate.is_file():
            return str(candidate)
    except PackageNotFoundError:
        pass

    return os.path.join(
        get_package_share_directory('camrod_snapshot'),
        'param',
        'camrod_topics.params.yaml',
    )


def generate_launch_description():
    default_params = _resolve_default_params()

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Snapshotter topic selection and buffer limits',
    )

    snapshotter = Node(
        package='camrod_snapshot',
        executable='snapshotter',
        name='snapshotter',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file,
        snapshotter,
    ])
