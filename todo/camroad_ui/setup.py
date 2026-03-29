"""
setup.py — ROS2 Python 패키지(ament_python) 빌드 설정 파일

역할:
  - colcon build 시 패키지 메타데이터와 실행 가능한 엔트리포인트를 정의
  - ros2 run camroad_ui_pkg <노드이름> 으로 실행할 수 있게 해줌
"""

import os
from setuptools import setup, find_packages

package_name = 'camroad_ui_pkg'


def _frontend_data_files(src='frontend/build', dest_base='share/' + package_name):
    """frontend/build 디렉토리 전체를 share/camroad_ui_pkg/frontend/build 에 설치"""
    result = []
    for root, _, files in os.walk(src):
        if not files:
            continue
        # root = 'frontend/build', 'frontend/build/static/js', ...
        dest = os.path.join(dest_base, root)  # e.g. share/camroad_ui_pkg/frontend/build/static/js
        result.append((dest, [os.path.join(root, f) for f in files]))
    return result


setup(
    name=package_name,
    version='0.0.1',

    # 'camroad_ui_pkg/' 디렉토리 아래 Python 모듈을 자동 탐색 (test 제외)
    packages=find_packages(exclude=['test']),

    # ROS2 ament 빌드 시스템에 필요한 데이터 파일
    data_files=[
        # ament 인덱스에 패키지 등록 (ros2 pkg list에 표시되도록)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml을 share 디렉토리에 설치
        ('share/' + package_name, ['package.xml']),
        # React 빌드 결과물을 share/camroad_ui_pkg/frontend/build/ 에 설치
        *_frontend_data_files(),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Robot UI with WebUI toggle button and ROS2 publisher',
    license='MIT',

    # ROS2 실행 가능 노드 등록
    # ros2 run camroad_ui_pkg toggle_publisher → toggle_publisher.py의 main() 실행
    # ros2 run camroad_ui_pkg backend           → backend.py의 main() 실행
    entry_points={
        'console_scripts': [
            'toggle_publisher = camroad_ui_pkg.toggle_publisher:main',
            'backend = camroad_ui_pkg.backend:main',
        ],
    },
)
