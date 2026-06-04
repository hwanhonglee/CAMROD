# Ground Segmentation ROS 2 Node (GSeg3D)

> **⚠️ CAMROD Modified Version**
> 이 패키지는 CAMROD 프로젝트용으로 수정된 버전입니다.
> 원본과 달리 `ground_segmentation` 코어 라이브러리(header-only)가 **이 패키지 안에 통합**되어 있어,
> 별도의 라이브러리 클론 없이 단독 빌드가 가능합니다.
> 변경 내역은 [CAMROD Modifications](#camrod-modifications) 섹션을 참고하세요.

Core Library • Nav2 Plugin • Paper • Video

This package implements a ROS 2 node that wraps the GSeg3D ground segmentation algorithm and exposes it as a real-time perception component for robotic systems.

The node subscribes to LiDAR point clouds (optionally synchronized with IMU data), performs two-phase grid-based ground segmentation, and publishes ground and obstacle point clouds for downstream navigation and perception modules.

## Overview

The node:

- Receives a `sensor_msgs/PointCloud2` input
- Optionally fuses IMU orientation for gravity alignment
- Applies GSeg3D Phase I (coarse) and Phase II (refinement) segmentation
- Publishes:
  - Ground points
  - Obstacle (non-ground) points
  - Raw filtered input cloud

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/ground_segmentation/input_pointcloud` | `sensor_msgs/msg/PointCloud2` | Input LiDAR point cloud |
| `/ground_segmentation/input_imu` | `sensor_msgs/msg/Imu` | IMU orientation (optional) |

IMU synchronization is enabled via the parameter `use_imu_orientation`.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/ground_segmentation/ground_points` | `sensor_msgs/msg/PointCloud2` | Segmented ground points |
| `/ground_segmentation/obstacle_points` | `sensor_msgs/msg/PointCloud2` | Segmented non-ground points |
| `/ground_segmentation/raw_points` | `sensor_msgs/msg/PointCloud2` | Filtered input cloud |

## Processing Pipeline

1. **Input Filtering**
   - ROI cropping (min/max X, Y, Z)
   - Optional voxel downsampling
2. **Synthetic Ground Injection**
   - Injects a ground seed beneath the robot
   - Ensures reliable ground region initialization
3. **Frame Alignment**
   - Transforms point cloud into `robot_frame`
   - Uses TF2 for frame lookup
   - Uses IMU data (if enabled) to compute gravity-aligned orientation
4. **Two-Phase Ground Segmentation**
   - Phase I: Coarse grid segmentation (large Z resolution)
   - Phase II: Fine grid refinement (small Z resolution)
5. **Post-Processing**
   - Radius-based filtering around the robot
   - Final ground / obstacle separation
6. **Publishing**
   - Results published as ROS 2 PointCloud2 messages

## Key Parameters

| Parameter | Description |
|---|---|
| `robot_frame` | Target frame for segmentation |
| `use_imu_orientation` | Enable IMU-based gravity alignment |
| `cellSizeX`, `cellSizeY`, `cellSizeZ` | Grid resolution for Phase I (m) |
| `cellSizeZPhase2` | Grid resolution in Z for Phase II (m) |
| `slopeThresholdDegrees` | Max slope for ground (degrees). Controls which planes are considered drivable. |
| `groundInlierThreshold` | Plane fitting inlier threshold (m). Controls how far points may lie from the estimated plane. |
| `centroidSearchRadius` | KD-tree expansion radius (m) — nanoflann radius search 사용 |
| `lidar_to_ground` | LiDAR-to-ground distance (m) |
| `transform_tolerance` | Tolerance for fetching transforms from TF (sec) |
| `show_benchmark` | Enable precision/recall evaluation |
| `maxGroundHeightDeviation` | Controls allowed centroid height difference in meters during Phase II expansion. |

---

## CAMROD Modifications

CAMROD 프로젝트 통합 과정에서 다음 사항이 원본과 다르게 수정되었습니다.

### 1. ground_segmentation 코어 라이브러리 통합 (header-only)

원본은 [`ground_segmentation`](https://github.com/dfki-ric/ground_segmentation) 코어 라이브러리를
별도 패키지로 클론한 뒤 `find_package(ground_segmentation)`으로 링크하는 구조였습니다.

그러나 코어 라이브러리의 실체는 컴파일되는 소스 없이 **헤더 3개뿐인 header-only (INTERFACE) 라이브러리**이므로,
해당 헤더를 이 패키지의 `include/`에 직접 포함하여 단일 패키지로 통합했습니다.

```
include/
├── ground_detection.hpp          # GSeg3D 2-phase 지면 분리 알고리즘
├── ground_detection_types.hpp    # 셀/그리드 자료구조 정의
└── pointcloud_processor.hpp      # ROI crop, voxel downsample 등 전처리
```

**CMakeLists.txt 변경:**

```cmake
# 제거
find_package(ground_segmentation REQUIRED)
target_link_libraries(... ground_segmentation::ground_segmentation)

# 추가
find_package(nanoflann REQUIRED)
target_include_directories(ground_segmentation_ros2_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include   # 통합된 헤더
  ...
)
target_link_libraries(ground_segmentation_ros2_node
  ${PCL_LIBRARIES}
  nanoflann::nanoflann                  # 코어 라이브러리가 쓰던 의존성을 직접 선언
)
```

**package.xml 변경:**

```xml
<!-- 변경 전 -->
<depend>ground_segmentation</depend>

<!-- 변경 후 -->
<depend>nanoflann</depend>
```

### 2. 통합의 장단점

| 장점 | 단점 |
|---|---|
| 패키지 1개로 빌드/관리 단순화 | upstream(DFKI) 업데이트 시 헤더 수동 동기화 필요 |
| 별도 클론 단계 불필요 | |
| 빌드 순서 의존성 제거 | |

upstream 헤더가 업데이트되면 아래 저장소의 `include/*.hpp`를 이 패키지의 `include/`로 다시 복사하면 됩니다:
`https://github.com/dfki-ric/ground_segmentation`

---

## Dependency: nanoflann

### nanoflann이란?

**KD-tree 기반 최근접 이웃 탐색(Nearest Neighbor Search) header-only C++ 라이브러리**입니다.

- 원작자: Jose Luis Blanco-Claraco (MRPT 프로젝트)
- 저장소: https://github.com/jlblancoc/nanoflann
- FLANN 라이브러리의 KD-tree 부분만 떼어내 경량화한 것 ("nano" + FLANN)
- 실체는 `nanoflann.hpp` 헤더 파일 하나 (컴파일된 라이브러리 없음)

### 이 패키지에서의 역할

지면 분리 과정에서 각 셀의 centroid 주변 이웃을 탐색할 때 사용됩니다.
파라미터 `centroidSearchRadius`가 nanoflann의 radius search 반경에 해당하며,
Phase II 확장 단계에서 지면 높이의 연속성 판단에 활용됩니다.

무차별 탐색(O(N)) 대신 KD-tree 탐색(O(log N))을 사용하므로
대규모 포인트클라우드에서도 실시간(CPU) 처리가 가능합니다.

### 설치

rosdep이 `package.xml`의 `<depend>nanoflann</depend>`을 보고 자동 설치합니다.
수동 설치 시:

```bash
sudo apt install libnanoflann-dev
```

### 문제 발생 시 확인 지점

```bash
# 1. 설치 여부 / 버전
dpkg -l | grep nanoflann

# 2. CMake config 파일 위치 (find_package 실패 시)
dpkg -L libnanoflann-dev | grep cmake

# 3. 헤더 존재 확인 (컴파일 에러 시)
ls /usr/include/nanoflann.hpp

# 4. rosdep 매핑 확인 (다른 PC 배포 시)
rosdep resolve nanoflann   # → libnanoflann-dev 가 나오면 정상
```

`find_package(nanoflann REQUIRED)` 실패 시 다음 에러가 발생합니다:

```
Could not find a package configuration file provided by "nanoflann"
```

이 경우 `sudo apt install libnanoflann-dev`로 해결합니다.

---

## Quick Start

### System Requirements

- Ubuntu 22.04, Ubuntu 24.04
- ROS 2 Humble, ROS 2 Jazzy

### Prerequisite

> **CAMROD 버전 참고:** 코어 라이브러리가 통합되어 있으므로
> 원본과 달리 `ground_segmentation` 별도 클론이 **필요 없습니다**.

```bash
sudo apt update
sudo apt install cmake libpcl-dev libeigen3-dev libnanoflann-dev
```

### Build Instructions

```bash
cd ~/ros2_ws/src
# (CAMROD 워크스페이스에서는 camrod_sensing/external/ 아래에 이미 포함되어 있음)
cd ~/ros2_ws
source /opt/ros/YOUR_ROS_DISTRO/setup.bash
colcon build --packages-up-to ground_segmentation_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
source install/setup.bash
```

### Running the ROS 2 Node

```bash
ros2 launch ground_segmentation_ros2 ground_segmentation.launch.py \
  pointcloud_topic:=<POINTCLOUD_TOPIC> \
  imu_topic:=<IMU_TOPIC> \
  use_sim_time:=<true/false>
```

Launch argument `imu_topic` is optional and is used only when parameter `use_imu_orientation` is true.

CAMROD 파이프라인에서는 `camrod_sensing`의 `lidar_driver.launch.py`가 이 노드를 실행하며,
토픽은 다음과 같이 remapping 됩니다:

```
/ground_segmentation/input_pointcloud → /sensing/lidar/vanjee/points_raw
/ground_segmentation/obstacle_points  → /sensing/lidar/points_filtered
```

파라미터는 `camrod_sensing/config/lidar/ground_seg_params.yaml`에서 로드됩니다.

## Node Configuration

Parameters are loaded from `parameters.yaml`
(CAMROD에서는 `camrod_sensing/config/lidar/ground_seg_params.yaml` 사용)

## Benchmarking Mode

When `show_benchmark=true`:

- Runtime statistics are printed to console

## Intended Use

- Real-time ground segmentation for mobile robots
- Safety-critical navigation and obstacle detection
- Traversability analysis
- Research

## Notes

- Requires a valid TF tree between sensor, IMU, and robot frames
- Designed for CPU real-time execution

## License

BSD-3 Clause License.

## Citation

If you use this work, please cite:

```bibtex
@inproceedings{lodhi2025gseg3d,
  author    = {Muhammad Haider Khan Lodhi and Christoph Hertzberg},
  title     = {GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds},
  booktitle = {2025 7th International Conference on Robotics and Computer Vision (ICRCV)},
  pages     = {119-126},
  year      = {2025},
  doi       = {10.1109/ICRCV67407.2025.11349133}
}
```

© DFKI Robotics Innovation Center
