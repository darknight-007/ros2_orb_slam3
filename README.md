![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)
![Version](https://img.shields.io/badge/Version-2.0.0-blue.svg)

# ROS2 ORB-SLAM3 — Adaptive Visual SLAM for Drone Mapping

A ROS2 package wrapping [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared library with native ROS2 integration. Publishes camera poses, point clouds, and keyframe trajectories as standard ROS2 messages for downstream mapping and navigation.

Developed as part of the **SES 598 Space Robotics and AI** course at Arizona State University, alongside the [terrain_mapping_drone_control](https://github.com/DREAMS-lab/ses598-space-robotics-and-ai-2026/tree/main/assignments/terrain_mapping_drone_control) package for ORB-SLAM-based adaptive terrain mapping on a simulated PX4 drone.

Forked from [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) and substantially extended for ROS2 Jazzy / Ubuntu 24.04 (Noble).

## Changes from Upstream

- **ROS2 Jazzy / Ubuntu Noble** — ported includes, build system, and dependencies
- **Thirdparty libraries built from source** — DBoW2 and g2o are compiled via `add_subdirectory()` during `colcon build` (adopted from upstream jazzy branch). No more pre-built `.so` files; always matches system OpenCV.
- **SLAM output publishers** — camera pose (`PoseStamped`), point clouds (`PointCloud2`), keyframe trajectory (`Path`), tracking state, and TF broadcast
- **Configurable camera topic** — the Python driver accepts an `image_topic` parameter (defaults to `/drone_camera`)
- **Drone camera calibration** — `DroneCamera.yaml` config with intrinsics from the PX4 Gazebo simulation camera
- **OpenCV 4.6 minimum** — explicit requirement for Noble

## Architecture

The package uses a two-node design:

| Node | Language | Role |
|---|---|---|
| `mono_node_cpp` | C++ | Runs ORB-SLAM3 monocular tracking, publishes SLAM outputs, Pangolin viewer |
| `mono_driver_node.py` | Python | Subscribes to camera topic, performs handshake, relays images and timestamps |

**Data flow:**

```
/drone_camera ──> mono_driver_node.py ──> mono_node_cpp (ORB-SLAM3)
                                              │
                                              ├──> /orb_slam3/camera_pose      (PoseStamped)
                                              ├──> /orb_slam3/map_points       (PointCloud2)
                                              ├──> /orb_slam3/tracked_points   (PointCloud2)
                                              ├──> /orb_slam3/trajectory       (Path)
                                              ├──> /orb_slam3/tracking_state   (Int32)
                                              └──> TF: map → orb_slam3_camera
```

## Published Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/orb_slam3/camera_pose` | `geometry_msgs/PoseStamped` | Every frame | Camera pose in world frame (Twc) |
| `/orb_slam3/tracked_points` | `sensor_msgs/PointCloud2` | Every frame | Map points visible in the current frame |
| `/orb_slam3/map_points` | `sensor_msgs/PointCloud2` | Every 5th frame | Full accumulated map point cloud |
| `/orb_slam3/trajectory` | `nav_msgs/Path` | Every 5th frame | All keyframe poses as a path |
| `/orb_slam3/tracking_state` | `std_msgs/Int32` | Every frame | 0=not ready, 1=not init, 2=OK, 3=recently lost, 4=lost |
| TF: `map` -> `orb_slam3_camera` | `tf2` | Every frame | Camera transform for rviz/tf tree |

## Prerequisites

### Eigen3

```bash
sudo apt install libeigen3-dev
```

### Pangolin

```bash
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j$(nproc)
sudo cmake --install build
```

Then ensure `/usr/local/lib` is in your library path. Add to `~/.bashrc`:

```bash
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
fi
```

```bash
source ~/.bashrc
sudo ldconfig
```

### OpenCV

Ubuntu 24.04 ships OpenCV 4.6. Verify:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/darknight-007/ros2_orb_slam3.git
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_orb_slam3
```

DBoW2 and g2o are built automatically from source — no manual Thirdparty compilation needed.

## Usage

### Running with the Drone Camera

You need **two terminals**. Make sure the simulation (PX4 + Gazebo) is running and publishing `/drone_camera`.

**Terminal 1 — C++ SLAM node:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

**Terminal 2 — Python driver node:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=DroneCamera
```

### Visualizing in rviz2

Once both nodes are running and tracking:

```bash
rviz2
```

Add the following displays (set Fixed Frame to `map`):

- **PointCloud2** on `/orb_slam3/map_points` — full 3D map
- **PointCloud2** on `/orb_slam3/tracked_points` — current frame features
- **PoseStamped** on `/orb_slam3/camera_pose` — live camera pose
- **Path** on `/orb_slam3/trajectory` — keyframe trajectory
- **TF** — shows `map` -> `orb_slam3_camera` transform

### Using a Different Camera Topic

Override the `image_topic` parameter:

```bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
  -p settings_name:=DroneCamera \
  -p image_topic:=/some_other_topic
```

### Running the Built-in EuRoC Test

To verify the installation with the included EuRoC MH05 sample:

**Terminal 1:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

**Terminal 2:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

## Parameters

### `mono_node_cpp` (C++)

| Parameter | Default | Description |
|---|---|---|
| `node_name_arg` | `not_given` | Name identifier for the SLAM node |
| `voc_file_arg` | (auto) | Path to ORB vocabulary file. Auto-resolved if not set |
| `settings_file_path_arg` | (auto) | Path to settings directory. Auto-resolved if not set |

### `mono_driver_node.py` (Python)

| Parameter | Default | Description |
|---|---|---|
| `settings_name` | `ZED2` | Name of the YAML config in `orb_slam3/config/Monocular/` (without `.yaml`) |
| `image_topic` | `/drone_camera` | ROS2 image topic to subscribe to |

## Camera Configuration

Camera configs live in `orb_slam3/config/Monocular/`. The `DroneCamera.yaml` was generated from the `/drone_camera_info` topic:

| Parameter | Value |
|---|---|
| Resolution | 1280 x 720 |
| fx, fy | 410.94, 410.94 |
| cx, cy | 640.0, 360.0 |
| Distortion | None (rectified) |
| Color order | RGB |

To create a config for a new camera, copy an existing YAML and update the intrinsics from your camera's `CameraInfo` topic.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `libopencv_core.so.4.5d` error | Old pre-built DBoW2 binary | Clean rebuild: `rm -rf build/ install/` then `colcon build` |
| `Waiting to finish handshake ......` | Python driver not running | Start `mono_driver_node.py` in a second terminal |
| Pangolin window doesn't open | Display not set | Check `echo $DISPLAY`, reinstall Pangolin |
| ORB-SLAM3 loses tracking | Camera intrinsics mismatch | Verify `settings_name` YAML matches your camera |
| No data on `/orb_slam3/*` topics | Tracking state != OK | Check `/orb_slam3/tracking_state` — data publishes only when state is 2 (OK) |

## Credits

This package is based on the work of:

- **Azmyin Md. Kamal** — [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) — original ROS2 wrapper
- **ORB-SLAM3** — Campos et al., University of Zaragoza — [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **thien94** — [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros) — ROS1 port that influenced the project structure
- Build system improvements adapted from upstream [jazzy branch](https://github.com/Mechazo11/ros2_orb_slam3/tree/jazzy) (PR [#45](https://github.com/Mechazo11/ros2_orb_slam3/pull/45))

## Citations

```bibtex
@INPROCEEDINGS{kamal2024solving,
  author={Kamal, Azmyin Md. and Dadson, Nenyi Kweku Nkensen and Gegg, Donovan and Barbalata, Corina},
  booktitle={2024 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
  title={Solving Short-Term Relocalization Problems In Monocular Keyframe Visual SLAM Using Spatial And Semantic Data}, 
  year={2024},
  pages={615-622},
  doi={10.1109/AIM55361.2024.10637187}}
```

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```
