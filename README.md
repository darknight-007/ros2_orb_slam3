![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)
![Version](https://img.shields.io/badge/Version-1.6.0-blue.svg)

# ROS2 ORB-SLAM3 — Monocular Visual SLAM for Drone Applications

A ROS2 package wrapping [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared library with native ROS2 integration. Used in the **SES 598 Space Robotics and AI** course ([terrain_mapping_drone_control](../terrain_mapping_drone_control/)) for monocular visual SLAM on a simulated PX4 drone.

Forked from [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) and adapted for ROS2 Jazzy / Ubuntu 24.04 (Noble) with configurable camera topics.

## Architecture

The package uses a two-node design:

| Node | Language | Role |
|---|---|---|
| `mono_node_cpp` | C++ | Runs ORB-SLAM3 monocular tracking, Pangolin viewer |
| `mono_driver_node.py` | Python | Subscribes to camera topic, performs handshake, relays images and timestamps to the C++ node |

**Data flow:**

```
/drone_camera ──> mono_driver_node.py ──> /mono_py_driver/img_msg ──> mono_node_cpp (ORB-SLAM3)
                                      └─> /mono_py_driver/timestep_msg ─┘
```

## Changes from Upstream

- **ROS2 Jazzy / Ubuntu Noble support** — rebuilt Thirdparty libraries (DBoW2, g2o) against system OpenCV 4.6
- **Configurable camera topic** — the Python driver accepts an `image_topic` parameter (defaults to `/drone_camera`)
- **Drone camera calibration** — added `DroneCamera.yaml` config with intrinsics from the PX4 Gazebo simulation camera
- Upstream was hardcoded for ZED2 camera on ROS2 Humble / Ubuntu 22.04

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

> **Note:** If you see errors about `libopencv_core.so.4.5d` at runtime, the Thirdparty DBoW2 library was built against a different OpenCV version. Rebuild it:
>
> ```bash
> cd ~/ros2_ws/src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2
> rm -rf build lib/libDBoW2.so
> mkdir -p build && cd build
> cmake .. -DCMAKE_BUILD_TYPE=Release
> make -j$(nproc)
> ```

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/darknight-007/ros2_orb_slam3.git
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_orb_slam3
```

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

The driver performs a handshake with the C++ node, then begins relaying images from `/drone_camera`. The Pangolin viewer window should open showing the ORB feature extraction and map.

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
| `settings_name` | `ZED2` | Name of the YAML config file in `orb_slam3/config/Monocular/` (without `.yaml`) |
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
| `libopencv_core.so.4.5d: cannot open shared object` | DBoW2 was built against a different OpenCV | Rebuild DBoW2 (see Prerequisites) |
| `Waiting to finish handshake ......` hangs | Python driver not running or topic mismatch | Start the Python driver in a second terminal |
| Pangolin window doesn't open | Display not set or Pangolin not installed | Check `echo $DISPLAY`, reinstall Pangolin |
| ORB-SLAM3 loses tracking immediately | Camera intrinsics mismatch | Verify `settings_name` YAML matches your camera |

## Credits

This package is based on the work of:

- **Azmyin Md. Kamal** — [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) — original ROS2 wrapper
- **ORB-SLAM3** — Campos et al., University of Zaragoza — [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **thien94** — [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros) — ROS1 port that influenced the project structure

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
