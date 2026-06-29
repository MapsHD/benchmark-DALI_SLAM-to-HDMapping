## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-DALI_SLAM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-DALI_SLAM-to-HDMapping

Runs the [DALI_SLAM](https://github.com/DCSI2022/DALI_SLAM) degeneracy-aware
LiDAR-inertial odometry front-end (**DA-LIO**) on a ROS 1 bag file and converts
the output to an [HDMapping](https://github.com/MapsHD/HDMapping) session.

DALI_SLAM is *Degeneracy-Aware LiDAR-inertial SLAM with novel distortion
correction and accurate multi-constraint pose graph optimization* by Wu et al.,
ISPRS Journal of Photogrammetry and Remote Sensing, 2025
([paper](https://www.sciencedirect.com/science/article/pii/S0924271625000413)).
DA-LIO is FAST-LIO2-based.

## Prerequisites

- Docker
- A ROS 1 bag containing a LiDAR topic and an IMU topic matching the topic names
  declared in the chosen DA-LIO config (ROS 2 bags are automatically converted
  to ROS 1 format). For the Livox profiles the LiDAR topic must be a
  `livox_ros_driver/CustomMsg`; the other profiles expect a
  `sensor_msgs/PointCloud2`.

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-DALI_SLAM-to-HDMapping.git --recursive
cd benchmark-DALI_SLAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t dalislam_noetic .
```

This installs:
- Ubuntu 20.04 + ROS 1 Noetic
- Eigen3, PCL, OpenCV, Boost, OpenMP, TBB, glog/gflags
- GTSAM 4.0.3 and Ceres 2.1.0 (built from source at the prefixes DA-LIO expects)
- Livox-SDK + `livox_ros_driver` (provides `CustomMsg`)
- DA-LIO (compiled from the DALI_SLAM submodule; the MC-PGO back-end is skipped)
- catkin workspace with `da_lio` and `dalislam_to_hdmapping`

The build takes several minutes on first run (GTSAM and Ceres are built from
source).

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros1-dalislam.sh
./docker_session_run-ros1-dalislam.sh /path/to/input.bag /path/to/output/dir
```

Or with no arguments to use a GUI file selector (requires `zenity`):

```bash
./docker_session_run-ros1-dalislam.sh
```

By default the script uses the `helmet_mid` profile (the upstream default config,
which matches the Livox-Mid test bag). Pick a different one with the `SENSOR`
environment variable, e.g.:

```bash
SENSOR=velodyne ./docker_session_run-ros1-dalislam.sh /path/to/input.bag /path/to/output/dir
```

Available sensor profiles (from the `DALI_SLAM/DA_LIO/config/` directory):

| `SENSOR`      | Config file         | LiDAR type | Config LiDAR topic       | Config IMU topic    |
|---------------|---------------------|------------|--------------------------|---------------------|
| `helmet_mid`  | `helmet_mid.yaml`   | Livox (Mid) | `/livox/lidar`          | `/imu0`             |
| `helmet_avia` | `helmet_avia.yaml`  | Livox (Avia) | `/livox/lidar`         | `/imu0`             |
| `avia`        | `avia.yaml`         | Livox (Avia) | `/livox/lidar`         | `/livox/imu`        |
| `horizon`     | `horizon.yaml`      | Livox (Horizon) | `/livox/lidar`      | `/livox/imu`        |
| `hesai`       | `hesai.yaml`        | Hesai      | `/hesai/pandar`          | `/alphasense/imu`   |
| `ouster64`    | `ouster64.yaml`     | Ouster OS  | `/os_cloud_node/points`  | `/os_cloud_node/imu`|
| `velodyne`    | `velodyne.yaml`     | Velodyne   | `/velodyne_points`       | `/imu/data`         |

If your bag uses different topic names than the profile's config, remap them on
playback with `LIDAR_TOPIC` / `IMU_TOPIC` (the value is the name **in the bag**):

```bash
SENSOR=velodyne LIDAR_TOPIC=/points_raw IMU_TOPIC=/imu/data_raw \
  ./docker_session_run-ros1-dalislam.sh /path/to/input.bag /path/to/output/dir
```

**What happens:**

The script opens a Docker container with a tmux session containing five panes on
window 0 and a `control` window (window 1, the attach target):

| Pane | Role |
|------|------|
| 0 | `roscore` |
| 1 | `roslaunch da_lio run_dalio_bench.launch` — subscribes to the LiDAR + IMU topics, publishes `/Odometry` + `/cloud_registered` (+ RViz live view) |
| 2 | `rosbag record /Odometry /cloud_registered` — captures the odometry and the registered world cloud |
| 3 | `rosbag play --clock` — plays your input bag with simulated clock |
| 4 | diagnostics — shows active topics and publishing rates |

Press `Ctrl+b` then `0` to switch to window 0 and watch RViz. When playback
finishes, the control window stops the recorder, kills all nodes and RViz, and
exits tmux. A second Docker run then converts the recorded bag into the
HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-DALI_SLAM/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
scan_lio_1.laz
...
session.json
trajectory_lio_0.csv
trajectory_lio_1.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

## Notes on DA-LIO

DA-LIO (like FAST-LIO2) publishes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/Odometry` | `nav_msgs/Odometry` | the current 6-DoF body pose in the global frame |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | the current scan, already registered into the global frame |

Because `/cloud_registered` is already in the world frame, the converter does
**not** re-apply the pose to the points — it only uses `/Odometry` to build the
per-chunk trajectory files. The recorded topics are tunable via env vars:

| Variable | Meaning | Default |
|----------|---------|---------|
| `ODOM_TOPIC`  | DA-LIO odometry output            | `/Odometry` |
| `CLOUD_TOPIC` | DA-LIO registered cloud (world)   | `/cloud_registered` |

This benchmark captures DA-LIO's **online** odometry output, consistent with the
other LIO benchmarks in this repo. DALI_SLAM's MC-PGO back-end (offline
multi-constraint pose graph optimization) is a separate stage and is not part of
the recorded session.

## Contact

januszbedkowski@gmail.com
