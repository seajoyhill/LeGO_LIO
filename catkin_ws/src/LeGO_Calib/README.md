# LeGO_Calib

`lego_calib` is an online LiDAR--IMU extrinsic calibration package that uses
**LeGO-LIO's mapOptimization result** instead of GRIL-Calib's bundled LiDAR
odometry.  It is designed for the current RoboSense-32 setup:

| Input | Default topic | Type | Purpose |
| --- | --- | --- | --- |
| Mapped LiDAR odometry | `/aft_mapped_to_init` | `nav_msgs/Odometry` | Accurate LiDAR pose from `mapOptimization` |
| Raw LiDAR scan | `/velodyne_points` | `sensor_msgs/PointCloud2` | Per-scan ground-plane measurement |
| IMU | `/imu` | `sensor_msgs/Imu` | Angular velocity, acceleration, and optionally its orientation quaternion |

It deliberately contains **no FAST-LIO odometry and no Patchwork++**. The
orientation source is selectable: `imu/use_imu_oriention: true` uses the
quaternion in `sensor_msgs/Imu::orientation` directly; `false` restores the
original Fusion AHRS estimator. The plane constraint is computed from the raw scan using PCL
`SACSegmentation` + `SAC_RANSAC` only.  The migrated GRIL/LI-Init optimizer
then uses the mapped pose trajectory, B-spline derivatives, IMU samples, and
accepted ground planes to estimate rotation, translation, and temporal offset.

## Coordinate convention

The saved estimate is intentionally the requested **LiDAR relative to IMU**
transform:

```text
p_imu = R_lidar_to_imu * p_lidar + t_lidar_to_imu
```

The output file is:

```text
$(rospack find lego_calib)/result/lego_calib_result.yaml
```

It stores `R_lidar_to_imu`, `t_lidar_to_imu`, quaternion (xyzw), RPY in
degrees, IMU-to-LiDAR time lag, and estimated biases.

## Build

```bash
cd /Users/sjh/src/LOAM/catkin_ws
catkin_make --pkg lego_calib
source devel/setup.zsh
```

## Run online with LeGO-LIO

Terminal 1 starts the existing LeGO-LIO pipeline (or your existing launch that
publishes `/aft_mapped_to_init`):

```bash
roslaunch lego_lio run.launch
```

Terminal 2 starts calibration concurrently:

```bash
roslaunch lego_calib lego_calib.launch
```

Both nodes subscribe to the same `/velodyne_points` and `/imu`; `lego_calib`
also subscribes to LeGO-LIO's `/aft_mapped_to_init`.  It does not alter
LeGO-LIO topics, parameters, or its mapping result.

After the vehicle moves at least `runtime/movement_start_distance`, a sample is
accepted only when all of the following are available near the mapped-odom
timestamp:

1. a valid orientation from the selected source (`/imu.orientation` or Fusion AHRS);
2. a raw point cloud within `sync/max_cloud_odom_dt`;
3. a PCL-RANSAC plane satisfying configured inlier ratio, RMS, and tilt gates.

The default mapped odometry frequency is `3 Hz`, matching LeGO-LIO's default
`mappingProcessInterval: 0.3`.  Change `calibration/odom_frequency` if you
change that interval materially.

To switch orientation source, edit `config/lego_calib.yaml` and restart the
node:

```yaml
imu:
  use_imu_oriention: true   # use /imu.orientation directly
  # use_imu_oriention: false  # use the Fusion AHRS estimator
```

In direct mode, the IMU quaternion must use the calibration convention
`Ground -> IMU`; in AHRS mode this convention is produced by the Fusion
implementation internally.

## Ctrl+C behavior

- When automatic excitation becomes sufficient, calibration runs and writes a
  result immediately.
- When you press **Ctrl+C**, the node runs the same batch calibration from all
  accepted samples even if the excitation test has not passed.
- To avoid producing a meaningless solve, Ctrl+C only runs optimization after
  at least `runtime/minimum_samples_to_solve` accepted samples (default 8, the B-spline minimum).
  With fewer samples it writes a `not_solved` YAML status file explaining why.

A Ctrl+C result is a **provisional** result if the excitation criterion did not
pass.  For a reliable translation estimate, collect rotations around all three
LiDAR axes and enough acceleration changes while preserving clear ground.

## Important ground-plane tuning

The scan is expected to be in the native LiDAR frame, using ROS FLU axes where
up is roughly `+Z`.  The candidate gate defaults to points within 1--40 m and
`-3 <= z <= 1` m; tune `ground_plane.min_z`, `max_z`, `max_range`, and the
RANSAC thresholds in `config/lego_calib.yaml` for your mounting height and
terrain.  The raw cloud's `ring` and point time fields are not consumed by the
calibration node itself, but they remain required by the LeGO-LIO front end.
