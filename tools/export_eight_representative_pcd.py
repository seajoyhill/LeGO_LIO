#!/usr/bin/env python3
"""Export IMU-selected three-frame groups from eight.bag as ASCII PCD and IMU CSV."""
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

import numpy as np
import rosbag

# Relative times are measured from the first PointCloud2 header timestamp.
# A group contains a source scan plus targets one and two LiDAR periods later.
SELECTIONS = [
    {"id": "01_static_baseline", "label": "静止基线", "description": "IMU 角速度和线加速度波动均接近最低值；用于验证零/极小位姿变化下的 ICP。", "source_rel_s": 3.500},
    {"id": "02_gentle_left_turn", "label": "缓和左转", "description": "稳定的正 z 轴角速度（约 +0.30 rad/s）；适合中等旋转初值误差。", "source_rel_s": 20.003},
    {"id": "03_gentle_right_turn", "label": "缓和右转", "description": "稳定的负 z 轴角速度（约 -0.27 rad/s）；与左转形成反向旋转测试。", "source_rel_s": 44.992},
    {"id": "04_fast_left_turn", "label": "快速左转", "description": "较高的正 z 轴角速度（约 +0.48 rad/s）；更具挑战的旋转配准。", "source_rel_s": 64.990},
    {"id": "05_fast_right_turn", "label": "快速右转", "description": "较高的负 z 轴角速度（约 -0.52 rad/s）；更具挑战的反向旋转配准。", "source_rel_s": 80.689},
    {"id": "06_peak_left_yaw", "label": "峰值左转", "description": "接近全程最大正 z 轴角速度（约 +0.62 rad/s）；用于测试较大帧间角变化。", "source_rel_s": 89.288},
    {"id": "07_high_linear_dynamics", "label": "高线性动态", "description": "线加速度波动接近全程最大、转动较弱；用于测试快速平移/振动场景。", "source_rel_s": 96.187},
]

# role, nominal interval after the source point-cloud header timestamp.
GROUP_FRAME_OFFSETS = (("source", 0.0), ("target_0p1s", 0.1), ("target_0p2s", 0.2))

# PointCloud2 layout in this bag.
POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 26,
})

# IMU array layout: timestamp, quaternion xyzw, angular velocity xyz, linear acceleration xyz.
IMU_COLUMNS = [
    "timestamp_unix_s", "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "angular_velocity_x_rad_s", "angular_velocity_y_rad_s", "angular_velocity_z_rad_s",
    "linear_acceleration_x_m_s2", "linear_acceleration_y_m_s2", "linear_acceleration_z_m_s2",
]


def nearest(rows, time_s):
    return min(rows, key=lambda row: abs(row["time"] - time_s))


def imu_stats(imu: np.ndarray, timestamp: float, half_window_s: float = 0.45):
    chosen = imu[(imu[:, 0] >= timestamp - half_window_s) & (imu[:, 0] <= timestamp + half_window_s)]
    omega = np.linalg.norm(chosen[:, 5:8], axis=1)
    accel = np.linalg.norm(chosen[:, 8:11], axis=1)
    return {
        "window_s": half_window_s,
        "sample_count": int(len(chosen)),
        "angular_speed_mean_rad_s": float(np.mean(omega)),
        "angular_speed_max_rad_s": float(np.max(omega)),
        "yaw_rate_mean_rad_s": float(np.mean(chosen[:, 7])),
        "acceleration_norm_mean_m_s2": float(np.mean(accel)),
        "acceleration_norm_std_m_s2": float(np.std(accel)),
    }


def write_pcd_ascii(msg, path: Path):
    """Write finite XYZ points and return the exact point-time scan interval."""
    total = msg.width * msg.height
    if msg.point_step != POINT_DTYPE.itemsize:
        raise RuntimeError(f"Unexpected point_step: {msg.point_step}; expected {POINT_DTYPE.itemsize}")
    raw = np.frombuffer(msg.data, dtype=POINT_DTYPE, count=total)
    valid = (
        np.isfinite(raw["x"]) & np.isfinite(raw["y"]) & np.isfinite(raw["z"]) & np.isfinite(raw["timestamp"])
    )
    points = raw[valid]
    with path.open("w", encoding="ascii", newline="\n") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        f.write("FIELDS x y z intensity ring timestamp\nSIZE 4 4 4 4 2 8\nTYPE F F F F U F\nCOUNT 1 1 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        for point in points:
            # 17 significant digits preserve the float64 Unix timestamp values.
            f.write(
                f"{point['x']:.8g} {point['y']:.8g} {point['z']:.8g} {point['intensity']:.8g} "
                f"{int(point['ring'])} {point['timestamp']:.17g}\n"
            )
    point_times = points["timestamp"]
    return {
        "raw_points": int(total),
        "exported_points": int(len(points)),
        "dropped_nonfinite_fields": int(total - len(points)),
        "scan_start_unix_s": float(point_times.min()),
        "scan_end_unix_s": float(point_times.max()),
        "scan_duration_s": float(point_times.max() - point_times.min()),
    }


def write_scan_imu_csv(imu: np.ndarray, scan_start: float, scan_end: float, first_cloud_time: float, path: Path):
    """Export actual /imu messages whose header time falls in the LiDAR scan interval."""
    rows = imu[(imu[:, 0] >= scan_start) & (imu[:, 0] <= scan_end)]
    fields = ["timestamp_unix_s", "time_from_scan_start_s", "time_from_first_cloud_s", *IMU_COLUMNS[1:]]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for row in rows:
            writer.writerow([
                f"{row[0]:.17g}", f"{row[0] - scan_start:.9f}", f"{row[0] - first_cloud_time:.9f}",
                *[f"{value:.17g}" for value in row[1:]],
            ])
    return int(len(rows))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    out = args.output
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    imu_rows, cloud_rows = [], []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=["/imu", "/velodyne_points"]):
            timestamp = msg.header.stamp.to_sec() or bag_time.to_sec()
            if topic == "/imu":
                q, av, la = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                imu_rows.append((timestamp, q.x, q.y, q.z, q.w, av.x, av.y, av.z, la.x, la.y, la.z))
            else:
                cloud_rows.append({"time": timestamp, "msg": msg})

    imu = np.asarray(imu_rows, dtype=np.float64)
    first_time = cloud_rows[0]["time"]
    for cloud in cloud_rows:
        cloud["relative_time_s"] = cloud["time"] - first_time

    report = {
        "source_bag": str(args.bag), "pointcloud_topic": "/velodyne_points", "imu_topic": "/imu",
        "pcd_format": "ASCII PCD v0.7", "point_fields": list(POINT_DTYPE.names),
        "imu_csv_columns": ["timestamp_unix_s", "time_from_scan_start_s", "time_from_first_cloud_s", *IMU_COLUMNS[1:]],
        "relative_time_origin": {"description": "First /velodyne_points header timestamp", "unix_time_s": first_time},
        "groups": [],
    }

    for selection in SELECTIONS:
        group_dir = out / selection["id"]
        group_dir.mkdir()
        group_report = {key: selection[key] for key in ("id", "label", "description")}
        group_report["frames"] = []
        source_requested = selection["source_rel_s"]
        for role, nominal_offset in GROUP_FRAME_OFFSETS:
            requested = source_requested + nominal_offset
            cloud = nearest(cloud_rows, first_time + requested)
            actual_rel = cloud["relative_time_s"]
            pcd_name = f"{role}_{actual_rel:07.3f}s.pcd"
            pcd_path = group_dir / pcd_name
            pcd_stats = write_pcd_ascii(cloud["msg"], pcd_path)
            imu_name = f"{role}_{actual_rel:07.3f}s_imu.csv"
            imu_path = group_dir / imu_name
            imu_samples = write_scan_imu_csv(
                imu, pcd_stats["scan_start_unix_s"], pcd_stats["scan_end_unix_s"], first_time, imu_path
            )
            group_report["frames"].append({
                "role": role,
                "nominal_offset_from_source_s": nominal_offset,
                "pcd_file": str(pcd_path.relative_to(out)),
                "imu_csv_file": str(imu_path.relative_to(out)),
                "requested_relative_time_s": requested,
                "actual_relative_time_s": actual_rel,
                "unix_header_time_s": cloud["time"], "frame_id": cloud["msg"].header.frame_id,
                "imu_samples_during_scan": imu_samples,
                "imu_stats_around_header": imu_stats(imu, cloud["time"]),
                **pcd_stats,
            })
        source_time = group_report["frames"][0]["actual_relative_time_s"]
        for frame in group_report["frames"]:
            frame["actual_offset_from_source_s"] = frame["actual_relative_time_s"] - source_time
        report["groups"].append(group_report)

    with (out / "manifest.json").open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")
    with (out / "manifest.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["group", "label", "role", "pcd_file", "imu_csv_file", "relative_time_s", "offset_from_source_s", "scan_start_unix_s", "scan_end_unix_s", "scan_duration_s", "points", "imu_samples_during_scan", "yaw_rate_mean_rad_s", "angular_speed_mean_rad_s", "acceleration_norm_std_m_s2"])
        for group in report["groups"]:
            for frame in group["frames"]:
                stats = frame["imu_stats_around_header"]
                writer.writerow([group["id"], group["label"], frame["role"], frame["pcd_file"], frame["imu_csv_file"], f"{frame['actual_relative_time_s']:.6f}", f"{frame['actual_offset_from_source_s']:.6f}", f"{frame['scan_start_unix_s']:.17g}", f"{frame['scan_end_unix_s']:.17g}", f"{frame['scan_duration_s']:.9f}", frame["exported_points"], frame["imu_samples_during_scan"], f"{stats['yaw_rate_mean_rad_s']:.6f}", f"{stats['angular_speed_mean_rad_s']:.6f}", f"{stats['acceleration_norm_std_m_s2']:.6f}"])

    with (out / "README.md").open("w", encoding="utf-8") as f:
        f.write("# eight.bag：IMU 筛选的代表性 ASCII PCD 三帧组\n\n")
        f.write("每个子目录都有三帧：`source`、`target_0p1s`（相邻 LiDAR 帧，约 0.1 s）和 `target_0p2s`（隔一帧，约 0.2 s）。因此可分别测试相邻帧和较大位姿变化的 ICP。\n\n")
        f.write("每个 PCD 均配有同名后缀 `_imu.csv` 的 IMU 文件。CSV 只包含该 PCD 内实际有效点时间戳范围 `[scan_start_unix_s, scan_end_unix_s]` 内的 `/imu` 消息；不插值。PCD 保持原始 Velodyne 坐标系，未做去畸变、下采样、地面分割或坐标变换。\n\n")
        f.write("PCD 为 ASCII v0.7，字段 `x y z intensity ring timestamp`；`timestamp` 为已完整保留精度的 Unix 秒。各扫描的精确范围、点数和 IMU 文件位置见 `manifest.json` / `manifest.csv`。\n\n")
        f.write("| 组 | 场景 | source / +0.1 s / +0.2 s 实际相对时间 (s) |\n|---|---|---:|\n")
        for group in report["groups"]:
            times = " / ".join(f"{frame['actual_relative_time_s']:.3f}" for frame in group["frames"])
            f.write(f"| `{group['id']}` | {group['label']} | {times} |\n")

    total_frames = sum(len(group["frames"]) for group in report["groups"])
    print(f"Exported {len(report['groups'])} groups ({total_frames} ASCII PCD files and {total_frames} scan IMU CSV files) to {out}")


if __name__ == "__main__":
    main()
