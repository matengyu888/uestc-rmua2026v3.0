#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from pathlib import Path as FsPath

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.interpolate import PchipInterpolator, splprep, splev


def load_points(csv_file):
    points = []
    sort_idx = None
    with open(csv_file, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)

    if not rows:
        return np.empty((0, 3), dtype=float)

    header = [cell.strip().lower() for cell in rows[0]]
    if {"x", "y", "z"}.issubset(set(header)):
        x_idx = header.index("x")
        y_idx = header.index("y")
        z_idx = header.index("z")
        if "global_order" in header:
            sort_idx = header.index("global_order")
        data_rows = rows[1:]
    else:
        x_idx, y_idx, z_idx = 0, 1, 2
        data_rows = rows
        if len(rows[0]) >= 4:
            try:
                first_col = float(rows[0][0])
                second_col = float(rows[0][1])
                if abs(first_col) > 1e6 and abs(second_col) < 1e5:
                    x_idx, y_idx, z_idx = 1, 2, 3
            except ValueError:
                pass

    sortable_points = []
    for row in data_rows:
        if len(row) <= max(x_idx, y_idx, z_idx):
            continue
        try:
            point = [float(row[x_idx]), float(row[y_idx]), float(row[z_idx])]
            if sort_idx is not None and len(row) > sort_idx and row[sort_idx].strip():
                sortable_points.append((float(row[sort_idx]), point))
            else:
                points.append(point)
        except ValueError:
            continue

    if sortable_points:
        sortable_points.sort(key=lambda item: item[0])
        points.extend(point for _, point in sortable_points)

    return np.asarray(points, dtype=float)


def infer_path_mode(csv_file, rows):
    csv_name = FsPath(csv_file).name.lower()
    header = [cell.strip().lower() for cell in rows[0]] if rows else []
    if is_dense_reference_csv(rows):
        return "direct"
    if "anchors" in csv_name:
        return "anchor_interp"
    if {"global_order", "segment", "kind", "x", "y", "z"}.issubset(set(header)):
        return "anchor_interp"
    return "spline"


def is_dense_reference_csv(rows):
    if not rows:
        return False

    header = [cell.strip().lower() for cell in rows[0]]
    required = {"timestamp", "x", "y", "z"}
    if not required.issubset(set(header)):
        return False

    # 手工飞出的参考轨迹通常自带姿态列且点数已经很密，不需要 path_loader 再做二次重建。
    has_attitude = {"qx", "qy", "qz", "qw"}.issubset(set(header))
    return has_attitude and len(rows) > 500


def remove_duplicate_points(points, min_spacing):
    if len(points) <= 1:
        return points

    filtered = [points[0]]
    min_spacing = max(min_spacing, 1e-4)
    for point in points[1:]:
        if np.linalg.norm(point - filtered[-1]) >= min_spacing:
            filtered.append(point)
    return np.asarray(filtered, dtype=float)


def remove_duplicate_xy_points(points, min_xy_spacing):
    if len(points) <= 1:
        return points

    filtered = [points[0]]
    min_xy_spacing = max(min_xy_spacing, 1e-5)
    for point in points[1:]:
        if np.linalg.norm(point[:2] - filtered[-1][:2]) >= min_xy_spacing:
            filtered.append(point)
            continue

        # XY 几乎重合时，保留更新后的 Z，避免二维样条出现非严格递增参数。
        filtered[-1] = point
    return np.asarray(filtered, dtype=float)


def remove_isolated_spikes(points, spike_jump, reconnect_dist):
    if len(points) < 3:
        return points

    filtered = [points[0]]
    for i in range(1, len(points) - 1):
        prev_pt = points[i - 1]
        curr_pt = points[i]
        next_pt = points[i + 1]

        prev_dist = np.linalg.norm(curr_pt - prev_pt)
        next_dist = np.linalg.norm(next_pt - curr_pt)
        reconnect = np.linalg.norm(next_pt - prev_pt)
        if prev_dist > spike_jump and next_dist > spike_jump and reconnect < reconnect_dist:
            continue

        filtered.append(curr_pt)

    filtered.append(points[-1])
    return np.asarray(filtered, dtype=float)


def cumulative_xy(points):
    if len(points) == 0:
        return np.empty((0,), dtype=float)
    deltas = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(deltas)))


def compute_xy_curvature(points):
    curvature = np.zeros(len(points), dtype=float)
    if len(points) < 3:
        return curvature

    for i in range(1, len(points) - 1):
        p0 = points[i - 1, :2]
        p1 = points[i, :2]
        p2 = points[i + 1, :2]

        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            continue

        area2 = abs(
            (p1[0] - p0[0]) * (p2[1] - p0[1]) -
            (p1[1] - p0[1]) * (p2[0] - p0[0])
        )
        curvature[i] = (2.0 * area2) / (a * b * c)

    if len(points) >= 2:
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]
    return curvature


def compute_corner_weights(points, corner_gain):
    if len(points) < 3:
        return np.ones(len(points), dtype=float)

    weights = np.ones(len(points), dtype=float)
    for i in range(1, len(points) - 1):
        v_prev = points[i, :2] - points[i - 1, :2]
        v_next = points[i + 1, :2] - points[i, :2]
        n_prev = np.linalg.norm(v_prev)
        n_next = np.linalg.norm(v_next)
        if n_prev < 1e-6 or n_next < 1e-6:
            continue
        cos_angle = np.dot(v_prev, v_next) / (n_prev * n_next)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        turn_angle = np.arccos(cos_angle)
        weights[i] = 1.0 + corner_gain * (turn_angle / np.pi)

    weights[0] = max(weights[0], 3.0)
    weights[-1] = max(weights[-1], 3.0)
    return weights


def moving_average(values, window):
    window = max(int(window), 1)
    if window <= 1 or len(values) < 3:
        return values.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


def smooth_xy_path(points, smoothing, num_points, corner_gain):
    if len(points) < 4:
        return points[:, :2].copy(), cumulative_xy(points)

    arc = cumulative_xy(points)
    keep = np.ones(len(points), dtype=bool)
    keep[1:] = np.diff(arc) > 1e-8
    points = points[keep]
    arc = arc[keep]
    if len(points) < 4:
        return points[:, :2].copy(), arc

    total_length = arc[-1]
    if total_length < 1e-6:
        return points[:, :2].copy(), arc

    u = arc / total_length
    weights = compute_corner_weights(points, corner_gain)
    smooth_term = max(smoothing, 0.0) * len(points)
    k = min(3, len(points) - 1)

    tck, _ = splprep(
        [points[:, 0], points[:, 1]],
        u=u,
        w=weights,
        s=smooth_term,
        k=k,
    )

    sample_count = max(int(num_points), len(points))
    u_fine = np.linspace(0.0, 1.0, sample_count)
    x_new, y_new = splev(u_fine, tck)
    xy = np.column_stack((x_new, y_new))
    return xy, u_fine * total_length


def resample_distances(points, base_spacing, min_spacing, curvature_gain):
    if len(points) < 2:
        return np.array([0.0], dtype=float)

    arc = cumulative_xy(points)
    curvature = compute_xy_curvature(points)
    distances = [0.0]

    for i in range(len(points) - 1):
        seg_len = arc[i + 1] - arc[i]
        if seg_len < 1e-6:
            continue
        local_curvature = max(curvature[i], curvature[i + 1])
        local_spacing = base_spacing / (1.0 + curvature_gain * local_curvature)
        local_spacing = np.clip(local_spacing, min_spacing, base_spacing)
        subdivisions = max(int(np.ceil(seg_len / local_spacing)), 1)
        for j in range(1, subdivisions + 1):
            distances.append(arc[i] + seg_len * j / subdivisions)

    return np.asarray(distances, dtype=float)


def enforce_z_rate_limit(z_values, distances, max_slope):
    if len(z_values) < 2 or max_slope <= 0.0:
        return z_values

    limited = z_values.copy()
    for i in range(1, len(limited)):
        ds = max(distances[i] - distances[i - 1], 1e-6)
        dz_max = max_slope * ds
        limited[i] = np.clip(limited[i], limited[i - 1] - dz_max, limited[i - 1] + dz_max)

    for i in range(len(limited) - 2, -1, -1):
        ds = max(distances[i + 1] - distances[i], 1e-6)
        dz_max = max_slope * ds
        limited[i] = np.clip(limited[i], limited[i + 1] - dz_max, limited[i + 1] + dz_max)

    return limited


def rebuild_path(points, smoothing, num_points, base_spacing, min_spacing, curvature_gain,
                 corner_gain, z_window, max_z_slope):
    clean_arc = cumulative_xy(points)
    if len(clean_arc) < 2 or clean_arc[-1] < 1e-6:
        return points

    sample_distances = resample_distances(points, base_spacing, min_spacing, curvature_gain)
    xy_smooth, smooth_arc = smooth_xy_path(points, smoothing, num_points, corner_gain)

    x = np.interp(sample_distances, smooth_arc, xy_smooth[:, 0])
    y = np.interp(sample_distances, smooth_arc, xy_smooth[:, 1])

    z_filtered = moving_average(points[:, 2], z_window)
    z = np.interp(sample_distances, clean_arc, z_filtered)
    z = enforce_z_rate_limit(z, sample_distances, max_z_slope)

    return np.column_stack((x, y, z))


def rebuild_path_linear(points, base_spacing, min_spacing, z_window, max_z_slope):
    clean_arc = cumulative_xy(points)
    if len(clean_arc) < 2 or clean_arc[-1] < 1e-6:
        return points

    spacing = max(min(base_spacing, clean_arc[-1]), min_spacing, 1e-3)
    sample_distances = np.arange(0.0, clean_arc[-1], spacing, dtype=float)
    if sample_distances.size == 0 or abs(sample_distances[-1] - clean_arc[-1]) > 1e-6:
        sample_distances = np.append(sample_distances, clean_arc[-1])

    z_filtered = moving_average(points[:, 2], z_window)
    x = np.interp(sample_distances, clean_arc, points[:, 0])
    y = np.interp(sample_distances, clean_arc, points[:, 1])
    z = np.interp(sample_distances, clean_arc, z_filtered)
    z = enforce_z_rate_limit(z, sample_distances, max_z_slope)
    return np.column_stack((x, y, z))


def rebuild_path_anchor_interp(points, base_spacing, min_spacing, z_window, max_z_slope):
    clean_arc = cumulative_xy(points)
    if len(clean_arc) < 2 or clean_arc[-1] < 1e-6:
        return points

    keep = np.ones(len(points), dtype=bool)
    keep[1:] = np.diff(clean_arc) > 1e-8
    points = points[keep]
    clean_arc = clean_arc[keep]
    if len(clean_arc) < 2 or clean_arc[-1] < 1e-6:
        return points

    spacing = max(min(base_spacing, clean_arc[-1]), min_spacing, 1e-3)
    sample_distances = np.arange(0.0, clean_arc[-1], spacing, dtype=float)
    if sample_distances.size == 0 or abs(sample_distances[-1] - clean_arc[-1]) > 1e-6:
        sample_distances = np.append(sample_distances, clean_arc[-1])

    x_interp = PchipInterpolator(clean_arc, points[:, 0])
    y_interp = PchipInterpolator(clean_arc, points[:, 1])
    z_filtered = moving_average(points[:, 2], z_window)
    z_interp = PchipInterpolator(clean_arc, z_filtered)

    x = x_interp(sample_distances)
    y = y_interp(sample_distances)
    z = z_interp(sample_distances)
    z = enforce_z_rate_limit(z, sample_distances, max_z_slope)
    return np.column_stack((x, y, z))


def build_path(points, frame_id):
    path = Path()
    path.header.frame_id = frame_id

    for point in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return path


def resolve_csv_path(csv_file):
    csv_path = FsPath(csv_file)
    if csv_path.is_absolute():
        return csv_path
    return (FsPath(__file__).resolve().parent / csv_file).resolve()


def publish_saved_path():
    rospy.init_node("path_loader_node", anonymous=True)

    default_csv = (FsPath(__file__).resolve().parent.parent / "path" / "rule_based_strict_route_anchors.csv").resolve()
    csv_file = rospy.get_param("~csv_path", str(default_csv))
    path_mode = rospy.get_param("~path_mode", "auto").strip().lower()
    smoothing = rospy.get_param("~smoothing", 0.18)
    num_points = rospy.get_param("~num_points", 1500)
    min_spacing = rospy.get_param("~min_spacing", 0.05)
    resample_spacing = rospy.get_param("~resample_spacing", 0.30)
    min_curve_spacing = rospy.get_param("~min_curve_spacing", 0.10)
    curvature_gain = rospy.get_param("~curvature_gain", 6.0)
    spike_jump = rospy.get_param("~spike_jump", 4.0)
    reconnect_dist = rospy.get_param("~reconnect_dist", 1.0)
    corner_gain = rospy.get_param("~corner_gain", 1.0)
    z_smooth_window = rospy.get_param("~z_smooth_window", 9)
    max_z_slope = rospy.get_param("~max_z_slope", 0.35)

    raw_pub = rospy.Publisher("/drone_1/raw_path", Path, queue_size=1, latch=True)
    path_pub = rospy.Publisher("/drone_1/saved_path", Path, queue_size=1, latch=True)

    csv_path = resolve_csv_path(csv_file)
    rospy.loginfo("正在从 %s 读取并重建控制轨迹...", str(csv_path))

    try:
        with open(str(csv_path), "r", encoding="utf-8-sig", newline="") as f:
            rows = list(csv.reader(f))

        if path_mode == "auto":
            path_mode = infer_path_mode(str(csv_path), rows)

        raw_points = load_points(str(csv_path))
        if len(raw_points) < 4:
            rospy.logerr("CSV 点数太少，无法构建路径。")
            return

        clean_points = remove_duplicate_points(raw_points, min_spacing)
        clean_points = remove_isolated_spikes(clean_points, spike_jump, reconnect_dist)
        clean_points = remove_duplicate_xy_points(clean_points, min_spacing * 0.5)
        if path_mode in ("direct", "raw", "passthrough"):
            rebuilt_points = clean_points.copy()
        elif path_mode == "linear":
            rebuilt_points = rebuild_path_linear(
                clean_points,
                resample_spacing,
                min_curve_spacing,
                z_smooth_window,
                max_z_slope,
            )
        elif path_mode in ("anchor_interp", "interp", "pchip"):
            rebuilt_points = rebuild_path_anchor_interp(
                clean_points,
                resample_spacing,
                min_curve_spacing,
                z_smooth_window,
                max_z_slope,
            )
        else:
            rebuilt_points = rebuild_path(
                clean_points,
                smoothing,
                num_points,
                resample_spacing,
                min_curve_spacing,
                curvature_gain,
                corner_gain,
                z_smooth_window,
                max_z_slope,
            )

        raw_path = build_path(clean_points, "world")
        rebuilt_path = build_path(rebuilt_points, "world")

        rospy.loginfo(
            "路径处理完成: mode=%s raw=%d clean=%d smooth=%d | smoothing=%.3f spacing=%.2f min_curve_spacing=%.2f curvature_gain=%.2f z_window=%d max_z_slope=%.2f",
            path_mode,
            len(raw_points),
            len(clean_points),
            len(rebuilt_points),
            smoothing,
            resample_spacing,
            min_curve_spacing,
            curvature_gain,
            int(z_smooth_window),
            max_z_slope,
        )

        now = rospy.Time.now()
        raw_path.header.stamp = now
        rebuilt_path.header.stamp = now
        raw_pub.publish(raw_path)
        path_pub.publish(rebuilt_path)
        rospy.loginfo("路径已通过 latched topic 发布完成，等待节点使用。")
        rospy.spin()

    except Exception as exc:
        rospy.logerr("处理路径出错: %s", str(exc))


if __name__ == "__main__":
    publish_saved_path()
