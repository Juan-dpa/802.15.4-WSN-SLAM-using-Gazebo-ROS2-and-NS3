#!/usr/bin/env python3
import csv
import json
import math
import re
from pathlib import Path as FilePath

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rssi_slam_core import RssiRangeISAM, RssiSlamConfig
except ImportError:
    import sys

    sys.path.append(str(FilePath(__file__).resolve().parent))
    from rssi_slam_core import RssiRangeISAM, RssiSlamConfig


def resolve_existing_path(path):
    raw = FilePath(str(path)).expanduser()
    candidates = [
        raw,
        FilePath.cwd() / raw,
        FilePath(__file__).resolve().parent / raw,
        FilePath(__file__).resolve().parent / ".." / raw,
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())
    return str(raw)


def seconds_to_stamp(seconds):
    seconds = max(0.0, float(seconds))
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def make_point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def color_for_mac(mac, alpha=1.0):
    palette = [
        (0.121, 0.466, 0.705),
        (1.000, 0.498, 0.054),
        (0.172, 0.627, 0.172),
        (0.839, 0.153, 0.157),
        (0.580, 0.404, 0.741),
        (0.549, 0.337, 0.294),
    ]
    idx = sum(ord(char) for char in mac) % len(palette)
    r, g, b = palette[idx]
    return r, g, b, alpha


def error_color(error_m, good_m, bad_m, alpha=1.0):
    if bad_m <= good_m:
        bad_m = good_m + 1e-6
    t = min(max((error_m - good_m) / (bad_m - good_m), 0.0), 1.0)
    green = (0.172, 0.627, 0.172)
    yellow = (1.000, 0.780, 0.120)
    red = (0.839, 0.153, 0.157)
    if t < 0.5:
        local = t / 0.5
        start, end = green, yellow
    else:
        local = (t - 0.5) / 0.5
        start, end = yellow, red
    color = tuple(start[i] * (1.0 - local) + end[i] * local for i in range(3))
    return color[0], color[1], color[2], alpha


def read_odom_csv(path):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if not raw or raw[0].strip().startswith("#"):
                continue
            if len(raw) < 4:
                continue
            try:
                rows.append((float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3])))
            except ValueError:
                continue
    rows.sort(key=lambda row: row[0])
    if not rows:
        raise ValueError(f"No odometry rows found in {path}")
    return rows


def read_rssi_csv(path):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if not raw or raw[0].strip().startswith("#"):
                continue
            if len(raw) < 4:
                continue
            try:
                rows.append((float(raw[0]), raw[1].strip(), float(raw[3])))
            except ValueError:
                continue
    rows.sort(key=lambda row: row[0])
    if not rows:
        raise ValueError(f"No RSSI rows found in {path}")
    return rows


def ordered_macs_from_rssi_rows(rows):
    macs = []
    seen = set()
    for _, mac, _ in rows:
        if mac not in seen:
            seen.add(mac)
            macs.append(mac)
    return macs


def read_positions_csv(path, macs, position_start_row):
    if not path:
        return {}
    resolved_path = resolve_existing_path(path)
    if not FilePath(resolved_path).exists():
        return {}

    rows = []
    with open(resolved_path, newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if not raw or raw[0].strip().startswith("#"):
                continue
            if len(raw) < 2:
                continue
            try:
                z = float(raw[2]) if len(raw) > 2 else 0.0
                rows.append((float(raw[0]), float(raw[1]), z))
            except ValueError:
                continue

    true_positions = {}
    for idx, mac in enumerate(macs):
        row_idx = int(position_start_row) + idx
        if row_idx >= len(rows):
            break
        true_positions[mac] = rows[row_idx]
    return true_positions


class RssiSlamReplayNode(Node):
    def __init__(self):
        super().__init__("rssi_slam_replay_node")
        self.declare_parameter("odom_csv", "Inputs/trajectory_odom-synthetic.csv")
        self.declare_parameter("rssi_csv", "Outputs/slam_dataset_run1-synthetic.csv")
        self.declare_parameter("positions_csv", "Inputs/positions-synthetic.csv")
        self.declare_parameter("position_start_row", 1)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_frame_id", "base_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("odom_rows_per_tick", 1)
        self.declare_parameter("loop", False)
        self.declare_parameter("use_csv_time_stamps", False)
        self.declare_parameter("publish_clock", False)

        self.declare_parameter("ap_marker_z", 1.5)
        self.declare_parameter("ap_marker_scale", 1.5)
        self.declare_parameter("true_ap_marker_z", 1.5)
        self.declare_parameter("true_ap_marker_scale", 1.0)
        self.declare_parameter("show_true_ap_positions", True)
        self.declare_parameter("show_ap_error_vectors", True)
        self.declare_parameter("mapping_error_good_m", 3.0)
        self.declare_parameter("mapping_error_bad_m", 15.0)
        self.declare_parameter("heatmap_z", 0.05)

        self.declare_parameter("path_loss_exponent", 4.4)
        self.declare_parameter("rssi_sigma_db", 6.0)
        self.declare_parameter("min_landmark_init_baseline_m", 60.0)
        self.declare_parameter("heatmap_resolution_m", 2.0)
        self.declare_parameter("heatmap_decay", 0.995)
        self.declare_parameter("heatmap_min_x", 0.0)
        self.declare_parameter("heatmap_max_x", 100.0)
        self.declare_parameter("heatmap_min_y", 0.0)
        self.declare_parameter("heatmap_max_y", 100.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.robot_frame_id = self.get_parameter("robot_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.odom_csv = resolve_existing_path(self.get_parameter("odom_csv").value)
        self.rssi_csv = resolve_existing_path(self.get_parameter("rssi_csv").value)
        self.positions_csv = resolve_existing_path(self.get_parameter("positions_csv").value)
        self.odom_rows = read_odom_csv(self.odom_csv)
        self.rssi_rows = read_rssi_csv(self.rssi_csv)
        self.rssi_macs = ordered_macs_from_rssi_rows(self.rssi_rows)
        self.true_ap_positions = read_positions_csv(
            self.positions_csv,
            self.rssi_macs,
            self.get_parameter("position_start_row").value,
        )
        self.odom_index = 0
        self.rssi_index = 0
        self.path_msg = RosPath()
        self.path_msg.header.frame_id = self.frame_id

        cfg = RssiSlamConfig(
            path_loss_exponent=self.get_parameter("path_loss_exponent").value,
            rssi_sigma_db=self.get_parameter("rssi_sigma_db").value,
            min_landmark_init_baseline_m=self.get_parameter("min_landmark_init_baseline_m").value,
            heatmap_resolution_m=self.get_parameter("heatmap_resolution_m").value,
            heatmap_decay=self.get_parameter("heatmap_decay").value,
            heatmap_min_x=self.get_parameter("heatmap_min_x").value,
            heatmap_max_x=self.get_parameter("heatmap_max_x").value,
            heatmap_min_y=self.get_parameter("heatmap_min_y").value,
            heatmap_max_y=self.get_parameter("heatmap_max_y").value,
        )
        self.slam = RssiRangeISAM(cfg)
        self.latest_snapshot = self.slam.snapshot()
        self.current_pose = None

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom_replay", 10)
        self.path_pub = self.create_publisher(RosPath, "/path_replay", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ap_markers", 10)
        self.heatmap_pub = self.create_publisher(PointCloud2, "/rssi_heatmap", 10)
        self.heatmap_pubs_by_mac = {}
        self.heatmap_topics_by_mac = {}
        self.status_pub = self.create_publisher(String, "/rssi_slam_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(float(self.get_parameter("publish_period_s").value), self.on_timer)
        self.get_logger().info(
            "RSSI SLAM replay ready: "
            f"odom_rows={len(self.odom_rows)}, rssi_rows={len(self.rssi_rows)}, "
            f"odom_csv={self.odom_csv}, rssi_csv={self.rssi_csv}"
        )
        if self.true_ap_positions:
            self.get_logger().info(f"Loaded true AP positions for {len(self.true_ap_positions)} MACs from {self.positions_csv}")
        else:
            self.get_logger().warn("No true AP positions loaded; AP mapping error overlay disabled.")

    def on_timer(self):
        rows_per_tick = max(1, int(self.get_parameter("odom_rows_per_tick").value))
        for _ in range(rows_per_tick):
            if self.odom_index >= len(self.odom_rows):
                if self.get_parameter("loop").value:
                    self.reset_replay()
                else:
                    self.publish_visuals()
                    return
            self.step_once()
        self.publish_visuals()

    def reset_replay(self):
        self.odom_index = 0
        self.rssi_index = 0
        self.path_msg.poses.clear()
        self.slam = RssiRangeISAM(self.slam.cfg)
        self.latest_snapshot = self.slam.snapshot()
        self.current_pose = None

    def step_once(self):
        time_s, x, y, theta = self.odom_rows[self.odom_index]
        while self.rssi_index < len(self.rssi_rows) and self.rssi_rows[self.rssi_index][0] <= time_s:
            rssi_time, mac, rssi_dbm = self.rssi_rows[self.rssi_index]
            self.latest_snapshot = self.slam.add_rssi(rssi_time, mac, rssi_dbm)
            self.rssi_index += 1

        self.latest_snapshot = self.slam.add_odometry(time_s, x, y, theta)
        self.current_pose = (time_s, x, y, theta)
        self.append_path_pose(time_s, x, y, theta)
        self.odom_index += 1

    def append_path_pose(self, time_s, x, y, theta):
        stamp = self.current_stamp(time_s)
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)

    def publish_visuals(self):
        if self.current_pose is None:
            return
        time_s, x, y, theta = self.current_pose
        stamp = self.current_stamp(time_s)
        if self.get_parameter("publish_clock").value:
            self.clock_pub.publish(Clock(clock=seconds_to_stamp(time_s)))
        self.publish_tf(stamp, x, y, theta)
        self.odom_pub.publish(self.make_odom(stamp, x, y, theta))
        self.path_pub.publish(self.path_msg)
        self.marker_pub.publish(self.make_ap_markers(self.latest_snapshot, stamp))
        self.heatmap_pub.publish(self.make_heatmap_cloud(self.latest_snapshot["heatmap_points"], stamp))
        self.publish_heatmaps_by_mac(self.latest_snapshot, stamp)
        self.status_pub.publish(String(data=json.dumps(self.make_status(self.latest_snapshot), sort_keys=True)))

    def current_stamp(self, csv_time_s):
        if self.get_parameter("use_csv_time_stamps").value:
            return seconds_to_stamp(csv_time_s)
        return self.get_clock().now().to_msg()

    def publish_tf(self, stamp, x, y, theta):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.robot_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(theta)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def make_odom(self, stamp, x, y, theta):
        msg = Odometry()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.child_frame_id = self.robot_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg

    def make_ap_markers(self, snapshot, stamp):
        markers = MarkerArray()
        marker_z = float(self.get_parameter("ap_marker_z").value)
        marker_scale = float(self.get_parameter("ap_marker_scale").value)

        if self.get_parameter("show_true_ap_positions").value:
            self.add_true_ap_markers(markers, stamp)

        for landmark in snapshot["landmarks"]:
            marker_id = int(landmark["id"])
            r, g, b, a = color_for_mac(landmark["mac"], 0.95)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "ap_estimates"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = landmark["x"]
            sphere.pose.position.y = landmark["y"]
            sphere.pose.position.z = marker_z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = marker_scale
            sphere.scale.y = marker_scale
            sphere.scale.z = marker_scale
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = a
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "ap_labels"
            label.id = 10000 + marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = landmark["x"]
            label.pose.position.y = landmark["y"]
            label.pose.position.z = marker_z + marker_scale
            label.pose.orientation.w = 1.0
            label.scale.z = 0.8
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = landmark["mac"]
            markers.markers.append(label)

            error = self.mapping_error_for_landmark(landmark)
            if error is not None:
                self.add_mapping_error_markers(markers, landmark, error, stamp)

        return markers

    def add_true_ap_markers(self, markers, stamp):
        true_z = float(self.get_parameter("true_ap_marker_z").value)
        true_scale = float(self.get_parameter("true_ap_marker_scale").value)
        for idx, mac in enumerate(self.rssi_macs, start=1):
            if mac not in self.true_ap_positions:
                continue
            true_x, true_y, _ = self.true_ap_positions[mac]
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "ap_true_positions"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = true_x
            marker.pose.position.y = true_y
            marker.pose.position.z = true_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = true_scale
            marker.scale.y = true_scale
            marker.scale.z = true_scale
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "ap_true_labels"
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = true_x
            label.pose.position.y = true_y
            label.pose.position.z = true_z + true_scale
            label.pose.orientation.w = 1.0
            label.scale.z = 0.65
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"true {mac}"
            markers.markers.append(label)

    def mapping_error_for_landmark(self, landmark):
        mac = landmark["mac"]
        if mac not in self.true_ap_positions:
            return None
        true_x, true_y, true_z = self.true_ap_positions[mac]
        dx = landmark["x"] - true_x
        dy = landmark["y"] - true_y
        error_m = math.hypot(dx, dy)
        return {
            "mac": mac,
            "true_x": true_x,
            "true_y": true_y,
            "true_z": true_z,
            "dx": dx,
            "dy": dy,
            "error_m": error_m,
        }

    def mapping_errors(self, snapshot):
        errors = []
        for landmark in snapshot["landmarks"]:
            error = self.mapping_error_for_landmark(landmark)
            if error is None:
                continue
            errors.append(
                {
                    "mac": error["mac"],
                    "estimated": [landmark["x"], landmark["y"]],
                    "true": [error["true_x"], error["true_y"]],
                    "dx_m": error["dx"],
                    "dy_m": error["dy"],
                    "error_m": error["error_m"],
                }
            )
        return errors

    @staticmethod
    def mapping_error_summary(errors):
        if not errors:
            return None
        values = [item["error_m"] for item in errors]
        rmse = math.sqrt(sum(value * value for value in values) / len(values))
        return {
            "count": len(values),
            "mean_m": sum(values) / len(values),
            "rmse_m": rmse,
            "max_m": max(values),
        }

    def add_mapping_error_markers(self, markers, landmark, error, stamp):
        marker_id = int(landmark["id"])
        true_z = float(self.get_parameter("true_ap_marker_z").value)
        est_z = float(self.get_parameter("ap_marker_z").value)
        good_m = float(self.get_parameter("mapping_error_good_m").value)
        bad_m = float(self.get_parameter("mapping_error_bad_m").value)
        r, g, b, a = error_color(error["error_m"], good_m, bad_m, 0.95)

        if self.get_parameter("show_ap_error_vectors").value:
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = stamp
            arrow.ns = "ap_mapping_error_vectors"
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                make_point(error["true_x"], error["true_y"], true_z),
                make_point(landmark["x"], landmark["y"], est_z),
            ]
            arrow.scale.x = 0.25
            arrow.scale.y = 0.75
            arrow.scale.z = 0.75
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = a
            markers.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "ap_mapping_error_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = 0.5 * (error["true_x"] + landmark["x"])
            label.pose.position.y = 0.5 * (error["true_y"] + landmark["y"])
            label.pose.position.z = max(true_z, est_z) + 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.75
            label.color.r = r
            label.color.g = g
            label.color.b = b
            label.color.a = 1.0
            label.text = f"{error['mac']} err={error['error_m']:.2f} m"
            markers.markers.append(label)

    def publish_heatmaps_by_mac(self, snapshot, stamp):
        for mac, points in snapshot["heatmap_points_by_mac"].items():
            publisher = self.heatmap_publisher_for_mac(mac)
            publisher.publish(self.make_heatmap_cloud(points, stamp))

    def heatmap_publisher_for_mac(self, mac):
        if mac not in self.heatmap_pubs_by_mac:
            suffix = self.topic_suffix_from_mac(mac)
            topic = f"/rssi_heatmap/{suffix}"
            self.heatmap_pubs_by_mac[mac] = self.create_publisher(PointCloud2, topic, 10)
            self.heatmap_topics_by_mac[mac] = topic
            self.get_logger().info(f"Publishing AP-specific heatmap for {mac} on {topic}")
        return self.heatmap_pubs_by_mac[mac]

    @staticmethod
    def topic_suffix_from_mac(mac):
        safe = re.sub(r"[^A-Za-z0-9_]", "_", mac.strip())
        return f"ap_{safe}"

    def make_heatmap_cloud(self, heatmap_points, stamp):
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = stamp
        z = float(self.get_parameter("heatmap_z").value)
        points = [(x, y, z, intensity) for x, y, intensity in heatmap_points]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def make_status(self, snapshot):
        errors = self.mapping_errors(snapshot)
        return {
            "csv_time_s": snapshot["time_s"],
            "initialized": snapshot["initialized"],
            "update_index": snapshot["update_index"],
            "pose_key": snapshot["pose_key"],
            "landmarks": snapshot["landmarks"],
            "ap_mapping_errors": errors,
            "ap_mapping_error_summary": self.mapping_error_summary(errors),
            "heatmap_topics_by_mac": self.heatmap_topics_by_mac,
            "pending_rssi": snapshot["pending_rssi"],
        }


def main(args=None):
    rclpy.init(args=args)
    node = RssiSlamReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
