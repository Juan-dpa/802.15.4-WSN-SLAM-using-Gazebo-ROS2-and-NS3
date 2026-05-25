#!/usr/bin/env python3
import argparse
import csv
import math
import os
from collections import OrderedDict

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Point2, Pose2


NM = gtsam.noiseModel


TX_POWER_DBM = 0.0
PATH_LOSS_1M_DB = 22.0
PATH_LOSS_EXPONENT = 4.4
REFERENCE_DISTANCE_M = 1.0
MIN_RANGE_M = 0.5
MAX_RANGE_M = 150.0

RSSI_SIGMA_DB = 6.0
MIN_RANGE_SIGMA_M = 2.0
USE_ROBUST_RANGE_LOSS = True
HUBER_K = 1.345

PRIOR_SIGMA_XY_M = 1.0
PRIOR_SIGMA_THETA_RAD = math.pi
ODOM_SIGMA_X_M = 0.05
ODOM_SIGMA_Y_M = 0.01
ODOM_SIGMA_THETA_RAD = 0.1
ODOM_POSE_PRIOR_SIGMA_XY_M = 0.25
ODOM_POSE_PRIOR_SIGMA_THETA_RAD = 0.2

MIN_INITIAL_RANGES = 30
RANGES_PER_UPDATE = 10
MIN_LANDMARK_INIT_RANGES = 50
MIN_LANDMARK_INIT_BASELINE_M = 60.0
MIN_LANDMARK_INIT_RSSI_SPAN_DB = 15.0
PATH_LOSS_GRID_SIZE = 31
PATH_LOSS_GRID_REFINEMENTS = 4
PATH_LOSS_SEARCH_MARGIN_M = 25.0
COMPRESSED_RANGE_BASELINE_RATIO = 0.85
ISAM_FACTORIZATION = "QR"
LIVE_PLOT_UPDATE_STRIDE = 5

POSITION_START_ROW = 1


def read_numeric_csv(path, expected_cols):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        for raw_row in reader:
            if not raw_row:
                continue
            if raw_row[0].strip().startswith("#"):
                continue
            if len(raw_row) < expected_cols:
                continue
            rows.append([float(value.strip()) for value in raw_row[:expected_cols]])
    return rows


def load_absolute_odometry(path):
    rows = read_numeric_csv(path, 4)
    if len(rows) < 2:
        raise ValueError("Odometry file must contain at least two poses.")

    times = [row[0] for row in rows]
    poses = [Pose2(row[1], row[2], row[3]) for row in rows]
    relative_odometry = OrderedDict()

    for idx in range(1, len(poses)):
        prev_pose = poses[idx - 1]
        curr_pose = poses[idx]
        relative_odometry[times[idx]] = prev_pose.between(curr_pose)

    return times, poses, relative_odometry


def load_ap_positions(path, macs, position_start_row):
    if path is None:
        return {}
    rows = read_numeric_csv(path, 3)
    if len(rows) < position_start_row + len(macs):
        raise ValueError("positions file does not contain enough rows for selected MAC mapping.")

    ap_rows = rows[position_start_row : position_start_row + len(macs)]
    return {mac: Point2(ap_rows[idx][0], ap_rows[idx][1]) for idx, mac in enumerate(macs)}


def load_ground_truth_trajectory(path):
    if path is None:
        return None
    rows = read_numeric_csv(path, 4)
    if not rows:
        return None
    return np.array([[row[1], row[2]] for row in rows])


def rssi_to_range_m(rssi_dbm, tx_power_dbm, path_loss_1m_db, path_loss_exponent, reference_distance_m):
    rssi_at_reference = tx_power_dbm - path_loss_1m_db
    exponent = (rssi_at_reference - rssi_dbm) / (10.0 * path_loss_exponent)
    return reference_distance_m * (10.0 ** exponent)


def bounded_rssi_to_range_m(args, rssi_dbm, path_loss_1m_db):
    range_m = rssi_to_range_m(
        rssi_dbm,
        args.tx_power_dbm,
        path_loss_1m_db,
        args.path_loss_exponent,
        args.reference_distance_m,
    )
    return min(max(range_m, args.min_range_m), args.max_range_m)


def infer_macs_from_rssi(path):
    macs = []
    seen = set()
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            mac = row["Src_MAC"].strip()
            if mac not in seen:
                seen.add(mac)
                macs.append(mac)
    return macs


def load_rssi_measurements(path, mac_to_landmark_id):
    measurements = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            mac = row["Src_MAC"].strip()
            if mac not in mac_to_landmark_id:
                continue

            time_s = float(row["Time_s"])
            rssi_dbm = float(row["RSSI_dBm"])
            measurements.append((time_s, mac_to_landmark_id[mac], mac, rssi_dbm))

    measurements.sort(key=lambda item: item[0])
    return measurements


def print_range_stats(measurements, landmark_id_to_mac, args):
    print("Initial RSSI-derived range summary:")
    for landmark_id, mac in sorted(landmark_id_to_mac.items()):
        ranges = np.array(
            [
                bounded_rssi_to_range_m(args, item[3], args.path_loss_1m_db)
                for item in measurements
                if item[1] == landmark_id
            ],
            dtype=float,
        )
        if len(ranges) == 0:
            continue
        print(
            f"  {mac}: min={np.min(ranges):.2f} m, "
            f"median={np.median(ranges):.2f} m, max={np.max(ranges):.2f} m"
        )


def estimate_landmark_from_past_samples(samples, args):
    if len(samples) < MIN_LANDMARK_INIT_RANGES:
        return None, None

    xs = np.array([sample["x"] for sample in samples], dtype=float)
    ys = np.array([sample["y"] for sample in samples], dtype=float)
    rssis = np.array([sample["rssi"] for sample in samples], dtype=float)
    baseline = max(float(np.ptp(xs)), float(np.ptp(ys)))
    if baseline < MIN_LANDMARK_INIT_BASELINE_M:
        return None, None
    if float(np.ptp(rssis)) < MIN_LANDMARK_INIT_RSSI_SPAN_DB:
        return None, None

    default_ranges = np.array(
        [bounded_rssi_to_range_m(args, rssi, args.path_loss_1m_db) for rssi in rssis],
        dtype=float,
    )
    default_point = estimate_landmark_from_known_ranges(xs, ys, default_ranges)
    if default_point is not None and float(np.max(default_ranges)) >= COMPRESSED_RANGE_BASELINE_RATIO * baseline:
        return default_point, args.path_loss_1m_db

    min_x = float(np.min(xs) - PATH_LOSS_SEARCH_MARGIN_M)
    max_x = float(np.max(xs) + PATH_LOSS_SEARCH_MARGIN_M)
    min_y = float(np.min(ys) - PATH_LOSS_SEARCH_MARGIN_M)
    max_y = float(np.max(ys) + PATH_LOSS_SEARCH_MARGIN_M)
    best = None

    for _ in range(PATH_LOSS_GRID_REFINEMENTS):
        grid_x = np.linspace(min_x, max_x, PATH_LOSS_GRID_SIZE)
        grid_y = np.linspace(min_y, max_y, PATH_LOSS_GRID_SIZE)
        for candidate_x in grid_x:
            dx = xs - candidate_x
            for candidate_y in grid_y:
                distances = np.hypot(dx, ys - candidate_y)
                distances = np.maximum(distances, args.reference_distance_m)
                log_distances = np.log10(distances / args.reference_distance_m)
                path_loss_samples = args.tx_power_dbm - rssis - 10.0 * args.path_loss_exponent * log_distances
                path_loss_1m_db = float(np.median(path_loss_samples))
                predicted_rssi = args.tx_power_dbm - path_loss_1m_db - 10.0 * args.path_loss_exponent * log_distances
                residuals = predicted_rssi - rssis
                cost = float(np.mean(residuals ** 2))
                if best is None or cost < best[0]:
                    best = (cost, float(candidate_x), float(candidate_y), path_loss_1m_db)

        _, center_x, center_y, _ = best
        half_width = max((max_x - min_x) / (PATH_LOSS_GRID_SIZE - 1), 0.5)
        half_height = max((max_y - min_y) / (PATH_LOSS_GRID_SIZE - 1), 0.5)
        min_x = center_x - half_width
        max_x = center_x + half_width
        min_y = center_y - half_height
        max_y = center_y + half_height

    _, landmark_x, landmark_y, path_loss_1m_db = best
    return Point2(landmark_x, landmark_y), path_loss_1m_db


def estimate_landmark_from_known_ranges(xs, ys, ranges):
    x0 = xs[0]
    y0 = ys[0]
    r0 = ranges[0]
    a = np.column_stack((2.0 * (x0 - xs[1:]), 2.0 * (y0 - ys[1:])))
    b = ranges[1:] ** 2 - r0 ** 2 - xs[1:] ** 2 + x0 ** 2 - ys[1:] ** 2 + y0 ** 2
    if np.linalg.matrix_rank(a) < 2:
        return None
    solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    return Point2(float(solution[0]), float(solution[1]))


def range_sigma_m(args, measured_range):
    coeff = math.log(10.0) / (10.0 * args.path_loss_exponent)
    return max(args.min_range_sigma_m, coeff * measured_range * args.rssi_sigma_db)


def huber_weight(normalized_residual, huber_k):
    abs_residual = abs(normalized_residual)
    if abs_residual <= huber_k:
        return 1.0
    return huber_k / abs_residual


def make_range_noise(args, measured_range):
    sigma = range_sigma_m(args, measured_range)
    gaussian = NM.Isotropic.Sigma(1, sigma)
    if args.robust:
        return NM.Robust.Create(NM.mEstimator.Huber.Create(args.huber_k), gaussian)
    return gaussian


def extract_pose_rows(result):
    poses = gtsam.utilities.allPose2s(result)
    pose_rows = []
    for key in poses.keys():
        pose = poses.atPose2(key)
        pose_rows.append((key, pose.x(), pose.y(), pose.theta()))
    pose_rows.sort(key=lambda row: row[0])
    return pose_rows


def extract_landmark_rows(result, landmark_ids):
    landmark_rows = []
    for landmark_id in sorted(landmark_ids):
        key = gtsam.symbol("L", landmark_id)
        try:
            point = result.atPoint2(key)
        except RuntimeError:
            continue
        landmark_rows.append((landmark_id, point[0], point[1]))
    return landmark_rows


def compute_visual_edges(result, range_records, landmark_id_to_mac, args):
    edges = []
    for record in range_records:
        landmark_key = gtsam.symbol("L", record["landmark_id"])
        try:
            pose = result.atPose2(record["pose_index"])
            landmark = result.atPoint2(landmark_key)
        except RuntimeError:
            continue

        pose_x = pose.x()
        pose_y = pose.y()
        landmark_x = float(landmark[0])
        landmark_y = float(landmark[1])
        dx = landmark_x - pose_x
        dy = landmark_y - pose_y
        predicted_range = math.hypot(dx, dy)
        measured_range = record["range"]
        residual = predicted_range - measured_range
        sigma = range_sigma_m(args, measured_range)
        normalized_residual = residual / sigma
        weight = huber_weight(normalized_residual, args.huber_k) if args.robust else 1.0

        if predicted_range > 1e-9:
            target_x = pose_x + dx * measured_range / predicted_range
            target_y = pose_y + dy * measured_range / predicted_range
        else:
            target_x = landmark_x
            target_y = landmark_y

        edges.append(
            {
                "poseIndex": record["pose_index"],
                "mac": landmark_id_to_mac[record["landmark_id"]],
                "px": pose_x,
                "py": pose_y,
                "lx": landmark_x,
                "ly": landmark_y,
                "targetX": target_x,
                "targetY": target_y,
                "measured": measured_range,
                "predicted": predicted_range,
                "residual": residual,
                "sigma": sigma,
                "weight": weight,
            }
        )
    return edges


def build_visual_snapshot(update_index, pose_index, result, initialized_landmark_keys, landmark_id_to_mac, range_records, args):
    pose_rows = extract_pose_rows(result)
    landmark_ids = {int(gtsam.Symbol(key).index()) for key in initialized_landmark_keys}
    landmark_rows = extract_landmark_rows(result, landmark_ids)
    edges = compute_visual_edges(result, range_records, landmark_id_to_mac, args)
    abs_residuals = [abs(edge["residual"]) for edge in edges]
    weights = [edge["weight"] for edge in edges]

    return {
        "iteration": update_index,
        "poseIndex": pose_index,
        "poses": [[row[1], row[2]] for row in pose_rows],
        "landmarks": [
            {"id": landmark_id, "mac": landmark_id_to_mac[landmark_id], "x": x, "y": y}
            for landmark_id, x, y in landmark_rows
        ],
        "edges": edges,
        "meanAbsResidual": float(np.mean(abs_residuals)) if abs_residuals else 0.0,
        "maxAbsResidual": float(np.max(abs_residuals)) if abs_residuals else 0.0,
        "meanWeight": float(np.mean(weights)) if weights else 1.0,
    }


class LivePoseGraphPlot:
    def __init__(self, ground_truth_xy=None, true_landmarks=None):
        self.ground_truth_xy = ground_truth_xy
        self.true_landmarks = true_landmarks or {}
        self.fig, (self.ax_map, self.ax_metrics) = plt.subplots(
            1,
            2,
            figsize=(13, 6),
            gridspec_kw={"width_ratios": [2.2, 1.0]},
        )
        self.ax_weight = self.ax_metrics.twinx()
        self.metric_iterations = []
        self.metric_mean_residuals = []
        self.metric_max_residuals = []
        self.metric_weights = []
        plt.ion()
        self.fig.show()

    def update(self, snapshot):
        self.metric_iterations.append(snapshot["iteration"])
        self.metric_mean_residuals.append(snapshot["meanAbsResidual"])
        self.metric_max_residuals.append(snapshot["maxAbsResidual"])
        self.metric_weights.append(snapshot["meanWeight"])

        self.ax_map.clear()
        self.ax_metrics.clear()
        self.ax_weight.clear()
        self._draw_map(snapshot)
        self._draw_metrics(snapshot)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def finish(self):
        plt.ioff()
        plt.show()

    def _draw_map(self, snapshot):
        if self.ground_truth_xy is not None and len(self.ground_truth_xy) > 0:
            self.ax_map.plot(
                self.ground_truth_xy[:, 0],
                self.ground_truth_xy[:, 1],
                "k--",
                linewidth=1.3,
                label="Reference trajectory",
            )

        poses = np.array(snapshot["poses"], dtype=float)
        if len(poses) > 0:
            self.ax_map.plot(poses[:, 0], poses[:, 1], color="#1f77b4", linewidth=1.7, label="Estimated trajectory")
            self.ax_map.scatter(poses[-1, 0], poses[-1, 1], s=60, color="#1f77b4", zorder=5)

        for edge in snapshot["edges"]:
            color = self._edge_color(edge["weight"])
            linewidth = 0.6 + 2.4 * (1.0 - edge["weight"])
            alpha = 0.18 + 0.55 * (1.0 - edge["weight"])
            self.ax_map.plot([edge["px"], edge["lx"]], [edge["py"], edge["ly"]], color=color, alpha=alpha, linewidth=linewidth)
            self.ax_map.annotate(
                "",
                xy=(edge["targetX"], edge["targetY"]),
                xytext=(edge["lx"], edge["ly"]),
                arrowprops={"arrowstyle": "->", "color": color, "alpha": 0.45, "linewidth": 1.3},
            )

        for landmark in snapshot["landmarks"]:
            self.ax_map.scatter(landmark["x"], landmark["y"], marker="^", s=90, color="#f59e0b", edgecolor="black", linewidth=0.5, zorder=6)
            self.ax_map.text(landmark["x"], landmark["y"], f" {landmark['mac']}", fontsize=9)

        for mac, point in self.true_landmarks.items():
            self.ax_map.scatter(float(point[0]), float(point[1]), marker="x", s=100, color="#d62728", linewidth=2.0, zorder=7)
            self.ax_map.text(float(point[0]), float(point[1]), f" {mac}", fontsize=9, color="#d62728")

        self.ax_map.set_title(f"Pose graph update {snapshot['iteration']} | pose {snapshot['poseIndex']}")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.grid(True, alpha=0.28)
        self.ax_map.axis("equal")
        self.ax_map.legend(loc="best")

    def _draw_metrics(self, snapshot):
        self.ax_metrics.plot(self.metric_iterations, self.metric_mean_residuals, color="#1f77b4", label="mean |range residual|")
        self.ax_metrics.plot(self.metric_iterations, self.metric_max_residuals, color="#d62728", alpha=0.75, label="max |range residual|")
        self.ax_metrics.set_xlabel("iSAM update")
        self.ax_metrics.set_ylabel("Residual (m)")
        self.ax_metrics.grid(True, alpha=0.28)
        self.ax_weight.plot(self.metric_iterations, self.metric_weights, color="#2ca02c", label="mean Huber weight")
        self.ax_weight.set_ylim(0.0, 1.05)
        self.ax_weight.set_ylabel("Mean weight")
        lines, labels = self.ax_metrics.get_legend_handles_labels()
        twin_lines, twin_labels = self.ax_weight.get_legend_handles_labels()
        self.ax_metrics.legend(lines + twin_lines, labels + twin_labels, loc="upper right", fontsize=8)

    @staticmethod
    def _edge_color(weight):
        weight = min(max(weight, 0.0), 1.0)
        red = np.array([214, 39, 40], dtype=float)
        green = np.array([44, 160, 44], dtype=float)
        color = red * (1.0 - weight) + green * weight
        return color / 255.0


def run_isam(odometry, absolute_poses, pose0, measurements, landmark_id_to_mac, args, visualizer=None):
    prior_noise = NM.Diagonal.Sigmas(gtsam.Point3(args.prior_sigma_xy_m, args.prior_sigma_xy_m, args.prior_sigma_theta_rad))
    odo_noise = NM.Diagonal.Sigmas(gtsam.Point3(args.odom_sigma_x_m, args.odom_sigma_y_m, args.odom_sigma_theta_rad))
    absolute_odom_prior_noise = None
    if args.odom_pose_prior_sigma_xy_m > 0.0:
        absolute_odom_prior_noise = NM.Diagonal.Sigmas(
            gtsam.Point3(
                args.odom_pose_prior_sigma_xy_m,
                args.odom_pose_prior_sigma_xy_m,
                args.odom_pose_prior_sigma_theta_rad,
            )
        )
    isam_params = gtsam.ISAM2Params()
    isam_params.setFactorization(args.isam_factorization)
    isam = gtsam.ISAM2(isam_params)
    new_factors = gtsam.NonlinearFactorGraph()
    new_factors.addPriorPose2(0, pose0, prior_noise)

    initial = gtsam.Values()
    initial.insert(0, pose0)

    last_pose = pose0
    initialized_landmarks = set()
    initialized = False
    range_index = 0
    new_range_count = 0
    pose_index = 1
    pending_landmark_samples = {}
    recent_range_records = []
    update_index = 0
    path_loss_1m_by_mac = {}

    for time_s, relative_pose in odometry.items():
        new_factors.add(gtsam.BetweenFactorPose2(pose_index - 1, pose_index, relative_pose, odo_noise))
        if absolute_odom_prior_noise is not None:
            new_factors.addPriorPose2(pose_index, absolute_poses[pose_index], absolute_odom_prior_noise)

        predicted_pose = last_pose.compose(relative_pose)
        last_pose = predicted_pose
        initial.insert(pose_index, predicted_pose)

        while range_index < len(measurements) and measurements[range_index][0] <= time_s:
            _, landmark_id, mac, rssi_dbm = measurements[range_index]
            landmark_key = gtsam.symbol("L", landmark_id)

            if landmark_key in initialized_landmarks:
                measured_range = bounded_rssi_to_range_m(args, rssi_dbm, path_loss_1m_by_mac[mac])
                new_factors.add(gtsam.RangeFactor2D(pose_index, landmark_key, measured_range, make_range_noise(args, measured_range)))
                recent_range_records.append({"pose_index": pose_index, "landmark_id": landmark_id, "range": measured_range})
                new_range_count += 1
            else:
                samples = pending_landmark_samples.setdefault(mac, [])
                samples.append(
                    {
                        "pose_index": pose_index,
                        "x": predicted_pose.x(),
                        "y": predicted_pose.y(),
                        "rssi": rssi_dbm,
                    }
                )
                initial_point, path_loss_1m_db = estimate_landmark_from_past_samples(samples, args)
                if initial_point is None:
                    range_index += 1
                    continue

                path_loss_1m_by_mac[mac] = path_loss_1m_db
                initial.insert(landmark_key, initial_point)
                initialized_landmarks.add(landmark_key)
                for sample in samples:
                    measured_range = bounded_rssi_to_range_m(args, sample["rssi"], path_loss_1m_db)
                    new_factors.add(
                        gtsam.RangeFactor2D(
                            sample["pose_index"],
                            landmark_key,
                            measured_range,
                            make_range_noise(args, measured_range),
                        )
                    )
                    recent_range_records.append(
                        {
                            "pose_index": sample["pose_index"],
                            "landmark_id": landmark_id,
                            "range": measured_range,
                        }
                    )
                    new_range_count += 1
                pending_landmark_samples[mac] = []

            range_index += 1

        if (range_index > args.min_initial_ranges) and (new_range_count > args.ranges_per_update):
            if not initialized:
                batch_optimizer = gtsam.LevenbergMarquardtOptimizer(new_factors, initial)
                initial = batch_optimizer.optimize()
                initialized = True

            isam.update(new_factors, initial)
            result = isam.calculateEstimate()
            update_index += 1
            if visualizer is not None and update_index % LIVE_PLOT_UPDATE_STRIDE == 0:
                visualizer.update(
                    build_visual_snapshot(
                        update_index,
                        pose_index,
                        result,
                        initialized_landmarks,
                        landmark_id_to_mac,
                        recent_range_records,
                        args,
                    )
                )
            last_pose = result.atPose2(pose_index)
            new_factors = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            new_range_count = 0
            recent_range_records = []

        pose_index += 1

    if new_factors.size() > 0:
        if not initialized:
            batch_optimizer = gtsam.LevenbergMarquardtOptimizer(new_factors, initial)
            initial = batch_optimizer.optimize()
        isam.update(new_factors, initial)
        result = isam.calculateEstimate()
        update_index += 1
        if visualizer is not None:
            visualizer.update(
                build_visual_snapshot(
                    update_index,
                    pose_index - 1,
                    result,
                    initialized_landmarks,
                    landmark_id_to_mac,
                    recent_range_records,
                    args,
                )
            )

    return isam.calculateEstimate(), initialized_landmarks, path_loss_1m_by_mac


def compute_landmark_errors(landmark_rows, landmark_id_to_mac, true_landmarks):
    if not true_landmarks:
        return [], None

    error_rows = []
    for landmark_id, est_x, est_y in landmark_rows:
        mac = landmark_id_to_mac.get(landmark_id)
        if mac is None or mac not in true_landmarks:
            continue

        true_point = true_landmarks[mac]
        true_x = float(true_point[0])
        true_y = float(true_point[1])
        dx_m = est_x - true_x
        dy_m = est_y - true_y
        error_m = math.hypot(est_x - true_x, est_y - true_y)
        error_rows.append((landmark_id, mac, est_x, est_y, true_x, true_y, dx_m, dy_m, error_m))

    if not error_rows:
        return [], None

    errors = np.array([row[-1] for row in error_rows], dtype=float)
    summary = {
        "count": len(error_rows),
        "mean": float(np.mean(errors)),
        "rmse": float(math.sqrt(np.mean(errors ** 2))),
        "max": float(np.max(errors)),
    }
    return error_rows, summary


def default_input_path(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(script_dir, "..", "Inputs", filename))
    return candidate if os.path.exists(candidate) else None


def main():
    parser = argparse.ArgumentParser(description="RSSI-adapted Range ISAM2 synthetic outdoor example.")
    parser.add_argument("--rssi_csv", required=True)
    parser.add_argument("--odom_csv", required=True)
    args = parser.parse_args()

    args.macs = infer_macs_from_rssi(args.rssi_csv)
    if not args.macs:
        raise ValueError("No usable AP MACs found in the RSSI dataset.")
    args.positions_csv = default_input_path("positions-synthetic.csv")
    args.trajectory_csv = default_input_path("trajectory-synthetic.csv")
    args.position_start_row = POSITION_START_ROW

    args.tx_power_dbm = TX_POWER_DBM
    args.path_loss_1m_db = PATH_LOSS_1M_DB
    args.path_loss_exponent = PATH_LOSS_EXPONENT
    args.reference_distance_m = REFERENCE_DISTANCE_M
    args.min_range_m = MIN_RANGE_M
    args.max_range_m = MAX_RANGE_M
    args.rssi_sigma_db = RSSI_SIGMA_DB
    args.min_range_sigma_m = MIN_RANGE_SIGMA_M
    args.robust = USE_ROBUST_RANGE_LOSS
    args.huber_k = HUBER_K
    args.prior_sigma_xy_m = PRIOR_SIGMA_XY_M
    args.prior_sigma_theta_rad = PRIOR_SIGMA_THETA_RAD
    args.odom_sigma_x_m = ODOM_SIGMA_X_M
    args.odom_sigma_y_m = ODOM_SIGMA_Y_M
    args.odom_sigma_theta_rad = ODOM_SIGMA_THETA_RAD
    args.odom_pose_prior_sigma_xy_m = ODOM_POSE_PRIOR_SIGMA_XY_M
    args.odom_pose_prior_sigma_theta_rad = ODOM_POSE_PRIOR_SIGMA_THETA_RAD
    args.min_initial_ranges = MIN_INITIAL_RANGES
    args.ranges_per_update = RANGES_PER_UPDATE
    args.isam_factorization = ISAM_FACTORIZATION

    _, absolute_poses, odometry = load_absolute_odometry(args.odom_csv)
    pose0 = absolute_poses[0]
    mac_to_landmark_id = {mac: idx + 1 for idx, mac in enumerate(args.macs)}
    landmark_id_to_mac = {landmark_id: mac for mac, landmark_id in mac_to_landmark_id.items()}

    measurements = load_rssi_measurements(args.rssi_csv, mac_to_landmark_id)

    if not measurements:
        raise ValueError("No RSSI rows matched the selected MAC list.")
    print_range_stats(measurements, landmark_id_to_mac, args)

    true_landmarks = load_ap_positions(args.positions_csv, args.macs, args.position_start_row)
    ground_truth_xy = load_ground_truth_trajectory(args.trajectory_csv)
    visualizer = LivePoseGraphPlot(ground_truth_xy, true_landmarks)

    result, initialized_landmarks, path_loss_1m_by_mac = run_isam(
        odometry,
        absolute_poses,
        pose0,
        measurements,
        landmark_id_to_mac,
        args,
        visualizer,
    )
    landmark_ids = {mac_to_landmark_id[mac] for mac in args.macs if gtsam.symbol("L", mac_to_landmark_id[mac]) in initialized_landmarks}
    landmark_rows = extract_landmark_rows(result, landmark_ids)
    landmark_error_rows, landmark_error_summary = compute_landmark_errors(
        landmark_rows,
        landmark_id_to_mac,
        true_landmarks,
    )

    print("RSSI Range-ISAM synthetic run complete.")
    print(f"Odometry factors: {len(odometry)}")
    print(f"RSSI range factors: {len(measurements)}")
    print(f"MACs: {', '.join(args.macs)}")
    print(f"Path loss: tx={args.tx_power_dbm:.1f} dBm, n={args.path_loss_exponent:.2f}")
    if path_loss_1m_by_mac:
        print("Online RSSI offset estimates:")
        for mac in args.macs:
            if mac in path_loss_1m_by_mac:
                print(f"  {mac}: PL(1m)={path_loss_1m_by_mac[mac]:.2f} dB")
    if landmark_error_summary is not None:
        print("AP mapping residuals:")
        for _, mac, est_x, est_y, true_x, true_y, dx_m, dy_m, error_m in landmark_error_rows:
            print(
                f"  {mac}: est=({est_x:.3f}, {est_y:.3f}) m, "
                f"true=({true_x:.3f}, {true_y:.3f}) m, "
                f"dx={dx_m:.3f} m, dy={dy_m:.3f} m, error={error_m:.3f} m"
            )
        print(
            "AP mapping error: "
            f"mean={landmark_error_summary['mean']:.3f} m, "
            f"RMSE={landmark_error_summary['rmse']:.3f} m, "
            f"max={landmark_error_summary['max']:.3f} m "
            f"over {landmark_error_summary['count']} APs"
        )
    visualizer.finish()


if __name__ == "__main__":
    main()
