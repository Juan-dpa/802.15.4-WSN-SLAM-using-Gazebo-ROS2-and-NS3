#!/usr/bin/env python3
import argparse
import csv
import math
import os
from collections import OrderedDict

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Point2, Pose2


NM = gtsam.noiseModel


TX_POWER_DBM = 0.0
PATH_LOSS_1M_DB = 22.0
PATH_LOSS_EXPONENT = 3.8
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
MIN_LANDMARK_INIT_RANGES = 8
MIN_LANDMARK_INIT_BASELINE_M = 10.0
ISAM_FACTORIZATION = "QR"

POSITION_START_ROW = 0
DEFAULT_OUTPUT_PREFIX = "rssi_range_isam_synthetic"


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


def load_rssi_as_ranges(path, mac_to_landmark_id, args):
    triples = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            mac = row["Src_MAC"].strip()
            if mac not in mac_to_landmark_id:
                continue

            time_s = float(row["Time_s"])
            rssi_dbm = float(row["RSSI_dBm"])
            range_m = rssi_to_range_m(
                rssi_dbm,
                args.tx_power_dbm,
                args.path_loss_1m_db,
                args.path_loss_exponent,
                args.reference_distance_m,
            )
            range_m = min(max(range_m, args.min_range_m), args.max_range_m)
            triples.append((time_s, mac_to_landmark_id[mac], range_m, mac, rssi_dbm))

    triples.sort(key=lambda item: item[0])
    return triples


def print_range_stats(triples, landmark_id_to_mac):
    print("RSSI-derived range summary:")
    for landmark_id, mac in sorted(landmark_id_to_mac.items()):
        ranges = np.array([item[2] for item in triples if item[1] == landmark_id], dtype=float)
        if len(ranges) == 0:
            continue
        print(
            f"  {mac}: min={np.min(ranges):.2f} m, "
            f"median={np.median(ranges):.2f} m, max={np.max(ranges):.2f} m"
        )


def estimate_landmark_from_past_samples(samples):
    if len(samples) < MIN_LANDMARK_INIT_RANGES:
        return None

    xs = np.array([sample["x"] for sample in samples], dtype=float)
    ys = np.array([sample["y"] for sample in samples], dtype=float)
    ranges = np.array([sample["range"] for sample in samples], dtype=float)
    baseline = max(float(np.ptp(xs)), float(np.ptp(ys)))
    if baseline < MIN_LANDMARK_INIT_BASELINE_M:
        return None

    x0 = xs[0]
    y0 = ys[0]
    r0 = ranges[0]
    a = np.column_stack((2.0 * (x0 - xs[1:]), 2.0 * (y0 - ys[1:])))
    b = ranges[1:] ** 2 - r0 ** 2 - xs[1:] ** 2 + x0 ** 2 - ys[1:] ** 2 + y0 ** 2
    if np.linalg.matrix_rank(a) < 2:
        return None

    solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    return Point2(float(solution[0]), float(solution[1]))


def make_range_noise(args, measured_range):
    coeff = math.log(10.0) / (10.0 * args.path_loss_exponent)
    sigma = max(args.min_range_sigma_m, coeff * measured_range * args.rssi_sigma_db)

    gaussian = NM.Isotropic.Sigma(1, sigma)
    if args.robust:
        return NM.Robust.Create(NM.mEstimator.Huber.Create(args.huber_k), gaussian)
    return gaussian


def run_isam(odometry, absolute_poses, pose0, triples, args):
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

    for time_s, relative_pose in odometry.items():
        new_factors.add(gtsam.BetweenFactorPose2(pose_index - 1, pose_index, relative_pose, odo_noise))
        if absolute_odom_prior_noise is not None:
            new_factors.addPriorPose2(pose_index, absolute_poses[pose_index], absolute_odom_prior_noise)

        predicted_pose = last_pose.compose(relative_pose)
        last_pose = predicted_pose
        initial.insert(pose_index, predicted_pose)

        while range_index < len(triples) and triples[range_index][0] <= time_s:
            _, landmark_id, measured_range, mac, _ = triples[range_index]
            landmark_key = gtsam.symbol("L", landmark_id)

            if landmark_key in initialized_landmarks:
                new_factors.add(gtsam.RangeFactor2D(pose_index, landmark_key, measured_range, make_range_noise(args, measured_range)))
                new_range_count += 1
            else:
                samples = pending_landmark_samples.setdefault(mac, [])
                samples.append(
                    {
                        "pose_index": pose_index,
                        "x": predicted_pose.x(),
                        "y": predicted_pose.y(),
                        "range": measured_range,
                    }
                )
                initial_point = estimate_landmark_from_past_samples(samples)
                if initial_point is None:
                    range_index += 1
                    continue

                initial.insert(landmark_key, initial_point)
                initialized_landmarks.add(landmark_key)
                for sample in samples:
                    new_factors.add(
                        gtsam.RangeFactor2D(
                            sample["pose_index"],
                            landmark_key,
                            sample["range"],
                            make_range_noise(args, sample["range"]),
                        )
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
            last_pose = result.atPose2(pose_index)
            new_factors = gtsam.NonlinearFactorGraph()
            initial = gtsam.Values()
            new_range_count = 0

        pose_index += 1

    if new_factors.size() > 0:
        if not initialized:
            batch_optimizer = gtsam.LevenbergMarquardtOptimizer(new_factors, initial)
            initial = batch_optimizer.optimize()
        isam.update(new_factors, initial)

    return isam.calculateEstimate(), initialized_landmarks


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


def save_outputs(result, output_prefix, landmark_ids, landmark_id_to_mac, true_landmarks=None, ground_truth_xy=None):
    poses = gtsam.utilities.allPose2s(result)
    pose_rows = []
    for key in poses.keys():
        pose = poses.atPose2(key)
        pose_rows.append((key, pose.x(), pose.y(), pose.theta()))
    pose_rows.sort(key=lambda row: row[0])

    with open(f"{output_prefix}_poses.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["PoseIndex", "X", "Y", "Theta"])
        writer.writerows(pose_rows)

    landmark_rows = []
    for landmark_id in sorted(landmark_ids):
        key = gtsam.symbol("L", landmark_id)
        point = result.atPoint2(key)
        landmark_rows.append((landmark_id, point[0], point[1]))

    with open(f"{output_prefix}_landmarks.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["LandmarkId", "X", "Y"])
        writer.writerows(landmark_rows)

    landmark_error_rows, landmark_error_summary = compute_landmark_errors(
        landmark_rows, landmark_id_to_mac, true_landmarks
    )
    with open(f"{output_prefix}_landmark_errors.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["LandmarkId", "MAC", "Estimated_X", "Estimated_Y", "True_X", "True_Y", "Delta_X", "Delta_Y", "Error_m"])
        writer.writerows(landmark_error_rows)

    if pose_rows:
        pose_array = np.array([[row[1], row[2]] for row in pose_rows])
        landmark_array = np.array([[row[1], row[2]] for row in landmark_rows]) if landmark_rows else np.empty((0, 2))

        plt.figure(figsize=(8, 8))
        if ground_truth_xy is not None and len(ground_truth_xy) > 0:
            plt.plot(ground_truth_xy[:, 0], ground_truth_xy[:, 1], "k--", linewidth=1.4, label="Synthetic trajectory")
        plt.plot(pose_array[:, 0], pose_array[:, 1], "-", linewidth=1.4, label="Estimated trajectory")
        if len(landmark_array) > 0:
            plt.scatter(landmark_array[:, 0], landmark_array[:, 1], marker="^", s=90, label="Estimated APs")
        if true_landmarks:
            true_array = np.array([[point[0], point[1]] for point in true_landmarks.values()])
            plt.scatter(true_array[:, 0], true_array[:, 1], marker="x", s=110, linewidths=2.0, label="Synthetic APs")
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.tight_layout()
        plt.savefig(f"{output_prefix}_plot.png", dpi=160)
        plt.close()

    return landmark_error_summary, landmark_error_rows


def default_input_path(filename):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(script_dir, "..", "Inputs", filename))
    return candidate if os.path.exists(candidate) else None


def main():
    parser = argparse.ArgumentParser(description="RSSI-adapted Range ISAM2 synthetic outdoor example.")
    parser.add_argument("--rssi_csv", required=True)
    parser.add_argument("--odom_csv", required=True)
    parser.add_argument("--output_prefix", default=DEFAULT_OUTPUT_PREFIX)
    args = parser.parse_args()

    args.macs = infer_macs_from_rssi(args.rssi_csv)
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

    triples = load_rssi_as_ranges(args.rssi_csv, mac_to_landmark_id, args)

    if not triples:
        raise ValueError("No RSSI rows matched the selected MAC list.")
    print_range_stats(triples, landmark_id_to_mac)

    result, initialized_landmarks = run_isam(odometry, absolute_poses, pose0, triples, args)
    landmark_ids = {mac_to_landmark_id[mac] for mac in args.macs if gtsam.symbol("L", mac_to_landmark_id[mac]) in initialized_landmarks}
    true_landmarks = load_ap_positions(args.positions_csv, args.macs, args.position_start_row)
    ground_truth_xy = load_ground_truth_trajectory(args.trajectory_csv)
    landmark_error_summary, landmark_error_rows = save_outputs(
        result,
        args.output_prefix,
        landmark_ids,
        landmark_id_to_mac,
        true_landmarks,
        ground_truth_xy,
    )

    print("RSSI Range-ISAM synthetic run complete.")
    print(f"Odometry factors: {len(odometry)}")
    print(f"RSSI range factors: {len(triples)}")
    print(f"MACs: {', '.join(args.macs)}")
    print(f"Path loss: tx={args.tx_power_dbm:.1f} dBm, PL(1m)={args.path_loss_1m_db:.2f} dB, n={args.path_loss_exponent:.2f}")
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
    print(
        "Outputs: "
        f"{args.output_prefix}_poses.csv, "
        f"{args.output_prefix}_landmarks.csv, "
        f"{args.output_prefix}_landmark_errors.csv, "
        f"{args.output_prefix}_plot.png"
    )


if __name__ == "__main__":
    main()
