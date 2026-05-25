#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rssi_range_isam_synthetic as slam


BASELINE_VALUES = [30, 40, 50, 60, 70, 80, 90]
RSSI_SIGMA_VALUES = [3, 4, 5, 6, 8, 10, 12]
PATH_LOSS_EXPONENT_VALUES = [3.2, 3.5, 3.8, 4.1, 4.4]

# The synthetic field is 100 x 100 m. Errors above this scale are not
# meaningful mapping errors; they indicate a numerically divergent graph.
DIVERGENCE_ERROR_M = 500.0

LEVELS = {
    "min": {
        "baseline": BASELINE_VALUES[0],
        "rssi_sigma": RSSI_SIGMA_VALUES[0],
        "path_loss_exponent": PATH_LOSS_EXPONENT_VALUES[0],
    },
    "med": {
        "baseline": BASELINE_VALUES[len(BASELINE_VALUES) // 2],
        "rssi_sigma": RSSI_SIGMA_VALUES[len(RSSI_SIGMA_VALUES) // 2],
        "path_loss_exponent": PATH_LOSS_EXPONENT_VALUES[len(PATH_LOSS_EXPONENT_VALUES) // 2],
    },
    "max": {
        "baseline": BASELINE_VALUES[-1],
        "rssi_sigma": RSSI_SIGMA_VALUES[-1],
        "path_loss_exponent": PATH_LOSS_EXPONENT_VALUES[-1],
    },
}

SWEEP_SPECS = {
    "baseline": {
        "values": BASELINE_VALUES,
        "xlabel": "MIN_LANDMARK_INIT_BASELINE_M (m)",
        "filename": "baseline",
    },
    "rssi_sigma": {
        "values": RSSI_SIGMA_VALUES,
        "xlabel": "RSSI_SIGMA_DB (dB)",
        "filename": "rssi_sigma",
    },
    "path_loss_exponent": {
        "values": PATH_LOSS_EXPONENT_VALUES,
        "xlabel": "PATH_LOSS_EXPONENT",
        "filename": "path_loss_exponent",
    },
}

T_CRITICAL_975 = {
    1: 12.706,
    2: 4.303,
    3: 3.182,
    4: 2.776,
    5: 2.571,
    6: 2.447,
    7: 2.365,
    8: 2.306,
    9: 2.262,
    10: 2.228,
    11: 2.201,
    12: 2.179,
    13: 2.160,
    14: 2.145,
    15: 2.131,
    16: 2.120,
    17: 2.110,
    18: 2.101,
    19: 2.093,
    20: 2.086,
    21: 2.080,
    22: 2.074,
    23: 2.069,
    24: 2.064,
    25: 2.060,
    26: 2.056,
    27: 2.052,
    28: 2.048,
    29: 2.045,
    30: 2.042,
}


def t_critical_95(n):
    if n <= 1:
        return float("nan")
    df = n - 1
    return T_CRITICAL_975.get(df, 1.96)


def mean_ci(values):
    values = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    n = len(values)
    if n == 0:
        return float("nan"), float("nan"), float("nan"), 0
    mean = float(np.mean(values))
    if n == 1:
        return mean, mean, mean, n
    half_width = t_critical_95(n) * float(np.std(values, ddof=1)) / math.sqrt(n)
    return mean, mean - half_width, mean + half_width, n


def rmse_ci(errors):
    errors = np.asarray(
        [value for value in errors if np.isfinite(value) and abs(value) <= DIVERGENCE_ERROR_M],
        dtype=float,
    )
    n = len(errors)
    if n == 0:
        return float("nan"), float("nan"), float("nan"), 0
    squared = errors ** 2
    mse = float(np.mean(squared))
    rmse = math.sqrt(mse)
    if n == 1:
        return rmse, rmse, rmse, n
    half_width = t_critical_95(n) * float(np.std(squared, ddof=1)) / math.sqrt(n)
    low = math.sqrt(max(0.0, mse - half_width))
    high = math.sqrt(max(0.0, mse + half_width))
    return rmse, low, high, n


def numeric_dataset_key(path):
    match = re.search(r"run(\d+)-synthetic", Path(path).name)
    if match:
        return int(match.group(1))
    return Path(path).name


def make_algorithm_args(macs, params):
    return SimpleNamespace(
        macs=macs,
        tx_power_dbm=slam.TX_POWER_DBM,
        path_loss_1m_db=slam.PATH_LOSS_1M_DB,
        path_loss_exponent=params["path_loss_exponent"],
        reference_distance_m=slam.REFERENCE_DISTANCE_M,
        min_range_m=slam.MIN_RANGE_M,
        max_range_m=slam.MAX_RANGE_M,
        rssi_sigma_db=params["rssi_sigma"],
        min_range_sigma_m=slam.MIN_RANGE_SIGMA_M,
        robust=slam.USE_ROBUST_RANGE_LOSS,
        huber_k=slam.HUBER_K,
        prior_sigma_xy_m=slam.PRIOR_SIGMA_XY_M,
        prior_sigma_theta_rad=slam.PRIOR_SIGMA_THETA_RAD,
        odom_sigma_x_m=slam.ODOM_SIGMA_X_M,
        odom_sigma_y_m=slam.ODOM_SIGMA_Y_M,
        odom_sigma_theta_rad=slam.ODOM_SIGMA_THETA_RAD,
        odom_pose_prior_sigma_xy_m=slam.ODOM_POSE_PRIOR_SIGMA_XY_M,
        odom_pose_prior_sigma_theta_rad=slam.ODOM_POSE_PRIOR_SIGMA_THETA_RAD,
        min_initial_ranges=slam.MIN_INITIAL_RANGES,
        ranges_per_update=slam.RANGES_PER_UPDATE,
        isam_factorization=slam.ISAM_FACTORIZATION,
    )


def run_single_dataset(rssi_csv, odom_csv, positions_csv, params):
    old_baseline = slam.MIN_LANDMARK_INIT_BASELINE_M
    slam.MIN_LANDMARK_INIT_BASELINE_M = params["baseline"]
    try:
        macs = slam.infer_macs_from_rssi(str(rssi_csv))
        args = make_algorithm_args(macs, params)
        _, absolute_poses, odometry = slam.load_absolute_odometry(str(odom_csv))
        pose0 = absolute_poses[0]
        mac_to_landmark_id = {mac: idx + 1 for idx, mac in enumerate(macs)}
        landmark_id_to_mac = {landmark_id: mac for mac, landmark_id in mac_to_landmark_id.items()}
        measurements = slam.load_rssi_measurements(str(rssi_csv), mac_to_landmark_id)

        result, initialized_landmarks, path_loss_1m_by_mac = slam.run_isam(
            odometry,
            absolute_poses,
            pose0,
            measurements,
            landmark_id_to_mac,
            args,
            visualizer=None,
        )

        landmark_ids = {
            mac_to_landmark_id[mac]
            for mac in macs
            if slam.gtsam.symbol("L", mac_to_landmark_id[mac]) in initialized_landmarks
        }
        true_landmarks = slam.load_ap_positions(str(positions_csv), macs, slam.POSITION_START_ROW)
        landmark_rows = slam.extract_landmark_rows(result, landmark_ids)
        error_rows, summary = slam.compute_landmark_errors(landmark_rows, landmark_id_to_mac, true_landmarks)
        errors_by_mac = {row[1]: row[-1] for row in error_rows}
        finite_errors = [error for error in errors_by_mac.values() if np.isfinite(error)]
        if len(finite_errors) != len(macs):
            raise RuntimeError("Not all APs were initialized with finite estimates.")
        if any(error > DIVERGENCE_ERROR_M for error in finite_errors):
            raise RuntimeError(
                f"Divergent AP estimate detected; max error={max(finite_errors):.3g} m "
                f"above threshold={DIVERGENCE_ERROR_M:.1f} m."
            )
        return {
            "ok": True,
            "macs": macs,
            "errors_by_mac": errors_by_mac,
            "max_error": summary["max"] if summary else float("nan"),
            "path_loss_1m_by_mac": path_loss_1m_by_mac,
        }
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "macs": [],
            "errors_by_mac": {},
            "max_error": float("nan"),
            "path_loss_1m_by_mac": {},
        }
    finally:
        slam.MIN_LANDMARK_INIT_BASELINE_M = old_baseline


def build_required_configs():
    configs = {}
    plot_specs = []
    for x_param, spec in SWEEP_SPECS.items():
        for level_name, fixed_values in LEVELS.items():
            fixed = {}
            for param in ["baseline", "rssi_sigma", "path_loss_exponent"]:
                if param != x_param:
                    fixed[param] = fixed_values[param]

            plot_points = []
            for x_value in spec["values"]:
                params = {
                    "baseline": fixed.get("baseline", x_value),
                    "rssi_sigma": fixed.get("rssi_sigma", x_value),
                    "path_loss_exponent": fixed.get("path_loss_exponent", x_value),
                }
                key = config_key(params)
                configs[key] = params
                plot_points.append((x_value, key))

            plot_specs.append(
                {
                    "x_param": x_param,
                    "level": level_name,
                    "fixed": fixed,
                    "points": plot_points,
                }
            )
    return configs, plot_specs


def config_key(params):
    return (
        float(params["baseline"]),
        float(params["rssi_sigma"]),
        float(params["path_loss_exponent"]),
    )


def aggregate_config(trial_rows, params):
    grouped = defaultdict(list)
    max_errors = []
    failed_trials = 0
    for row in trial_rows:
        if not row["ok"]:
            failed_trials += 1
            continue
        max_errors.append(row["max_error"])
        for mac, error in row["errors_by_mac"].items():
            grouped[mac].append(error)

    by_mac = {}
    for mac, errors in grouped.items():
        rmse, rmse_low, rmse_high, n = rmse_ci(errors)
        max_observed = float(np.max(errors)) if errors else float("nan")
        mean_error, mean_low, mean_high, _ = mean_ci(errors)
        by_mac[mac] = {
            "n": n,
            "rmse": rmse,
            "rmse_low": rmse_low,
            "rmse_high": rmse_high,
            "mean_error": mean_error,
            "mean_error_low": mean_low,
            "mean_error_high": mean_high,
            "max_observed": max_observed,
        }

    mean_max, max_low, max_high, n_max = mean_ci(max_errors)
    return {
        "params": params,
        "by_mac": by_mac,
        "mean_max_error": mean_max,
        "mean_max_error_low": max_low,
        "mean_max_error_high": max_high,
        "n_max": n_max,
        "failed_trials": failed_trials,
        "total_trials": len(trial_rows),
    }


def fixed_label(fixed):
    labels = []
    if "baseline" in fixed:
        labels.append(f"baseline={fixed['baseline']} m")
    if "rssi_sigma" in fixed:
        labels.append(f"RSSI sigma={fixed['rssi_sigma']} dB")
    if "path_loss_exponent" in fixed:
        labels.append(f"n={fixed['path_loss_exponent']}")
    return ", ".join(labels)


def configure_axes(ax, xlabel, ylabel, title, fixed):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    legend = ax.legend(title=f"Fixed: {fixed_label(fixed)}", fontsize=8)
    if legend is not None:
        legend.get_title().set_fontsize(8)


def plot_rmse(plot_spec, summaries, output_dir):
    x_param = plot_spec["x_param"]
    x_values = [point[0] for point in plot_spec["points"]]
    macs = sorted({mac for _, key in plot_spec["points"] for mac in summaries[key]["by_mac"]})

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    for mac in macs:
        y = []
        yerr_low = []
        yerr_high = []
        for _, key in plot_spec["points"]:
            stats = summaries[key]["by_mac"].get(mac)
            if stats is None:
                y.append(float("nan"))
                yerr_low.append(0.0)
                yerr_high.append(0.0)
                continue
            y.append(stats["rmse"])
            yerr_low.append(max(0.0, stats["rmse"] - stats["rmse_low"]))
            yerr_high.append(max(0.0, stats["rmse_high"] - stats["rmse"]))
        ax.errorbar(x_values, y, yerr=[yerr_low, yerr_high], marker="o", capsize=4, linewidth=1.5, label=mac)

    configure_axes(
        ax,
        SWEEP_SPECS[x_param]["xlabel"],
        "RMSE AP mapping error (m)",
        f"RMSE vs {SWEEP_SPECS[x_param]['xlabel']}",
        plot_spec["fixed"],
    )
    fig.tight_layout()
    path = output_dir / f"rmse_vs_{SWEEP_SPECS[x_param]['filename']}_fixed_{plot_spec['level']}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_max_error(plot_spec, summaries, output_dir):
    x_param = plot_spec["x_param"]
    x_values = [point[0] for point in plot_spec["points"]]
    y = []
    yerr_low = []
    yerr_high = []
    for _, key in plot_spec["points"]:
        stats = summaries[key]
        y.append(stats["mean_max_error"])
        yerr_low.append(max(0.0, stats["mean_max_error"] - stats["mean_max_error_low"]))
        yerr_high.append(max(0.0, stats["mean_max_error_high"] - stats["mean_max_error"]))

    fig, ax = plt.subplots(figsize=(8.2, 5.0))
    ax.errorbar(
        x_values,
        y,
        yerr=[yerr_low, yerr_high],
        marker="o",
        capsize=4,
        linewidth=1.6,
        label="max AP error per dataset",
        color="#d62728",
    )
    configure_axes(
        ax,
        SWEEP_SPECS[x_param]["xlabel"],
        "Mean max AP mapping error (m)",
        f"Max error vs {SWEEP_SPECS[x_param]['xlabel']}",
        plot_spec["fixed"],
    )
    fig.tight_layout()
    path = output_dir / f"max_error_vs_{SWEEP_SPECS[x_param]['filename']}_fixed_{plot_spec['level']}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_summary_csv(path, summaries):
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "baseline",
                "rssi_sigma",
                "path_loss_exponent",
                "mac",
                "n",
                "rmse",
                "rmse_ci95_low",
                "rmse_ci95_high",
                "mean_error",
                "mean_error_ci95_low",
                "mean_error_ci95_high",
                "max_observed_error",
                "mean_max_error_global",
                "mean_max_error_global_ci95_low",
                "mean_max_error_global_ci95_high",
                "failed_trials",
                "total_trials",
            ]
        )
        for key in sorted(summaries):
            summary = summaries[key]
            params = summary["params"]
            for mac, stats in sorted(summary["by_mac"].items()):
                writer.writerow(
                    [
                        params["baseline"],
                        params["rssi_sigma"],
                        params["path_loss_exponent"],
                        mac,
                        stats["n"],
                        stats["rmse"],
                        stats["rmse_low"],
                        stats["rmse_high"],
                        stats["mean_error"],
                        stats["mean_error_low"],
                        stats["mean_error_high"],
                        stats["max_observed"],
                        summary["mean_max_error"],
                        summary["mean_max_error_low"],
                        summary["mean_max_error_high"],
                        summary["failed_trials"],
                        summary["total_trials"],
                    ]
                )


def main():
    parser = argparse.ArgumentParser(description="Parameter sweep for RSSI Range-iSAM mapping.")
    parser.add_argument("--rssi_glob", default="../Outputs/slam_dataset_run*-synthetic.csv")
    parser.add_argument("--odom_csv", default="../Inputs/trajectory_odom-synthetic.csv")
    parser.add_argument("--positions_csv", default="../Inputs/positions-synthetic.csv")
    parser.add_argument("--output_dir", default="param_sweep_results")
    parser.add_argument("--limit_datasets", type=int, default=None)
    parser.add_argument("--max_configs", type=int, default=None)
    args = parser.parse_args()

    rssi_files = sorted(glob.glob(args.rssi_glob), key=numeric_dataset_key)
    if args.limit_datasets is not None:
        rssi_files = rssi_files[: args.limit_datasets]
    if not rssi_files:
        raise ValueError(f"No RSSI datasets matched: {args.rssi_glob}")

    odom_csv = Path(args.odom_csv)
    positions_csv = Path(args.positions_csv)
    output_dir = Path(args.output_dir)
    plots_dir = output_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    configs, plot_specs = build_required_configs()
    if args.max_configs is not None:
        limited_keys = list(configs.keys())[: args.max_configs]
        configs = {key: configs[key] for key in limited_keys}
        plot_specs = [
            {
                **spec,
                "points": [point for point in spec["points"] if point[1] in configs],
            }
            for spec in plot_specs
        ]
        plot_specs = [spec for spec in plot_specs if spec["points"]]

    print(f"Datasets: {len(rssi_files)}")
    print(f"Unique parameter configurations: {len(configs)}")

    all_trials = {}
    for idx, key in enumerate(sorted(configs), start=1):
        params = configs[key]
        print(
            f"[{idx}/{len(configs)}] baseline={params['baseline']}, "
            f"RSSI sigma={params['rssi_sigma']}, n={params['path_loss_exponent']}"
        )
        trial_rows = []
        for rssi_csv in rssi_files:
            result = run_single_dataset(Path(rssi_csv), odom_csv, positions_csv, params)
            result["dataset"] = Path(rssi_csv).name
            result["params"] = params
            trial_rows.append(result)
        all_trials[key] = trial_rows

    summaries = {key: aggregate_config(trials, configs[key]) for key, trials in all_trials.items()}
    write_summary_csv(output_dir / "sweep_summary.csv", summaries)

    generated = []
    for plot_spec in plot_specs:
        generated.append(plot_rmse(plot_spec, summaries, plots_dir))
    for plot_spec in plot_specs:
        generated.append(plot_max_error(plot_spec, summaries, plots_dir))

    print(f"Generated summary: {output_dir / 'sweep_summary.csv'}")
    print(f"Generated plots: {len(generated)}")
    for path in generated:
        print(path)


if __name__ == "__main__":
    main()
