#!/usr/bin/env python3
import math
from collections import defaultdict, deque
from dataclasses import dataclass

import gtsam
import numpy as np
from gtsam import Point2, Pose2


NM = gtsam.noiseModel


@dataclass
class RssiSlamConfig:
    tx_power_dbm: float = 0.0
    path_loss_1m_db: float = 22.0
    path_loss_exponent: float = 4.4
    reference_distance_m: float = 1.0
    min_range_m: float = 0.5
    max_range_m: float = 150.0
    rssi_sigma_db: float = 6.0
    min_range_sigma_m: float = 2.0
    robust: bool = True
    huber_k: float = 1.345

    prior_sigma_xy_m: float = 1.0
    prior_sigma_theta_rad: float = math.pi
    odom_sigma_x_m: float = 0.05
    odom_sigma_y_m: float = 0.01
    odom_sigma_theta_rad: float = 0.1
    odom_pose_prior_sigma_xy_m: float = 0.25
    odom_pose_prior_sigma_theta_rad: float = 0.2

    min_initial_ranges: int = 30
    ranges_per_update: int = 10
    min_landmark_init_ranges: int = 50
    min_landmark_init_baseline_m: float = 60.0
    min_landmark_init_rssi_span_db: float = 15.0
    path_loss_grid_size: int = 31
    path_loss_grid_refinements: int = 4
    path_loss_search_margin_m: float = 25.0
    compressed_range_baseline_ratio: float = 0.85
    isam_factorization: str = "QR"

    heatmap_min_x: float = 0.0
    heatmap_max_x: float = 100.0
    heatmap_min_y: float = 0.0
    heatmap_max_y: float = 100.0
    heatmap_resolution_m: float = 2.0
    heatmap_decay: float = 0.995


@dataclass
class RssiMeasurement:
    time_s: float
    mac: str
    rssi_dbm: float


def rssi_to_range_m(rssi_dbm, cfg):
    rssi_at_reference = cfg.tx_power_dbm - cfg.path_loss_1m_db
    exponent = (rssi_at_reference - rssi_dbm) / (10.0 * cfg.path_loss_exponent)
    range_m = cfg.reference_distance_m * (10.0 ** exponent)
    return min(max(range_m, cfg.min_range_m), cfg.max_range_m)


def range_sigma_m(cfg, measured_range):
    coeff = math.log(10.0) / (10.0 * cfg.path_loss_exponent)
    return max(cfg.min_range_sigma_m, coeff * measured_range * cfg.rssi_sigma_db)


def huber_weight(normalized_residual, huber_k):
    abs_residual = abs(normalized_residual)
    if abs_residual <= huber_k:
        return 1.0
    return huber_k / abs_residual


def make_range_noise(cfg, measured_range):
    gaussian = NM.Isotropic.Sigma(1, range_sigma_m(cfg, measured_range))
    if cfg.robust:
        return NM.Robust.Create(NM.mEstimator.Huber.Create(cfg.huber_k), gaussian)
    return gaussian


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


class RssiRangeISAM:
    def __init__(self, config=None):
        self.cfg = config or RssiSlamConfig()
        self.pose_key = 0
        self.last_odom_pose = None
        self.last_graph_pose = None
        self.last_time_s = None
        self.initialized = False
        self.update_index = 0
        self.new_range_count = 0

        params = gtsam.ISAM2Params()
        params.setFactorization(self.cfg.isam_factorization)
        self.isam = gtsam.ISAM2(params)
        self.factors = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        self.mac_to_landmark_id = {}
        self.landmark_id_to_mac = {}
        self.initialized_landmark_keys = set()
        self.pending_landmark_samples = defaultdict(list)
        self.pending_rssi = deque()
        self.path_loss_1m_by_mac = {}
        self.recent_range_records = deque(maxlen=250)
        self.latest_result = None

        self._init_heatmaps()

    def _init_heatmaps(self):
        xs = np.arange(self.cfg.heatmap_min_x, self.cfg.heatmap_max_x + 1e-9, self.cfg.heatmap_resolution_m)
        ys = np.arange(self.cfg.heatmap_min_y, self.cfg.heatmap_max_y + 1e-9, self.cfg.heatmap_resolution_m)
        self.grid_x, self.grid_y = np.meshgrid(xs, ys)
        self.heatmaps = {}

    def add_rssi(self, time_s, mac, rssi_dbm):
        self._ensure_mac(mac)
        self.pending_rssi.append(RssiMeasurement(float(time_s), mac.strip(), float(rssi_dbm)))
        if self.last_time_s is not None:
            self._process_pending_rssi(self.last_time_s, self.last_graph_pose)
            self._maybe_update()
        return self.snapshot()

    def add_odometry(self, time_s, x, y, theta):
        odom_pose = Pose2(float(x), float(y), float(theta))
        if self.last_odom_pose is None:
            self._add_initial_pose(odom_pose)
            self.last_time_s = float(time_s)
            return self.snapshot()

        self.pose_key += 1
        relative_pose = self.last_odom_pose.between(odom_pose)
        self.last_odom_pose = odom_pose
        predicted_pose = self.last_graph_pose.compose(relative_pose)
        self.last_graph_pose = predicted_pose
        self.last_time_s = float(time_s)

        odo_noise = NM.Diagonal.Sigmas(
            gtsam.Point3(self.cfg.odom_sigma_x_m, self.cfg.odom_sigma_y_m, self.cfg.odom_sigma_theta_rad)
        )
        self.factors.add(gtsam.BetweenFactorPose2(self.pose_key - 1, self.pose_key, relative_pose, odo_noise))
        self.initial.insert(self.pose_key, predicted_pose)

        if self.cfg.odom_pose_prior_sigma_xy_m > 0.0:
            prior_noise = NM.Diagonal.Sigmas(
                gtsam.Point3(
                    self.cfg.odom_pose_prior_sigma_xy_m,
                    self.cfg.odom_pose_prior_sigma_xy_m,
                    self.cfg.odom_pose_prior_sigma_theta_rad,
                )
            )
            self.factors.addPriorPose2(self.pose_key, odom_pose, prior_noise)

        self._process_pending_rssi(self.last_time_s, predicted_pose)
        self._maybe_update()
        return self.snapshot()

    def _add_initial_pose(self, pose):
        self.last_odom_pose = pose
        self.last_graph_pose = pose
        prior_noise = NM.Diagonal.Sigmas(
            gtsam.Point3(self.cfg.prior_sigma_xy_m, self.cfg.prior_sigma_xy_m, self.cfg.prior_sigma_theta_rad)
        )
        self.factors.addPriorPose2(0, pose, prior_noise)
        self.initial.insert(0, pose)

    def _ensure_mac(self, mac):
        if mac in self.mac_to_landmark_id:
            return
        landmark_id = len(self.mac_to_landmark_id) + 1
        self.mac_to_landmark_id[mac] = landmark_id
        self.landmark_id_to_mac[landmark_id] = mac
        self.heatmaps[mac] = np.zeros_like(self.grid_x, dtype=float)

    def _process_pending_rssi(self, time_s, pose):
        while self.pending_rssi and self.pending_rssi[0].time_s <= time_s:
            measurement = self.pending_rssi.popleft()
            self._process_rssi(measurement, pose)

    def _process_rssi(self, measurement, pose):
        mac = measurement.mac
        self._ensure_mac(mac)
        landmark_id = self.mac_to_landmark_id[mac]
        landmark_key = gtsam.symbol("L", landmark_id)
        self._update_heatmap(mac, pose, measurement.rssi_dbm)

        if landmark_key in self.initialized_landmark_keys:
            measured_range = rssi_to_range_m(measurement.rssi_dbm, self._cfg_for_mac(mac))
            self.factors.add(gtsam.RangeFactor2D(self.pose_key, landmark_key, measured_range, make_range_noise(self.cfg, measured_range)))
            self.recent_range_records.append((self.pose_key, landmark_id, measured_range))
            self.new_range_count += 1
            return

        samples = self.pending_landmark_samples[mac]
        samples.append({"pose_key": self.pose_key, "x": pose.x(), "y": pose.y(), "rssi": measurement.rssi_dbm})
        initial_point, path_loss_1m_db = self._estimate_landmark_from_past_samples(samples)
        if initial_point is None:
            return

        self.path_loss_1m_by_mac[mac] = path_loss_1m_db
        self.initial.insert(landmark_key, initial_point)
        self.initialized_landmark_keys.add(landmark_key)
        for sample in samples:
            measured_range = rssi_to_range_m(sample["rssi"], self._cfg_for_mac(mac))
            self.factors.add(
                gtsam.RangeFactor2D(
                    sample["pose_key"],
                    landmark_key,
                    measured_range,
                    make_range_noise(self.cfg, measured_range),
                )
            )
            self.recent_range_records.append((sample["pose_key"], landmark_id, measured_range))
            self.new_range_count += 1
        self.pending_landmark_samples[mac] = []

    def _cfg_for_mac(self, mac):
        cfg = RssiSlamConfig(**self.cfg.__dict__)
        cfg.path_loss_1m_db = self.path_loss_1m_by_mac.get(mac, self.cfg.path_loss_1m_db)
        return cfg

    def _estimate_landmark_from_past_samples(self, samples):
        if len(samples) < self.cfg.min_landmark_init_ranges:
            return None, None

        xs = np.array([sample["x"] for sample in samples], dtype=float)
        ys = np.array([sample["y"] for sample in samples], dtype=float)
        rssis = np.array([sample["rssi"] for sample in samples], dtype=float)
        baseline = max(float(np.ptp(xs)), float(np.ptp(ys)))
        if baseline < self.cfg.min_landmark_init_baseline_m:
            return None, None
        if float(np.ptp(rssis)) < self.cfg.min_landmark_init_rssi_span_db:
            return None, None

        default_ranges = np.array([rssi_to_range_m(rssi, self.cfg) for rssi in rssis], dtype=float)
        default_point = estimate_landmark_from_known_ranges(xs, ys, default_ranges)
        if default_point is not None and float(np.max(default_ranges)) >= self.cfg.compressed_range_baseline_ratio * baseline:
            return default_point, self.cfg.path_loss_1m_db

        return self._estimate_landmark_and_path_loss(xs, ys, rssis)

    def _estimate_landmark_and_path_loss(self, xs, ys, rssis):
        min_x = float(np.min(xs) - self.cfg.path_loss_search_margin_m)
        max_x = float(np.max(xs) + self.cfg.path_loss_search_margin_m)
        min_y = float(np.min(ys) - self.cfg.path_loss_search_margin_m)
        max_y = float(np.max(ys) + self.cfg.path_loss_search_margin_m)
        best = None

        for _ in range(self.cfg.path_loss_grid_refinements):
            grid_x = np.linspace(min_x, max_x, self.cfg.path_loss_grid_size)
            grid_y = np.linspace(min_y, max_y, self.cfg.path_loss_grid_size)
            for candidate_x in grid_x:
                dx = xs - candidate_x
                for candidate_y in grid_y:
                    distances = np.maximum(np.hypot(dx, ys - candidate_y), self.cfg.reference_distance_m)
                    log_distances = np.log10(distances / self.cfg.reference_distance_m)
                    path_loss_samples = self.cfg.tx_power_dbm - rssis - 10.0 * self.cfg.path_loss_exponent * log_distances
                    path_loss_1m_db = float(np.median(path_loss_samples))
                    predicted_rssi = self.cfg.tx_power_dbm - path_loss_1m_db - 10.0 * self.cfg.path_loss_exponent * log_distances
                    cost = float(np.mean((predicted_rssi - rssis) ** 2))
                    if best is None or cost < best[0]:
                        best = (cost, float(candidate_x), float(candidate_y), path_loss_1m_db)

            _, center_x, center_y, _ = best
            half_width = max((max_x - min_x) / (self.cfg.path_loss_grid_size - 1), 0.5)
            half_height = max((max_y - min_y) / (self.cfg.path_loss_grid_size - 1), 0.5)
            min_x, max_x = center_x - half_width, center_x + half_width
            min_y, max_y = center_y - half_height, center_y + half_height

        _, landmark_x, landmark_y, path_loss_1m_db = best
        return Point2(landmark_x, landmark_y), path_loss_1m_db

    def _update_heatmap(self, mac, pose, rssi_dbm):
        measured_range = rssi_to_range_m(rssi_dbm, self._cfg_for_mac(mac))
        sigma = range_sigma_m(self.cfg, measured_range)
        distances = np.hypot(self.grid_x - pose.x(), self.grid_y - pose.y())
        likelihood = np.exp(-0.5 * ((distances - measured_range) / sigma) ** 2)
        self.heatmaps[mac] = self.cfg.heatmap_decay * self.heatmaps[mac] + likelihood

    def _maybe_update(self):
        if self.pose_key <= self.cfg.min_initial_ranges or self.new_range_count <= self.cfg.ranges_per_update:
            return

        if not self.initialized:
            self.initial = gtsam.LevenbergMarquardtOptimizer(self.factors, self.initial).optimize()
            self.initialized = True

        self.isam.update(self.factors, self.initial)
        self.latest_result = self.isam.calculateEstimate()
        self.update_index += 1
        self.last_graph_pose = self.latest_result.atPose2(self.pose_key)
        self.factors = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()
        self.new_range_count = 0

    def snapshot(self):
        result = self.latest_result
        landmarks = []
        if result is not None:
            for landmark_key in sorted(self.initialized_landmark_keys):
                landmark_id = int(gtsam.Symbol(landmark_key).index())
                try:
                    point = result.atPoint2(landmark_key)
                except RuntimeError:
                    continue
                landmarks.append(
                    {
                        "id": landmark_id,
                        "mac": self.landmark_id_to_mac[landmark_id],
                        "x": float(point[0]),
                        "y": float(point[1]),
                        "path_loss_1m_db": self.path_loss_1m_by_mac.get(self.landmark_id_to_mac[landmark_id]),
                    }
                )

        return {
            "time_s": self.last_time_s,
            "pose_key": self.pose_key,
            "update_index": self.update_index,
            "landmarks": landmarks,
            "heatmap_points": self.heatmap_points(),
            "heatmap_points_by_mac": self.heatmap_points_by_mac(),
            "pending_rssi": len(self.pending_rssi),
            "initialized": self.initialized,
        }

    def heatmap_points(self):
        if not self.heatmaps:
            return []
        combined = np.maximum.reduce(list(self.heatmaps.values()))
        return self._heatmap_to_points(combined)

    def heatmap_points_by_mac(self):
        return {
            mac: self._heatmap_to_points(heatmap)
            for mac, heatmap in sorted(self.heatmaps.items())
        }

    def _heatmap_to_points(self, heatmap):
        max_value = float(np.max(heatmap))
        if max_value <= 0.0:
            return []
        normalized = heatmap / max_value
        points = []
        for y_idx in range(normalized.shape[0]):
            for x_idx in range(normalized.shape[1]):
                intensity = float(normalized[y_idx, x_idx])
                if intensity < 0.02:
                    continue
                points.append((float(self.grid_x[y_idx, x_idx]), float(self.grid_y[y_idx, x_idx]), intensity))
        return points
