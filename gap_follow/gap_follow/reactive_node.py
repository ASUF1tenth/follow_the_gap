import csv
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ReactiveFollowGap(Node):
    """Reactive Follow-the-Gap controller publishing Ackermann drive commands."""

    def __init__(self) -> None:
        super().__init__("gap_follower")

        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("publish_autodrive_commands", False)
        self.declare_parameter("throttle_topic", "/autodrive/roboracer_1/throttle_command")
        self.declare_parameter("steering_topic", "/autodrive/roboracer_1/steering_command")
        self.declare_parameter("speed_to_throttle_gain", 0.2)
        self.declare_parameter("max_lidar_dist", 10.0)
        self.declare_parameter("preprocess_window", 3)
        self.declare_parameter("disparity_threshold", 0.4)
        self.declare_parameter("safety_radius_m", 0.28)
        self.declare_parameter("gap_min_range_m", 1.2)
        self.declare_parameter("max_steering_angle", 0.4189)
        self.declare_parameter("speed_straight", 4.5)
        self.declare_parameter("speed_corner", 2.5)
        self.declare_parameter("speed_emergency", 0.8)
        self.declare_parameter("caution_distance_m", 0.55)
        self.declare_parameter("emergency_distance_m", 0.30)
        self.declare_parameter("no_gap_speed", 1.2)
        self.declare_parameter("corner_angle_threshold", 0.22)
        self.declare_parameter("steering_smoothing_alpha", 0.35)
        self.declare_parameter("max_steering_rate_rad_s", 3.2)
        self.declare_parameter("best_point_margin_m", 0.25)
        self.declare_parameter("best_point_center_penalty", 0.004)
        self.declare_parameter("fov_half_angle_rad", 1.57)
        self.declare_parameter("enable_file_logging", True)
        self.declare_parameter("log_dir", "logs")
        self.declare_parameter("console_status_every_n", 20)
        self.declare_parameter("near_collision_distance_m", 0.35)

        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.publish_autodrive_commands = bool(self.get_parameter("publish_autodrive_commands").value)
        self.throttle_topic = str(self.get_parameter("throttle_topic").value)
        self.steering_topic = str(self.get_parameter("steering_topic").value)
        self.speed_to_throttle_gain = float(self.get_parameter("speed_to_throttle_gain").value)
        self.max_lidar_dist = float(self.get_parameter("max_lidar_dist").value)
        self.preprocess_window = max(1, int(self.get_parameter("preprocess_window").value))
        self.disparity_threshold = float(self.get_parameter("disparity_threshold").value)
        self.safety_radius_m = float(self.get_parameter("safety_radius_m").value)
        self.gap_min_range_m = float(self.get_parameter("gap_min_range_m").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.speed_straight = float(self.get_parameter("speed_straight").value)
        self.speed_corner = float(self.get_parameter("speed_corner").value)
        self.speed_emergency = float(self.get_parameter("speed_emergency").value)
        self.caution_distance_m = float(self.get_parameter("caution_distance_m").value)
        self.emergency_distance_m = float(self.get_parameter("emergency_distance_m").value)
        self.no_gap_speed = float(self.get_parameter("no_gap_speed").value)
        self.corner_angle_threshold = float(self.get_parameter("corner_angle_threshold").value)
        self.steering_smoothing_alpha = float(self.get_parameter("steering_smoothing_alpha").value)
        self.max_steering_rate_rad_s = float(self.get_parameter("max_steering_rate_rad_s").value)
        self.best_point_margin_m = float(self.get_parameter("best_point_margin_m").value)
        self.best_point_center_penalty = float(self.get_parameter("best_point_center_penalty").value)
        self.fov_half_angle_rad = float(self.get_parameter("fov_half_angle_rad").value)
        self.enable_file_logging = bool(self.get_parameter("enable_file_logging").value)
        self.log_dir = str(self.get_parameter("log_dir").value)
        self.console_status_every_n = max(1, int(self.get_parameter("console_status_every_n").value))
        self.near_collision_distance_m = float(self.get_parameter("near_collision_distance_m").value)
        self.caution_distance_m = max(self.caution_distance_m, self.emergency_distance_m + 1e-3)
        self._last_steering_cmd = 0.0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.throttle_pub = self.create_publisher(Float32, self.throttle_topic, 10)
        self.steering_pub = self.create_publisher(Float32, self.steering_topic, 10)
        self._got_first_scan = False
        self._cycle_count = 0
        self._last_callback_time = None
        self._csv_file = None
        self._csv_writer = None
        self._init_csv_logger()
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

    def _init_csv_logger(self) -> None:
        if not self.enable_file_logging:
            return

        log_dir_path = Path(self.log_dir)
        if not log_dir_path.is_absolute():
            log_dir_path = Path.cwd() / log_dir_path
        log_dir_path.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = log_dir_path / f"ftg_run_{stamp}.csv"
        self._csv_file = open(csv_path, "w", newline="", encoding="ascii")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "cycle",
                "stamp_sec",
                "dt_sec",
                "processing_ms",
                "min_range_m",
                "best_range_m",
                "gap_found",
                "gap_start",
                "gap_end",
                "gap_length",
                "best_idx",
                "steering_rad",
                "speed_mps",
                "throttle_cmd",
                "steering_cmd",
            ]
        )
        self._csv_file.flush()
        self.get_logger().info(f"Verbose logging enabled: {csv_path}")

    def _write_cycle_log(
        self,
        stamp_sec: float,
        dt_sec: float,
        processing_ms: float,
        min_range_m: float,
        best_range_m: float,
        gap: Optional[Tuple[int, int]],
        best_idx: int,
        steering: float,
        speed: float,
        throttle_cmd: float,
        steering_cmd: float,
    ) -> None:
        if not self._csv_writer:
            return

        gap_found = 1 if gap is not None else 0
        gap_start = gap[0] if gap is not None else -1
        gap_end = gap[1] if gap is not None else -1
        gap_length = (gap_end - gap_start + 1) if gap is not None else 0

        self._csv_writer.writerow(
            [
                self._cycle_count,
                round(stamp_sec, 6),
                round(dt_sec, 6),
                round(processing_ms, 3),
                round(min_range_m, 4),
                round(best_range_m, 4),
                gap_found,
                gap_start,
                gap_end,
                gap_length,
                best_idx,
                round(steering, 6),
                round(speed, 6),
                round(throttle_cmd, 6),
                round(steering_cmd, 6),
            ]
        )

        if self._cycle_count % 10 == 0 and self._csv_file:
            self._csv_file.flush()

    def preprocess_lidar(self, msg: LaserScan) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * msg.angle_increment

        mask = np.abs(angles) <= self.fov_half_angle_rad
        if not np.any(mask):
            return np.array([], dtype=np.float32), np.array([], dtype=np.float32), np.array([], dtype=np.float32)

        front_ranges = ranges[mask]
        front_angles = angles[mask]
        front_indices = np.flatnonzero(mask)

        front_ranges = np.nan_to_num(front_ranges, nan=0.0, posinf=self.max_lidar_dist, neginf=0.0)
        front_ranges = np.clip(front_ranges, 0.0, self.max_lidar_dist)

        if self.preprocess_window > 1 and front_ranges.size >= self.preprocess_window:
            kernel = np.ones(self.preprocess_window, dtype=np.float32) / float(self.preprocess_window)
            front_ranges = np.convolve(front_ranges, kernel, mode="same")

        return front_ranges, front_angles, front_indices

    def apply_safety_bubble(self, proc_ranges: np.ndarray, angle_increment: float) -> np.ndarray:
        filtered = proc_ranges.copy()
        if filtered.size <= 1:
            return filtered

        ref_ranges = proc_ranges.copy()  # Use original depths for disparity and radius checks

        # 1. Closest point bubble
        closest_idx = int(np.argmin(ref_ranges))
        closest_dist = max(0.05, float(ref_ranges[closest_idx]))
        c_radius_idx = int(self.safety_radius_m / max(angle_increment * closest_dist, 1e-4))
        c_start = max(0, closest_idx - c_radius_idx)
        c_end = min(filtered.size, closest_idx + c_radius_idx + 1)
        filtered[c_start:c_end] = 0.0

        # 2. Add bubbles to disparities (edges)
        diffs = np.abs(np.diff(ref_ranges))
        disparity_ids = np.flatnonzero(diffs > self.disparity_threshold)

        for idx in disparity_ids:
            left = ref_ranges[idx]
            right = ref_ranges[idx + 1]
            close_idx = idx if left < right else idx + 1
            close_dist = max(0.05, float(ref_ranges[close_idx]))
            radius_idx = int(self.safety_radius_m / max(angle_increment * close_dist, 1e-4))
            start = max(0, close_idx - radius_idx)
            end = min(filtered.size, close_idx + radius_idx + 1)
            filtered[start:end] = 0.0

        return filtered

    def find_max_gap(self, ranges: np.ndarray) -> Optional[Tuple[int, int]]:
        free = ranges > self.gap_min_range_m
        if not np.any(free):
            return None

        padded = np.pad(free.astype(np.int8), (1, 1), mode="constant")
        transitions = np.diff(padded)
        starts = np.flatnonzero(transitions == 1)
        ends = np.flatnonzero(transitions == -1) - 1

        if starts.size == 0:
            return None

        lengths = ends - starts + 1
        best_i = int(np.argmax(lengths))
        return int(starts[best_i]), int(ends[best_i])

    def find_best_point(self, ranges: np.ndarray, gap: Tuple[int, int]) -> int:
        start, end = gap
        gap_ranges = ranges[start : end + 1]
        if gap_ranges.size == 0:
            return ranges.size // 2

        center_idx = ranges.size // 2
        global_indices = np.arange(start, end + 1)

        # Base score is range, minus a penalty for off-center
        scores = gap_ranges - self.best_point_center_penalty * np.abs(global_indices - center_idx)

        # Find the single best scoring ray
        best_i = int(np.argmax(scores))

        # Additional safety check: If best point is < 1.0m away, fall back to center of the gap 
        # to ensure we don't steer into a sharp wall corner that the radius bubble barely cleared.
        best_global = int(global_indices[best_i])
        best_val = float(gap_ranges[best_i])

        if best_val < 1.0:
            return (start + end) // 2

        return best_global

    def _limit_steering(self, target_steering: float, dt_sec: float) -> float:
        alpha = float(np.clip(self.steering_smoothing_alpha, 0.0, 1.0))
        smoothed = alpha * target_steering + (1.0 - alpha) * self._last_steering_cmd

        ref_dt = dt_sec if dt_sec > 1e-4 else 0.06
        max_delta = self.max_steering_rate_rad_s * ref_dt
        limited = float(np.clip(smoothed, self._last_steering_cmd - max_delta, self._last_steering_cmd + max_delta))
        self._last_steering_cmd = limited
        return limited

    def _compute_speed(self, min_range: float, steering_abs: float, gap_found: bool) -> float:
        base_speed = self.speed_corner if steering_abs > self.corner_angle_threshold else self.speed_straight
        if not gap_found:
            base_speed = min(base_speed, self.no_gap_speed)

        if min_range <= self.emergency_distance_m:
            return self.speed_emergency
        if min_range < self.caution_distance_m:
            ratio = (min_range - self.emergency_distance_m) / (self.caution_distance_m - self.emergency_distance_m)
            ratio = float(np.clip(ratio, 0.0, 1.0))
            return self.speed_emergency + ratio * (base_speed - self.speed_emergency)
        return base_speed

    def lidar_callback(self, msg: LaserScan) -> None:
        t0 = time.perf_counter()
        if not self._got_first_scan:
            self.get_logger().info(f"Received first scan on {self.lidar_topic}")
            self._got_first_scan = True

        proc_ranges, proc_angles, _ = self.preprocess_lidar(msg)
        if proc_ranges.size == 0:
            return

        now_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dt_sec = 0.0 if self._last_callback_time is None else max(0.0, now_sec - self._last_callback_time)
        self._last_callback_time = now_sec
        self._cycle_count += 1

        safe_ranges = self.apply_safety_bubble(proc_ranges, msg.angle_increment)
        gap = self.find_max_gap(safe_ranges)
        if gap is None:
            # No valid gap means obstacle field is dense; default to center and slow down.
            best_idx = safe_ranges.size // 2
            best_range = float(safe_ranges[best_idx])
        else:
            best_idx = self.find_best_point(safe_ranges, gap)
            best_range = float(safe_ranges[best_idx])

        target_steering = float(np.clip(proc_angles[best_idx], -self.max_steering_angle, self.max_steering_angle))
        min_range = float(np.min(proc_ranges))
        steering = self._limit_steering(target_steering, dt_sec)
        speed = self._compute_speed(min_range=min_range, steering_abs=abs(steering), gap_found=(gap is not None))

        cmd = AckermannDriveStamped()
        cmd.header = msg.header
        cmd.drive.steering_angle = steering
        cmd.drive.speed = speed
        self.drive_pub.publish(cmd)

        throttle_cmd = float("nan")
        steering_cmd = float("nan")

        if self.publish_autodrive_commands:
            # AutoDRIVE bridge expects normalized command range [-1, 1].
            throttle_cmd = float(np.clip(speed * self.speed_to_throttle_gain, -1.0, 1.0))
            steering_cmd = float(np.clip(steering / max(self.max_steering_angle, 1e-6), -1.0, 1.0))
            self.throttle_pub.publish(Float32(data=throttle_cmd))
            self.steering_pub.publish(Float32(data=steering_cmd))

        processing_ms = (time.perf_counter() - t0) * 1000.0
        self._write_cycle_log(
            stamp_sec=now_sec,
            dt_sec=dt_sec,
            processing_ms=processing_ms,
            min_range_m=min_range,
            best_range_m=best_range,
            gap=gap,
            best_idx=best_idx,
            steering=steering,
            speed=speed,
            throttle_cmd=throttle_cmd,
            steering_cmd=steering_cmd,
        )

        if self._cycle_count % self.console_status_every_n == 0:
            self.get_logger().info(
                (
                    f"cycle={self._cycle_count} dt={dt_sec:.3f}s proc={processing_ms:.2f}ms "
                    f"min_range={min_range:.2f}m best_range={best_range:.2f}m "
                    f"steer={steering:.3f}rad speed={speed:.2f}mps gap_found={gap is not None}"
                )
            )
        if min_range < self.near_collision_distance_m:
            self.get_logger().warn(
                f"near_collision: min_range={min_range:.2f}m < {self.near_collision_distance_m:.2f}m"
            )

    def destroy_node(self) -> bool:
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()