import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ReactiveFollowGap(Node):
    """Reactive Follow-the-Gap controller publishing Ackermann drive commands."""

    def __init__(self) -> None:
        super().__init__("gap_follower")

        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("max_lidar_dist", 10.0)
        self.declare_parameter("preprocess_window", 3)
        self.declare_parameter("disparity_threshold", 0.4)
        self.declare_parameter("safety_radius_m", 0.28)
        self.declare_parameter("gap_min_range_m", 1.2)
        self.declare_parameter("max_steering_angle", 0.4189)
        self.declare_parameter("speed_straight", 4.5)
        self.declare_parameter("speed_corner", 2.5)
        self.declare_parameter("corner_angle_threshold", 0.22)
        self.declare_parameter("fov_half_angle_rad", 1.57)

        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.max_lidar_dist = float(self.get_parameter("max_lidar_dist").value)
        self.preprocess_window = max(1, int(self.get_parameter("preprocess_window").value))
        self.disparity_threshold = float(self.get_parameter("disparity_threshold").value)
        self.safety_radius_m = float(self.get_parameter("safety_radius_m").value)
        self.gap_min_range_m = float(self.get_parameter("gap_min_range_m").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.speed_straight = float(self.get_parameter("speed_straight").value)
        self.speed_corner = float(self.get_parameter("speed_corner").value)
        self.corner_angle_threshold = float(self.get_parameter("corner_angle_threshold").value)
        self.fov_half_angle_rad = float(self.get_parameter("fov_half_angle_rad").value)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

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

        diffs = np.abs(np.diff(filtered))
        disparity_ids = np.flatnonzero(diffs > self.disparity_threshold)

        for idx in disparity_ids:
            left = filtered[idx]
            right = filtered[idx + 1]
            close_idx = idx if left < right else idx + 1
            close_dist = max(0.05, float(filtered[close_idx]))
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

        max_val = float(np.max(gap_ranges))
        candidate_local = np.flatnonzero(gap_ranges >= (max_val - 0.15))
        if candidate_local.size == 0:
            return (start + end) // 2
        return int(start + np.median(candidate_local))

    def lidar_callback(self, msg: LaserScan) -> None:
        proc_ranges, proc_angles, _ = self.preprocess_lidar(msg)
        if proc_ranges.size == 0:
            return

        safe_ranges = self.apply_safety_bubble(proc_ranges, msg.angle_increment)
        gap = self.find_max_gap(safe_ranges)
        if gap is None:
            best_idx = int(np.argmax(proc_ranges))
        else:
            best_idx = self.find_best_point(safe_ranges, gap)

        steering = float(np.clip(proc_angles[best_idx], -self.max_steering_angle, self.max_steering_angle))
        speed = self.speed_corner if abs(steering) > self.corner_angle_threshold else self.speed_straight

        cmd = AckermannDriveStamped()
        cmd.header = msg.header
        cmd.drive.steering_angle = steering
        cmd.drive.speed = speed
        self.drive_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()