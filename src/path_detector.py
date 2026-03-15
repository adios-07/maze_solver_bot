#!/usr/bin/env python3
"""
Path Detector Node
------------------
Reads LiDAR scan data and logs which directions the robot can move.
Divides the scan into sectors (forward, left, right, backward) and
checks if the minimum range in each sector exceeds a clearance threshold.

Subscribes:
  /scan   (sensor_msgs/LaserScan)

Output:
  Logger prints available directions e.g. "Available paths: forward, left"
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan


class PathDetector(Node):
    def __init__(self):
        super().__init__('path_detector')

        # ── Tunable parameters ──────────────────────────────────────────────
        # Minimum clearance distance to consider a direction passable (metres)
        self.declare_parameter('clearance_threshold', 0.5)

        # Angular half-width of each direction sector (degrees)
        # e.g. 30 means forward = -30° to +30°
        self.declare_parameter('sector_half_width_deg', 30.0)
        # ────────────────────────────────────────────────────────────────────

        self.clearance_threshold    = self.get_parameter('clearance_threshold').value
        self.sector_half_width_deg  = self.get_parameter('sector_half_width_deg').value

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            f'PathDetector started | clearance={self.clearance_threshold}m '
            f'| sector_half_width={self.sector_half_width_deg}°'
        )

    def get_sector_min(self, ranges, angles_deg, center_deg, half_width_deg):
        """Return the minimum valid range within a directional sector."""
        low  = center_deg - half_width_deg
        high = center_deg + half_width_deg
        mask = (angles_deg >= low) & (angles_deg <= high)
        sector_ranges = ranges[mask]

        # Filter out zeros (invalid returns)
        valid = sector_ranges[sector_ranges > 0]
        if len(valid) == 0:
            return float('inf')  # no data = assume clear
        return float(np.min(valid))

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Replace inf/nan with 0 (treated as invalid)
        ranges = np.where(np.isfinite(ranges), ranges, 0.0)

        # Build angle array in degrees for each range reading
        # ROS LiDAR: angle_min is the start angle, increments by angle_increment
        n = len(ranges)
        angles_rad = np.linspace(msg.angle_min, msg.angle_max, n)
        angles_deg = np.degrees(angles_rad)

        hw = self.sector_half_width_deg
        threshold = self.clearance_threshold

        # Check each direction
        # Convention: 0° = forward, 90° = left, -90° = right, 180°/−180° = backward
        directions = {
            'forward':  self.get_sector_min(ranges, angles_deg,   0.0, hw),
            'left':     self.get_sector_min(ranges, angles_deg,  90.0, hw),
            'right':    self.get_sector_min(ranges, angles_deg, -90.0, hw),
            'backward': self.get_sector_min(ranges, angles_deg, 180.0, hw),
        }

        available = [
            direction for direction, min_range in directions.items()
            if min_range > threshold
        ]

        if available:
            self.get_logger().info(
                f'Available paths: {", ".join(available)} | '
                f'Clearances → ' +
                ' | '.join(f'{d}: {r:.2f}m' for d, r in directions.items()),
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().warn(
                'No available paths! Robot is surrounded.',
                throttle_duration_sec=0.5
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()