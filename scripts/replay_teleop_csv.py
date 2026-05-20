#!/usr/bin/env python3
"""Publish a teleop waypoint CSV onto /teleop at a fixed rate.

Companion to teleop_bag_to_waypoints.py — replays the resampled (t, speed,
steering_angle) table so the mux/VESC see a smooth, regular command stream
instead of the bursty cadence produced by `ros2 bag play` on a joy_teleop bag.

Usage:
    python3 replay_teleop_csv.py <csv_path> [--topic /teleop] [--rate 100] [--loops 1]

The CSV must have columns: t_rel,speed,steering_angle (as written by
teleop_bag_to_waypoints.py). Publication ends one tick after the last row.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class TeleopReplay(Node):
    def __init__(self, topic: str, rate: float, waypoints: np.ndarray, loops: int):
        super().__init__("teleop_replay")
        self._pub = self.create_publisher(AckermannDriveStamped, topic, 10)
        self._wp = waypoints  # shape (N, 3): t_rel, speed, steer
        self._loops = loops
        self._loop_idx = 0
        self._i = 0
        self._dt = 1.0 / rate
        self._start_wall = None
        self._duration = float(waypoints[-1, 0])
        self.get_logger().info(
            f"Replaying {len(waypoints)} samples on {topic} at {rate:.1f} Hz "
            f"({self._duration:.2f}s per loop, {loops} loop(s))"
        )
        self.create_timer(self._dt, self._tick)

    def _tick(self):
        now = time.monotonic()
        if self._start_wall is None:
            self._start_wall = now
        t = now - self._start_wall

        if t > self._duration:
            self._loop_idx += 1
            if self._loop_idx >= self._loops:
                # Publish a final zero command for safety, then shut down
                msg = AckermannDriveStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                self._pub.publish(msg)
                self.get_logger().info(
                    "Replay finished — published zero command, shutting down"
                )
                rclpy.shutdown()
                return
            self._start_wall = now
            self._i = 0
            t = 0.0

        # Advance pointer to the row whose t_rel <= t (ZOH lookup)
        while self._i + 1 < len(self._wp) and self._wp[self._i + 1, 0] <= t:
            self._i += 1

        speed = float(self._wp[self._i, 1])
        steer = float(self._wp[self._i, 2])

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description="Replay a teleop waypoint CSV onto a topic"
    )
    parser.add_argument("csv_path", help="Path to teleop_waypoints.csv")
    parser.add_argument(
        "--topic", default="/teleop", help="Topic to publish on (default /teleop)"
    )
    parser.add_argument(
        "--rate", type=float, default=100.0, help="Publish rate in Hz (default 100)"
    )
    parser.add_argument(
        "--loops", type=int, default=1, help="Number of times to replay the sequence"
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=1.0,
        help="Seconds to wait before starting (lets subscribers connect; default 1.0)",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    if not csv_path.is_file():
        sys.exit(f"CSV not found: {csv_path}")

    # Tolerate both old (uncommented column header) and new (commented) CSVs:
    # treat anything that isn't a numeric row as a header by filtering manually.
    rows = []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                rows.append([float(x) for x in line.split(",")])
            except ValueError:
                continue  # column-name row
    wp = np.array(rows)
    if wp.ndim != 2 or wp.shape[1] != 3:
        sys.exit(
            f"Expected 3 columns (t_rel,speed,steering_angle), got shape {wp.shape}"
        )

    rclpy.init(args=sys.argv)
    node = TeleopReplay(args.topic, args.rate, wp, args.loops)
    if args.delay > 0:
        # Sleep after the publisher exists so DDS discovery can match subscribers.
        print(f"Sleeping {args.delay}s for subscriber discovery ...")
        time.sleep(args.delay)
        node._start_wall = None  # reset so the timer rebases t=0 to post-sleep
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
