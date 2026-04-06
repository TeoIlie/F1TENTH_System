#!/usr/bin/env python3
"""Compare /odom XY trajectory against /vrpn_mocap/f110/pose ground truth.

Reads a ROS2 bag recorded with `ros2 bag record -a`, extracts odometry and
Vicon poses, aligns them in time, computes RMSE, and saves a comparison plot.

Usage:
    python3 measure_odom.py <bag_dir> [-o output.png] [--odom-topic /early_fusion/odom]

The odom frame is arbitrary (starts at origin), so we align it to Vicon by
matching the initial position and heading, then computing error in Vicon frame.
"""

import argparse
import sqlite3
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from transforms3d.euler import quat2euler

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------

TYPE_MAP = {
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "nav_msgs/msg/Odometry": Odometry,
}

DEFAULT_ODOM_TOPIC = "/odom"
VICON_TOPIC = "/vrpn_mocap/f110/pose"


def read_bag(
    bag_dir: str, odom_topic: str = DEFAULT_ODOM_TOPIC
) -> dict[str, list[tuple[float, object]]]:
    topics = {VICON_TOPIC, odom_topic}
    bag_path = Path(bag_dir)
    db_files = sorted(bag_path.glob("*.db3"))
    if not db_files:
        sys.exit(f"No .db3 files found in {bag_path}")

    results: dict[str, list] = {t: [] for t in topics}

    for db_file in db_files:
        conn = sqlite3.connect(str(db_file))
        cursor = conn.cursor()

        cursor.execute("SELECT id, name, type FROM topics")
        topic_map = {}
        for tid, name, typename in cursor.fetchall():
            if name in topics and typename in TYPE_MAP:
                topic_map[tid] = (name, TYPE_MAP[typename])

        cursor.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        )
        for tid, timestamp, data in cursor.fetchall():
            if tid not in topic_map:
                continue
            name, msg_class = topic_map[tid]
            msg = deserialize_message(data, msg_class)
            t_sec = timestamp * 1e-9
            results[name].append((t_sec, msg))

        conn.close()

    return results


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def yaw_from_quat(q) -> float:
    _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
    return yaw


def rotate_2d(x, y, angle):
    """Rotate (x, y) by angle."""
    c, s = np.cos(angle), np.sin(angle)
    return c * x - s * y, s * x + c * y


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Compare odometry trajectory against Vicon ground truth"
    )
    parser.add_argument("bag_dir", help="Path to the ROS2 bag directory")
    parser.add_argument("-o", "--output", default=None, help="Output plot path")
    parser.add_argument(
        "--odom-topic",
        default=DEFAULT_ODOM_TOPIC,
        help=f"Odometry topic to compare against Vicon (default: {DEFAULT_ODOM_TOPIC})",
    )
    args = parser.parse_args()

    odom_topic = args.odom_topic

    print(f"Reading bag from {args.bag_dir} ...")
    print(f"  Odom topic: {odom_topic}")
    data = read_bag(args.bag_dir, odom_topic=odom_topic)

    for topic in [VICON_TOPIC, odom_topic]:
        print(f"  {topic}: {len(data[topic])} messages")

    if not data[odom_topic]:
        sys.exit(f"No {odom_topic} messages found in bag")
    if not data[VICON_TOPIC]:
        sys.exit(f"No {VICON_TOPIC} messages found in bag")

    # --- Extract arrays ---
    vicon_t = np.array([t for t, _ in data[VICON_TOPIC]])
    vicon_x = np.array([m.pose.position.x for _, m in data[VICON_TOPIC]])
    vicon_y = np.array([m.pose.position.y for _, m in data[VICON_TOPIC]])
    vicon_yaw = np.unwrap(
        [yaw_from_quat(m.pose.orientation) for _, m in data[VICON_TOPIC]]
    )

    odom_t = np.array([t for t, _ in data[odom_topic]])
    odom_x = np.array([m.pose.pose.position.x for _, m in data[odom_topic]])
    odom_y = np.array([m.pose.pose.position.y for _, m in data[odom_topic]])
    odom_yaw = np.unwrap(
        [yaw_from_quat(m.pose.pose.orientation) for _, m in data[odom_topic]]
    )

    # --- Resample to common time grid ---
    t_start = max(vicon_t[0], odom_t[0])
    t_end = min(vicon_t[-1], odom_t[-1])
    if t_end <= t_start:
        sys.exit("No time overlap between /odom and Vicon")

    t_common = np.arange(t_start, t_end, 0.01)  # 100 Hz grid

    vx = np.interp(t_common, vicon_t, vicon_x)
    vy = np.interp(t_common, vicon_t, vicon_y)
    vyaw = np.interp(t_common, vicon_t, vicon_yaw)

    ox = np.interp(t_common, odom_t, odom_x)
    oy = np.interp(t_common, odom_t, odom_y)
    oyaw = np.interp(t_common, odom_t, odom_yaw)

    # --- Align odom to Vicon frame ---
    # Rotate and translate odom so that its initial pose matches Vicon's initial pose.
    delta_yaw = vyaw[0] - oyaw[0]
    ox_centered = ox - ox[0]
    oy_centered = oy - oy[0]
    ox_rot, oy_rot = rotate_2d(ox_centered, oy_centered, delta_yaw)
    ox_aligned = ox_rot + vx[0]
    oy_aligned = oy_rot + vy[0]

    # --- Compute errors ---
    err_x = vx - ox_aligned
    err_y = vy - oy_aligned
    err_dist = np.sqrt(err_x**2 + err_y**2)

    rmse = np.sqrt(np.mean(err_dist**2))
    max_err = np.max(err_dist)
    mean_err = np.mean(err_dist)

    t_rel = t_common - t_common[0]
    duration = t_rel[-1]

    print(f"\n--- Odometry vs Vicon Results ---")
    print(f"  Duration:        {duration:.2f} s")
    print(f"  Samples:         {len(t_common)}")
    print(f"  RMSE:            {rmse:.4f} m")
    print(f"  Mean error:      {mean_err:.4f} m")
    print(f"  Max error:       {max_err:.4f} m")
    print(f"  Final error:     {err_dist[-1]:.4f} m")

    # Total distance traveled (Vicon ground truth)
    total_dist = np.sum(np.sqrt(np.diff(vx) ** 2 + np.diff(vy) ** 2))
    print(f"  Total distance:  {total_dist:.2f} m")
    print(
        f"  RMSE / distance: {rmse / total_dist * 100:.2f}%" if total_dist > 0 else ""
    )

    # --- Plot ---
    if args.output:
        plot_path = args.output
    else:
        data_dir = Path.home() / "f1tenth_ws" / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        bag_name = Path(args.bag_dir).resolve().name
        topic_suffix = odom_topic.strip("/").replace("/", "_")
        plot_path = str(data_dir / f"{bag_name}_{topic_suffix}_vs_vicon.png")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Panel 1: XY trajectory
    ax = axes[0, 0]
    ax.plot(vx, vy, linewidth=1.5, label="Vicon (ground truth)", color="C0")
    ax.plot(
        ox_aligned,
        oy_aligned,
        linewidth=1.5,
        linestyle="--",
        label="Odom (aligned)",
        color="C1",
    )
    ax.scatter(
        [vx[0]], [vy[0]], marker="o", s=80, color="green", zorder=5, label="Start"
    )
    ax.scatter([vx[-1]], [vy[-1]], marker="x", s=80, color="red", zorder=5, label="End")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2: Position error over time
    ax = axes[0, 1]
    ax.plot(t_rel, err_dist, linewidth=1, color="C3")
    ax.axhline(
        rmse, color="C3", linestyle="--", alpha=0.7, label=f"RMSE = {rmse:.4f} m"
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (m)")
    ax.set_title("Euclidean Position Error Over Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: X and Y over time
    ax = axes[1, 0]
    ax.plot(t_rel, vx, linewidth=1, label="Vicon X", color="C0")
    ax.plot(
        t_rel,
        ox_aligned,
        linewidth=1,
        linestyle="--",
        label="Odom X",
        color="C0",
        alpha=0.7,
    )
    ax.plot(t_rel, vy, linewidth=1, label="Vicon Y", color="C1")
    ax.plot(
        t_rel,
        oy_aligned,
        linewidth=1,
        linestyle="--",
        label="Odom Y",
        color="C1",
        alpha=0.7,
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("X/Y Position Over Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4: Error components
    ax = axes[1, 1]
    ax.plot(t_rel, err_x, linewidth=1, label="X error", color="C0")
    ax.plot(t_rel, err_y, linewidth=1, label="Y error", color="C1")
    ax.axhline(0, color="gray", linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("X/Y Error Components")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        f"{odom_topic} vs Vicon  |  RMSE: {rmse:.4f} m  |  Max: {max_err:.4f} m  |  "
        f"Distance: {total_dist:.2f} m",
        fontsize=12,
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    print(f"\nSaved plot to {plot_path}")


if __name__ == "__main__":
    main()
