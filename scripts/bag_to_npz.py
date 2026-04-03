#!/usr/bin/env python3
"""Parse a ROS2 bag into a .npz file for sim2real trajectory comparison.

Extracts control commands, Vicon ground truth, VESC telemetry, and odometry
from a bag recorded with:
    cd ~/f1tenth_ws/bags
    ros2 bag record /ackermann_cmd /vrpn_mocap/f110/pose /vrpn_mocap/f110/twist /sensors/core /odom

Usage:
    python3 bag_to_npz.py <bag_dir> [-o output.npz]

Outputs two files:
    <base>_original.npz  — raw variable-rate data from each topic
    <base>_100Hz.npz     — all signals resampled to a uniform 100 Hz grid
                           (commands via zero-order hold, poses/velocities via linear interp)

Default base is ~/f1tenth_ws/data/<bag_name> (creates ~/f1tenth_ws/data/ if needed).

Example:
    python3 bag_to_npz.py ~/f1tenth_ws/bags/rosbag2_2026_04_03-16_18_43
    python3 bag_to_npz.py ~/f1tenth_ws/bags/rosbag2_2026_04_03-16_18_43 -o custom
"""

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped

# ---------------------------------------------------------------------------
# Bag reading via sqlite3 (same pattern as validate_obs.py)
# ---------------------------------------------------------------------------

TYPE_MAP = {
    "ackermann_msgs/msg/AckermannDriveStamped": AckermannDriveStamped,
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "geometry_msgs/msg/TwistStamped": TwistStamped,
    "nav_msgs/msg/Odometry": Odometry,
    "vesc_msgs/msg/VescStateStamped": VescStateStamped,
}

TOPICS_OF_INTEREST = {
    "/ackermann_cmd",
    "/vrpn_mocap/f110/pose",
    "/vrpn_mocap/f110/twist",
    "/sensors/core",
    "/odom",
}


def read_bag(bag_dir: str) -> dict[str, list[tuple[float, object]]]:
    """Read a ROS2 bag (sqlite3) and return {topic: [(t_sec, msg), ...]}."""
    bag_path = Path(bag_dir)
    db_files = sorted(bag_path.glob("*.db3"))
    if not db_files:
        sys.exit(f"No .db3 files found in {bag_path}")

    results: dict[str, list] = {t: [] for t in TOPICS_OF_INTEREST}

    for db_file in db_files:
        conn = sqlite3.connect(str(db_file))
        cursor = conn.cursor()

        cursor.execute("SELECT id, name, type FROM topics")
        topic_map = {}
        for tid, name, typename in cursor.fetchall():
            if name in TOPICS_OF_INTEREST and typename in TYPE_MAP:
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

# VESC config (from vesc.yaml)
SPEED_TO_ERPM_GAIN = 4600.0
WHEEL_RADIUS = 0.049


def yaw_from_quat(q) -> float:
    from transforms3d.euler import quat2euler

    _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
    return yaw


def body_vel(world_vx: float, world_vy: float, yaw: float):
    """Rotate world-frame velocity into body frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return world_vx * c + world_vy * s, -world_vx * s + world_vy * c


def speed_from_erpm(erpm: float) -> float:
    return erpm / SPEED_TO_ERPM_GAIN


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Parse a ROS2 bag into .npz for sim2real comparison"
    )
    parser.add_argument("bag_dir", help="Path to the ROS2 bag directory")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output base name; produces <base>_original.npz and <base>_100Hz.npz",
    )
    args = parser.parse_args()

    print(f"Reading bag from {args.bag_dir} ...")
    data = read_bag(args.bag_dir)

    for topic, msgs in data.items():
        print(f"  {topic}: {len(msgs)} messages")

    if not data["/ackermann_cmd"]:
        sys.exit("No /ackermann_cmd messages found in bag")
    if not data["/vrpn_mocap/f110/pose"]:
        sys.exit("No /vrpn_mocap/f110/pose messages found in bag")

    # --- Control commands from /ackermann_cmd ---
    cmd_t = np.array([t for t, _ in data["/ackermann_cmd"]])
    cmd_speed = np.array([m.drive.speed for _, m in data["/ackermann_cmd"]])
    cmd_steer = np.array([m.drive.steering_angle for _, m in data["/ackermann_cmd"]])

    # --- Vicon pose ---
    vicon_t = np.array([t for t, _ in data["/vrpn_mocap/f110/pose"]])
    vicon_x = np.array([m.pose.position.x for _, m in data["/vrpn_mocap/f110/pose"]])
    vicon_y = np.array([m.pose.position.y for _, m in data["/vrpn_mocap/f110/pose"]])
    vicon_yaw = np.array(
        [yaw_from_quat(m.pose.orientation) for _, m in data["/vrpn_mocap/f110/pose"]]
    )

    # --- Vicon twist → body-frame velocities ---
    twist_t = np.array([t for t, _ in data["/vrpn_mocap/f110/twist"]])
    twist_world_vx = np.array(
        [m.twist.linear.x for _, m in data["/vrpn_mocap/f110/twist"]]
    )
    twist_world_vy = np.array(
        [m.twist.linear.y for _, m in data["/vrpn_mocap/f110/twist"]]
    )
    twist_r = np.array([m.twist.angular.z for _, m in data["/vrpn_mocap/f110/twist"]])

    # Rotate world velocities to body frame using nearest Vicon yaw
    vicon_body_vx = np.empty_like(twist_t)
    vicon_body_vy = np.empty_like(twist_t)
    for i, t in enumerate(twist_t):
        yaw_idx = np.argmin(np.abs(vicon_t - t))
        yaw = vicon_yaw[yaw_idx]
        vx, vy = body_vel(twist_world_vx[i], twist_world_vy[i], yaw)
        vicon_body_vx[i] = vx
        vicon_body_vy[i] = vy

    # --- VESC telemetry from /sensors/core ---
    arrays_to_save = {}
    if data["/sensors/core"]:
        core_t = np.array([t for t, _ in data["/sensors/core"]])
        core_speed = np.array(
            [speed_from_erpm(m.state.speed) for _, m in data["/sensors/core"]]
        )
        core_voltage = np.array(
            [m.state.voltage_input for _, m in data["/sensors/core"]]
        )
        arrays_to_save.update(
            core_t=core_t, core_speed=core_speed, core_voltage=core_voltage
        )
    else:
        print("  WARNING: No /sensors/core messages — VESC telemetry will be missing")

    # --- Odometry ---
    if data["/odom"]:
        odom_t = np.array([t for t, _ in data["/odom"]])
        odom_x = np.array([m.pose.pose.position.x for _, m in data["/odom"]])
        odom_y = np.array([m.pose.pose.position.y for _, m in data["/odom"]])
        odom_vx = np.array([m.twist.twist.linear.x for _, m in data["/odom"]])
        arrays_to_save.update(
            odom_t=odom_t, odom_x=odom_x, odom_y=odom_y, odom_vx=odom_vx
        )
    else:
        print("  WARNING: No /odom messages — odometry will be missing")

    # --- Determine output paths ---
    if args.output:
        base = args.output.removesuffix(".npz")
    else:
        data_dir = Path.home() / "f1tenth_ws" / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        bag_name = Path(args.bag_dir).resolve().name
        base = str(data_dir / bag_name)

    original_path = f"{base}_original.npz"
    resampled_path = f"{base}_100Hz.npz"

    # --- Save original (variable-rate) ---
    np.savez(
        original_path,
        # Control commands
        cmd_t=cmd_t,
        cmd_speed=cmd_speed,
        cmd_steer=cmd_steer,
        # Vicon pose
        vicon_t=vicon_t,
        vicon_x=vicon_x,
        vicon_y=vicon_y,
        vicon_yaw=vicon_yaw,
        # Vicon twist (body frame)
        twist_t=twist_t,
        vicon_body_vx=vicon_body_vx,
        vicon_body_vy=vicon_body_vy,
        vicon_r=twist_r,
        # VESC + odom (if present)
        **arrays_to_save,
    )

    print(f"\nSaved original to {original_path}")
    print(f"  Commands: {len(cmd_t)} samples, " f"{cmd_t[-1] - cmd_t[0]:.2f}s duration")
    print(
        f"  Vicon:    {len(vicon_t)} samples, "
        f"{vicon_t[-1] - vicon_t[0]:.2f}s duration"
    )
    if data["/sensors/core"]:
        print(f"  VESC:     {len(core_t)} samples")
    if data["/odom"]:
        print(f"  Odom:     {len(odom_t)} samples")

    # --- Resample to 100 Hz ---
    t_start = max(cmd_t[0], vicon_t[0], twist_t[0])
    t_end = min(cmd_t[-1], vicon_t[-1], twist_t[-1])
    sim_t = np.arange(t_start, t_end, 0.01)

    # Commands: zero-order hold (hold last received command)
    cmd_idx = np.clip(np.searchsorted(cmd_t, sim_t, side="right") - 1, 0, len(cmd_t) - 1)
    rs_cmd_speed = cmd_speed[cmd_idx]
    rs_cmd_steer = cmd_steer[cmd_idx]

    # Vicon pose: linear interpolation (continuous signals)
    rs_vicon_x = np.interp(sim_t, vicon_t, vicon_x)
    rs_vicon_y = np.interp(sim_t, vicon_t, vicon_y)
    rs_vicon_yaw = np.interp(sim_t, vicon_t, np.unwrap(vicon_yaw))

    # Vicon twist: linear interpolation
    rs_body_vx = np.interp(sim_t, twist_t, vicon_body_vx)
    rs_body_vy = np.interp(sim_t, twist_t, vicon_body_vy)
    rs_r = np.interp(sim_t, twist_t, twist_r)

    resampled_arrays = {}
    if data["/sensors/core"]:
        rs_core_idx = np.clip(
            np.searchsorted(core_t, sim_t, side="right") - 1, 0, len(core_t) - 1
        )
        resampled_arrays["rs_core_speed"] = core_speed[rs_core_idx]
    if data["/odom"]:
        resampled_arrays["rs_odom_x"] = np.interp(sim_t, odom_t, odom_x)
        resampled_arrays["rs_odom_y"] = np.interp(sim_t, odom_t, odom_y)
        resampled_arrays["rs_odom_vx"] = np.interp(sim_t, odom_t, odom_vx)

    # Normalize time to start at 0
    rs_t = sim_t - sim_t[0]

    np.savez(
        resampled_path,
        t=rs_t,
        cmd_speed=rs_cmd_speed,
        cmd_steer=rs_cmd_steer,
        vicon_x=rs_vicon_x,
        vicon_y=rs_vicon_y,
        vicon_yaw=rs_vicon_yaw,
        vicon_body_vx=rs_body_vx,
        vicon_body_vy=rs_body_vy,
        vicon_r=rs_r,
        **resampled_arrays,
    )

    print(f"\nSaved resampled to {resampled_path}")
    print(f"  {len(rs_t)} steps at 100 Hz, {rs_t[-1]:.2f}s duration")


if __name__ == "__main__":
    main()
