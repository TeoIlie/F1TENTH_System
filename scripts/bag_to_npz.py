#!/usr/bin/env python3
"""Parse a ROS2 bag into a .npz file for sim2real trajectory comparison.

Extracts control commands, Vicon ground truth, VESC telemetry, and odometry
from a bag recorded with:
    cd ~/f1tenth_ws/bags
    ros2 bag record /ackermann_cmd /vrpn_mocap/f110/pose /vrpn_mocap/f110/twist /sensors/core /odom

Usage:
    python3 bag_to_npz.py <bag_dir> [-o output.npz]

Outputs three files:
    <base>_original.npz  — raw variable-rate data from each topic
    <base>_100Hz.npz     — all signals resampled to a uniform 100 Hz grid
                           (commands via zero-order hold, poses/velocities via linear interp)
    <base>_summary.png   — 4-panel validation plot (XY, velocity, steering, yaw)

Also prints PASS/WARN validation checks (ZOH fidelity, interpolation error, rates).
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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip validation plot generation",
    )
    args = parser.parse_args()

    print(f"Reading bag from {args.bag_dir} ...")
    data = read_bag(args.bag_dir)

    for topic, msgs in data.items():
        print(f"  {topic}: {len(msgs)} messages")

    required_topics = [
        "/ackermann_cmd",
        "/vrpn_mocap/f110/pose",
        "/vrpn_mocap/f110/twist",
    ]
    missing = [t for t in required_topics if not data[t]]
    if missing:
        sys.exit(f"Required topics missing from bag: {', '.join(missing)}")

    # --- Control commands from /ackermann_cmd ---
    cmd_t = np.array([t for t, _ in data["/ackermann_cmd"]])
    cmd_speed = np.array([m.drive.speed for _, m in data["/ackermann_cmd"]])
    cmd_steer = np.array([m.drive.steering_angle for _, m in data["/ackermann_cmd"]])

    # --- Vicon pose ---
    vicon_t = np.array([t for t, _ in data["/vrpn_mocap/f110/pose"]])
    vicon_x = np.array([m.pose.position.x for _, m in data["/vrpn_mocap/f110/pose"]])
    vicon_y = np.array([m.pose.position.y for _, m in data["/vrpn_mocap/f110/pose"]])
    vicon_yaw_raw = np.array(
        [yaw_from_quat(m.pose.orientation) for _, m in data["/vrpn_mocap/f110/pose"]]
    )
    vicon_yaw = np.unwrap(vicon_yaw_raw)

    # --- Vicon twist → body-frame velocities ---
    twist_t = np.array([t for t, _ in data["/vrpn_mocap/f110/twist"]])
    twist_world_vx = np.array(
        [m.twist.linear.x for _, m in data["/vrpn_mocap/f110/twist"]]
    )
    twist_world_vy = np.array(
        [m.twist.linear.y for _, m in data["/vrpn_mocap/f110/twist"]]
    )
    twist_r = np.array([m.twist.angular.z for _, m in data["/vrpn_mocap/f110/twist"]])

    # Rotate world velocities to body frame using interpolated Vicon yaw
    yaw_at_twist = np.interp(twist_t, vicon_t, vicon_yaw)
    cos_yaw = np.cos(yaw_at_twist)
    sin_yaw = np.sin(yaw_at_twist)
    vicon_body_vx = twist_world_vx * cos_yaw + twist_world_vy * sin_yaw
    vicon_body_vy = -twist_world_vx * sin_yaw + twist_world_vy * cos_yaw

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
    cmd_idx = np.clip(
        np.searchsorted(cmd_t, sim_t, side="right") - 1, 0, len(cmd_t) - 1
    )
    rs_cmd_speed = cmd_speed[cmd_idx]
    rs_cmd_steer = cmd_steer[cmd_idx]

    # Vicon pose: linear interpolation (continuous signals)
    rs_vicon_x = np.interp(sim_t, vicon_t, vicon_x)
    rs_vicon_y = np.interp(sim_t, vicon_t, vicon_y)
    rs_vicon_yaw = np.interp(sim_t, vicon_t, vicon_yaw)

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

    # --- Validation checks ---
    validate(
        cmd_t=cmd_t,
        cmd_speed=cmd_speed,
        cmd_steer=cmd_steer,
        vicon_t=vicon_t,
        vicon_x=vicon_x,
        vicon_y=vicon_y,
        vicon_yaw=vicon_yaw,
        twist_t=twist_t,
        vicon_body_vx=vicon_body_vx,
        rs_t=rs_t,
        rs_cmd_speed=rs_cmd_speed,
        rs_cmd_steer=rs_cmd_steer,
        rs_vicon_x=rs_vicon_x,
        rs_vicon_y=rs_vicon_y,
        rs_vicon_yaw=rs_vicon_yaw,
        rs_body_vx=rs_body_vx,
        rs_body_vy=rs_body_vy,
        rs_r=rs_r,
        sim_t=sim_t,
        core_t=arrays_to_save.get("core_t"),
        core_speed=arrays_to_save.get("core_speed"),
        rs_core_speed=resampled_arrays.get("rs_core_speed"),
        odom_t=arrays_to_save.get("odom_t"),
        odom_x=arrays_to_save.get("odom_x"),
        odom_y=arrays_to_save.get("odom_y"),
        rs_odom_x=resampled_arrays.get("rs_odom_x"),
        rs_odom_y=resampled_arrays.get("rs_odom_y"),
    )

    # --- Summary plot ---
    if not args.no_plot:
        plot_path = f"{base}_summary.png"
        make_summary_plot(
            plot_path=plot_path,
            cmd_t=cmd_t - sim_t[0],
            cmd_speed=cmd_speed,
            cmd_steer=cmd_steer,
            vicon_t=vicon_t - sim_t[0],
            vicon_x=vicon_x,
            vicon_y=vicon_y,
            vicon_yaw=vicon_yaw,
            twist_t=twist_t - sim_t[0],
            vicon_body_vx=vicon_body_vx,
            rs_t=rs_t,
            rs_cmd_speed=rs_cmd_speed,
            rs_cmd_steer=rs_cmd_steer,
            rs_vicon_x=rs_vicon_x,
            rs_vicon_y=rs_vicon_y,
            rs_vicon_yaw=rs_vicon_yaw,
            rs_body_vx=rs_body_vx,
            core_t=(
                (arrays_to_save["core_t"] - sim_t[0])
                if "core_t" in arrays_to_save
                else None
            ),
            core_speed=arrays_to_save.get("core_speed"),
            rs_core_speed=resampled_arrays.get("rs_core_speed"),
            odom_x=arrays_to_save.get("odom_x"),
            odom_y=arrays_to_save.get("odom_y"),
            rs_odom_x=resampled_arrays.get("rs_odom_x"),
            rs_odom_y=resampled_arrays.get("rs_odom_y"),
        )
        print(f"\nSaved summary plot to {plot_path}")


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def validate(
    *,
    cmd_t,
    cmd_speed,
    cmd_steer,
    vicon_t,
    vicon_x,
    vicon_y,
    vicon_yaw,
    twist_t,
    vicon_body_vx,
    rs_t,
    rs_cmd_speed,
    rs_cmd_steer,
    rs_vicon_x,
    rs_vicon_y,
    rs_vicon_yaw,
    rs_body_vx,
    rs_body_vy,
    rs_r,
    sim_t,
    core_t,
    core_speed,
    rs_core_speed,
    odom_t,
    odom_x,
    odom_y,
    rs_odom_x,
    rs_odom_y,
):
    """Print PASS/WARN validation checks to stdout."""
    print("\n--- Validation ---")

    # 1. Topic rates
    def _rate(t_arr, name):
        if t_arr is None or len(t_arr) < 2:
            return
        dt = np.diff(t_arr)
        rate = 1.0 / np.mean(dt)
        print(f"  {name}: avg {rate:.1f} Hz ({len(t_arr)} msgs)")

    _rate(cmd_t, "/ackermann_cmd")
    _rate(vicon_t, "/vrpn_mocap/f110/pose")
    _rate(twist_t, "/vrpn_mocap/f110/twist")
    _rate(core_t, "/sensors/core")
    _rate(odom_t, "/odom")

    # 2. Time coverage — warn if overlap is much shorter than any topic
    overlap = rs_t[-1]
    for name, t_arr in [("cmd", cmd_t), ("vicon", vicon_t), ("twist", twist_t)]:
        dur = t_arr[-1] - t_arr[0]
        if overlap < 0.5 * dur:
            print(
                f"  WARN: resampled window ({overlap:.2f}s) is <50% of {name} duration ({dur:.2f}s)"
            )

    # 3. ZOH fidelity — at each 100Hz grid point, the resampled value should equal the
    #    last original command that arrived before that grid point.
    for sig_name, orig_t, orig_vals, rs_vals in [
        ("cmd_speed", cmd_t, cmd_speed, rs_cmd_speed),
        ("cmd_steer", cmd_t, cmd_steer, rs_cmd_steer),
    ]:
        expected_idx = np.clip(
            np.searchsorted(orig_t, sim_t, side="right") - 1, 0, len(orig_vals) - 1
        )
        expected = orig_vals[expected_idx]
        max_err = np.max(np.abs(expected - rs_vals))
        status = "PASS" if max_err < 1e-6 else "WARN"
        print(f"  {status}: ZOH {sig_name} max error vs expected = {max_err:.2e}")

    # 4. Interpolation fidelity — resampled should be close to original at original sample times
    for sig_name, orig_t, orig_vals, rs_vals in [
        ("vicon_x", vicon_t, vicon_x, rs_vicon_x),
        ("vicon_y", vicon_t, vicon_y, rs_vicon_y),
        ("vicon_yaw", vicon_t, vicon_yaw, rs_vicon_yaw),
    ]:
        mask = (orig_t >= sim_t[0]) & (orig_t <= sim_t[-1])
        if not np.any(mask):
            continue
        rs_at_orig = np.interp(orig_t[mask], sim_t, rs_vals)
        max_err = np.max(np.abs(orig_vals[mask] - rs_at_orig))
        thresh = 0.01 if "yaw" not in sig_name else 0.02  # 1cm or ~1 deg
        status = "PASS" if max_err < thresh else "WARN"
        unit = "rad" if "yaw" in sig_name else "m"
        print(f"  {status}: interp {sig_name} max error = {max_err:.4f} {unit}")

    # 5. Velocity consistency — did the car actually move?
    max_cmd = np.max(np.abs(cmd_speed))
    max_vicon_vx = np.max(np.abs(vicon_body_vx)) if len(vicon_body_vx) > 0 else 0
    if max_cmd > 0.1:
        ratio = max_vicon_vx / max_cmd
        status = "PASS" if ratio > 0.3 else "WARN"
        print(
            f"  {status}: peak vicon_vx / peak cmd_speed = {ratio:.2f} "
            f"({max_vicon_vx:.2f} / {max_cmd:.2f} m/s)"
        )


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def make_summary_plot(
    *,
    plot_path,
    cmd_t,
    cmd_speed,
    cmd_steer,
    vicon_t,
    vicon_x,
    vicon_y,
    vicon_yaw,
    twist_t,
    vicon_body_vx,
    rs_t,
    rs_cmd_speed,
    rs_cmd_steer,
    rs_vicon_x,
    rs_vicon_y,
    rs_vicon_yaw,
    rs_body_vx,
    core_t,
    core_speed,
    rs_core_speed,
    odom_x,
    odom_y,
    rs_odom_x,
    rs_odom_y,
):
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))

    # --- Panel 1: XY trajectory ---
    ax = axes[0]
    ax.scatter(vicon_x, vicon_y, s=3, alpha=0.4, label="Vicon original", zorder=2)
    ax.plot(rs_vicon_x, rs_vicon_y, linewidth=1, label="Vicon 100 Hz", zorder=3)
    if odom_x is not None:
        ax.plot(odom_x, odom_y, linewidth=0.8, alpha=0.6, label="Odom original")
    if rs_odom_x is not None:
        ax.plot(
            rs_odom_x,
            rs_odom_y,
            linewidth=0.8,
            linestyle="--",
            alpha=0.6,
            label="Odom 100 Hz",
        )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Panel 2: Velocity profile ---
    ax = axes[1]
    ax.step(
        cmd_t,
        cmd_speed,
        where="post",
        linewidth=0.8,
        color="C0",
        alpha=0.5,
        label="cmd_speed original (step)",
    )
    ax.plot(rs_t, rs_cmd_speed, linewidth=1, color="C0", label="cmd_speed 100 Hz")
    ax.scatter(
        twist_t,
        vicon_body_vx,
        s=2,
        alpha=0.3,
        color="C1",
        label="vicon_body_vx original",
    )
    ax.plot(rs_t, rs_body_vx, linewidth=1, color="C1", label="vicon_body_vx 100 Hz")
    if core_t is not None:
        ax.scatter(
            core_t, core_speed, s=2, alpha=0.3, color="C2", label="VESC speed original"
        )
    if rs_core_speed is not None:
        ax.plot(rs_t, rs_core_speed, linewidth=1, color="C2", label="VESC speed 100 Hz")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Velocity Profile — Command vs Measured")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Panel 3: Steering command ---
    ax = axes[2]
    ax.step(
        cmd_t,
        np.degrees(cmd_steer),
        where="post",
        linewidth=0.8,
        color="C4",
        alpha=0.5,
        label="cmd_steer original (step)",
    )
    ax.plot(
        rs_t,
        np.degrees(rs_cmd_steer),
        linewidth=1,
        color="C4",
        label="cmd_steer 100 Hz",
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Steering angle (deg)")
    ax.set_title("Steering Command")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Panel 4: Yaw ---
    ax = axes[3]
    ax.scatter(
        vicon_t,
        np.degrees(vicon_yaw),
        s=3,
        alpha=0.4,
        color="C3",
        label="Vicon yaw original",
    )
    ax.plot(
        rs_t,
        np.degrees(rs_vicon_yaw),
        linewidth=1,
        color="C3",
        label="Vicon yaw 100 Hz (unwrapped)",
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw (deg)")
    ax.set_title("Heading")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
