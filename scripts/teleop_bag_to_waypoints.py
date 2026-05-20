#!/usr/bin/env python3
"""Convert a /teleop ROS2 bag into a uniform-rate CSV of (t, speed, steer) waypoints.

The recorded bag is variable-rate and bursty (joy_teleop only publishes on stick
events), which causes choppy `ros2 bag play` replay. This script resamples it
onto a uniform 100 Hz grid (matching recovery.yaml's `rate`) so a downstream
replay node can publish at a fixed cadence the mux/VESC expect.

Usage:
    python3 teleop_bag_to_waypoints.py <bag_dir> [--rate 100] [--smooth-steer-tau 0.02]
                                                 [--start 1.5] [--end 8.0]

Examples:
    # Default: 100 Hz, light smoothing on both channels
    python3 teleop_bag_to_waypoints.py ~/bags/teleop_2026_05_20

    # Trim the output to seconds 1.5 .. 8.0 of the active window
    python3 teleop_bag_to_waypoints.py ~/bags/teleop_2026_05_20 --start 1.5 --end 8.0

    # Disable steering smoothing, drop output to 50 Hz
    python3 teleop_bag_to_waypoints.py ~/bags/teleop_2026_05_20 --rate 50 --smooth-steer-tau 0

Output: <bag_dir>/teleop_waypoints.csv with columns:
    t_rel, speed, steering_angle

t_rel starts at 0 at the first non-zero speed command. The window is trimmed
to [first_nonzero, last_nonzero] of speed. Steering is low-pass filtered with
a 1st-order IIR (default tau=20 ms) to remove stick noise — set tau=0 to disable.

Also writes <bag_dir>/teleop_waypoints.png comparing raw vs resampled signals.
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

from ackermann_msgs.msg import AckermannDriveStamped

TELEOP_TOPIC = "/teleop"
TELEOP_TYPE = "ackermann_msgs/msg/AckermannDriveStamped"


def read_teleop(bag_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (t, speed, steer) arrays read from /teleop in a sqlite3 bag."""
    db_files = sorted(bag_dir.glob("*.db3"))
    if not db_files:
        sys.exit(f"No .db3 files found in {bag_dir}")

    ts, speeds, steers = [], [], []
    for db_file in db_files:
        conn = sqlite3.connect(str(db_file))
        cur = conn.cursor()
        cur.execute("SELECT id, name, type FROM topics")
        topic_id = None
        for tid, name, typename in cur.fetchall():
            if name == TELEOP_TOPIC and typename == TELEOP_TYPE:
                topic_id = tid
                break
        if topic_id is None:
            conn.close()
            continue

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
        for timestamp, data in cur.fetchall():
            msg = deserialize_message(data, AckermannDriveStamped)
            # Prefer header stamp (when teleop generated the cmd); fall back to bag stamp
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp <= 0:
                stamp = timestamp * 1e-9
            ts.append(stamp)
            speeds.append(msg.drive.speed)
            steers.append(msg.drive.steering_angle)
        conn.close()

    if not ts:
        sys.exit(f"No {TELEOP_TOPIC} messages found in {bag_dir}")

    return np.array(ts), np.array(speeds), np.array(steers)


def first_last_nonzero(speed: np.ndarray, eps: float = 1e-3) -> tuple[int, int]:
    nz = np.where(np.abs(speed) > eps)[0]
    if len(nz) == 0:
        sys.exit("All speed commands are zero — nothing to replay.")
    return int(nz[0]), int(nz[-1])


def lowpass(signal: np.ndarray, dt: float, tau: float) -> np.ndarray:
    """1st-order IIR low-pass. y[k] = y[k-1] + alpha*(x[k] - y[k-1])."""
    if tau <= 0:
        return signal.copy()
    alpha = dt / (tau + dt)
    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, len(signal)):
        out[i] = out[i - 1] + alpha * (signal[i] - out[i - 1])
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Resample a /teleop bag into a uniform-rate waypoint CSV"
    )
    parser.add_argument("bag_dir", help="Path to a ROS2 bag directory (sqlite3)")
    parser.add_argument(
        "--rate", type=float, default=100.0, help="Output rate in Hz (default 100)"
    )
    parser.add_argument(
        "--smooth-steer-tau",
        type=float,
        default=0.02,
        help="Low-pass time constant for steering, seconds. 0 disables (default 0.02)",
    )
    parser.add_argument(
        "--smooth-speed-tau",
        type=float,
        default=0.02,
        help="Low-pass time constant for speed, seconds. 0 disables (default 0.02)",
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="Skip the comparison plot"
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="Trim start, seconds relative to first non-zero speed command",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="Trim end, seconds relative to first non-zero speed command",
    )
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir).resolve()
    if not bag_dir.is_dir():
        sys.exit(f"Not a directory: {bag_dir}")

    print(f"Reading {TELEOP_TOPIC} from {bag_dir} ...")
    t_raw, speed_raw, steer_raw = read_teleop(bag_dir)
    print(f"  {len(t_raw)} messages, {t_raw[-1] - t_raw[0]:.2f}s duration")

    # Trim to active window (first → last non-zero speed command)
    i0, i1 = first_last_nonzero(speed_raw)
    t_start = t_raw[i0]
    t_end = t_raw[i1]
    active_t_start = t_start
    active_t_end = t_end
    active_duration = active_t_end - active_t_start
    if args.start is not None:
        if args.start < 0 or args.start >= active_duration:
            sys.exit(
                f"--start={args.start} out of range "
                f"(active window is 0..{active_duration:.2f}s)"
            )
        t_start = active_t_start + args.start
    if args.end is not None:
        if args.end <= 0 or args.end > active_duration:
            sys.exit(
                f"--end={args.end} out of range "
                f"(active window is 0..{active_duration:.2f}s)"
            )
        t_end = active_t_start + args.end
    if t_end <= t_start:
        sys.exit(
            f"Trim window is empty: start={args.start}, end={args.end} "
            f"(active window is 0..{active_duration:.2f}s)"
        )
    trimmed = args.start is not None or args.end is not None
    dt = 1.0 / args.rate
    grid = np.arange(0.0, t_end - t_start, dt)
    abs_grid = grid + t_start

    # Zero-order hold: at each grid point, hold the last command whose timestamp <= grid point
    idx = np.clip(np.searchsorted(t_raw, abs_grid, side="right") - 1, 0, len(t_raw) - 1)
    speed_zoh = speed_raw[idx]
    steer_zoh = steer_raw[idx]

    # Low-pass on the uniform grid
    speed_out = lowpass(speed_zoh, dt, args.smooth_speed_tau)
    steer_out = lowpass(steer_zoh, dt, args.smooth_steer_tau)

    # Write CSV
    out_csv = bag_dir / "teleop_waypoints.csv"
    trim_lo = args.start if args.start is not None else 0.0
    trim_hi = args.end if args.end is not None else active_t_end - active_t_start
    trim_note = f" trimmed to [{trim_lo:.3f}, {trim_hi:.3f}]s" if trimmed else ""
    header = (
        f"Generated from {bag_dir.name} at {args.rate:.1f} Hz "
        f"(speed tau={args.smooth_speed_tau}s, steer tau={args.smooth_steer_tau}s)"
        f"{trim_note}\n"
        "t_rel,speed,steering_angle"
    )
    np.savetxt(
        out_csv,
        np.column_stack([grid, speed_out, steer_out]),
        delimiter=",",
        header=header,
        comments="# ",
        fmt=("%.4f", "%.6f", "%.6f"),
    )
    print(f"Wrote {out_csv} ({len(grid)} samples, {grid[-1]:.2f}s)")

    if args.no_plot:
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    t_raw_rel = t_raw - t_start

    ax = axes[0]
    ax.step(
        t_raw_rel, speed_raw, where="post", alpha=0.4, color="C0", label="raw (ZOH)"
    )
    ax.plot(
        grid,
        speed_zoh,
        linewidth=0.8,
        color="C0",
        alpha=0.6,
        label="resampled (pre-filter)",
    )
    ax.plot(
        grid,
        speed_out,
        linewidth=1.2,
        color="C3",
        label=f"resampled (tau={args.smooth_speed_tau}s)",
    )
    ax.axvspan(grid[0], grid[-1], color="gray", alpha=0.08, label="active window")
    ax.set_ylabel("speed (m/s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.step(
        t_raw_rel,
        np.degrees(steer_raw),
        where="post",
        alpha=0.4,
        color="C1",
        label="raw (ZOH)",
    )
    ax.plot(
        grid,
        np.degrees(steer_zoh),
        linewidth=0.8,
        color="C1",
        alpha=0.6,
        label="resampled (pre-filter)",
    )
    ax.plot(
        grid,
        np.degrees(steer_out),
        linewidth=1.2,
        color="C3",
        label=f"resampled (tau={args.smooth_steer_tau}s)",
    )
    ax.set_xlabel("t_rel (s)")
    ax.set_ylabel("steering (deg)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    if trimmed:
        fig.suptitle(
            f"Trimmed to [{trim_lo:.3f}, {trim_hi:.3f}]s "
            "(relative to first non-zero speed command)"
        )
    fig.tight_layout()
    out_png = bag_dir / "teleop_waypoints.png"
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"Wrote {out_png}")


if __name__ == "__main__":
    main()
