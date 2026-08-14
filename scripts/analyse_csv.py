#!/usr/bin/env python3
"""Summarise every column of a CSV: mean, min, max, std dev, and a histogram.

Numeric columns get full statistics plus an ASCII histogram. Non-numeric
columns (timestamps, controller names, outcomes, ...) get a value-count
histogram instead. Empty cells are ignored.

Usage:
    python3 analyse_csv.py <csv_path> [--bins N] [--columns col1,col2]
                                      [--plot [out.png]] [--width N]

Example:
    python3 analyse_csv.py ~/f1tenth_ws/experiments/recovery_episodes.csv --plot
"""

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

# Non-numeric columns with more distinct values than this are summarised
# rather than listed exhaustively (e.g. timestamps).
MAX_CATEGORIES = 15

# Plots land next to the episode CSVs written by recovery_controller
# (recovery.yaml: log_path: ~/f1tenth_ws/experiments/recovery_episodes.csv).
EXPERIMENTS_DIR = Path("~/f1tenth_ws/experiments").expanduser()
DEFAULT_PLOT_NAME = "hist.png"


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def read_csv(path):
    """Return (fieldnames, list of row dicts)."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            sys.exit(f"{path}: empty file")
        rows = [r for r in reader if any((v or "").strip() for v in r.values())]
    return reader.fieldnames, rows


def column_values(rows, name):
    """Non-empty raw string values of a column."""
    return [(r.get(name) or "").strip() for r in rows if (r.get(name) or "").strip()]


def as_floats(values):
    """Parse values as floats, or return None if any value is not numeric."""
    out = []
    for v in values:
        try:
            out.append(float(v))
        except ValueError:
            return None
    return out


# ---------------------------------------------------------------------------
# Histograms
# ---------------------------------------------------------------------------


def bar(count, largest, width):
    if largest <= 0:
        return ""
    return "#" * max(1, round(count * width / largest)) if count else ""


def numeric_histogram(values, bins, width):
    """Return a list of printable histogram lines for numeric data."""
    lo, hi = min(values), max(values)
    if lo == hi:
        return [f"  [{lo:.4g}] {len(values):>4}  {'#' * width}"]

    step = (hi - lo) / bins
    counts = [0] * bins
    for v in values:
        idx = int((v - lo) / step)
        counts[min(idx, bins - 1)] += 1  # right edge falls in the last bin

    largest = max(counts)
    lines = []
    for i, count in enumerate(counts):
        edge_lo = lo + i * step
        edge_hi = edge_lo + step
        lines.append(
            f"  [{edge_lo:>10.4g}, {edge_hi:>10.4g}) {count:>4}  {bar(count, largest, width)}"
        )
    return lines


def category_histogram(values, width):
    """Return printable histogram lines for categorical data."""
    counts = {}
    for v in values:
        counts[v] = counts.get(v, 0) + 1

    ordered = sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))
    if len(ordered) > MAX_CATEGORIES:
        lines = [f"  {len(ordered)} distinct values, top {MAX_CATEGORIES}:"]
        shown = ordered[:MAX_CATEGORIES]
    else:
        lines = []
        shown = ordered

    label_width = max(len(v) for v, _ in shown)
    largest = shown[0][1]
    for value, count in shown:
        lines.append(
            f"  {value:<{label_width}} {count:>4}  {bar(count, largest, width)}"
        )
    return lines


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def report_column(name, raw, total_rows, bins, width):
    missing = total_rows - len(raw)
    print(f"\n{name}")
    print("-" * len(name))

    if not raw:
        print(f"  no values ({total_rows} empty)")
        return

    values = as_floats(raw)
    if values is None:
        print(f"  type      non-numeric   n={len(raw)}  missing={missing}")
        for line in category_histogram(raw, width):
            print(line)
        return

    mean = statistics.fmean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    print(f"  n         {len(values)}  (missing {missing})")
    print(f"  mean      {mean:.6g}")
    print(f"  std dev   {std:.6g}")
    print(f"  min       {min(values):.6g}")
    print(f"  median    {statistics.median(values):.6g}")
    print(f"  max       {max(values):.6g}")
    for line in numeric_histogram(values, bins, width):
        print(line)


def resolve_plot_path(path):
    """Bare filenames go to the experiments directory; explicit paths are kept."""
    path = path.expanduser()
    if path.parent == Path("."):
        path = EXPERIMENTS_DIR / path.name
    return path


def save_plots(path, columns, rows, bins):
    """Save a grid of histograms for the numeric columns to an image file."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    numeric = []
    for name in columns:
        values = as_floats(column_values(rows, name))
        if values:
            numeric.append((name, values))

    if not numeric:
        print("\nno numeric columns to plot", file=sys.stderr)
        return

    cols = min(4, len(numeric))
    grid_rows = math.ceil(len(numeric) / cols)
    fig, axes = plt.subplots(
        grid_rows, cols, figsize=(4 * cols, 3 * grid_rows), squeeze=False
    )
    for ax, (name, values) in zip(axes.flat, numeric):
        ax.hist(values, bins=bins, color="#4c72b0", edgecolor="white")
        ax.set_title(name, fontsize=9)
        ax.tick_params(labelsize=7)
    for ax in axes.flat[len(numeric) :]:
        ax.axis("off")

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    print(f"\nwrote {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="path to the CSV file")
    parser.add_argument(
        "--bins", type=int, default=10, help="histogram bins (default: 10)"
    )
    parser.add_argument(
        "--columns", help="comma-separated subset of columns to analyse"
    )
    parser.add_argument("--width", type=int, default=40, help="ASCII bar width")
    parser.add_argument(
        "--plot",
        type=Path,
        nargs="?",
        const=Path(DEFAULT_PLOT_NAME),
        help=f"also save histograms to an image (default: {EXPERIMENTS_DIR}/{DEFAULT_PLOT_NAME}; "
        "a bare filename is written to that directory)",
    )
    args = parser.parse_args()

    if args.bins < 1:
        sys.exit("--bins must be >= 1")
    if not args.csv_path.is_file():
        sys.exit(f"{args.csv_path}: no such file")

    fieldnames, rows = read_csv(args.csv_path)
    if not rows:
        sys.exit(f"{args.csv_path}: no data rows")

    columns = fieldnames
    if args.columns:
        wanted = [c.strip() for c in args.columns.split(",") if c.strip()]
        unknown = [c for c in wanted if c not in fieldnames]
        if unknown:
            sys.exit(f"unknown column(s): {', '.join(unknown)}")
        columns = wanted

    print(f"{args.csv_path}: {len(rows)} rows, {len(fieldnames)} columns")
    for name in columns:
        report_column(name, column_values(rows, name), len(rows), args.bins, args.width)

    if args.plot:
        save_plots(resolve_plot_path(args.plot), columns, rows, args.bins)


if __name__ == "__main__":
    main()
