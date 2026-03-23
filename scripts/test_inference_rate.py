"""Benchmark inference rate of an ONNX policy model.

Measures pure session.run() latency without any environment overhead,
to determine achievable control loop frequency on real hardware.

Usage:
    python scripts/test_inference_rate.py
    python scripts/test_inference_rate.py --path /path/to/model.onnx --n 50000
"""

import argparse
import time

import numpy as np
import onnxruntime as ort

DEFAULT_MODEL = "/home/jetson/f1tenth_ws/src/F1TENTH_System/recovery_controller/models/bwcm7l05/model.onnx"


def benchmark_model(
    session: ort.InferenceSession, obs_dim: int, n_iters: int, warmup: int
) -> np.ndarray:
    """Run inference in a tight loop and return per-call times in seconds."""
    # Warmup
    for _ in range(warmup):
        obs = np.random.randn(1, obs_dim).astype(np.float32)
        session.run(["action"], {"obs": obs})

    # Timed runs
    times = np.empty(n_iters)
    for i in range(n_iters):
        obs = np.random.randn(1, obs_dim).astype(np.float32)
        t0 = time.perf_counter()
        session.run(["action"], {"obs": obs})
        times[i] = time.perf_counter() - t0

    return times


def print_report(times: np.ndarray, provider: str):
    """Print timing statistics."""
    ms = times * 1000
    mean_ms = np.mean(ms)
    hz = 1000.0 / mean_ms

    print(f"\n{'=' * 50}")
    print(f"  Provider: {provider}")
    print(f"  Iterations: {len(ms)}")
    print(f"{'=' * 50}")
    print(f"  Mean:   {mean_ms:.4f} ms  ({hz:.0f} Hz)")
    print(f"  Median: {np.median(ms):.4f} ms")
    print(f"  Std:    {np.std(ms):.4f} ms")
    print(f"  Min:    {np.min(ms):.4f} ms")
    print(f"  Max:    {np.max(ms):.4f} ms")
    print(f"  P95:    {np.percentile(ms, 95):.4f} ms")
    print(f"  P99:    {np.percentile(ms, 99):.4f} ms")
    print()

    targets = [100, 200, 500, 1000]
    print("  Control loop feasibility:")
    for rate in targets:
        budget_ms = 1000.0 / rate
        ok = "YES" if mean_ms < budget_ms else "NO"
        print(f"    {rate:>4d} Hz ({budget_ms:.1f} ms budget): {ok}")
    print()


def main():
    parser = argparse.ArgumentParser(description="Benchmark ONNX policy inference rate")
    parser.add_argument(
        "--path",
        default=DEFAULT_MODEL,
        help=f"Path to ONNX model (default: {DEFAULT_MODEL})",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=10000,
        help="Number of inference iterations (default: 10000)",
    )
    parser.add_argument(
        "--warmup", type=int, default=100, help="Warmup iterations (default: 100)"
    )
    args = parser.parse_args()

    # CPU benchmark
    print(f"Loading model from {args.path}")
    session_cpu = ort.InferenceSession(args.path, providers=["CPUExecutionProvider"])
    obs_dim = session_cpu.get_inputs()[0].shape[1]
    act_dim = session_cpu.get_outputs()[0].shape[1]
    print(f"Observation dim: {obs_dim}")
    print(f"Action dim: {act_dim}")

    times_cpu = benchmark_model(session_cpu, obs_dim, args.n, args.warmup)
    print_report(times_cpu, "CPUExecutionProvider")

    # CUDA benchmark if available
    available = ort.get_available_providers()
    if "CUDAExecutionProvider" in available:
        session_cuda = ort.InferenceSession(
            args.path, providers=["CUDAExecutionProvider"]
        )
        times_cuda = benchmark_model(session_cuda, obs_dim, args.n, args.warmup)
        print_report(times_cuda, "CUDAExecutionProvider")
    else:
        print("CUDAExecutionProvider not available, skipping GPU benchmark.")


if __name__ == "__main__":
    main()
