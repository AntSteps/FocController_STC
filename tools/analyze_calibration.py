#!/usr/bin/env python3
"""Visualize calibration samples captured by host_pc.py."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_samples(path: Path) -> dict[int, np.ndarray]:
    grouped: dict[int, list[list[float]]] = {1: [], -1: []}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            direction = int(row["direction"])
            grouped.setdefault(direction, []).append(
                [
                    float(row["command_theta"]),
                    float(row["mechanical_angle"]),
                    float(row["electrical_error"]),
                ]
            )
    return {key: np.asarray(value, dtype=float) for key, value in grouped.items() if value}


def periodic_interp(samples: np.ndarray, grid: np.ndarray) -> np.ndarray:
    ordered = samples[np.argsort(samples[:, 1])]
    angle = ordered[:, 1]
    error = ordered[:, 2]
    return np.interp(
        grid,
        np.r_[angle - 2.0 * np.pi, angle, angle + 2.0 * np.pi],
        np.r_[error, error, error],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    grouped = load_samples(args.csv)
    grid = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    forward = periodic_interp(grouped[1], grid)
    reverse = periodic_interp(grouped[-1], grid)
    average = 0.5 * (forward + reverse)
    direction_delta = forward - reverse

    mean = float(np.mean(average))
    residual = average - mean
    harmonic_components = []
    for harmonic in (1, 2):
        cosine = 2.0 * np.mean(residual * np.cos(harmonic * grid))
        sine = 2.0 * np.mean(residual * np.sin(harmonic * grid))
        component = cosine * np.cos(harmonic * grid) + sine * np.sin(harmonic * grid)
        harmonic_components.append(component)
        residual -= component

    output = args.output or args.csv.with_name(args.csv.stem + "_analysis.png")
    fig, axes = plt.subplots(3, 2, figsize=(15, 13), constrained_layout=True)
    ax = axes[0, 0]
    for direction, color in ((1, "tab:blue"), (-1, "tab:orange")):
        data = grouped[direction]
        order = np.argsort(data[:, 0]) if direction == 1 else np.argsort(-data[:, 0])
        ordered = data[order]
        unwrapped = np.unwrap(ordered[:, 1])
        ax.plot(ordered[:, 0], unwrapped, ".", ms=2, alpha=0.65,
                label=("forward" if direction == 1 else "reverse"), color=color)
    ax.set_title("Raw mechanical trajectory")
    ax.set_xlabel("command electrical angle (rad)")
    ax.set_ylabel("unwrapped mechanical angle (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[0, 1]
    for direction, label, color in ((1, "正向", "tab:blue"), (-1, "反向", "tab:orange")):
        data = grouped[direction]
        ax.plot(data[:, 0], data[:, 2], ".", ms=2, alpha=0.65, label=("forward" if direction == 1 else "reverse"), color=color)
    ax.set_title("Raw samples: error vs command electrical angle")
    ax.set_xlabel("command theta (rad)")
    ax.set_ylabel("electrical error (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[1, 0]
    for direction, color in ((1, "tab:blue"), (-1, "tab:orange")):
        data = grouped[direction]
        order = np.argsort(data[:, 0]) if direction == 1 else np.argsort(-data[:, 0])
        ordered = data[order]
        unwrapped = np.unwrap(ordered[:, 1])
        theta = ordered[:, 0]
        # Both sweeps use command_theta as the signed trajectory variable.
        ideal = unwrapped[0] + (theta - theta[0]) / 7.0
        ax.plot(theta, unwrapped - ideal, ".", ms=2, alpha=0.65,
                label=("forward" if direction == 1 else "reverse"), color=color)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title("Trajectory following residual")
    ax.set_xlabel("command electrical angle (rad)")
    ax.set_ylabel("mechanical residual (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(grid, forward, label="forward", color="tab:blue")
    ax.plot(grid, reverse, label="reverse", color="tab:orange")
    ax.plot(grid, average, label="average", color="black", linewidth=2)
    ax.set_title("Paired by mechanical angle")
    ax.set_xlabel("mechanical angle (rad)")
    ax.set_ylabel("electrical error (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[2, 0]
    ax.plot(grid, average - mean, label="mean error minus DC", color="tab:green")
    ax.plot(grid, direction_delta, label="forward - reverse", color="tab:red")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title("Compensable vs direction-dependent error")
    ax.set_xlabel("mechanical angle (rad)")
    ax.set_ylabel("error (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[2, 1]
    ax.plot(grid, average - mean, label="mean error", color="0.65")
    ax.plot(grid, residual, label="residual after 1st/2nd harmonics", color="tab:purple")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title("Residual after harmonic fit")
    ax.set_xlabel("mechanical angle (rad)")
    ax.set_ylabel("error (rad)")
    ax.grid(alpha=0.25)
    ax.legend()

    rms_delta = float(np.sqrt(np.mean(direction_delta * direction_delta)))
    rms_residual = float(np.sqrt(np.mean(residual * residual)))
    fig.suptitle(
        f"{args.csv.name} | mean={mean:.4f} rad | "
        f"direction RMS={rms_delta:.4f} rad | residual RMS={rms_residual:.4f} rad"
    )
    fig.savefig(output, dpi=160)
    print(f"saved: {output}")
    print(f"samples: forward={len(grouped[1])}, reverse={len(grouped[-1])}")
    print(f"mean error: {mean:.6f} rad")
    print(f"direction difference RMS: {rms_delta:.6f} rad")
    print(f"residual RMS after 1/2 harmonics: {rms_residual:.6f} rad")


if __name__ == "__main__":
    main()
