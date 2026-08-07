#!/usr/bin/env python3
"""Estimate speed-observer ripple from a calibration CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def normalize_pi(value: float) -> float:
    while value > np.pi:
        value -= 2.0 * np.pi
    while value < -np.pi:
        value += 2.0 * np.pi
    return value


def load(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    groups = {}
    for direction in (1, -1):
        data = np.array(
            [[float(r["mechanical_angle"]), float(r["electrical_error"])]
             for r in rows if int(r["direction"]) == direction],
            dtype=float,
        )
        data = data[np.argsort(data[:, 0])]
        groups[direction] = np.interp(
            np.linspace(0.0, 2.0 * np.pi, 1440, endpoint=False),
            np.r_[data[:, 0] - 2.0 * np.pi, data[:, 0], data[:, 0] + 2.0 * np.pi],
            np.r_[data[:, 1], data[:, 1], data[:, 1]],
        )
    return groups[1], groups[-1]


def pll_update(measured: float, state: list[float], fs: float, bandwidth: float, zeta: float) -> float:
    kp = 2.0 * zeta * 2.0 * np.pi * bandwidth
    ki = (2.0 * np.pi * bandwidth) ** 2
    est_angle, integral, prev_error, prev_speed = state
    error = normalize_pi(measured - est_angle)
    half_dt = 0.5 / fs
    integral += (error + prev_error) * ki * half_dt
    speed = kp * error + integral
    est_angle = normalize_pi(est_angle + (speed + prev_speed) * half_dt)
    state[:] = [est_angle, integral, error, speed]
    return speed


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--pole-pairs", type=float, default=7.0)
    parser.add_argument("--speed", type=float, default=0.714, help="assumed mechanical speed (rad/s)")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--fs", type=float, default=300.0)
    parser.add_argument("--bandwidth", type=float, default=10.0)
    parser.add_argument("--zeta", type=float, default=0.707)
    args = parser.parse_args()

    forward, reverse = load(args.csv)
    angle_grid = np.linspace(0.0, 2.0 * np.pi, len(forward), endpoint=False)
    error_elec = 0.5 * (forward + reverse)
    dc = float(np.mean(error_elec))
    residual = error_elec - dc
    fitted = np.zeros_like(residual)
    for harmonic in (1, 2):
        c = 2.0 * np.mean(residual * np.cos(harmonic * angle_grid))
        s = 2.0 * np.mean(residual * np.sin(harmonic * angle_grid))
        component = c * np.cos(harmonic * angle_grid) + s * np.sin(harmonic * angle_grid)
        fitted += component
    correction_mech = (dc + fitted) / args.pole_pairs

    t = np.arange(0.0, args.duration, 1.0 / args.fs)
    true_angle = args.speed * t
    phase = np.mod(true_angle, 2.0 * np.pi)
    periodic_error = np.interp(phase, np.r_[angle_grid, 2.0 * np.pi], np.r_[residual, residual[0]]) / args.pole_pairs
    raw_angle = true_angle + periodic_error
    raw_encoder_angle = raw_angle
    harmonic_angle = raw_angle - np.interp(phase, np.r_[angle_grid, 2.0 * np.pi], np.r_[fitted, fitted[0]]) / args.pole_pairs
    harmonic_model_angle = true_angle + np.interp(phase, np.r_[angle_grid, 2.0 * np.pi], np.r_[fitted, fitted[0]]) / args.pole_pairs
    ideal_angle = true_angle

    traces = {}
    for name, measured in (("raw_encoder", raw_encoder_angle),
                           ("harmonic_corrected", harmonic_angle),
                           ("harmonic_component", harmonic_model_angle),
                           ("ideal", ideal_angle)):
        state = [float(measured[0]), 0.0, 0.0, 0.0]
        values = np.array([pll_update(float(x), state, args.fs, args.bandwidth, args.zeta) for x in measured])
        traces[name] = values

    start = int(2.0 * args.fs)
    print(f"assumptions: speed={args.speed:.3f} rad/s, fs={args.fs:.1f} Hz, PLL BW={args.bandwidth:.1f} Hz")
    print("case                  mean(rad/s)   ripple RMS   ripple peak-to-peak")
    for name in ("raw_encoder", "harmonic_corrected", "harmonic_component", "ideal"):
        values = traces[name][start:]
        ripple = values - np.mean(values)
        print(f"{name:12s} {np.mean(values):12.6f} {np.std(ripple):12.6f} {np.ptp(ripple):18.6f}")

    output = args.output or args.csv.with_name(args.csv.stem + "_speed_simulation.png")
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), constrained_layout=True)
    axes[0].plot(t, raw_angle - true_angle, label="measured error (full CSV)")
    axes[0].plot(t, raw_angle - harmonic_angle, label="1st/2nd harmonic component")
    axes[0].set_ylabel("angle error (rad)")
    axes[0].set_title("Synthetic angle error at constant speed")
    axes[0].grid(alpha=0.25)
    axes[0].legend()
    for name, color in (("raw_encoder", "tab:red"),
                        ("harmonic_corrected", "tab:blue"),
                        ("harmonic_component", "tab:orange"),
                        ("ideal", "black")):
        axes[1].plot(t, traces[name], label=name, color=color, linewidth=1.0)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("PLL speed (rad/s)")
    axes[1].set_title("Simulated 300 Hz PLL speed estimate")
    axes[1].grid(alpha=0.25)
    axes[1].legend()
    fig.savefig(output, dpi=160)
    print(f"saved: {output}")


if __name__ == "__main__":
    main()
