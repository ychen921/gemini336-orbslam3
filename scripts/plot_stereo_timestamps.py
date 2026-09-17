#!/usr/bin/env python3
"""Plot CSV output from analyze_stereo_timestamps.py; requires matplotlib."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def plot_results(directory: Path) -> None:
    pairs = read_rows(directory / "pairs.csv")
    elapsed = [int(row["elapsed_ns"]) / 1e9 for row in pairs]
    signed_delta = [int(row["signed_delta_ns"]) / 1e6 for row in pairs]
    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(elapsed, signed_delta, linewidth=.8, label="Left minus right")
    for threshold, color in ((.5, "tab:orange"), (2, "tab:red")):
        axes[0].axhline(threshold, color=color, linestyle="--", label=f"±{threshold} ms")
        axes[0].axhline(-threshold, color=color, linestyle="--")
    axes[0].set_ylabel("Header timestamp difference (ms)")
    axes[0].legend(loc="lower right")
    for side in ("left", "right"):
        rows = read_rows(directory / f"{side}_timestamps.csv")
        first_stamp = int(rows[0]["header_ns"])
        times = [(int(row["header_ns"]) - first_stamp) / 1e9 for row in rows[1:]]
        intervals = [int(row["interval_ns"]) / 1e6 for row in rows[1:]]
        axes[1].plot(times, intervals, linewidth=.7, alpha=.8, label=side)
    axes[1].set_ylabel("Consecutive frame interval (ms)")
    axes[1].set_xlabel("Elapsed header time (s)")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    figure.suptitle("Gemini336 stereo bag: offline timestamp analysis")
    figure.tight_layout()
    figure.savefig(directory / "timestamps.png", dpi=180)
    figure.savefig(directory / "timestamps.svg")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    plot_results(parser.parse_args().directory)


if __name__ == "__main__":
    main()
