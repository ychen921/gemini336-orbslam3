#!/usr/bin/env python3
"""Read ROS 2 CDR Image headers from a SQLite bag without ROS dependencies."""

import argparse
import csv
import json
import sqlite3
import statistics
import struct
from bisect import bisect_left
from pathlib import Path


def read_stamps(database: Path, topic: str) -> list[tuple[int, int]]:
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as connection:
        topic_row = connection.execute(
            "SELECT id, type, serialization_format FROM topics WHERE name=?", (topic,)
        ).fetchone()
        if topic_row is None or topic_row[1:] != ("sensor_msgs/msg/Image", "cdr"):
            raise ValueError(f"Expected CDR Image topic: {topic}")
        # The Image header starts with Time immediately after CDR encapsulation.
        rows = connection.execute(
            "SELECT timestamp, substr(data, 1, 12) FROM messages "
            "WHERE topic_id=? ORDER BY timestamp, id", (topic_row[0],)
        )
        stamps = []
        for recorded_ns, header in rows:
            if len(header) != 12 or header[:2] not in (b"\x00\x00", b"\x00\x01"):
                raise ValueError("Unsupported or truncated CDR header")
            seconds, nanoseconds = struct.unpack_from(
                "<iI" if header[1] == 1 else ">iI", header, 4
            )
            if nanoseconds >= 1_000_000_000:
                raise ValueError("Invalid nanosecond field")
            stamps.append((recorded_ns, seconds * 1_000_000_000 + nanoseconds))
    if len(stamps) < 2:
        raise ValueError(f"Not enough images: {topic}")
    return stamps


def percentile(values: list[int], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def write_csv(path: Path, columns: list[str], rows: list[list[object]]) -> None:
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(columns)
        writer.writerows(rows)


def nearest_index(stamps: list[int], target: int) -> int:
    insertion = bisect_left(stamps, target)
    candidates = range(max(0, insertion - 1), min(len(stamps), insertion + 1))
    return min(candidates, key=lambda index: abs(stamps[index] - target))


def analyze(database: Path, output: Path, left_topic: str, right_topic: str) -> None:
    output.mkdir(parents=True, exist_ok=True)
    streams = {side: read_stamps(database, topic) for side, topic in
               (("left", left_topic), ("right", right_topic))}
    summary: dict[str, object] = {"database": str(database), "topics": [left_topic, right_topic]}
    intervals: dict[str, list[int]] = {}
    for side, records in streams.items():
        stamps = [stamp for _, stamp in records]
        gaps = [current - previous for previous, current in zip(stamps, stamps[1:])]
        intervals[side] = gaps
        median = statistics.median(gaps)
        summary[side] = {
            "count": len(stamps), "first_ns": stamps[0], "last_ns": stamps[-1],
            "duplicates": len(stamps) - len(set(stamps)),
            "backwards": sum(gap < 0 for gap in gaps),
            "interval_median_ns": median, "interval_p95_ns": percentile(gaps, .95),
            "interval_max_ns": max(gaps),
            "suspected_gap_indices": [index + 2 for index, gap in enumerate(gaps) if gap > 1.5 * median],
        }
        write_csv(output / f"{side}_timestamps.csv",
                  ["index", "recorded_ns", "header_ns", "interval_ns"],
                  [[index + 1, recorded, stamp, "" if index == 0 else gaps[index - 1]]
                   for index, (recorded, stamp) in enumerate(records)])
    # Do not sort sensor timestamps: that would hide backwards timestamps.
    if any(any(gap <= 0 for gap in gaps) for gaps in intervals.values()):
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        raise ValueError("Non-increasing sensor timestamps; correspondence needs manual review")
    left = [stamp for _, stamp in streams["left"]]
    right = [stamp for _, stamp in streams["right"]]
    # Mutual nearest neighbours inside half a nominal period provide conservative,
    # unique correspondence without filtering at either synchronization threshold.
    radius = min(statistics.median(intervals[side]) for side in streams) / 2
    pairs: list[tuple[int, int]] = []
    for left_index, stamp in enumerate(left):
        right_index = nearest_index(right, stamp)
        if abs(stamp - right[right_index]) < radius and nearest_index(left, right[right_index]) == left_index:
            pairs.append((left_index, right_index))
    paired_left = {first for first, _ in pairs}
    paired_right = {second for _, second in pairs}
    summary["correspondence"] = {
        "method": "mutual nearest timestamps, strictly within half median frame period; not hardware exposure proof",
        "radius_ns": radius, "count": len(pairs),
        "unmatched_left": [index + 1 for index in range(len(left)) if index not in paired_left],
        "unmatched_right": [index + 1 for index in range(len(right)) if index not in paired_right],
    }
    if not pairs:
        raise ValueError("No unambiguous timestamp correspondence")
    pair_rows: list[list[object]] = []
    differences = []
    for left_index, right_index in pairs:
        difference = left[left_index] - right[right_index]
        differences.append(difference)
        pair_rows.append([left_index + 1, right_index + 1, left[left_index], right[right_index],
                          left[left_index] - left[0], difference, abs(difference),
                          "" if left_index == 0 else intervals["left"][left_index - 1],
                          "" if right_index == 0 else intervals["right"][right_index - 1],
                          abs(difference) <= 500_000, abs(difference) <= 2_000_000])
    columns = ["left_index", "right_index", "left_ns", "right_ns", "elapsed_ns", "signed_delta_ns",
               "absolute_delta_ns", "left_interval_ns", "right_interval_ns", "within_0_5_ms", "within_2_ms"]
    write_csv(output / "pairs.csv", columns, pair_rows)
    write_csv(output / "tail20.csv", columns, [row for row in pair_rows if int(row[0]) > len(left) - 20])
    absolute = [abs(value) for value in differences]
    summary["delta"] = {"signed_min_ns": min(differences), "signed_max_ns": max(differences),
                        "absolute_median_ns": statistics.median(absolute),
                        "absolute_p95_ns": percentile(absolute, .95), "absolute_max_ns": max(absolute)}
    segments: list[list[object]] = []
    for threshold in (500_000, 2_000_000):
        active: list[int] = []
        for index in range(len(pairs) + 1):
            exceeds = index < len(pairs) and absolute[index] > threshold
            contiguous = not active or (index < len(pairs) and pairs[index] ==
                                       (pairs[active[-1]][0] + 1, pairs[active[-1]][1] + 1))
            if active and (not exceeds or not contiguous):
                first, last = active[0], active[-1]
                segments.append([threshold, pairs[first][0] + 1, pairs[last][0] + 1, len(active),
                                 left[pairs[first][0]] - left[0], left[pairs[last][0]] - left[0],
                                 left[pairs[last][0]] - left[pairs[first][0]], max(absolute[item] for item in active)])
                active = []
            if exceeds:
                active.append(index)
        summary[f"over_{threshold}_ns"] = sum(value > threshold for value in absolute)
    write_csv(output / "threshold_segments.csv", ["threshold_ns", "first_left_index", "last_left_index", "count",
              "start_elapsed_ns", "end_elapsed_ns", "first_to_last_duration_ns", "max_absolute_ns"], segments)
    windows: list[list[object]] = []
    for start in range(0, (left[-1] - left[0]) // 5_000_000_000 + 1):
        selected = [(left[first] - left[0], left[first] - right[second]) for first, second in pairs
                    if (left[first] - left[0]) // 5_000_000_000 == start]
        if not selected:
            continue
        times = [time / 1e9 for time, _ in selected]
        values = [value for _, value in selected]
        mean_time, mean_value = statistics.mean(times), statistics.mean(values)
        denominator = sum((time - mean_time) ** 2 for time in times)
        slope = sum((time - mean_time) * (value - mean_value) for time, value in zip(times, values)) / denominator if denominator else 0
        windows.append([start * 5, len(values), min(values), statistics.median(values), max(values), slope])
    write_csv(output / "windows5s.csv", ["start_sec", "count", "signed_min_ns", "signed_median_ns", "signed_max_ns", "slope_ns_per_sec"], windows)
    times = [(left[first] - left[0]) / 1e9 for first, _ in pairs]
    mean_time, mean_delta = statistics.mean(times), statistics.mean(differences)
    summary["global_slope_ns_per_sec"] = sum((time - mean_time) * (delta - mean_delta) for time, delta in zip(times, differences)) / sum((time - mean_time) ** 2 for time in times)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--left-topic", default="/camera/left_ir/image_raw")
    parser.add_argument("--right-topic", default="/camera/right_ir/image_raw")
    args = parser.parse_args()
    analyze(args.database, args.output, args.left_topic, args.right_topic)


if __name__ == "__main__":
    main()
