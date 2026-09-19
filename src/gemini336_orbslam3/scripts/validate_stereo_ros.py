#!/usr/bin/env python3
"""Replay EuRoC through the installed SlamNode and retain auditable ROS evidence."""

import argparse
from array import array
from bisect import bisect_left
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FRAME = re.compile(r"Stereo frame: index=(\d+) timestamp=([\d.]+) track_ms=([\d.]+) state=(\w+)")
STATS = re.compile(r"Stereo stats: (.*)")


def read_camera(path):
    with path.open() as stream:
        rows = [(int(row[0]), row[1]) for row in csv.reader(stream)
                if row and not row[0].startswith('#')]
    if not rows or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"Empty or non-increasing camera CSV: {path}")
    return rows


def sha256(path):
    with path.open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path, fields, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def image_message(path, stamp):
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 2 or image.dtype.name != 'uint8':
        raise ValueError(f"Expected a MONO8 image: {path}")

    # Preserve acquisition time while constructing the ROS MONO8 payload.
    msg = Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.height, msg.width = image.shape
    msg.encoding = 'mono8'
    msg.step = msg.width
    # A typed byte array avoids per-pixel Python validation in the generated ROS setter.
    msg.data = array('B', image.tobytes())

    return msg


def summarize(published, frames, stats, expected, code, error, drained, log):
    # StereoFrame stores epoch seconds as double. Match within 1 us, far below
    # EuRoC's 50 ms spacing, instead of claiming nanosecond round-trip precision.
    stamps = [row['timestamp_ns'] for row in published]
    matched = []
    unexpected = []
    for frame in frames:
        stamp = round(float(frame['timestamp']) * 10**9)
        pos = bisect_left(stamps, stamp)
        candidates = [i for i in (pos - 1, pos) if 0 <= i < len(stamps)]
        nearest = min(candidates, key=lambda i: abs(stamps[i] - stamp)) if candidates else None
        if nearest is None or abs(stamps[nearest] - stamp) > 1000:
            unexpected.append(frame)
        else:
            matched.append(nearest)

    # Audit missing/duplicate output independently of the aggregate log statistics.
    counts = Counter(matched)
    missing = [dict(index=i, timestamp_ns=stamp) for i, stamp in enumerate(stamps) if not counts[i]]
    times = sorted(float(frame['track_ms']) for frame in frames)

    # Recompute totals from per-frame evidence rather than trusting summary logs alone.
    total = next((row for row in stats if row['scope'] == 'total'), None)
    windows = [row for row in stats if row['scope'] in ('window', 'tail')]
    stats_ok = bool(total) and sum(int(row['frames']) for row in windows) == len(frames)
    stats_ok = stats_ok and int(total['frames']) == len(frames)
    stats_ok = stats_ok and sum(int(row['intervals']) for row in windows) == max(0, len(frames) - 1)
    if total and times:
        mean = sum(times) / len(times)
        stats_ok = stats_ok and abs(float(total['track_mean_ms']) - mean) < 0.00001
        stats_ok = stats_ok and abs(float(total['track_max_ms']) - max(times)) < 0.00001
        intervals = [(float(b['timestamp']) - float(a['timestamp'])) * 1000
                     for a, b in zip(frames, frames[1:])]
        if intervals:
            stats_ok = stats_ok and int(total['intervals']) == len(intervals)
            for key, value in [('interval_min_ms', min(intervals)),
                               ('interval_mean_ms', sum(intervals) / len(intervals)),
                               ('interval_max_ms', max(intervals))]:
                stats_ok = stats_ok and abs(float(total[key]) - value) < 0.001
    for row in stats:
        elapsed = float(row['elapsed_sec'])
        stats_ok = stats_ok and elapsed > 0
        if elapsed > 0:
            stats_ok = stats_ok and math.isclose(float(row['rate_hz']), int(row['frames']) / elapsed,
                                                rel_tol=0.0001, abs_tol=0.0001)

    ordered = all(b > a for a, b in zip(matched, matched[1:]))
    shutdown_ok = ('Stereo input stopped;' in log and 'Stereo SLAM shutdown returned' in log
                   and log.index('Stereo input stopped;') < log.index('Stereo SLAM shutdown returned'))
    passed = (not error and drained and code == 0 and len(published) == expected
              and not missing and not unexpected and ordered and len(matched) == len(counts)
              and stats_ok and shutdown_ok and len(frames) == expected)

    return {
        'passed': passed, 'error': error, 'node_exit_code': code,
        'drain_completed': drained, 'shutdown_order_ok': shutdown_ok,
        'statistics_consistent': stats_ok, 'expected': expected,
        'published': len(published), 'processed': len(frames),
        'missing': missing, 'unexpected': unexpected,
        'duplicate_processed': sum(count - 1 for count in counts.values()),
        'non_increasing_processed': sum(b <= a for a, b in zip(matched, matched[1:])),
        'states': dict(Counter(frame['state'] for frame in frames)),
        'track_mean_ms': sum(times) / len(times) if times else None,
        'track_p95_ms': times[math.ceil(len(times) * 0.95) - 1] if times else None,
        'track_max_ms': max(times) if times else None,
        'publish_max_lateness_ms': max((row['lateness_ms'] for row in published), default=None),
        'publish_over_50ms': sum(row['lateness_ms'] > 50 for row in published),
        'statistics': stats,
    }


def observations(published, summary, log):
    received = {side: [int(stamp) for stamp in re.findall(
        rf'Stereo receive: side={side} timestamp_ns=(\d+)', log)] for side in ('left', 'right')}
    pairs = [(int(left), int(right)) for left, right in re.findall(
        r'Stereo sync: left_ns=(\d+) right_ns=(\d+)', log)]
    left_set, right_set = set(received['left']), set(received['right'])
    pair_set = set(pairs)

    # Locate the first observable loss stage for each published pair.
    gaps = []
    missing_processed = {row['index'] for row in summary['missing']}
    for row in published:
        stamp = row['timestamp_ns']
        left, right, synced = stamp in left_set, stamp in right_set, (stamp, stamp) in pair_set
        if not (left and right and synced) or row['index'] in missing_processed:
            gaps.append({'index': row['index'], 'timestamp_ns': stamp,
                         'left_received': left, 'right_received': right, 'synced': synced,
                         'processed': row['index'] not in missing_processed})

    return {'received_counts': {side: len(rows) for side, rows in received.items()},
            'receive_duplicates': {side: len(rows) - len(set(rows)) for side, rows in received.items()},
            'receive_non_increasing': {side: sum(b <= a for a, b in zip(rows, rows[1:]))
                                       for side, rows in received.items()},
            'sync_count': len(pairs), 'sync_unequal_timestamps': sum(a != b for a, b in pairs),
            'gaps': gaps}


def run(args):
    left = read_camera(args.mav0 / 'cam0/data.csv')
    right = read_camera(args.mav0 / 'cam1/data.csv')
    if [row[0] for row in left] != [row[0] for row in right]:
        raise ValueError('Left and right CSV timestamps must match exactly')
    if args.max_frames:
        left, right = left[:args.max_frames], right[:args.max_frames]

    # Never silently overwrite evidence from an earlier run.
    args.output.mkdir(parents=True, exist_ok=False)
    executable = Path(get_package_prefix('gemini336_orbslam3')) / 'lib/gemini336_orbslam3/slam_node'
    command = [str(executable), '--ros-args', '-p', f'settings_path:={args.settings}',
               '-p', f'vocabulary_path:={args.vocabulary}',
               '-p', f'left_image_topic:={args.left_topic}', '-p', f'right_image_topic:={args.right_topic}',
               '-p', 'stereo.sync_queue_size:=10', '-p', 'stereo.max_time_diff_sec:=0.0005',
               '--log-level', 'slam_node:=debug']
    manifest = {'command': command, 'arguments': {key: str(value) for key, value in vars(args).items()},
                'environment': {key: os.environ.get(key) for key in
                                ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE')},
                'sha256': {str(path): sha256(path) for path in
                           [args.settings, args.vocabulary, args.mav0 / 'cam0/data.csv', args.mav0 / 'cam1/data.csv']}}
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    # The publisher retains its own evidence separately from the SLAM process log.
    published, frames, stats = [], [], []
    error, process, drained = None, None, False
    rclpy.init()
    node = rclpy.create_node('stereo_validation_publisher')
    pubs = [node.create_publisher(Image, topic, qos_profile_sensor_data)
            for topic in (args.left_topic, args.right_topic)]
    path = args.output / 'node.log'

    with path.open('w') as log, path.open() as reader:
        pending = ''

        def collect():
            nonlocal pending
            # A read can end between writes of the same log line; retain its unfinished tail.
            lines = (pending + reader.read()).split('\n')
            pending = lines.pop()
            for line in lines:
                match = FRAME.search(line)
                if match:
                    frames.append(dict(zip(('index', 'timestamp', 'track_ms', 'state'), match.groups())))
                match = STATS.search(line)
                if match:
                    stats.append(dict(item.split('=', 1) for item in match.group(1).strip().split()))

        try:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            while any(pub.get_subscription_count() != 1 for pub in pubs):
                if process.poll() is not None:
                    raise RuntimeError('Node exited during discovery')
                if time.monotonic() > deadline:
                    raise TimeoutError('Subscription discovery timed out (expected one subscriber per topic)')
                time.sleep(0.1)

            time.sleep(args.discovery_delay)
            started = time.monotonic()
            last_progress = started

            # Read each pair before its deadline. Record lateness without skipping frames.
            for index, ((stamp, left_name), (_, right_name)) in enumerate(zip(left, right)):
                messages = [image_message(args.mav0 / camera / 'data' / name, stamp)
                            for camera, name in [('cam0', left_name), ('cam1', right_name)]]
                target = started + (stamp - left[0][0]) / 1e9
                time.sleep(max(0, target - time.monotonic()))
                if process.poll() is not None:
                    raise RuntimeError('Node exited while publishing')
                before = time.monotonic()
                for pub, message in zip(pubs, messages):
                    pub.publish(message)
                after = time.monotonic()
                published.append(dict(index=index, timestamp_ns=stamp, scheduled_sec=target - started,
                                      publish_start_sec=before - started, publish_end_sec=after - started,
                                      lateness_ms=(before - target) * 1000))
                collect()
                if after - last_progress >= 5:
                    print(f'Published {len(published)}/{len(left)}; processed {len(frames)}', flush=True)
                    last_progress = after

            # No sentinel/resend: an unprocessed final frame must remain visible as a gap.
            deadline = time.monotonic() + args.drain_timeout
            while len(frames) < len(published) and time.monotonic() < deadline:
                collect()
                if process.poll() is not None:
                    raise RuntimeError('Node exited while draining')
                time.sleep(0.05)
            drained = len(frames) == len(published)
            if not drained:
                error = 'Timed out waiting for all published frames to finish tracking'
        except (Exception, KeyboardInterrupt) as exc:
            error = f'{type(exc).__name__}: {exc}'
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=args.shutdown_timeout)
                except subprocess.TimeoutExpired:
                    error = (error + '; ' if error else '') + 'Shutdown timed out; process killed'
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            collect()
            node.destroy_node()
            rclpy.shutdown()

    # Shutdown completes the log before cross-checking and writing the final report.
    summary = summarize(published, frames, stats, len(left), process.returncode if process else None,
                        error, drained, path.read_text())
    summary['observations'] = observations(published, summary, path.read_text())

    write_csv(args.output / 'published.csv',
              ['index', 'timestamp_ns', 'scheduled_sec', 'publish_start_sec', 'publish_end_sec', 'lateness_ms'], published)
    write_csv(args.output / 'processed.csv', ['index', 'timestamp', 'track_ms', 'state'], frames)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: value for key, value in summary.items() if key != 'statistics'}, indent=2), flush=True)

    return 0 if summary['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('mav0', 'settings', 'vocabulary', 'output'):
        parser.add_argument('--' + name, type=lambda value: Path(value).resolve(), required=True)
    parser.add_argument('--max-frames', type=int, default=0, help='0 means the complete sequence')
    parser.add_argument('--left-topic', default='/phase3c/left')
    parser.add_argument('--right-topic', default='/phase3c/right')
    parser.add_argument('--startup-timeout', type=float, default=60)
    parser.add_argument('--discovery-delay', type=float, default=2)
    parser.add_argument('--drain-timeout', type=float, default=10)
    parser.add_argument('--shutdown-timeout', type=float, default=20)

    args = parser.parse_args()
    if args.max_frames < 0 or any(not math.isfinite(value) or value <= 0 for value in
                                (args.startup_timeout, args.discovery_delay, args.drain_timeout, args.shutdown_timeout)):
        parser.error('Frame limit must be nonnegative; delays/timeouts must be finite and positive')

    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
