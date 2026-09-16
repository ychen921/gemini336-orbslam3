#!/usr/bin/env python3
"""Validate a bounded bag clip through rosbag2 playback and the installed SlamNode."""

import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time

import rclpy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from rosbag2_interfaces.srv import Resume
from sensor_msgs.msg import Image

from validate_stereo_ros import FRAME, STATS, sha256, write_csv


def extract_clip(args):
    meta = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if meta['storage_identifier'] != 'sqlite3':
        raise ValueError('Only SQLite input is supported')
    start = meta['starting_time']['nanoseconds_since_epoch'] + round(args.start_sec * 1e9)
    end = start + round(args.duration_sec * 1e9)
    clip = args.output / 'clip'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(clip), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    sides = {args.left_topic: 'left', args.right_topic: 'right'}
    created, rows = set(), []
    # Copy serialized messages and bag timestamps unchanged; only select a finite interval.
    for filename in meta['relative_file_paths']:
        with sqlite3.connect((args.bag / filename).as_uri() + '?mode=ro', uri=True) as db:
            topics = {r[0]: r[1:] for r in db.execute('SELECT id,name,type,serialization_format,offered_qos_profiles FROM topics') if r[1] in sides}
            for _, (name, kind, fmt, qos) in topics.items():
                if kind != 'sensor_msgs/msg/Image' or fmt != 'cdr':
                    raise ValueError('Expected CDR sensor_msgs/Image topics')
                if name not in created:
                    writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind,
                                        serialization_format=fmt, offered_qos_profiles=qos))
                    created.add(name)
            for tid, stamp, data in db.execute('SELECT topic_id,timestamp,data FROM messages WHERE timestamp>=? AND timestamp<? ORDER BY timestamp,id', (start, end)):
                if tid not in topics:
                    continue
                name = topics[tid][0]
                msg = deserialize_message(data, Image)
                header = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                writer.write(name, data, stamp)
                rows.append({'side': sides[name], 'stamp_ns': header, 'record_ns': stamp})
    del writer  # Finalize metadata before opening the clip with the player.
    if any(not any(r['side'] == side for r in rows) for side in ('left', 'right')):
        raise ValueError('Selected clip must contain both image topics')
    for side in ('left', 'right'):
        stamps = [r['stamp_ns'] for r in rows if r['side'] == side]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('Clip image header timestamps must be strictly increasing')
    return clip, rows


def reference_pairs(rows, tolerance):
    sides = [[r['stamp_ns'] for r in rows if r['side'] == side] for side in ('left', 'right')]
    left, right = sides
    i, j, pairs = 0, 0, []
    while i < len(left) and j < len(right):
        delta = left[i] - right[j]
        if abs(delta) <= tolerance:
            pairs.append((left[i], right[j]))
            i, j = i+1, j+1
        elif delta < 0:
            i += 1
        else:
            j += 1
    return sides, pairs


def parse_log(path):
    text = path.read_text()
    received = [[int(s) for s in re.findall(rf'Stereo receive: side={side} timestamp_ns=(\d+)', text)]
                for side in ('left', 'right')]
    pairs = [(int(a), int(b)) for a, b in re.findall(r'Stereo sync: left_ns=(\d+) right_ns=(\d+)', text)]
    frames = [dict(zip(('index', 'timestamp', 'track_ms', 'state'), match)) for match in FRAME.findall(text)]
    stats = [dict(item.split('=', 1) for item in line.strip().split()) for line in STATS.findall(text)]
    return text, received, pairs, frames, stats


def analyse(rows, tolerance, path, node_code, player_code, error, drained):
    expected, reference = reference_pairs(rows, tolerance)
    text, received, pairs, frames, stats = parse_log(path)
    missing_rx = {side: sorted(set(expected[i]) - set(received[i])) for i, side in enumerate(('left', 'right'))}
    unexpected_rx = {side: sorted(set(received[i]) - set(expected[i])) for i, side in enumerate(('left', 'right'))}
    missing_pairs = sorted(set(reference) - set(pairs))
    unexpected_pairs = sorted(set(pairs) - set(reference))
    # Tracking uses left timestamp as double seconds; match within 1 us only for that conversion.
    left_stamps = sorted(a for a, _ in pairs)
    tracked, unexpected_tracking = [], []
    for frame in frames:
        ns = round(float(frame['timestamp']) * 1e9)
        pos = bisect_left(left_stamps, ns)
        candidates = [s for s in left_stamps[max(0, pos-1):pos+1] if abs(s-ns) <= 1000]
        if len(candidates) == 1:
            tracked.append(candidates[0])
        else:
            unexpected_tracking.append(frame)
    missing_tracking = sorted(set(left_stamps) - set(tracked))
    duplicates = any(len(v) != len(set(v)) for v in [*received, pairs, tracked])
    backwards = any(any(b <= a for a, b in zip(v, v[1:])) for v in [*received, tracked])
    totals = [s for s in stats if s['scope'] == 'total']
    stats_ok = (len(totals) == 1 and int(totals[0]['frames']) == len(frames)
                and sum(int(s['frames']) for s in stats if s['scope'] in ('window', 'tail')) == len(frames))
    shutdown_ok = ('Stereo input stopped;' in text and 'Stereo SLAM shutdown returned' in text
                   and text.index('Stereo input stopped;') < text.index('Stereo SLAM shutdown returned'))
    durations = sorted(float(f['track_ms']) for f in frames)
    states = dict(Counter(f['state'] for f in frames))
    passed = (not error and drained and node_code == 0 and player_code == 0 and stats_ok and shutdown_ok
              and not any(missing_rx.values()) and not any(unexpected_rx.values()) and not missing_pairs
              and not unexpected_pairs and not missing_tracking and not unexpected_tracking
              and not duplicates and not backwards and bool(states.get('Ok')))
    return {'passed': passed, 'error': error, 'node_exit_code': node_code, 'player_exit_code': player_code,
            'drained': drained, 'statistics_consistent': stats_ok, 'shutdown_order_ok': shutdown_ok,
            'bag_image_counts': dict(zip(('left', 'right'), map(len, expected))),
            'received_counts': dict(zip(('left', 'right'), map(len, received))),
            'reference_pairs': len(reference), 'sync_count': len(pairs), 'processed': len(frames),
            'outside_tolerance_counts': {s: len(expected[i])-len(reference) for i, s in enumerate(('left', 'right'))},
            'missing_received': missing_rx, 'unexpected_received': unexpected_rx,
            'missing_reference_pairs': missing_pairs, 'unexpected_sync_pairs': unexpected_pairs,
            'missing_tracking': missing_tracking, 'unexpected_tracking': unexpected_tracking,
            'duplicates': duplicates, 'non_increasing': backwards, 'states': states,
            'first_ok_timestamp': next((f['timestamp'] for f in frames if f['state'] == 'Ok'), None),
            'track_mean_ms': sum(durations)/len(durations) if durations else None,
            'track_p95_ms': durations[math.ceil(len(durations)*0.95)-1] if durations else None,
            'track_max_ms': max(durations) if durations else None, 'statistics': stats}


def stop(process, timeout):
    if process is None or process.poll() is not None:
        return True
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        return False


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    clip, rows = extract_clip(args)
    write_csv(args.output / 'bag_images.csv', ['side', 'stamp_ns', 'record_ns'], rows)
    tolerance = round(args.max_time_diff_sec * 1e9)
    expected, reference = reference_pairs(rows, tolerance)
    executable = Path(get_package_prefix('gemini336_orbslam3')) / 'lib/gemini336_orbslam3/slam_node'
    node_cmd = [str(executable), '--ros-args', '-p', f'settings_path:={args.settings}',
                '-p', f'left_image_topic:={args.left_topic}', '-p', f'right_image_topic:={args.right_topic}',
                '-p', f'stereo.max_time_diff_sec:={args.max_time_diff_sec}', '--log-level', 'slam_node:=debug']
    player_cmd = ['ros2', 'bag', 'play', str(clip), '--rate', '1', '--start-paused', '--disable-keyboard-controls']
    manifest = {'arguments': {k: str(v) for k, v in vars(args).items()}, 'node_command': node_cmd,
                'player_command': player_cmd, 'environment': {k: os.environ.get(k) for k in
                ('RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'FASTRTPS_DEFAULT_PROFILES_FILE')},
                'sha256': {str(p): sha256(p) for p in [args.settings, args.bag/'metadata.yaml', clip/'metadata.yaml']}}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    rclpy.init()
    observer = rclpy.create_node('stereo_bag_validation')
    client = observer.create_client(Resume, '/rosbag2_player/resume')
    node = player = None
    error, drained = None, False
    node_path = args.output/'node.log'
    try:
        with node_path.open('w') as log, (args.output/'player.log').open('w') as plog:
            node = subprocess.Popen(node_cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+60
            while 'Stereo SLAM initialized:' not in node_path.read_text():
                if node.poll() is not None:
                    raise RuntimeError('SLAM initialization failed; see node.log')
                if time.monotonic() > deadline:
                    raise TimeoutError('SLAM initialization timed out')
                time.sleep(0.1)
            player = subprocess.Popen(player_cmd, stdout=plog, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30
            while not client.wait_for_service(timeout_sec=0.1) or any(
                    observer.count_publishers(topic) != 1 or observer.count_subscribers(topic) != 1
                    for topic in (args.left_topic, args.right_topic)):
                if player.poll() is not None or node.poll() is not None:
                    raise RuntimeError('Process exited during discovery')
                if time.monotonic() > deadline:
                    raise TimeoutError('Expected exactly one publisher and subscriber per image topic')
            manifest['publisher_qos'] = {topic: [str(info.qos_profile) for info in observer.get_publishers_info_by_topic(topic)]
                                         for topic in (args.left_topic, args.right_topic)}
            (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            time.sleep(2)
            future = client.call_async(Resume.Request())
            rclpy.spin_until_future_complete(observer, future, timeout_sec=5)
            if not future.done() or future.exception() is not None:
                raise RuntimeError('Could not resume rosbag playback')
            deadline = time.monotonic()+args.duration_sec+30
            last_report = time.monotonic()
            while player.poll() is None:
                if node.poll() is not None:
                    raise RuntimeError('SLAM exited during playback')
                if time.monotonic() > deadline:
                    raise TimeoutError('Playback timed out')
                if time.monotonic()-last_report >= 5:
                    _, rx, pairs, frames, _ = parse_log(node_path)
                    print(f'Received {len(rx[0])}/{len(rx[1])}; sync={len(pairs)} track={len(frames)}', flush=True)
                    last_report = time.monotonic()
                time.sleep(0.1)
            deadline = time.monotonic()+args.drain_timeout
            while time.monotonic() < deadline:
                _, rx, pairs, frames, _ = parse_log(node_path)
                if list(map(len, rx)) == list(map(len, expected)) and len(pairs) == len(reference) and len(frames) == len(pairs):
                    drained = True
                    break
                if node.poll() is not None:
                    raise RuntimeError('SLAM exited while draining')
                time.sleep(0.1)
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        for process in (player, node):
            if not stop(process, 20):
                error = (error+'; ' if error else '')+'Process shutdown timed out; killed'
        observer.destroy_node()
        rclpy.shutdown()
    summary = analyse(rows, tolerance, node_path, node.returncode if node else None,
                      player.returncode if player else None, error, drained)
    _, _, pairs, frames, _ = parse_log(node_path)
    write_csv(args.output/'processed.csv', ['index', 'timestamp', 'track_ms', 'state'], frames)
    write_csv(args.output/'sync.csv', ['left_stamp_ns', 'right_stamp_ns'],
              [{'left_stamp_ns': a, 'right_stamp_ns': b} for a, b in pairs])
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k not in ('statistics', 'missing_received', 'unexpected_received', 'missing_reference_pairs', 'unexpected_sync_pairs', 'missing_tracking', 'unexpected_tracking')}, indent=2), flush=True)
    return 0 if summary['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'settings', 'output'):
        parser.add_argument('--'+name, required=True, type=lambda p: Path(p).resolve())
    parser.add_argument('--start-sec', type=float, default=0)
    parser.add_argument('--duration-sec', type=float, default=10)
    parser.add_argument('--max-time-diff-sec', type=float, default=0.0005)
    parser.add_argument('--drain-timeout', type=float, default=3)
    parser.add_argument('--left-topic', default='/camera/left_ir/image_raw')
    parser.add_argument('--right-topic', default='/camera/right_ir/image_raw')
    args = parser.parse_args()
    if not math.isfinite(args.start_sec) or args.start_sec < 0 or any(not math.isfinite(x) or x <= 0 for x in
            (args.duration_sec, args.max_time_diff_sec, args.drain_timeout)):
        parser.error('Start must be nonnegative; duration, tolerance and drain timeout must be positive')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
