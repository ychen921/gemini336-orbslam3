"""Run 6D-1 in the project Docker image; retain evidence in a fresh /validation."""
import bisect
import collections
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu

ROOT = Path('/workspaces/gemini336-orbslam3')
OUT = Path('/validation')
LEFT = '/camera/left_ir/image_raw'
RIGHT = '/camera/right_ir/image_raw'
IMU = '/camera/gyro_accel/sample'
TOPICS = [LEFT, RIGHT, IMU]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + '\n')


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def prepare():
    # Select complete corresponding image sequences by header time, preserving
    # original serialized bytes and recorded playback order in a new bag.
    source = ROOT / 'bags/test_gemini336_stereo_imu/test_gemini336_stereo_imu_0.db3'
    connection = sqlite3.connect(f'file:{source}?mode=ro', uri=True)
    topics = {row[0]: row[1:] for row in connection.execute(
        'SELECT id,name,type,serialization_format,offered_qos_profiles FROM topics')}
    samples = {name: [] for name in TOPICS}
    for ident, topic_id, recorded, data in connection.execute(
            'SELECT id,topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
        name = topics[topic_id][0]
        if name not in samples:
            continue
        message = deserialize_message(data, Imu if name == IMU else Image)
        samples[name].append((ident, recorded, stamp(message)))
    first = samples[LEFT][0][2]
    left = [row for row in samples[LEFT] if row[2] <= first + 60_000_000_000]
    right = samples[RIGHT][:len(left)]
    assert len(right) == len(left)
    assert all(abs(a[2] - b[2]) <= 2_000_000 for a, b in zip(left, right))
    imu_times = [row[2] for row in samples[IMU]]
    last_image = max(left[-1][2], right[-1][2])
    final_imu = bisect.bisect_right(imu_times, last_image)
    assert final_imu < len(imu_times)
    imu = samples[IMU][:final_imu + 1]
    assert imu[0][2] <= min(left[0][2], right[0][2])
    assert max(b[2] - a[2] for a, b in zip(imu, imu[1:])) <= 20_000_000
    selected = {LEFT: left, RIGHT: right, IMU: imu}
    ids = {row[0] for rows in selected.values() for row in rows}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(OUT / 'clip'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, kind, serialization, qos in topics.values():
        if name in TOPICS:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=name, type=kind, serialization_format=serialization,
                offered_qos_profiles=qos))
    for ident, topic_id, recorded, data in connection.execute(
            'SELECT id,topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
        if ident in ids:
            writer.write(topics[topic_id][0], data, recorded)
    del writer
    connection.close()
    manifest = {name: {'count': len(rows), 'first_header_ns': rows[0][2],
                      'last_header_ns': rows[-1][2]} for name, rows in selected.items()}
    manifest['image_duration_sec'] = (left[-1][2] - first) / 1e9
    save('clip_manifest.json', manifest)
    save('clip_timestamps.json', {name: [row[2] for row in rows] for name, rows in selected.items()})
    return manifest, left


def stop(process):
    # Cleanup never counts as successful natural shutdown.
    if process is None or process.poll() is not None:
        return False
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    return True


def main():
    if (OUT / 'result.json').exists() or (OUT / 'clip').exists():
        raise RuntimeError('Use a fresh output directory; existing evidence is never overwritten')
    manifest, left = prepare()
    prefix = Path(subprocess.check_output(
        ['ros2', 'pkg', 'prefix', 'gemini336_orbslam3'], text=True).strip())
    params = prefix / 'share/gemini336_orbslam3/config/stereo_imu_slam.yaml'
    command = [str(prefix / 'lib/gemini336_orbslam3/slam_node'), '--ros-args',
               '--params-file', str(params), '--log-level', 'slam_node:=debug']
    player_command = ['ros2', 'bag', 'play', str(OUT / 'clip'), '--rate', '1.0',
                      '--delay', '2', '--disable-keyboard-controls']
    (OUT / 'parameters_source.yaml').write_text(params.read_text())
    (OUT / 'settings.yaml').write_text((ROOT / 'configs/Gemini_336_stereo_imu.yaml').read_text())
    save('command.json', {'node': command, 'player': player_command,
                          'environment': {k: os.environ.get(k) for k in
                                          ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION']}})
    result = {'passed': False}
    node = player = None
    with (OUT / 'node.log').open('w') as node_log, (OUT / 'player.log').open('w') as player_log:
        try:
            node = subprocess.Popen(command, cwd=OUT, stdout=node_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 60
            while 'Stereo SLAM initialized:' not in (OUT / 'node.log').read_text():
                if node.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Backend startup failed or exceeded 60 seconds')
                time.sleep(0.2)
            # Query the actual graph before playback; avoid unrelated publishers.
            for name, topic in zip(['left', 'right', 'imu'], TOPICS):
                graph = subprocess.check_output(
                    ['ros2', 'topic', 'info', topic, '--verbose', '--no-daemon'],
                    text=True, stderr=subprocess.STDOUT, timeout=20)
                (OUT / (name + '_topic.txt')).write_text(graph)
                assert 'Publisher count: 0' in graph and 'Subscription count: 1' in graph
            snapshot = subprocess.check_output(['ros2', 'param', 'dump', '/slam_node'],
                                                text=True, timeout=20)
            (OUT / 'parameters.yaml').write_text(snapshot)
            player = subprocess.Popen(player_command, cwd=OUT, stdout=player_log,
                                      stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 100
            while player.poll() is None:
                if node.poll() is not None:
                    raise RuntimeError('Node exited before playback finished')
                if time.monotonic() > deadline:
                    raise RuntimeError('Playback watchdog exceeded 100 seconds')
                time.sleep(0.2)
            result['player_exit_code'] = player.returncode
            result['node_exit_code'] = node.wait(timeout=30)
        except Exception as error:
            result['error'] = str(error)
        finally:
            result['player_cleanup_required'] = stop(player)
            result['node_cleanup_required'] = stop(node)
            result['node_exit_code'] = node.returncode if node else None
            result['player_exit_code'] = player.returncode if player else None

    log = (OUT / 'node.log').read_text()
    frames = re.findall(r'Stereo frame: index=(\d+) timestamp=([\d.]+) track_ms=([\d.]+) state=(\w+)', log)
    save('frames.json', [{'index': int(i), 'timestamp': t, 'track_ms': float(ms), 'state': s}
                         for i, t, ms, s in frames])
    result['processed_debug_frames'] = len(frames)
    result['states'] = dict(collections.Counter(row[3] for row in frames))
    result['initialization_evidence'] = [line for line in log.splitlines()
                                         if 'VIBA' in line]
    result['reset_or_motion_events'] = [line for line in log.splitlines()
                                        if any(word in line.lower() for word in
                                               ['reset', 'not enough acceleration', 'not enough motion', 'scale too small'])]
    coordination = re.search(r'coordination: final=true enqueued=(\d+) processed=(\d+) startup_discarded=(\d+) pending=(\d+) pending_peak=(\d+)', log)
    imu_stats = re.search(r'Final IMU input: received=(\d+) accepted=(\d+) backwards=(\d+) overflow=(\d+) buffered=(\d+)', log)
    checks = {}
    if coordination:
        enqueued, processed, discarded, pending, peak = map(int, coordination.groups())
        result['coordination'] = dict(zip(['enqueued', 'processed', 'startup_discarded', 'pending', 'pending_peak'],
                                          [enqueued, processed, discarded, pending, peak]))
        checks['accounting'] = enqueued == processed + discarded + pending
        checks['drained'] = pending == 0
        # Accept only the known final-pair omission, never an interior gap.
        observed = [float(row[1]) for row in frames]
        expected = [row[2] / 1e9 for row in left]
        checks['image_prefix'] = (len(observed) in [len(expected), len(expected)-1]
                                  and all(abs(a-b) < 1e-6 for a,b in zip(observed, expected)))
        checks['debug_count_matches'] = len(frames) == processed
    if imu_stats:
        received, accepted, backwards, overflow, buffered = map(int, imu_stats.groups())
        result['imu'] = dict(zip(['received', 'accepted', 'backwards', 'overflow', 'buffered'],
                                 [received, accepted, backwards, overflow, buffered]))
        checks['imu_counts'] = received == accepted == manifest[IMU]['count']
        checks['imu_order'] = backwards == 0
    checks['natural_exit'] = (result['node_exit_code'] == result['player_exit_code'] == 0
                              and not result['node_cleanup_required'] and not result['player_cleanup_required'])
    checks['final_reports'] = all(log.count(text) == 1 for text in
                                  ['scope=total', 'coordination: final=true', 'Stereo SLAM shutdown returned'])
    checks['initialization_evidence'] = 'start VIBA 1' in log
    checks['required_statistics'] = coordination is not None and imu_stats is not None
    result['checks'] = checks
    result['passed'] = all(checks.values()) and 'error' not in result
    save('result.json', result)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
