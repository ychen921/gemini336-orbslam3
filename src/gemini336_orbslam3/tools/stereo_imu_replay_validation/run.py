"""Run 6D-1 or 6D-2 in Docker; retain evidence in a fresh /validation."""
import argparse
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
from validation_common import analyze_folder

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


def prepare(full_bag):
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
    left = samples[LEFT] if full_bag else [
        row for row in samples[LEFT] if row[2] <= first + 60_000_000_000]
    right = samples[RIGHT][:len(left)]
    assert len(right) == len(left)
    assert all(abs(a[2] - b[2]) <= 2_000_000 for a, b in zip(left, right))
    imu_times = [row[2] for row in samples[IMU]]
    last_image = max(left[-1][2], right[-1][2])
    final_imu = bisect.bisect_right(imu_times, last_image)
    assert final_imu < len(imu_times)
    imu = samples[IMU] if full_bag else samples[IMU][:final_imu + 1]
    assert imu[0][2] <= min(left[0][2], right[0][2])
    assert max(b[2] - a[2] for a, b in zip(imu, imu[1:])) <= 20_000_000
    selected = {LEFT: left, RIGHT: right, IMU: imu}
    # Full replay reads the original bag directly, including all trailing IMU.
    # Only the short replay needs a separate bag to define its end precisely.
    if not full_bag:
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
    manifest['full_bag'] = full_bag
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full-bag', action='store_true',
                        help='Run 6D-2 against the complete original bag')
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument('--viewer', action='store_true',
                        help='Enable the real viewer for the diagnostic comparison')
    args = parser.parse_args()
    if (OUT / 'result.json').exists() or (OUT / 'clip').exists():
        raise RuntimeError('Use a fresh output directory; existing evidence is never overwritten')
    manifest, left = prepare(args.full_bag)
    prefix = Path(subprocess.check_output(
        ['ros2', 'pkg', 'prefix', 'gemini336_orbslam3'], text=True).strip())
    params = prefix / 'share/gemini336_orbslam3/config/stereo_imu_slam.yaml'
    command = [str(prefix / 'lib/gemini336_orbslam3/slam_node'), '--ros-args',
               '--params-file', str(params), '--log-level', 'slam_node:=debug',
               '-p', 'logging.directory:=/validation/project_logs', '-p', 'logging.level:=debug']
    if args.diagnostics:
        command += ["-p", "diagnostics.trace_path:=/validation/trace.csv"]
    if args.viewer:
        command += ['-p', 'enable_viewer:=true']
    bag = ROOT / 'bags/test_gemini336_stereo_imu' if args.full_bag else OUT / 'clip'
    player_command = ['ros2', 'bag', 'play', str(bag), '--rate', '1.0',
                      '--delay', '2', '--disable-keyboard-controls', '--topics', *TOPICS]
    playback_timeout = manifest['image_duration_sec'] + 40
    (OUT / 'parameters_source.yaml').write_text(params.read_text())
    (OUT / 'settings.yaml').write_text((ROOT / 'configs/Gemini_336_stereo_imu.yaml').read_text())
    save('command.json', {'node': command, 'player': player_command,
                          'environment': {k: os.environ.get(k) for k in
                                          ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION']}})
    result = {'passed': False}
    resources = []
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
            playback_start = time.monotonic()
            deadline = playback_start + playback_timeout
            next_sample = playback_start
            while player.poll() is None:
                # Monitor outside the node executor; retain raw CPU counters.
                if args.diagnostics and time.monotonic() >= next_sample:
                    sample = {'steady_ns': time.monotonic_ns(),
                              'clock_ticks': os.sysconf('SC_CLK_TCK'),
                              'loadavg': Path('/proc/loadavg').read_text(),
                              'cpu': Path('/proc/stat').read_text().splitlines()[0],
                              'memory': Path('/proc/meminfo').read_text()}
                    for name, process in [('node', node), ('player', player)]:
                        try:
                            sample[name] = {'pid': process.pid,
                                'stat': Path(f'/proc/{process.pid}/stat').read_text(),
                                'status': Path(f'/proc/{process.pid}/status').read_text()}
                        except FileNotFoundError:
                            sample[name] = None
                    resources.append(sample)
                    next_sample = time.monotonic() + 1.0
                if node.poll() is not None:
                    raise RuntimeError('Node exited before playback finished')
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Playback watchdog exceeded {playback_timeout:.3f} seconds')
                time.sleep(0.2)
            result['player_exit_code'] = player.returncode
            result['player_wall_sec'] = time.monotonic() - playback_start
            shutdown_start = time.monotonic()
            result['node_exit_code'] = node.wait(timeout=30)
            result['exit_after_player_sec'] = time.monotonic() - shutdown_start
        except Exception as error:
            result['error'] = str(error)
        finally:
            result['player_cleanup_required'] = stop(player)
            result['node_cleanup_required'] = stop(node)
            result['node_exit_code'] = node.returncode if node else None
            result['player_exit_code'] = player.returncode if player else None
            if args.diagnostics:
                save('resources.json', resources)

    # Preserve runner facts before offline parsing; analysis never supplies process exit codes.
    save('result.json', result)
    (OUT / 'node_exit_code.txt').write_text(str(result['node_exit_code']) + '\n')
    logs = list((OUT / 'project_logs').glob('*/slam.log'))
    if len(logs) == 1:
        save('artifacts.json', {'slam_log': str(logs[0].relative_to(OUT))})
    analysis = analyze_folder(OUT)
    save('stop_analysis.json', analysis)
    print(json.dumps(analysis, indent=2))
    return 1 if any(c['status'] == 'fail' for c in analysis['checks'].values()) else 2 if not analysis['passed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
