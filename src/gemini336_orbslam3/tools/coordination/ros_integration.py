"""Exercise the production main/executor with real ROS inputs and a stub backend."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

rclpy.init()
node = rclpy.create_node('coordination_test_publisher')
pubs = [node.create_publisher(Image, topic, qos_profile_sensor_data)
        for topic in ['/camera/left_ir/image_raw', '/camera/right_ir/image_raw']]
imu_pub = node.create_publisher(Imu, '/camera/gyro_accel/sample', qos_profile_sensor_data)

def imu(ms):
    msg = Imu()
    msg.header.stamp.sec, ns = divmod(ms, 1000)
    msg.header.stamp.nanosec = ns * 1000000
    msg.header.frame_id = 'imu'
    msg.linear_acceleration.z = 9.81
    imu_pub.publish(msg)

def frames():
    for i in range(10):
        msg = Image()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = i * 30000000
        msg.height = msg.width = msg.step = 2
        msg.encoding = 'mono8'
        msg.data = [0, 0, 0, 0]
        for pub in pubs:
            pub.publish(msg)
        time.sleep(0.015)

try:
    for name in ['stereo', 'imu', 'gap', 'backwards', 'timeout', 'backend', 'sigint']:
        env = os.environ.copy()
        if name == 'backend':
            env['TEST_FAIL_ON'] = '2'
        params = {'settings_path': '/stub/settings', 'sensor_mode': 'stereo' if name == 'stereo' else 'stereo_imu',
                  'imu.max_gap_sec': '0.02', 'input_timeout_sec': '0.8',
                  'stereo_imu.wait_timeout_sec': '0.6'}
        args = [sys.argv[1], '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        log_path = Path(sys.argv[2]) / f'{name}.log'
        with log_path.open('w') as output:
            process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, env=env)
            try:
                deadline = time.monotonic() + 8
                while not all(pub.get_subscription_count() for pub in pubs):
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f'{name}: discovery failed')
                    time.sleep(0.02)
                if name != 'stereo':
                    while imu_pub.get_subscription_count() == 0:
                        if time.monotonic() > deadline:
                            raise RuntimeError(f'{name}: IMU discovery failed')
                        time.sleep(0.02)
                time.sleep(0.2)
                if name in ['imu', 'backend']:
                    for ms in range(995, 1310, 5):
                        imu(ms)
                        time.sleep(0.005)
                elif name == 'gap':
                    imu(1000); time.sleep(0.02); imu(1030)
                elif name == 'backwards':
                    imu(1030); time.sleep(0.02); imu(1020)
                frames()
                if name == 'sigint':
                    process.send_signal(signal.SIGINT)
                result = process.wait(timeout=8)
            finally:
                if process.poll() is None:
                    process.kill(); process.wait()
        log = log_path.read_text()
        expected = 0 if name in ['stereo', 'imu', 'sigint'] else 1
        assert result == expected, (name, result, log)
        assert log.count('Stereo SLAM shutdown returned') == 1, (name, log)
        if name in ['stereo', 'imu']:
            assert 'First stereo frame processed' in log, (name, log)
        if name == 'imu':
            assert 'pending=0' in log and 'startup_discarded=0' in log, log
        if name == 'gap':
            assert 'interval=(' in log and 'gap' in log, log
        if name == 'backwards':
            assert 'IMU timestamp moved backwards' in log, log
        if name == 'timeout':
            assert 'wait timed out' in log and 'threshold_sec=' in log, log
        if name == 'backend':
            assert 'injected backend failure' in log and 'processed=1' in log, log
        print(f'PASS ROS {name}: exit={result}')
        deadline = time.monotonic() + 8
        while any(pub.get_subscription_count() for pub in pubs + [imu_pub]):
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name}: stale subscription discovery')
            time.sleep(0.05)
finally:
    node.destroy_node()
    rclpy.shutdown()
