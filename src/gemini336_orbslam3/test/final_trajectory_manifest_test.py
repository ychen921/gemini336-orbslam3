#!/usr/bin/env python3
"""Check the JSON contract produced by the finite C++ publisher fixtures."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile


with tempfile.TemporaryDirectory(prefix='gemini-final-trajectory-json-') as temporary:
    root = Path(temporary)
    subprocess.run([sys.argv[1], str(root)], check=True)
    for mode, target in (('stereo', 'left_camera'), ('stereo_imu', 'imu_body')):
        packet = root / mode
        report = json.loads((packet / 'manifest.json').read_text())
        assert report['schema_version'] == 1 and report['evaluation_schema_version'] == 3
        assert report['export_complete'] is True and report['available'] is True
        assert report['status'] == 'complete' and report['backend_shutdown_returned'] is True
        assert report['run_id'] == 'run"id\n' and report['sensor_mode'] == mode
        assert report['map_id'] == 0 and report['origin_keyframe_id'] == 0
        assert report['pose_target'] == target and report['all_gba_completed'] is None
        assert report['timestamp_unit'] == 'ns' and report['quaternion_order'] == 'xyzw'
        assert report['frame_origin_in_map']['translation_m'] == [1, 2, 3]
        assert report['frame_origin_in_map']['quaternion_xyzw'] == [0, 0, 0, 1]
        if mode == 'stereo_imu':
            assert report['imu_initialized'] is True
            assert report['body_from_camera']['translation_m'] == [0.1, 0.2, 0.3]
        else:
            assert report['imu_initialized'] is None and report['body_from_camera'] is None
        for info in report['files'].values():
            path = packet / info['path']
            assert info['present'] is True and info['bytes'] == path.stat().st_size
            assert info['written'] == info['considered'] == len(path.read_text().splitlines()) == 1
            assert info['first_timestamp_sec'] == info['last_timestamp_sec'] == 1.25
    for status in ('no_map', 'no_keyframes', 'no_frame_poses'):
        report = json.loads((root / status / 'manifest.json').read_text())
        assert report['export_complete'] is True and report['available'] is False
        assert report['status'] == status and report['files']['frames']['first_timestamp_sec'] is None
        if status != 'no_frame_poses':
            assert report['map_id'] is None and report['frame_origin_in_map'] is None
            assert all(info['present'] is False and info['bytes'] is None for info in report['files'].values())

print('Final trajectory manifest JSON checks passed')
