"""Finite artifacts only; no ROS, backend workers or rosbag replay."""

import contextlib
import csv
import importlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

from t02_analysis_test import HEADER, ORIGIN_NS, event_text, make_row, write_metrics, z_rotation


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
FINAL = importlib.import_module('analyze_t02_final')


class FinalAnalysisTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.paths = {mode: self.root / mode for mode in ('stereo', 'stereo_imu')}
        self.rows = {}
        self.keyframes = {}
        for mode, path in self.paths.items():
            directory = path / 'evaluation'
            directory.mkdir(parents=True)
            metadata = dict(schema_version=3, sensor_mode=mode, run_id=mode,
                            bag_id='scene_00', pose_enabled=True, final_trajectory_enabled=True)
            (directory / 'metadata.json').write_text(json.dumps(metadata))
            with (directory / 'frames.csv').open('w', newline='') as output:
                writer = csv.DictWriter(output, fieldnames=HEADER)
                writer.writeheader()
                writer.writerows(make_row(i, mode == 'stereo_imu') for i in range(13))
            write_metrics(directory / 'summary.csv', dict(
                processed=13, recorded=13, dropped=0, core_snapshot_frames=13,
                tracked_mappoint_inliers_valid_samples=13, keyframes_inserted_last_frame=1,
                tracking_lost_events=0, relocalization_events=0))
            write_metrics(directory / 'inputs.csv', {'accounting_valid': 'true'})
            write_metrics(directory / 'backend_summary.csv', dict(
                backend_summary_available=1, keyframes_inserted_total=1,
                **{'event_' + name: 0 for name in FINAL.live.t01.EVENT_TYPES}))
            (directory / 'export_complete.txt').write_text('schema_version=3\nrecorded=13\ndropped=0\n')
            (path / 'slam.log').write_text(event_text('map_baseline', 1, 9_999_999_999))
            (path / 'console.log').write_text(
                'STOP_PROCESS schema_version=1 cleanup_failed=false logging_finish_status=ok '
                'logging_dropped_messages=0 exit_code=0\n')

            frames, keyframes = [], []
            for index in range(13):
                position, angle = ((0, 0, 0), 0) if index <= 3 else (
                    ((3, 4, 0), 60) if index >= 9 else ((1, 2, 0), 20))
                quaternion = z_rotation(angle)
                if index % 2:
                    quaternion = tuple(-v for v in quaternion)
                # Export time is one ns below the exact sensor time, exercising
                # matching at the original inclusive window boundaries.
                stamp = ORIGIN_NS + index * 1_000_000_000 - 1
                frames.append((stamp, *position, *quaternion))
                # Independent known T_map_origin: +90 deg about Z, then (10,-7,2).
                # Do not use the analyzer's compose() to create its own oracle.
                x, y, z = position
                keyframes.append((stamp, 10-y, -7+x, 2+z, *z_rotation(angle+90)))
            self.rows[mode], self.keyframes[mode] = frames, keyframes
            self.write_export(mode)

    def write_export(self, mode):
        directory = self.paths[mode] / 'evaluation/final_trajectory'
        directory.mkdir(exist_ok=True)
        files = {}
        for kind, rows in (('frames', self.rows[mode]), ('keyframes', self.keyframes[mode])):
            path = directory / FINAL.FILES[kind]
            path.write_text(''.join(str(row[0]) + '.000000 ' +
                                    ' '.join(str(v) for v in row[1:]) + '\n' for row in rows))
            files[kind] = dict(path=path.name, present=True, bytes=path.stat().st_size,
                               considered=len(rows), written=len(rows), lost=0,
                               invalid_reference=0, other_map=0, invalid_pose=0, invalid_timestamp=0,
                               first_timestamp_sec=rows[0][0]/1e9 if rows else None,
                               last_timestamp_sec=rows[-1][0]/1e9 if rows else None)
        manifest = dict(
            schema_version=1, evaluation_schema_version=3, sensor_mode=mode, run_id=mode,
            backend_shutdown_returned=True, export_complete=True, available=True, status='complete',
            map_selected=True, map_id=0, timestamp_unit='ns', translation_unit='m',
            quaternion_order='xyzw', frame_pose_convention='T_origin_sensor',
            keyframe_pose_convention='T_map_sensor',
            pose_target='imu_body' if mode == 'stereo_imu' else 'left_camera',
            frame_origin_in_map=dict(translation_m=[10, -7, 2], quaternion_xyzw=z_rotation(90)),
            body_from_camera=(dict(translation_m=[1, 0, 0],
                                   quaternion_xyzw=[math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
                              if mode == 'stereo_imu' else None), files=files)
        (directory / 'manifest.json').write_text(json.dumps(manifest))

    def manifest_path(self, mode='stereo'):
        return self.paths[mode] / 'evaluation/final_trajectory/manifest.json'

    def analyze(self):
        return FINAL.analyze(self.paths['stereo'], self.paths['stereo_imu'])

    def test_known_camera_endpoints_and_independent_keyframe_agreement(self):
        report = self.analyze()
        self.assertTrue(report['comparison_available'])
        self.assertTrue(report['input_artifacts_unchanged'])
        stereo, imu = report['runs']
        self.assertAlmostEqual(stereo['final_optimized_endpoint']['translation_m'], 5)
        # Rotating the body lever arm changes the camera displacement. This also
        # exercises a nonidentity body-to-camera rotation and both pose origins.
        self.assertAlmostEqual(imu['final_optimized_endpoint']['translation_m'],
                               math.hypot(2.5, 4 + math.sqrt(3)/2))
        for run, start in ((stereo, (10, -7, 2)), (imu, (10, -6, 2))):
            self.assertAlmostEqual(run['final_optimized_endpoint']['rotation_deg'], 60)
            for actual, expected in zip(run['windows']['start']['translation_mean_m'], start):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(run['windows']['start']['samples'], 4)
            self.assertEqual(run['windows']['end']['samples'], 4)
            agreement = run['keyframe_frame_agreement']
            self.assertEqual(agreement['matched_keyframes'], 13)
            self.assertLess(agreement['max_translation_m'], 1e-12)
            self.assertLess(agreement['max_rotation_deg'], 1e-10)
        self.assertEqual(report['shared_windows_ns']['start'][0], ORIGIN_NS)
        self.assertEqual(stereo['timestamp_alignment']['frames']['max_rounding_delta_ns'], 1)

    def test_missing_original_start_is_not_replaced_by_first_pose(self):
        self.rows['stereo_imu'] = self.rows['stereo_imu'][4:]
        self.write_export('stereo_imu')
        report = self.analyze()
        run = report['runs'][1]
        self.assertFalse(report['comparison_available'])
        self.assertIsNone(run['final_optimized_endpoint']['translation_m'])
        self.assertEqual(run['windows']['start']['samples'], 0)
        self.assertTrue(run['first_valid_pose_to_last_pose']['available'])
        self.assertEqual(run['first_valid_pose_to_last_pose']['first_offset_sec'], 4)
        self.assertEqual(report['shared_windows_ns']['start'][0], ORIGIN_NS)

    def test_identical_history_duplicates_do_not_overweight_window(self):
        expected = self.analyze()['runs'][0]['final_optimized_endpoint']
        self.rows['stereo'][1:1] = [self.rows['stereo'][0]] * 10
        self.write_export('stereo')
        run = self.analyze()['runs'][0]
        self.assertEqual(run['final_optimized_endpoint'], expected)
        self.assertEqual(run['windows']['start']['samples'], 4)
        self.assertEqual(run['timestamp_alignment']['frames']['duplicate_rows_removed'], 10)

    def test_conflicting_duplicate_is_rejected(self):
        row = list(self.rows['stereo'][0])
        row[1] = 10
        self.rows['stereo'].insert(1, tuple(row))
        self.write_export('stereo')
        with self.assertRaisesRegex(ValueError, 'conflicting poses'):
            self.analyze()

    def test_nonfinite_pose_and_nonunit_rotation_are_rejected(self):
        original = self.rows['stereo'][0]
        for index, value in ((1, float('nan')), (7, 2)):
            with self.subTest(index=index):
                row = list(original)
                row[index] = value
                self.rows['stereo'][0] = tuple(row)
                self.write_export('stereo')
                with self.assertRaises(ValueError):
                    self.analyze()

    def test_wrong_manifest_units_are_rejected(self):
        path = self.manifest_path()
        manifest = json.loads(path.read_text())
        manifest['timestamp_unit'] = 's'
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'incompatible manifest'):
            self.analyze()

    def test_file_counts_are_checked_after_size_matches(self):
        path = self.manifest_path()
        manifest = json.loads(path.read_text())
        manifest['files']['frames']['written'] += 1
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'row counts'):
            self.analyze()

    def test_timestamp_without_unique_match_is_rejected(self):
        row = list(self.rows['stereo'][0])
        row[0] += 10_000
        self.rows['stereo'][0] = tuple(row)
        self.write_export('stereo')
        with self.assertRaisesRegex(ValueError, 'unique recorded sensor match'):
            self.analyze()

    def test_process_completion_is_required(self):
        (self.paths['stereo'] / 'console.log').write_text('')
        report = self.analyze()
        self.assertFalse(report['runs'][0]['final_optimized_endpoint']['available'])
        self.assertFalse(report['runs'][0]['collection_checks']['process_ok'])

    def test_missing_manifest_is_explicitly_unavailable(self):
        self.manifest_path().unlink()
        run = self.analyze()['runs'][0]
        self.assertIn('final_trajectory_unavailable', run['final_optimized_endpoint']['reasons'])

    def test_cli_refuses_to_overwrite_input(self):
        path = self.manifest_path()
        before = path.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            FINAL.main([str(self.paths['stereo']), str(self.paths['stereo_imu']), '--output', str(path)])
        self.assertEqual(error.exception.code, 1)
        self.assertEqual(path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
