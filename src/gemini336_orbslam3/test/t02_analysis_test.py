"""Finite synthetic artifacts only; no ROS, rosbag replay, or backend workers."""

import contextlib
import csv
import hashlib
import importlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
T02 = importlib.import_module('analyze_t02')


ORIGIN_NS = 1700000000000000001  # One-nanosecond precision beyond a double epoch.
HEADER = (
    'sensor_timestamp_ns', 'enqueue_sequence', 'processed_index', 'enqueue_steady_ns',
    'adapter_start_steady_ns', 'adapter_return_steady_ns', 'raw_tracking_state',
    'pending_before_call', 'in_flight_before_call', 'outstanding_before_call', 'imu_samples',
    'adapter_time_ms', 'enqueue_to_return_ms', 'core_snapshot_valid', 'core_frame_id',
    'tracking_map_valid', 'tracking_map_id', 'tracked_mappoint_inliers_valid',
    'tracked_mappoint_inliers', 'keyframes_inserted_total', 'tracking_lost_events',
    'relocalization_events', 'pose_enabled', 'pose_valid', 'pose_capture_steady_ns',
    'map_change_index_valid', 'map_change_index', 'imu_initialized_valid', 'imu_initialized',
    'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw',
)


def z_rotation(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_row(index, inertial):
    row = {name: 0 for name in HEADER}
    start = 10_000_000_000 + index * 10_000_000
    row.update(sensor_timestamp_ns=ORIGIN_NS + index * 1_000_000_000,
               enqueue_sequence=index + 1, processed_index=index + 1,
               enqueue_steady_ns=start - 1_000_000, adapter_start_steady_ns=start,
               adapter_return_steady_ns=start + 2_000_000, raw_tracking_state=2,
               core_snapshot_valid=1, core_frame_id=index, tracking_map_valid=1,
               tracked_mappoint_inliers_valid=1, tracked_mappoint_inliers=80,
               keyframes_inserted_total=1, pose_enabled=1, pose_valid=1,
               pose_capture_steady_ns=start + 1_000_000,
               map_change_index_valid=1, map_change_index=index,
               imu_initialized_valid=int(inertial), imu_initialized=1 if inertial else 'N/A')
    if index <= 3:
        translation, quaternion = (0.0, 0.0, 0.0), z_rotation(0)
    elif index >= 9:
        translation = (0.3, 0.4, 0.0) if inertial else (3.0, 4.0, 0.0)
        quaternion = z_rotation(60)
    else:
        translation, quaternion = (1.0, 2.0, 0.0), z_rotation(20)
    # Alternate antipodal encodings of the same physical rotation.
    if index % 2:
        quaternion = tuple(-value for value in quaternion)
    row.update(zip(('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'), translation + quaternion))
    return row


def event_text(kind, sequence, steady_ns, map_id=0):
    return (f'T01_EVENT type={kind} sequence={sequence} steady_ns={steady_ns} '
            f'frame_valid=0 frame_id=0 sensor_timestamp_sec=0 map_valid=1 map_id={map_id} '
            'target_map_valid=0 target_map_id=0 result_map_valid=0 result_map_id=0 '
            'keyframes_valid=0 keyframe_id=0 target_keyframe_id=0\n')


def write_metrics(path, values):
    with path.open('w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(('metric', 'value'))
        writer.writerows(values.items())


class RotationTests(unittest.TestCase):
    def test_antipodal_mean_and_near_180(self):
        rotation = z_rotation(175)
        mean = T02.quaternion_mean([rotation, tuple(-v for v in rotation), rotation])
        self.assertAlmostEqual(T02.rotation_distance_deg(mean, rotation), 0, places=9)
        mean = T02.quaternion_mean([z_rotation(179), z_rotation(-179)])
        self.assertAlmostEqual(T02.rotation_distance_deg(mean, z_rotation(180)), 0, places=9)
        self.assertAlmostEqual(T02.rotation_distance_deg(mean, z_rotation(179)), 1, places=9)

    def test_ambiguous_rotation_does_not_choose_an_axis(self):
        with self.assertRaisesRegex(ValueError, 'ambiguous_rotation_mean'):
            T02.quaternion_mean([z_rotation(0), z_rotation(180)])

    def test_small_angle_is_preserved(self):
        self.assertAlmostEqual(T02.rotation_distance_deg(z_rotation(0), z_rotation(1e-6)),
                               1e-6, places=12)

    def test_general_axis_rotation_mean(self):
        # Equal rotations about an arbitrary axis exercise all six matrix
        # off-diagonal entries, beyond the planar fixtures used below.
        norm = math.sqrt(14.0)
        half = math.radians(110) / 2
        rotation = tuple(v * math.sin(half) / norm for v in (1, 2, 3)) + (math.cos(half),)
        mean = T02.quaternion_mean([rotation, tuple(-v for v in rotation)] * 3)
        self.assertAlmostEqual(T02.rotation_distance_deg(mean, rotation), 0, places=9)


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.paths = {mode: self.root / mode for mode in ('stereo', 'stereo_imu')}
        self.rows = {mode: [make_row(i, mode == 'stereo_imu') for i in range(13)]
                     for mode in self.paths}
        self.metadata = {mode: {} for mode in self.paths}
        self.summary = {mode: {} for mode in self.paths}
        self.events = {mode: [('map_baseline', 1, 9_999_999_999)] for mode in self.paths}
        self.extra_log = {mode: '' for mode in self.paths}
        self.logger_losses = {mode: 0 for mode in self.paths}
        self.accounting = {mode: 'true' for mode in self.paths}
        self.write_sessions()

    def write_sessions(self):
        for mode, path in self.paths.items():
            directory = path / 'evaluation'
            directory.mkdir(parents=True, exist_ok=True)
            metadata = dict(schema_version=3, sensor_mode=mode, pose_enabled=True,
                            bag_id='scene_00', run_id=mode, repository_commit='unknown',
                            core_commit='unknown', repository_dirty='unknown', core_dirty='unknown')
            metadata.update(self.metadata[mode])
            (directory / 'metadata.json').write_text(json.dumps(metadata))
            rows, events = self.rows[mode], self.events[mode]
            with (directory / 'frames.csv').open('w', newline='') as output:
                writer = csv.DictWriter(output, fieldnames=HEADER)
                writer.writeheader()
                writer.writerows(rows)
            summary = dict(processed=len(rows), recorded=len(rows), dropped=0,
                           core_snapshot_frames=len(rows),
                           tracked_mappoint_inliers_valid_samples=len(rows),
                           keyframes_inserted_last_frame=1,
                           tracking_lost_events=sum(
                               kind == 'tracking_lost' for kind, _, _ in events),
                           relocalization_events=sum(
                               kind == 'relocalized' for kind, _, _ in events))
            summary.update(self.summary[mode])
            write_metrics(directory / 'summary.csv', summary)
            write_metrics(directory / 'inputs.csv', {'accounting_valid': self.accounting[mode]})
            (directory / 'export_complete.txt').write_text(
                f"schema_version=3\nrecorded={summary['recorded']}\ndropped={summary['dropped']}\n")
            backend = dict(backend_summary_available=1, keyframes_inserted_total=1)
            backend.update({'event_' + name: sum(kind == name for kind, _, _ in events)
                            for name in T02.t01.EVENT_TYPES})
            write_metrics(directory / 'backend_summary.csv', backend)
            # Reverse delivery order: decisions must use source steady_ns.
            (path / 'slam.log').write_text(
                ''.join(event_text(*event, map_id=rows[0]['tracking_map_id'] if rows else 0)
                        for event in reversed(events)) + self.extra_log[mode])
            (path / 'console.log').write_text(
                'STOP_PROCESS schema_version=1 cleanup_failed=false logging_finish_status=ok '
                f'logging_dropped_messages={self.logger_losses[mode]} exit_code=0\n')

    def analyze(self, **kwargs):
        self.write_sessions()
        return T02.analyze(self.paths['stereo'], self.paths['stereo_imu'], **kwargs)

    def test_known_endpoint_and_zero_jitter(self):
        report = self.analyze()
        self.assertTrue(report['comparison_available'])
        self.assertFalse(report['stationary_windows_verified'])
        self.assertEqual(report['shared_windows_ns']['start'],
                         [ORIGIN_NS, ORIGIN_NS + 3_000_000_000])
        for run, distance in zip(report['runs'], (5.0, 0.5)):
            endpoint = run['live_endpoint']
            self.assertAlmostEqual(endpoint['translation_m'], distance, places=12)
            self.assertAlmostEqual(endpoint['rotation_deg'], 60, places=9)
            self.assertFalse(run['coordinate_context']['coordinate_version_confirmed'])
            self.assertGreater(run['coordinate_context']['map_change_index_distinct'], 1)
            self.assertFalse(run['final_optimized_endpoint']['available'])
            for window in run['windows'].values():
                self.assertEqual(window['valid_samples'], 4)
                self.assertAlmostEqual(window['translation_rms_m'], 0, places=12)
                self.assertAlmostEqual(window['rotation_rms_deg'], 0, places=9)
        self.assertEqual(report['runs'][0]['windows']['end']['translation_mean_m'],
                         (3.0, 4.0, 0.0))
        self.assertEqual(report['runs'][1]['windows']['end']['translation_mean_m'],
                         (0.3, 0.4, 0.0))

    def test_window_variation_uses_distance_and_rotation_from_mean(self):
        for index, (position, angle) in enumerate(((-1, -10), (0, 0), (1, 10), (0, 0))):
            self.rows['stereo'][index]['tx'] = position
            self.rows['stereo'][index].update(zip(('qx', 'qy', 'qz', 'qw'), z_rotation(angle)))
        window = self.analyze()['runs'][0]['windows']['start']
        self.assertAlmostEqual(window['translation_mean_m'][0], 0, places=12)
        self.assertAlmostEqual(window['translation_rms_m'], math.sqrt(0.5), places=12)
        self.assertAlmostEqual(window['translation_max_m'], 1, places=12)
        self.assertAlmostEqual(window['rotation_rms_deg'], math.sqrt(50), places=9)
        self.assertAlmostEqual(window['rotation_max_deg'], 10, places=9)

    def test_missing_origin_never_moves_to_first_valid_pose(self):
        for row in self.rows['stereo_imu'][:4]:
            row['pose_valid'] = 0  # Even zero/identity storage is not a measurement.
        report = self.analyze()
        run = report['runs'][1]
        self.assertFalse(run['live_endpoint']['available'])
        self.assertIsNone(run['live_endpoint']['translation_m'])
        self.assertEqual(run['windows']['start']['valid_samples'], 0)
        self.assertIsNone(run['windows']['start']['translation_mean_m'])
        self.assertEqual(run['windows']['start']['sensor_start_ns'], ORIGIN_NS)
        self.assertTrue(run['windows']['end']['available'])

    def test_invalid_numeric_pose_and_capture_are_excluded(self):
        for column, value, reason in (
                ('tx', 'nan', 'invalid_numeric_pose'), ('ty', 'inf', 'invalid_numeric_pose'),
                ('qz', 'nan', 'invalid_numeric_pose'), ('qw', 2, 'invalid_rotation'),
                ('pose_capture_steady_ns', 1, 'capture_time_invalid')):
            with self.subTest(column=column, value=value):
                self.rows['stereo'] = [make_row(i, False) for i in range(13)]
                for row in self.rows['stereo'][:4]:
                    row[column] = value
                run = self.analyze()['runs'][0]
                self.assertFalse(run['live_endpoint']['available'])
                self.assertEqual(run['windows']['start']['invalid_pose_counts'][reason], 4)

    def test_ambiguous_window_mean_is_unavailable(self):
        for index, row in enumerate(self.rows['stereo'][:4]):
            row.update(zip(('qx', 'qy', 'qz', 'qw'), z_rotation(180 if index % 2 else 0)))
        start = self.analyze()['runs'][0]['windows']['start']
        self.assertFalse(start['available'])
        self.assertIn('ambiguous_rotation_mean', start['reasons'])

    def test_same_windows_with_different_valid_samples(self):
        self.rows['stereo_imu'].pop(0)
        for index, row in enumerate(self.rows['stereo_imu'], 1):
            row.update(enqueue_sequence=index, processed_index=index)
        report = self.analyze()
        self.assertTrue(report['comparison_available'])
        start = [run['windows']['start'] for run in report['runs']]
        self.assertEqual([window['sensor_start_ns'] for window in start], [ORIGIN_NS] * 2)
        self.assertEqual([window['valid_samples'] for window in start], [4, 3])
        self.assertEqual(start[1]['first_valid_sensor_ns'], ORIGIN_NS + 1_000_000_000)

    def test_explicit_offsets_and_original_sensor_bounds(self):
        report = self.analyze(start_window=(1, 3), end_window=(9, 11))
        self.assertTrue(report['comparison_available'])
        self.assertEqual(report['shared_windows_ns']['start'],
                         [ORIGIN_NS + 1_000_000_000, ORIGIN_NS + 3_000_000_000])
        report = self.analyze(
            sensor_range_ns=(ORIGIN_NS - 2_000_000_000, ORIGIN_NS + 14_000_000_000))
        self.assertEqual(report['sensor_bounds']['source'], 'operator_supplied_sensor_bounds')
        self.assertFalse(report['comparison_available'])  # Only two endpoint samples remain.
        self.assertEqual(report['shared_windows_ns']['start'][0], ORIGIN_NS - 2_000_000_000)

    def test_coordinate_events_block_even_reused_map_ids(self):
        for kind in ('map_created', 'map_switched', 'active_map_reset',
                     'atlas_reset', 'merge_applied'):
            with self.subTest(event=kind):
                self.events['stereo'] = [('map_baseline', 1, 9_999_999_999),
                                         (kind, 2, 10_063_000_000)]
                run = self.analyze()['runs'][0]
                context = run['coordinate_context']
                self.assertEqual(context['tracking_map_ids'], [0])
                self.assertFalse(context['screening_passed'])
                self.assertEqual(context['critical_events'][0]['type'], kind)
                self.assertIn('coordinate_change_event', run['live_endpoint']['reasons'])
                self.assertIsNone(run['live_endpoint']['translation_m'])

    def test_events_outside_pose_interval_do_not_relabel_live_endpoints(self):
        self.events['stereo'].extend([
            ('active_map_reset', 2, 9_999_999_998),
            ('map_created', 3, 10_123_000_000),
        ])
        self.assertTrue(self.analyze()['runs'][0]['live_endpoint']['available'])

    def test_map_id_changes_and_counter_decreases(self):
        self.rows['stereo'][6]['tracking_map_id'] = 42
        run = self.analyze()['runs'][0]
        self.assertIn('map_identity_changed', run['live_endpoint']['reasons'])
        self.rows['stereo'][6]['tracking_map_id'] = 0
        self.rows['stereo'][6]['map_change_index'] = 0
        run = self.analyze()['runs'][0]
        self.assertIn('map_change_counter_decreased', run['live_endpoint']['reasons'])
        self.rows['stereo'][6]['map_change_index'] = 6
        self.rows['stereo'][3]['tracking_map_id'] = 42
        window = self.analyze()['runs'][0]['windows']['start']
        self.assertIn('mixed_maps_in_window', window['reasons'])
        self.assertIsNone(window['translation_mean_m'])

    def test_missing_route_context_is_unconfirmed(self):
        self.rows['stereo'][6]['map_change_index_valid'] = 0
        run = self.analyze()['runs'][0]
        self.assertIn('map_change_context_unavailable', run['live_endpoint']['reasons'])

    def test_map_ids_are_not_compared_across_independent_runs(self):
        for row in self.rows['stereo_imu']:
            row['tracking_map_id'] = 42
        report = self.analyze()
        self.assertTrue(report['comparison_available'])
        self.assertEqual(report['runs'][0]['coordinate_context']['tracking_map_ids'], [0])
        self.assertEqual(report['runs'][1]['coordinate_context']['tracking_map_ids'], [42])

    def test_sensor_and_capture_order_must_be_preserved(self):
        rows = self.rows['stereo']
        rows[5]['sensor_timestamp_ns'] = rows[4]['sensor_timestamp_ns']
        run = self.analyze()['runs'][0]
        self.assertIn('sensor_timestamps_not_ordered', run['live_endpoint']['reasons'])
        rows[5]['sensor_timestamp_ns'] = ORIGIN_NS + 5_000_000_000
        for name in ('adapter_start_steady_ns', 'adapter_return_steady_ns',
                     'pose_capture_steady_ns'):
            rows[5][name] = rows[4][name]
        run = self.analyze()['runs'][0]
        self.assertIn('capture_timestamps_not_ordered', run['live_endpoint']['reasons'])

    def test_imu_initialization_changes_or_missing_context(self):
        for row in self.rows['stereo_imu'][:4]:
            row['imu_initialized'] = 0
        run = self.analyze()['runs'][1]
        self.assertTrue(run['windows']['start']['available'])
        self.assertIn('imu_initialization_changed', run['live_endpoint']['reasons'])
        self.rows['stereo_imu'][3]['imu_initialized'] = 1
        start = self.analyze()['runs'][1]['windows']['start']
        self.assertIn('imu_initialization_changed_in_window', start['reasons'])
        for row in self.rows['stereo_imu']:
            row['imu_initialized_valid'] = 0
        start = self.analyze()['runs'][1]['windows']['start']
        self.assertIn('imu_initialization_context_unavailable', start['reasons'])

    def test_uninitialized_imu_is_reported_separately_from_pose_validity(self):
        for row in self.rows['stereo_imu']:
            row['imu_initialized'] = 0
        report = self.analyze()
        self.assertTrue(report['runs'][1]['live_endpoint']['available'])
        self.assertTrue(any('remained uninitialized' in warning for warning in report['warnings']))

    def test_overflow_log_loss_and_input_accounting_gate_endpoint(self):
        self.summary['stereo'].update(processed=15, dropped=2, core_snapshot_frames=15)
        run = self.analyze()['runs'][0]
        self.assertIn('frame_records_incomplete', run['live_endpoint']['reasons'])
        self.summary['stereo'].clear()
        self.logger_losses['stereo'] = 1
        run = self.analyze()['runs'][0]
        self.assertIn('event_history_unconfirmed', run['live_endpoint']['reasons'])
        self.logger_losses['stereo'] = 0
        self.accounting['stereo'] = 'false'
        run = self.analyze()['runs'][0]
        self.assertIn('input_accounting_unconfirmed', run['live_endpoint']['reasons'])

    def test_missing_process_and_backend_evidence_remain_unavailable(self):
        (self.paths['stereo'] / 'console.log').unlink()
        report = T02.analyze(self.paths['stereo'], self.paths['stereo_imu'])
        self.assertIn('process_completion_unconfirmed',
                      report['runs'][0]['live_endpoint']['reasons'])
        (self.paths['stereo'] / 'evaluation' / 'backend_summary.csv').unlink()
        report = T02.analyze(self.paths['stereo'], self.paths['stereo_imu'])
        self.assertIn('backend_totals_unconfirmed', report['runs'][0]['live_endpoint']['reasons'])

    def test_external_console_paths(self):
        console = self.root / 'external_console.log'
        (self.paths['stereo'] / 'console.log').rename(console)
        report = T02.analyze(self.paths['stereo'], self.paths['stereo_imu'],
                             stereo_console_log=console)
        self.assertTrue(report['comparison_available'])
        self.assertEqual(report['runs'][0]['console_log'], str(console))

    def test_loop_gba_annotations_are_not_optimized_pose_claims(self):
        self.extra_log['stereo'] = ('EVENT=LOOP_ACCEPT KF=10\nEVENT=GBA_START LOOP_KF=10\n'
                                    'EVENT=GBA_APPLY_BEGIN LOOP_KF=10\n'
                                    'EVENT=GBA_APPLY_END LOOP_KF=10\n')
        run = self.analyze()['runs'][0]
        self.assertTrue(run['live_endpoint']['available'])
        self.assertEqual(run['loop_gba_annotations']['counts'],
                         {'LOOP_ACCEPT': 1, 'GBA_START': 1,
                          'GBA_APPLY_BEGIN': 1, 'GBA_APPLY_END': 1})
        self.assertFalse(run['coordinate_context']['coordinate_version_confirmed'])
        self.assertFalse(run['final_optimized_endpoint']['available'])

    def test_unknown_provenance_is_not_a_descriptive_metric_gate(self):
        for mode in self.paths:
            self.metadata[mode]['bag_id'] = 'unknown'
        report = self.analyze()
        self.assertIsNone(report['same_bag_confirmed'])
        self.assertTrue(report['comparison_available'])
        self.assertEqual(report['runs'][0]['provenance']['repository_commit'], 'unknown')
        self.metadata['stereo']['bag_id'] = 'scene_00'
        self.metadata['stereo_imu']['bag_id'] = 'other_bag'
        report = self.analyze()
        self.assertIs(report['same_bag_confirmed'], False)
        self.assertFalse(report['comparison_available'])
        self.assertTrue(all(run['live_endpoint']['available'] for run in report['runs']))

    def test_disabled_pose_is_not_an_identity_endpoint(self):
        self.metadata['stereo']['pose_enabled'] = False
        for row in self.rows['stereo']:
            row['pose_enabled'] = row['pose_valid'] = 0
        run = self.analyze()['runs'][0]
        self.assertIn('pose_collection_disabled', run['live_endpoint']['reasons'])
        self.assertIsNone(run['live_endpoint']['translation_m'])

    def test_invalid_window_options_and_old_schema(self):
        for options in ({'window_sec': 0}, {'window_sec': math.nan}, {'window_sec': 7},
                        {'min_samples': 2}, {'start_window': (-1, 3)},
                        {'start_window': (0, 6), 'end_window': (5, 8)}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.analyze(**options)
        self.metadata['stereo']['schema_version'] = 2
        with self.assertRaisesRegex(ValueError, 'schema_version=3'):
            self.analyze()

    def test_missing_pose_columns_are_not_treated_as_zero(self):
        path = self.paths['stereo'] / 'evaluation' / 'frames.csv'
        with path.open('w', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=HEADER[:-1], extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.rows['stereo'])
        with self.assertRaisesRegex(ValueError, 'columns are missing'):
            T02.analyze(self.paths['stereo'], self.paths['stereo_imu'])

    def test_cli_writes_only_report_and_returns_partial_status(self):
        before = {path: hashlib.sha256(path.read_bytes()).digest()
                  for session in self.paths.values() for path in session.rglob('*')
                  if path.is_file()}
        output = self.root / 'results' / 't02_report.json'
        arguments = [str(self.paths['stereo']), str(self.paths['stereo_imu']),
                     '--output', str(output), '--require-complete']
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(T02.main(arguments), 0)
        self.assertTrue(json.loads(output.read_text())['comparison_available'])
        self.assertEqual(before, {path: hashlib.sha256(path.read_bytes()).digest()
                                  for path in before})
        for row in self.rows['stereo'][:4]:
            row['pose_valid'] = 0
        self.write_sessions()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(T02.main(arguments), 2)
        report = json.loads(output.read_text())
        self.assertIsNone(report['runs'][0]['live_endpoint']['translation_m'])
        self.assertNotIn('NaN', output.read_text())

    def test_cli_refuses_to_replace_raw_input(self):
        path = self.paths['stereo'] / 'evaluation' / 'frames.csv'
        original = path.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as result:
            T02.main([str(self.paths['stereo']), str(self.paths['stereo_imu']),
                      '--output', str(path)])
        self.assertEqual(result.exception.code, 1)
        self.assertEqual(path.read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
