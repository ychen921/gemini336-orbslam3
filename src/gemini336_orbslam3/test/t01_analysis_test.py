import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SPEC = importlib.util.spec_from_file_location(
    'analyze_t01', Path(__file__).resolve().parents[1] / 'tools' / 'analyze_t01.py')
T01 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(T01)
FRAME_HEADER = ('enqueue_sequence,processed_index,tracked_mappoint_inliers_valid,'
                'tracked_mappoint_inliers\n')


def write_metrics(path, values):
    with path.open('w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(('metric', 'value', 'scope'))
        writer.writerows((name, 'N/A' if value is None else value, 'fixture')
                         for name, value in values.items())


def event(kind, sequence, time):
    return (f'[atlas] [info] T01_EVENT type={kind} sequence={sequence} steady_ns={time} '
            'frame_valid=0 frame_id=0 sensor_timestamp_sec=0 map_valid=1 map_id=0 '
            'target_map_valid=0 target_map_id=0 result_map_valid=0 result_map_id=0 '
            'keyframes_valid=0 keyframe_id=0 target_keyframe_id=0\n')


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.session = Path(self.temporary.name)
        self.directory = self.session / 'evaluation'
        self.directory.mkdir()
        metadata = {name: 'known' for name in (
            'run_id', 'bag_id', 'repository_commit', 'core_commit',
            'settings_id', 'vocabulary_id')}
        metadata.update(schema_version=2, sensor_mode='stereo',
                        repository_dirty='false', core_dirty='false')
        (self.directory / 'metadata.json').write_text(json.dumps(metadata))
        self.summary = dict(processed=2, recorded=2, dropped=0, core_snapshot_frames=2,
                            tracked_mappoint_inliers_valid_samples=1,
                            tracked_mappoint_inliers_mean=80, tracking_lost_events=1,
                            relocalization_events=0, keyframes_inserted_last_frame=2)
        write_metrics(self.directory / 'summary.csv', self.summary)
        # Match the runtime's two-column export and textual accounting flag.
        (self.directory / 'inputs.csv').write_text(
            'metric,value\naccounting_valid,true\nevaluation_dropped,0\n')
        (self.directory / 'frames.csv').write_text(FRAME_HEADER + '1,1,0,N/A\n2,2,1,80\n')
        (self.directory / 'export_complete.txt').write_text(
            'schema_version=2\nrecorded=2\ndropped=0\n')
        self.backend = {'backend_summary_available': 1, 'keyframes_inserted_total': 3}
        occurred = ('map_baseline', 'tracking_lost', 'map_created')
        self.backend.update({'event_' + name: int(name in occurred) for name in T01.EVENT_TYPES})
        write_metrics(self.directory / 'backend_summary.csv', self.backend)
        # Arrival order and sequence differ from source-time order.
        self.log = self.session / 'slam.log'
        self.log.write_text(event('map_created', 2, 30) + event('map_baseline', 1, 10) +
                            event('tracking_lost', 3, 20))
        self.console = self.session / 'console.log'
        self.console.write_text(
            'STOP_PROCESS schema_version=1 cleanup_failed=false logging_finish_status=ok '
            'logging_dropped_messages=0 exit_code=0\n')

    def analyze(self):
        return T01.analyze(self.session, self.console)

    def test_metric_values_preserve_types(self):
        path = self.directory / 'typed_metrics.csv'
        for text, expected, expected_type in (
                ('true', True, bool), ('false', False, bool),
                ('N/A', None, type(None)), ('0', 0, int), ('1', 1, int),
                ('1.25', 1.25, float), ('1e-3', 0.001, float), ('1E3', 1000.0, float)):
            with self.subTest(value=text):
                path.write_text(f'metric,value\nsample,{text}\n')
                value = T01.metrics(path)['sample']
                # bool compares equal to 0/1, so equality alone misses coercion.
                self.assertIs(type(value), expected_type)
                self.assertEqual(value, expected)

    def test_invalid_metric_text_rejected(self):
        path = self.directory / 'invalid_metrics.csv'
        for text in ('unknown', 'true-ish', ''):
            with self.subTest(value=text):
                path.write_text(f'metric,value\nsample,{text}\n')
                with self.assertRaises(ValueError):
                    T01.metrics(path)

    def test_complete_and_source_order(self):
        report, events = self.analyze()
        self.assertTrue(report['collection_complete'])
        self.assertTrue(report['comparison_ready'])
        self.assertIs(report['inputs']['accounting_valid'], True)
        self.assertEqual([row['type'] for row in events],
                         ['map_baseline', 'tracking_lost', 'map_created'])
        self.assertEqual(events[0]['map_id'], 0)  # Valid zero map ID.
        self.assertIsNone(events[0]['frame_id'])
        self.assertEqual(report['keyframes_inserted_last_frame'], 2)
        self.assertEqual(report['keyframes_inserted_after_join'], 3)

    def test_missing_join_and_console_remain_unavailable(self):
        (self.directory / 'backend_summary.csv').unlink()
        self.console.unlink()
        report, _ = self.analyze()
        self.assertTrue(report['frame_records_complete'])
        self.assertFalse(report['collection_complete'])
        self.assertIsNone(report['final_event_counts']['tracking_lost'])
        self.assertIsNone(report['logger_dropped_messages'])

    def test_log_loss_and_missing_event(self):
        self.console.write_text(self.console.read_text().replace('messages=0', 'messages=2'))
        report, _ = self.analyze()
        self.assertTrue(report['event_counts_match'])
        self.assertFalse(report['event_details_complete'])
        self.log.write_text(event('map_baseline', 1, 10) + event('map_created', 2, 30))
        report, _ = self.analyze()
        self.assertFalse(report['event_counts_match'])
        self.assertEqual(report['final_event_counts']['tracking_lost'], 1)

    def test_duplicate_or_malformed_event(self):
        with self.log.open('a') as output:
            output.write(event('tracking_lost', 3, 20))
            output.write('T01_EVENT type=unknown\n')
        report, _ = self.analyze()
        self.assertEqual(report['event_parse_errors'], 2)
        self.assertFalse(report['collection_complete'])

    def test_final_total_cannot_be_a_smaller_frame_sample(self):
        self.backend['keyframes_inserted_total'] = 1
        write_metrics(self.directory / 'backend_summary.csv', self.backend)
        report, _ = self.analyze()
        self.assertFalse(report['final_counters_consistent'])
        self.assertFalse(report['collection_complete'])

    def test_overflow_and_valid_zero(self):
        self.summary.update(processed=3, recorded=1, dropped=2, core_snapshot_frames=3)
        write_metrics(self.directory / 'summary.csv', self.summary)
        (self.directory / 'frames.csv').write_text(FRAME_HEADER + '1,1,1,0\n')
        (self.directory / 'export_complete.txt').write_text(
            'schema_version=2\nrecorded=1\ndropped=2\n')
        report, _ = self.analyze()
        self.assertTrue(report['export_complete'])
        self.assertFalse(report['frame_records_complete'])
        self.assertEqual(report['retained_valid_inlier_rows'], 1)

    def test_old_schema_and_invalid_inlier_flag(self):
        (self.directory / 'frames.csv').write_text(FRAME_HEADER + '1,1,1,N/A\n')
        with self.assertRaises(ValueError):
            self.analyze()
        (self.directory / 'metadata.json').write_text('{"schema_version": 1}')
        with self.assertRaisesRegex(ValueError, 'schema_version=2'):
            self.analyze()


if __name__ == '__main__':
    unittest.main()
