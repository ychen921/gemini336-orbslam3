import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[1] / 'tools/stereo_imu_replay_validation/validation_common.py'
spec = importlib.util.spec_from_file_location('validation_common', path)
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

ACCOUNT = 'STOP_ACCOUNTING enqueued=2 queued=1 in_flight=0 processed=1 startup_discarded=0 overload_discarded=0 outstanding=1 peak=2 accounting=Valid identities_valid=true\n'
WORK = 'STOP_WORK enqueue_sequence=2 timestamp_ns=2000000000 owner=queued stage=Queued batch_use=NotRequired batch_samples=0 related_sequence=0 interruption=none timestamp_sec=2 received_steady_ns=1 wait_ms=0 reason=none location=none failure_imu_status=none batch_interval_left_sec=none batch_interval_right_sec=none exception=""\n'
PROCESS = 'STOP_PROCESS schema_version=1 first_stop=InputIdle first_failure=none exception_present=false cleanup_failed=false backend_starts=1 logging_finish_status=ok logging_dropped_messages=0 exit_code=0\n'

class Tests(unittest.TestCase):
    def test_complete_and_contradiction(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'slam.log').write_text(ACCOUNT + WORK)
            (root/'console.log').write_text(PROCESS)
            (root/'node_exit_code.txt').write_text('0')
            result = v.analyze_folder(root)
            self.assertEqual(result['checks']['accounting']['status'], 'pass')
            self.assertEqual(result['checks']['work_identity']['status'], 'pass')
            self.assertEqual(result['checks']['stop_outcome']['status'], 'pass')
            self.assertFalse(result['passed'])
            (root/'slam.log').write_text(ACCOUNT.replace('enqueued=2', 'enqueued=3') + WORK)
            self.assertEqual(v.analyze_folder(root)['checks']['accounting']['status'], 'fail')

    def test_missing_duplicate_legacy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for text in ('', ACCOUNT + ACCOUNT, ACCOUNT.replace('queued=1 ', '', 1)):
                (root/'slam.log').write_text(text)
                self.assertEqual(v.analyze_folder(root)['checks']['accounting']['status'], 'insufficient')
            (root/'slam.log').unlink()
            (root/'node.log').write_text('coordination: final=true enqueued=1 processed=1')
            self.assertEqual(v.analyze_folder(root)['format'], 'legacy')

    def test_imu_and_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'slam.log').write_text('Final IMU input: received=2 accepted=1 stopped=1 unavailable=0 invalid_values=0 invalid_timestamps=0 timestamp_precision_rejections=0 duplicates=0 backwards=0 overflow=1 buffered=1\n')
            (root/'trace.csv').write_text('event,steady_ns,sensor_ns,value1,value2\nimu_reject_stopped,1,123,0,0\n# dropped_events=0\n')
            result = v.analyze_folder(root)
            self.assertEqual(result['checks']['imu_classification']['status'], 'pass')
            self.assertEqual(result['checks']['trace_integrity']['status'], 'pass')
            (root/'trace.csv').write_text('event,steady_ns,sensor_ns,value1,value2\n')
            self.assertEqual(v.analyze_folder(root)['checks']['trace_integrity']['status'], 'insufficient')

if __name__ == '__main__':
    unittest.main()
