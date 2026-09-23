"""Cross-check 6D-1/6D-2 logs against source timestamps, without replaying."""
import collections
import bisect
from decimal import Decimal
import json
from pathlib import Path
import re
import sys

out = Path(sys.argv[1])
log = (out / 'node.log').read_text()
stamps = json.loads((out / 'clip_timestamps.json').read_text())
frames = json.loads((out / 'frames.json').read_text())
result = json.loads((out / 'result.json').read_text())
manifest = json.loads((out / 'clip_manifest.json').read_text())
checks = {}
raw_counts = {}
raw_gaps = {}
for side in ['left', 'right']:
    observed = [int(t) for t in re.findall(r'Stereo receive: side=' + side + r' timestamp_ns=(\d+)', log)]
    expected = stamps['/camera/' + side + '_ir/image_raw']
    checks[side + '_raw_exact'] = observed == expected
    raw_counts[side] = len(observed)
    received = set(observed)
    # Separate interior reception gaps from the unplayed/unreceived suffix when
    # the node stops early. Source indices are one-based for manual comparison.
    raw_gaps[side] = {
        'last_received_source_index': bisect.bisect_right(expected, observed[-1]) if observed else 0,
        'missing_before_last_received': [index+1 for index, timestamp in enumerate(expected)
                                         if observed and timestamp <= observed[-1] and timestamp not in received],
        'max_received_header_gap_ms': max((b-a for a,b in zip(observed, observed[1:])), default=0)/1e6}
sync = [(int(a), int(b)) for a, b in re.findall(r'Stereo sync: left_ns=(\d+) right_ns=(\d+)', log)]
left = stamps['/camera/left_ir/image_raw']
right = stamps['/camera/right_ir/image_raw']
expected_pairs = list(zip(left, right))
checks['sync_exact_prefix'] = sync in (expected_pairs, expected_pairs[:-1])
checks['sync_processed_count'] = len(sync) == len(frames)
errors = [abs(int(Decimal(frame['timestamp']) * 1000000000) - timestamp)
          for frame, timestamp in zip(frames, left)]
checks['processed_timestamp_match'] = (len(frames) in (len(left), len(left)-1)
                                        and max(errors, default=0) <= 256)
lines = log.splitlines()
viba2 = next((i for i, line in enumerate(lines) if 'end VIBA 2' in line), None)
after = lines[viba2+1:] if viba2 is not None else []
post_states = collections.Counter(match.group(1) for line in after
                                  if (match := re.search(r'Stereo frame: .*state=(\w+)', line)))
reset_lines = [line for line in lines if re.search(r'reset|reseting', line, re.IGNORECASE)]
checks['viba2_observed'] = viba2 is not None
checks['post_viba2_tracking'] = post_states.get('Ok', 0) > 0 and set(post_states) == {'Ok'}
checks['no_reset_observed'] = not reset_lines
checks['no_ros_errors'] = '[ERROR]' not in log
# Derive state durations from sensor timestamps, excluding startup wall time.
# Each frame's state covers the interval to its successor; the final state has
# no known next sample and therefore adds no duration beyond its timestamp.
segments = []
state_duration = collections.Counter()
for index, frame in enumerate(frames):
    timestamp = Decimal(frame['timestamp'])
    if not segments or segments[-1]['state'] != frame['state']:
        segments.append({'state': frame['state'], 'first_index': frame['index'],
                         'first_timestamp': frame['timestamp'], 'count': 0})
    segment = segments[-1]
    segment.update(last_index=frame['index'], last_timestamp=frame['timestamp'])
    segment['count'] += 1
    if index + 1 < len(frames):
        state_duration[frame['state']] += float(Decimal(frames[index+1]['timestamp']) - timestamp)

coordination = []
for line in lines:
    if 'coordination:' not in line:
        continue
    values = dict(re.findall(r'(\w+)=([\w.]+)', line.split('coordination:', 1)[1]))
    coordination.append({key: value if key == 'final' else float(value)
                         for key, value in values.items()})
track_times = sorted(frame['track_ms'] for frame in frames)
tracking_ms = ({'mean': sum(track_times)/len(track_times),
                'p95': track_times[max(0, (95*len(track_times)+99)//100-1)],
                'max': track_times[-1]} if track_times else {})
imu_times = stamps['/camera/gyro_accel/sample']
gap_error = re.search(r'IMU data gap detected: interval=\(([^,]+), ([^\]]+)\]', log)
source_gap_check = None
if gap_error:
    lower, upper = [int(Decimal(value)*1000000000) for value in gap_error.groups()]
    begin = max(0, bisect.bisect_right(imu_times, lower)-1)
    end = bisect.bisect_right(imu_times, upper)+1
    covered = imu_times[begin:end]
    source_gap_check = {'request_interval': list(gap_error.groups()),
                        'source_samples_including_brackets': len(covered),
                        'first_ns': covered[0], 'last_ns': covered[-1],
                        'max_gap_ms': max(b-a for a,b in zip(covered, covered[1:]))/1e6}
limitations = ['No direct per-batch IMU trace or initialization-state API.',
               'VIBA evidence is indirect and does not validate calibration or optimization accuracy.',
               'Shutdown does not prove all upstream threads joined.',
               'Debug logging affects timing; no hardware or Stereo regression test.']
if not manifest.get('full_bag', False):
    limitations.append('No full-bag test in this run.')
if len(sync) == len(left)-1:
    limitations.append('One final synchronized pair remains omitted.')
summary = {'checks': checks, 'passed': result['passed'] and all(checks.values()),
           'full_bag': manifest.get('full_bag', False),
           'raw_left': raw_counts['left'], 'raw_right': raw_counts['right'], 'sync': len(sync),
           'raw_reception_gaps': raw_gaps,
           'source_imu_max_gap_ms': max(b-a for a,b in zip(imu_times, imu_times[1:]))/1e6,
           'source_at_runtime_gap': source_gap_check,
           'slow_tracking_frames': [frame for frame in frames if frame['track_ms'] >= 100],
           'maximum_timestamp_error_ns': max(errors, default=0),
           'post_viba2_states': dict(post_states), 'reset_lines': reset_lines,
           'insufficient_acceleration_messages': log.count('not enough acceleration'),
           'tracking_ms': tracking_ms,
           'state_segments': segments,
           'state_duration_sec': dict(state_duration),
           'map_events': [line for line in lines if re.search(
               r'new map|new atlas|map.*id|active map|merge|loop detected', line, re.IGNORECASE)],
           'coordination_samples': coordination,
           'coordination_reports': [line for line in lines if 'coordination:' in line],
           'limitations': limitations}
(out / 'analysis.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary['passed'] else 1)
