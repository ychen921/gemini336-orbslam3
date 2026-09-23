"""Cross-check 6D-1 logs against the exact clip manifest, without replaying."""
import collections
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
checks = {}
for side in ['left', 'right']:
    observed = [int(t) for t in re.findall(r'Stereo receive: side=' + side + r' timestamp_ns=(\d+)', log)]
    expected = stamps['/camera/' + side + '_ir/image_raw']
    checks[side + '_raw_exact'] = observed == expected
sync = [(int(a), int(b)) for a, b in re.findall(r'Stereo sync: left_ns=(\d+) right_ns=(\d+)', log)]
left = stamps['/camera/left_ir/image_raw']
right = stamps['/camera/right_ir/image_raw']
checks['sync_exact_prefix'] = sync == list(zip(left[:-1], right[:-1]))
checks['sync_processed_count'] = len(sync) == len(frames)
errors = [abs(int(Decimal(frame['timestamp']) * 1000000000) - left[i])
          for i, frame in enumerate(frames)]
checks['processed_timestamp_match'] = len(frames) == len(left)-1 and max(errors, default=0) <= 256
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
summary = {'checks': checks, 'passed': result['passed'] and all(checks.values()),
           'raw_left': len(left), 'raw_right': len(right), 'sync': len(sync),
           'maximum_timestamp_error_ns': max(errors, default=0),
           'post_viba2_states': dict(post_states), 'reset_lines': reset_lines,
           'insufficient_acceleration_messages': log.count('not enough acceleration'),
           'tracking_ms': {'mean': sum(f['track_ms'] for f in frames)/len(frames),
                           'max': max(f['track_ms'] for f in frames)},
           'coordination_reports': [line for line in lines if 'coordination:' in line],
           'limitations': ['No direct per-batch IMU trace or initialization-state API.',
                           'VIBA evidence is indirect and does not validate calibration or optimization accuracy.',
                           'One final synchronized pair remains omitted; shutdown does not prove all upstream threads joined.',
                           'Debug logging affects timing; no hardware, full-bag, or Stereo regression test.']}
(out / 'analysis.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary['passed'] else 1)
