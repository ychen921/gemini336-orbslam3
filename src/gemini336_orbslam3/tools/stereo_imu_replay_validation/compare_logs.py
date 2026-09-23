"""Compare existing 6D-2 logs; standard library only, no replay or source edits."""
import argparse
import bisect
import collections
import csv
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re


WALL = re.compile(r'^\[\w+\] \[([\d.]+)\]')
FRAME = re.compile(r'Stereo frame: index=(\d+) timestamp=([\d.]+) track_ms=([\d.]+) state=(\w+)')
RAW = re.compile(r'Stereo receive: side=(left|right) timestamp_ns=(\d+)')


def nanoseconds(value):
    return int(Decimal(value) * 1000000000)


def analyze(path, source):
    lines = path.read_text().splitlines()
    origin = source['/camera/left_ir/image_raw'][0]
    frames, raw, coordination, events = [], {'left': [], 'right': []}, [], []
    for number, line in enumerate(lines, 1):
        wall = WALL.search(line)
        wall_ns = nanoseconds(wall[1]) if wall else None
        match = FRAME.search(line)
        if match:
            frames.append(dict(line=number, index=int(match[1]), header_ns=nanoseconds(match[2]),
                               wall_ns=wall_ns, track_ms=float(match[3]), state=match[4]))
        match = RAW.search(line)
        if match:
            raw[match[1]].append(dict(line=number, header_ns=int(match[2]), wall_ns=wall_ns))
        if 'coordination:' in line:
            values = dict(re.findall(r'(\w+)=([\w.]+)', line.split('coordination:', 1)[1]))
            coordination.append(dict(line=number, wall_ns=wall_ns,
                                     **{k: v if k == 'final' else float(v) for k,v in values.items()}))
        if any(word in line for word in ['*Loop detected', 'BAD LOOP', 'Local Mapping STOP',
                                         'Local Mapping RELEASE', 'IMU data gap detected']):
            events.append(dict(line=number, message=line, wall_ns=wall_ns))
    if not frames or not raw['left']:
        raise ValueError(f'{path}: missing tracking or reception records')

    # ROS log wall timestamps are not steady-clock instrumentation. Align each
    # run with its first left callback; keep this axis distinct from bag time.
    first_wall = raw['left'][0]['wall_ns']
    first_header = raw['left'][0]['header_ns']
    for frame in frames:
        frame['bag_sec'] = (frame['header_ns']-origin)/1e9
        frame['callback_elapsed_sec'] = (frame['wall_ns']-first_wall)/1e9
        frame['relative_lag_sec'] = ((frame['wall_ns']-first_wall)
                                     -(frame['header_ns']-first_header))/1e9
    frame_lines = [frame['line'] for frame in frames]
    for event in coordination + events:
        position = bisect.bisect_right(frame_lines, event['line'])-1
        # Untimestamped backend messages get a line-order bracket, not a made-up
        # exact time: asynchronous output and buffering limit causal inference.
        before = frames[position] if position >= 0 else None
        after = frames[position+1] if position+1 < len(frames) else None
        event['bag_bracket_sec'] = [before['bag_sec'] if before else None,
                                    after['bag_sec'] if after else None]
        if event['wall_ns'] is not None:
            event['callback_elapsed_sec'] = (event['wall_ns']-first_wall)/1e9

    # Associate each missing source run with the observed callbacks bracketing
    # it. The unreceived suffix after shutdown is deliberately not called loss.
    gaps = {}
    for side, records in raw.items():
        expected = source[f'/camera/{side}_ir/image_raw']
        lookup = {timestamp: index for index,timestamp in enumerate(expected)}
        indices = [lookup[record['header_ns']] for record in records]
        if any(b <= a for a,b in zip(indices, indices[1:])):
            raise ValueError(f'{path}: nonmonotonic {side} reception')
        missing = []
        for previous, current, a, b in zip(records, records[1:], indices, indices[1:]):
            if b-a <= 1:
                continue
            wall_start, wall_end = previous['wall_ns'], current['wall_ns']
            overlap = []
            for frame in frames:
                # Approximate only: the log is emitted after timed tracking
                # and statistics, and timestamps use a different clock.
                finish = frame['wall_ns']
                start = finish-int(frame['track_ms']*1e6)
                if frame['track_ms'] >= 100 and start < wall_end and finish > wall_start:
                    overlap.append(frame['index'])
            missing.append(dict(first_source_index=a+2, last_source_index=b,
                                count=b-a-1, bag_start_sec=(expected[a+1]-origin)/1e9,
                                bag_end_sec=(expected[b-1]-origin)/1e9,
                                callback_gap_ms=(wall_end-wall_start)/1e6,
                                approximate_overlapping_slow_frames=overlap,
                                before_log_line=previous['line'], after_log_line=current['line']))
        initial_missing = indices[0]
        gaps[side] = dict(received=len(records), last_source_index=indices[-1]+1,
                          initial_missing=initial_missing,
                          missing_before_last=initial_missing+sum(item['count'] for item in missing),
                          missing_runs=missing)
    bins = collections.defaultdict(list)
    for frame in frames:
        bins[int(frame['bag_sec'])].append(frame['track_ms'])
    per_second = [dict(bag_second=second, frames=len(values), mean_ms=sum(values)/len(values),
                       max_ms=max(values)) for second,values in sorted(bins.items())]
    errors = []
    imu = source['/camera/gyro_accel/sample']
    for event in events:
        match = re.search(r'IMU data gap detected: interval=\(([^,]+), ([^\]]+)\]', event['message'])
        if not match:
            continue
        lower, upper = map(nanoseconds, match.groups())
        begin = max(0,bisect.bisect_right(imu,lower)-1)
        end = bisect.bisect_right(imu,upper)+1
        bracket = imu[begin:end]
        errors.append(dict(line=event['line'], bag_interval_sec=[(lower-origin)/1e9,(upper-origin)/1e9],
                           source_samples=len(bracket),
                           source_max_gap_ms=max(b-a for a,b in zip(bracket,bracket[1:]))/1e6))
    return dict(log=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                frames=frames, per_second=per_second, coordination=coordination, events=events,
                reception=gaps, imu_error_source_checks=errors,
                states=dict(collections.Counter(frame['state'] for frame in frames)),
                wall_clock_backwards=any(b['wall_ns'] < a['wall_ns'] for a,b in zip(frames,frames[1:])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--viewer', type=Path, required=True)
    parser.add_argument('--source-timestamps', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    # Refuse to replace previous evidence, including a partly completed run.
    args.output.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.source_timestamps.read_text())
    result = dict(source_sha256=hashlib.sha256(args.source_timestamps.read_bytes()).hexdigest(),
                  limitations=['ROS log times are not steady-clock callback start/end traces.',
                               'Loop timing uses adjacent tracking lines, not exact backend event timestamps.',
                               'Relative lag subtracts the first callback offset; it is not sensor-to-result latency.',
                               'Missing callbacks do not identify player, DDS, executor, or system causes.'],
                  runs={name: analyze(path,source) for name,path in
                        [('baseline',args.baseline),('viewer',args.viewer)]})
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,run in result['runs'].items():
        with (args.output/f'{name}_seconds.csv').open('w',newline='') as output:
            writer = csv.DictWriter(output,fieldnames=['bag_second','frames','mean_ms','max_ms'])
            writer.writeheader()
            writer.writerows(run['per_second'])
        print(name, 'frames=',len(run['frames']), 'missing=',
              {side:data['missing_before_last'] for side,data in run['reception'].items()},
              'source_gap_checks=',run['imu_error_source_checks'])
    print('Output:',args.output)


if __name__ == '__main__':
    main()
