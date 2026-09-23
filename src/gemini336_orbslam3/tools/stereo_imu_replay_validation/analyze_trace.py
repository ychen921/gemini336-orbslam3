"""Analyze opt-in steady-clock trace; missing source suffix is not counted as loss."""
import argparse
import bisect
import collections
import csv
import json
from pathlib import Path


def analyze(folder):
    text = (folder/'trace.csv').read_text().splitlines()
    dropped = int(text[-1].split('=')[1])
    rows = list(csv.DictReader(line for line in text if not line.startswith('#')))
    for row in rows:
        row['steady_ns'] = int(row['steady_ns'])
        row['sensor_ns'] = int(row['sensor_ns'])
        row['value1'] = float(row['value1'])
        row['value2'] = float(row['value2'])
    source = json.loads((folder/'clip_timestamps.json').read_text())['/camera/gyro_accel/sample']
    received = [r for r in rows if r['event']=='imu_received']
    accepted = [r for r in rows if r['event']=='imu_accepted']
    counts = collections.Counter(r['event'] for r in rows)
    # Pair calls in recorded execution order, including an unfinished call.
    calls, current = [], None
    for row in rows:
        if row['event']=='track_begin':
            if current is not None:
                raise ValueError('Overlapping tracking calls')
            current = row
        elif row['event']=='track_end':
            if current is None or current['value1'] != row['value1']:
                raise ValueError('Unmatched tracking return')
            calls.append({'begin_ns':current['steady_ns'], 'end_ns':row['steady_ns'],
                          'sensor_sec':row['value1'],
                          'duration_ms':(row['steady_ns']-current['steady_ns'])/1e6})
            current = None
    gaps = []
    for previous, following in zip(accepted, accepted[1:]):
        start, end = previous['sensor_ns'], following['sensor_ns']
        if end-start <= 20_000_000:
            continue
        lower, upper = previous['steady_ns'], following['steady_ns']
        overlap = [dict(call, overlap_ms=max(0,min(upper,call['end_ns'])-max(lower,call['begin_ns']))/1e6)
                   for call in calls if call['begin_ns']<upper and call['end_ns']>lower]
        gaps.append({'left_ns':start,'right_ns':end,'sensor_gap_ms':(end-start)/1e6,
                     'callback_gap_ms':(upper-lower)/1e6,
                     'missing_source_samples':max(0,bisect.bisect_left(source,end)-bisect.bisect_right(source,start)),
                     'tracking_overlap':overlap})
    seen = {row['sensor_ns'] for row in received}
    missing = [timestamp for timestamp in source if received and timestamp<=received[-1]['sensor_ns'] and timestamp not in seen]
    result = {'dropped_events':dropped,'counts':dict(counts),
              'trace_complete':dropped == 0,
              'steady_ordered':all(a['steady_ns']<=b['steady_ns'] for a,b in zip(rows,rows[1:])),
              'missing_imu_before_last_received':len(missing),
              'unknown_received_samples':len(seen-set(source)),
              'rejected_events':{k:v for k,v in counts.items() if k.startswith('imu_reject')},
              'accepted_gaps':gaps,
              'longest_callback_intervals': sorted([
                  {'left_ns':a['sensor_ns'], 'right_ns':b['sensor_ns'],
                   'sensor_gap_ms':(b['sensor_ns']-a['sensor_ns'])/1e6,
                   'callback_gap_ms':(b['steady_ns']-a['steady_ns'])/1e6}
                  for a,b in zip(received,received[1:])],
                  key=lambda item:item['callback_gap_ms'], reverse=True)[:10],
              'query_gap_events':[r for r in rows if r['event'] in ('imu_query_gap','imu_gap_endpoints')],
              'longest_calls':sorted(calls,key=lambda x:x['duration_ms'],reverse=True)[:10],
              'unfinished_call':current,
              'limitations':['DDS lost callback does not report every KEEP_LAST overwrite.',
                             'Overlapping tracking establishes executor unavailability, not the cause inside backend.',
                             'System sampling is one second and cannot exclude subsecond scheduling or memory stalls.']}
    # Validate the trace against independent final counters. Do not confuse
    # diagnostics integrity with a successful SLAM replay.
    replay = json.loads((folder/'result.json').read_text())
    result['integrity_checks'] = {
        'no_trace_omissions': dropped == 0,
        'steady_ordered': result['steady_ordered'],
        'received_matches_final': len(received) == replay.get('imu', {}).get('received'),
        'accepted_matches_final': len(accepted) == replay.get('imu', {}).get('accepted'),
        'tracking_returns_match_final': len(calls) == replay.get('coordination', {}).get('processed'),
        'received_classified': len(received) == len(accepted) + sum(result['rejected_events'].values()),
        'source_membership': result['unknown_received_samples'] == 0,
    }
    # /proc CPU ticks cover all process threads: 100% means one CPU core.
    resources=json.loads((folder/'resources.json').read_text())
    metrics=[]
    for previous,following in zip(resources,resources[1:]):
        dt=(following['steady_ns']-previous['steady_ns'])/1e9
        metric={'steady_ns':following['steady_ns'],'interval_sec':dt,'loadavg':following['loadavg'].strip()}
        for name in ['node','player']:
            if not previous[name] or not following[name]:
                continue
            a=previous[name]['stat'].rsplit(')',1)[1].split()
            b=following[name]['stat'].rsplit(')',1)[1].split()
            metric[name+'_cpu_percent']=100*((int(b[11])+int(b[12]))-(int(a[11])+int(a[12])))/following['clock_ticks']/dt
            metric[name+'_rss_kib']=next((int(line.split()[1]) for line in following[name]['status'].splitlines() if line.startswith('VmRSS:')),None)
        metrics.append(metric)
    result['resource_summary']={'samples':len(resources),
        'node_cpu_percent_max':max((x.get('node_cpu_percent',0) for x in metrics),default=0),
        'player_cpu_percent_max':max((x.get('player_cpu_percent',0) for x in metrics),default=0),
        'node_rss_kib_max':max((x.get('node_rss_kib') or 0 for x in metrics),default=0)}
    (folder/'resource_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    (folder/'trace_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    analyze(parser.parse_args().folder)
