"""Analyze opt-in steady-clock trace; missing source suffix is not counted as loss."""
import argparse
import bisect
import collections
import csv
import json
from pathlib import Path


def analyze(folder):
    from validation_common import analyze_folder
    result = analyze_folder(folder)
    # /proc CPU ticks cover all process threads: 100% means one CPU core.
    resources=json.loads((folder/'resources.json').read_text()) if (folder/'resources.json').exists() else []
    metrics=[]
    for previous,following in zip(resources,resources[1:]):
        dt=(following['steady_ns']-previous['steady_ns'])/1e9
        if dt <= 0:
            continue
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
    (folder/'resource_metrics.json').open('x').write(json.dumps(metrics,indent=2)+'\n')
    with (folder / 'trace_analysis.json').open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps(result, indent=2))
    return 1 if any(c['status'] == 'fail' for c in result['checks'].values()) else 2 if not result['passed'] else 0


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    raise SystemExit(analyze(parser.parse_args().folder))
