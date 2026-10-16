#!/usr/bin/env python3
"""Read a completed/partial T01 session; never launch ROS or replay a bag."""

import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path


EVENT_TYPES = (
    'map_baseline', 'tracking_lost', 'relocalized', 'map_created',
    'map_switched', 'active_map_reset', 'atlas_reset', 'merge_applied',
)
EVENT_COLUMNS = (
    'type', 'sequence', 'steady_ns', 'frame_id', 'sensor_timestamp_sec',
    'map_id', 'target_map_id', 'result_map_id', 'keyframe_id', 'target_keyframe_id',
)
TOKENS = re.compile(r'([a-z_]+)=([^\s]+)')


def metrics(path):
    """Missing post-join files are unavailable, never zero-valued totals."""
    if not path.exists():
        return {}
    with path.open(newline='') as source:
        rows = list(csv.DictReader(source))
    result = {}
    for row in rows:
        name = row['metric']
        if name in result:
            raise ValueError(f'duplicate metric {name} in {path}')
        text = row['value']
        # inputs.csv exports accounting_valid as lowercase text alongside numbers.
        if text in ('true', 'false'):
            result[name] = text == 'true'
        elif text == 'N/A':
            result[name] = None
        elif any(c in text.lower() for c in '.e'):
            result[name] = float(text)
        else:
            result[name] = int(text)
    return result


def read_events(path):
    events, errors, seen = [], 0, set()
    with path.open() as source:
        for line in source:
            if 'T01_EVENT ' not in line:
                continue
            try:
                data = dict(TOKENS.findall(line.split('T01_EVENT ', 1)[1]))
                if data['type'] not in EVENT_TYPES:
                    raise ValueError('unknown event')
                row = {'type': data['type']}
                row['sequence'] = int(data['sequence'])
                row['steady_ns'] = int(data['steady_ns'])
                if row['sequence'] <= 0 or row['steady_ns'] <= 0 or row['sequence'] in seen:
                    raise ValueError('invalid/duplicate event identity')
                for flag in ('frame_valid', 'map_valid', 'target_map_valid',
                             'result_map_valid', 'keyframes_valid'):
                    if data[flag] not in ('0', '1'):
                        raise ValueError('invalid validity flag')
                for name, flag in (
                    ('frame_id', 'frame_valid'), ('map_id', 'map_valid'),
                    ('target_map_id', 'target_map_valid'), ('result_map_id', 'result_map_valid'),
                    ('keyframe_id', 'keyframes_valid'), ('target_keyframe_id', 'keyframes_valid'),
                ):
                    row[name] = int(data[name]) if data[flag] == '1' else None
                row['sensor_timestamp_sec'] = (
                    float(data['sensor_timestamp_sec']) if data['frame_valid'] == '1' else None)
                object_fields = ('frame_id', 'map_id', 'target_map_id', 'result_map_id',
                                 'keyframe_id', 'target_keyframe_id')
                if any(row[name] is not None and row[name] < 0 for name in object_fields):
                    raise ValueError('negative object identity')
                if (row['sensor_timestamp_sec'] is not None and
                        not math.isfinite(row['sensor_timestamp_sec'])):
                    raise ValueError('nonfinite sensor time')
                seen.add(row['sequence'])
                events.append(row)
            except (KeyError, ValueError, TypeError):
                errors += 1
    # Source times can precede enqueue/log delivery; sequence breaks timestamp ties.
    events.sort(key=lambda row: (row['steady_ns'], row['sequence']))
    return events, errors


def read_stop_process(path):
    result = None
    if path is not None and path.exists():
        with path.open() as source:
            for line in source:
                if 'STOP_PROCESS ' in line:
                    result = dict(TOKENS.findall(line.split('STOP_PROCESS ', 1)[1]))
    return result


def analyze(session, console_log=None):
    directory = session / 'evaluation'
    with (directory / 'metadata.json').open() as source:
        metadata = json.load(source)
    if metadata.get('schema_version') != 2:
        raise ValueError('T01 numeric/event analysis requires evaluation schema_version=2')
    summary = metrics(directory / 'summary.csv')
    backend = metrics(directory / 'backend_summary.csv')
    inputs = metrics(directory / 'inputs.csv')
    marker_path = directory / 'export_complete.txt'
    marker = {}
    if marker_path.exists():
        marker = dict(line.split('=', 1) for line in marker_path.read_text().splitlines()
                      if '=' in line)

    frame_rows, inlier_rows, previous = 0, 0, None
    identities_ordered = True
    with (directory / 'frames.csv').open(newline='') as source:
        for row in csv.DictReader(source):
            sequence, processed = int(row['enqueue_sequence']), int(row['processed_index'])
            identities_ordered &= (previous is None or
                                   (sequence > previous[0] and processed > previous[1]))
            previous = sequence, processed
            frame_rows += 1
            if row['tracked_mappoint_inliers_valid'] == '1':
                # An actual zero is valid; N/A with a true validity flag is malformed.
                int(row['tracked_mappoint_inliers'])
                inlier_rows += 1

    events, parse_errors = read_events(session / 'slam.log')
    stop = read_stop_process(console_log)
    if stop is None:
        stop = read_stop_process(session / 'slam.log')
    process_ok = stop is not None and all(stop.get(name) == expected for name, expected in (
        ('exit_code', '0'), ('cleanup_failed', 'false'), ('logging_finish_status', 'ok'),
    ))
    logger_loss = None if stop is None else stop.get('logging_dropped_messages')
    observed = Counter(event['type'] for event in events)
    final_counts = {name: backend.get('event_' + name) for name in EVENT_TYPES}
    final_available = backend.get('backend_summary_available') == 1
    counts_match = (all(final_counts[name] is not None and
                        final_counts[name] == observed[name] for name in EVENT_TYPES)
                    if final_available else None)
    expected_events = sum(final_counts.values()) if counts_match else None
    sequence_complete = (expected_events is not None and
                         sorted(event['sequence'] for event in events) ==
                         list(range(1, expected_events + 1)))
    event_details_complete = bool(final_available and counts_match and sequence_complete and
                                  parse_errors == 0 and logger_loss == '0')
    final_consistent = None
    if final_available:
        final_consistent = all(final is not None and (sample is None or final >= sample)
                               for final, sample in (
                                   (backend.get('keyframes_inserted_total'),
                                    summary.get('keyframes_inserted_last_frame')),
                                   (final_counts['tracking_lost'],
                                    summary.get('tracking_lost_events')),
                                   (final_counts['relocalized'],
                                    summary.get('relocalization_events')),
                               ))
    processed, recorded, dropped = (summary.get(name)
                                    for name in ('processed', 'recorded', 'dropped'))
    export_complete = bool(marker.get('schema_version') == '2' and
                           marker.get('recorded') == str(recorded) and
                           marker.get('dropped') == str(dropped) and
                           recorded == frame_rows and processed is not None and
                           recorded is not None and dropped is not None and
                           recorded + dropped == processed and (directory / 'inputs.csv').exists())
    frame_complete = bool(export_complete and dropped == 0 and identities_ordered and
                          summary.get('core_snapshot_frames') == processed and processed > 0 and
                          inlier_rows == summary.get('tracked_mappoint_inliers_valid_samples'))
    collection_complete = bool(frame_complete and event_details_complete and
                               process_ok and final_consistent)
    provenance = ('run_id', 'bag_id', 'repository_commit', 'core_commit',
                  'settings_id', 'vocabulary_id')
    provenance_complete = all(metadata.get(name) not in (None, '', 'unknown')
                              for name in provenance)
    tracking_available = (summary.get('tracked_mappoint_inliers_valid_samples') or 0) > 0
    warnings = []
    if not event_details_complete:
        warnings.append('Event detail completeness is unconfirmed; inspect logger losses, '
                        'final counters and console STOP_PROCESS.')
    if dropped:
        warnings.append('Frame CSV and percentiles describe the retained prefix; '
                        'cumulative summaries keep all successful calls.')
    if not tracking_available:
        warnings.append('No valid TrackLocalMap inlier samples; '
                        'normal tracking metrics remain unavailable.')
    if final_consistent is False:
        warnings.append('Final counters are missing or smaller than pre-shutdown samples.')
    if not provenance_complete:
        warnings.append('Provenance is incomplete; fill bag/source/settings/vocabulary '
                        'identifiers before comparing runs.')
    if metadata.get('repository_dirty') != 'false' or metadata.get('core_dirty') != 'false':
        warnings.append('Clean source provenance is unconfirmed; '
                        'commit IDs alone may not identify the tested source.')
    report = {
        'schema_version': 1, 'evaluation_schema_version': 2,
        'session': str(session), 'sensor_mode': metadata.get('sensor_mode'),
        'export_complete': export_complete, 'frame_records_complete': frame_complete,
        'event_details_complete': event_details_complete, 'event_counts_match': counts_match,
        'final_counters_consistent': final_consistent,
        'process_ok': bool(process_ok), 'logger_dropped_messages': logger_loss,
        'collection_complete': collection_complete,
        'comparison_ready': bool(collection_complete and tracking_available and
                                 provenance_complete and
                                 metadata.get('repository_dirty') == 'false' and
                                 metadata.get('core_dirty') == 'false'),
        'recorded_rows': frame_rows, 'retained_valid_inlier_rows': inlier_rows,
        'event_parse_errors': parse_errors,
        'observed_event_counts': {name: observed[name] for name in EVENT_TYPES},
        'final_event_counts': final_counts,
        'keyframes_inserted_last_frame': summary.get('keyframes_inserted_last_frame'),
        'keyframes_inserted_after_join': backend.get('keyframes_inserted_total'),
        'summary': summary, 'inputs': inputs, 'warnings': warnings,
        'interpretation': ('Inliers count feature associations, not unique MapPoints. '
                           'Map IDs may change at merge; no pose/map-version or '
                           'ground-truth accuracy claim.'),
    }
    return report, events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='log_slam/<session> directory')
    parser.add_argument('--console-log', type=Path,
                        help='Captured stderr/console containing final STOP_PROCESS')
    parser.add_argument('--output-dir', type=Path, help='Default: session/evaluation')
    parser.add_argument('--require-complete', action='store_true',
                        help='Exit 2 for incomplete collection')
    args = parser.parse_args()
    try:
        report, events = analyze(args.session, args.console_log)
        destination = args.output_dir or args.session / 'evaluation'
        destination.mkdir(parents=True, exist_ok=True)
        with (destination / 'events.csv').open('w', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=EVENT_COLUMNS)
            writer.writeheader()
            writer.writerows({name: 'N/A' if value is None else value
                              for name, value in event.items()}
                             for event in events)
        text = json.dumps(report, indent=2, allow_nan=False)
        (destination / 't01_report.json').write_text(text + '\n')
        print(text)
        return 2 if args.require_complete and not report['collection_complete'] else 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'T01 analysis failed: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
