"""Read-only evidence parsing. No ROS imports or process execution."""
import argparse
import collections
import csv
import io
import json
import math
import re
from decimal import Decimal
from pathlib import Path

REJECTS = ('stopped', 'unavailable', 'invalid_values', 'invalid_timestamps',
           'timestamp_precision_rejections', 'duplicates', 'backwards')
REASONS = {'none', 'Signal', 'ContextShutdown', 'InputIdle', 'Capacity', 'Timeout',
           'SamplingError', 'BackendError', 'CallbackError', 'CancelError',
           'TraceWriteError', 'ShutdownError', 'Finalization'}


def check(status, reason, evidence=None):
    return dict(status=status, reason=reason, evidence=evidence or [])


def records(text, marker, path):
    result = []
    for number, line in enumerate(text.splitlines(), 1):
        if marker not in line:
            continue
        payload = line.split(marker, 1)[1]
        # Exception text has no complete escaping contract. Preserve it as opaque text.
        payload, separator, exception = payload.partition(' exception=')
        pairs = re.findall(r'(\w+)=([^\s]+)', payload)
        values = dict(pairs)
        if separator:
            values['exception'] = exception
        result.append(dict(values=values, duplicate_keys=len(pairs) != len(values) - bool(separator),
                           evidence=dict(file=str(path), line=number)))
    return result


def one(rows):
    if len(rows) != 1 or rows[0]['duplicate_keys']:
        raise ValueError('missing or ambiguous final record')
    return rows[0]['values']


def integers(values, keys):
    parsed = {}
    for key in keys:
        raw = values[key]
        if not re.fullmatch(r'\d+', str(raw)):
            raise ValueError('invalid nonnegative integer: ' + key)
        parsed[key] = int(raw)
    return parsed


def read_optional(path):
    return path.read_text(errors='replace') if path.is_file() else ''


def artifacts(folder):
    """Never follow a global latest symlink; resolve only this run's explicit manifest."""
    project = folder / 'slam.log'
    manifest_path = folder / 'artifacts.json'
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        candidate = (folder / manifest['slam_log']).resolve()
        if not candidate.is_relative_to(folder.resolve()):
            raise ValueError('slam_log escapes run directory')
        project = candidate
    console_path = folder / 'console.log'
    if not console_path.exists():
        console_path = folder / 'node.log'
    console = read_optional(console_path)
    project_text = read_optional(project)
    # Historical runs put project logs in node.log. Do not merge/double count them.
    if not project_text and 'STOP_ACCOUNTING ' in console:
        project, project_text = console_path, console
    return project, project_text, console_path, console


def trace_evidence(path):
    text = read_optional(path)
    lines = text.splitlines()
    footers = [line for line in lines if line.startswith('# dropped_events=')]
    try:
        if len(footers) != 1 or not lines or lines[-1] != footers[0]:
            raise ValueError('missing/ambiguous/truncated trace footer')
        dropped = int(footers[0].split('=')[1])
        if dropped < 0:
            raise ValueError('negative dropped count')
        rows = list(csv.DictReader(io.StringIO('\n'.join(line for line in lines if not line.startswith('#')))))
        for row in rows:
            row['steady_ns'] = int(row['steady_ns'])
            row['sensor_ns'] = int(row['sensor_ns'])
            row['value1'] = float(row['value1'])
            row['value2'] = float(row['value2'])
            if not math.isfinite(row['value1']) or not math.isfinite(row['value2']):
                raise ValueError('nonfinite trace value')
        ordered = all(a['steady_ns'] <= b['steady_ns'] for a, b in zip(rows, rows[1:]))
        state = 'fail' if not ordered else 'insufficient' if dropped else 'pass'
        return rows, check(state, 'steady order / bounded trace omissions', [str(path)]), dropped
    except (ValueError, KeyError, TypeError) as error:
        return [], check('insufficient', str(error), [str(path)]), None


def analyze_folder(folder):
    folder = Path(folder)
    project, text, console_path, console = artifacts(folder)
    result = dict(schema_version=1, format='stop' if 'STOP_ACCOUNTING ' in text else 'legacy',
                  checks={}, observations={}, limitations=[
                      'Trace begin/end bracket wrapper code, not exact backend execution.',
                      'No runtime success or DDS loss cause is inferred from accounting alone.'])
    checks = result['checks']
    account_rows = records(text, 'STOP_ACCOUNTING ', project)
    work_rows = records(text, 'STOP_WORK ', project)
    coverage_rows = records(text, 'STOP_IMU_COVERAGE ', project)
    imu_rows = records(text, 'Final IMU input: ', project)
    process_rows = records(console, 'STOP_PROCESS ', console_path)
    result['records'] = dict(accounting=account_rows, work=work_rows, coverage=coverage_rows,
                             imu=imu_rows, process=process_rows)
    account = None
    mode = None
    imu = None
    try:
        raw = one(account_rows)
        account = integers(raw, ('enqueued', 'queued', 'in_flight', 'processed', 'startup_discarded',
                                 'overload_discarded', 'outstanding', 'peak'))
        valid = (account['enqueued'] == sum(account[k] for k in
                 ('queued', 'in_flight', 'processed', 'startup_discarded', 'overload_discarded')) and
                 account['outstanding'] == account['queued'] + account['in_flight'] and
                 account['outstanding'] <= account['peak'] and account['in_flight'] <= 2)
        if raw.get('accounting') not in ('Valid', 'Invalid') or raw.get('identities_valid') not in ('true', 'false'):
            raise ValueError('unknown/missing accounting declaration')
        valid = valid and raw['accounting'] == 'Valid' and raw['identities_valid'] == 'true'
        checks['accounting'] = check('pass' if valid else 'fail', 'recomputed final accounting', account_rows)
        # Capacity and mode require actual parameters, not implicit defaults.
        parameters = read_optional(folder / 'parameters.yaml')
        mode = re.search(r'\bsensor_mode:\s*[\'"]?(stereo_imu|stereo)\b', parameters)
        capacity = re.search(r'\b' + (mode[1] if mode else '(?!)') + r'\.pending_frame_capacity:\s*(\d+)', parameters)
        if not mode or not capacity:
            checks['mode_capacity'] = check('insufficient', 'missing actual mode/capacity parameters')
        else:
            valid = account['peak'] <= int(capacity[1]) and (account['overload_discarded'] == 0 if mode[1] == 'stereo_imu'
                    else account['startup_discarded'] == 0 and account['in_flight'] <= 1)
            checks['mode_capacity'] = check('pass' if valid else 'fail', 'mode-specific capacity/discard rules')
    except (KeyError, ValueError) as error:
        checks['accounting'] = check('insufficient', str(error), account_rows)
    try:
        if account is None:
            raise ValueError('no complete accounting')
        identities, owners = [], collections.Counter()
        for row in work_rows:
            w = row['values']
            if row['duplicate_keys']:
                raise ValueError('duplicate work fields')
            numbers = integers(w, ('enqueue_sequence', 'batch_samples', 'related_sequence'))
            int(w['timestamp_ns'])
            for key in ('timestamp_sec', 'wait_ms'):
                if not math.isfinite(float(w[key])):
                    raise ValueError('nonfinite work value')
            integers(w, ('received_steady_ns',))
            if w['reason'] not in {'none', 'StopObserved', 'UnexpectedException', 'ImuBackwards', 'StartupTimeout',
                                    'FrameTimeout', 'ImuQueryException', 'ImuQueryResult', 'StartupDiscardException',
                                    'BeforeBackendException', 'BackendException', 'AfterBackendException'}:
                raise ValueError('unknown work reason')
            if w['location'] not in {'none', 'Coordination', 'ImuQuery', 'StartupDiscard', 'BeforeBackend',
                                      'Backend', 'Completion', 'AfterBackend', 'Finalization'}:
                raise ValueError('unknown work location')
            if w['failure_imu_status'] not in {'none', 'Stopped', 'Ready', 'WaitingForData', 'MissingHistory',
                                               'BufferOverflow', 'DataGap', 'InvalidRequest'}:
                raise ValueError('unknown IMU status')
            for key in ('batch_interval_left_sec', 'batch_interval_right_sec'):
                if w[key] != 'none' and not math.isfinite(float(w[key])):
                    raise ValueError('nonfinite batch interval')
            if 'exception' not in w:
                raise ValueError('missing exception field')
            if w['owner'] not in ('queued', 'primary', 'startup_next') or w['stage'] not in ('Queued', 'Reserved', 'Ready', 'Executing'):
                raise ValueError('unknown owner/stage')
            if w['batch_use'] not in ('NotRequired', 'NotAcquired', 'ConsumedUnused', 'DeliveredToBackend') or w['interruption'] not in ('none', 'Stopped', 'Failed'):
                raise ValueError('unknown batch/interruption')
            if (w['owner'] == 'queued') != (w['stage'] == 'Queued'):
                raise AssertionError('owner/stage conflict')
            if w['batch_use'] in ('ConsumedUnused', 'DeliveredToBackend'):
                if numbers['batch_samples'] == 0 or not float(w['batch_interval_left_sec']) < float(w['batch_interval_right_sec']):
                    raise AssertionError('invalid consumed batch')
            if w['batch_use'] == 'DeliveredToBackend' and w['stage'] != 'Executing':
                raise AssertionError('delivered batch was not executing')
            identities.append(numbers['enqueue_sequence'])
            owners[w['owner']] += 1
        valid = (len(identities) == account['outstanding'] and len(set(identities)) == len(identities) and
                 all(1 <= value <= account['enqueued'] for value in identities) and
                 owners['queued'] == account['queued'] and owners['primary'] + owners['startup_next'] == account['in_flight'] and
                 owners['primary'] <= 1 and owners['startup_next'] <= 1)
        checks['work_identity'] = check('pass' if valid else 'fail', 'unfinished identity/owner accounting', work_rows)
    except AssertionError as error:
        checks['work_identity'] = check('fail', str(error), work_rows)
    except (KeyError, ValueError) as error:
        checks['work_identity'] = check('insufficient', str(error), work_rows)
    try:
        coverage = one(coverage_rows)
        if coverage['source'] == 'saved_batch':
            matches = [row['values'] for row in work_rows if row['values'].get('enqueue_sequence') == coverage['enqueue_sequence']]
            w = matches[0] if len(matches) == 1 else {}
            valid = (w.get('batch_use') in ('ConsumedUnused', 'DeliveredToBackend') and
                     coverage['status'] == w['batch_use'] and coverage['batch_samples'] == w['batch_samples'] and
                     coverage['interval_left_sec'] == w['batch_interval_left_sec'] and
                     coverage['interval_right_sec'] == w['batch_interval_right_sec'])
            checks['coverage'] = check('pass' if valid else 'fail', 'saved batch matches unfinished work', coverage_rows)
        elif coverage['status'] == 'NoOutstanding' and account is not None and account['outstanding'] == 0:
            checks['coverage'] = check('not_applicable', 'no unfinished work', coverage_rows)
        elif coverage['status'] == 'NotRequired' and mode and mode[1] == 'stereo':
            checks['coverage'] = check('not_applicable', 'Stereo coverage', coverage_rows)
        else:
            checks['coverage'] = check('insufficient', 'inspection/empty coverage retained without independent batch evidence', coverage_rows)
    except (KeyError, ValueError) as error:
        checks['coverage'] = check('insufficient', str(error), coverage_rows)
    try:
        imu = integers(one(imu_rows), ('received', 'accepted', 'overflow', 'buffered') + REJECTS)
        valid = imu['received'] == imu['accepted'] + sum(imu[k] for k in REJECTS)
        checks['imu_classification'] = check('pass' if valid else 'fail', 'overflow excluded from reject sum', imu_rows)
    except (KeyError, ValueError) as error:
        checks['imu_classification'] = check('not_applicable' if mode and mode[1] == 'stereo' and not imu_rows else 'insufficient', str(error), imu_rows)
    rows, checks['trace_integrity'], dropped = trace_evidence(folder / 'trace.csv')
    result['observations']['trace_counts'] = dict(collections.Counter(r['event'] for r in rows))
    result['observations']['dropped_events'] = dropped
    if imu is not None and checks['trace_integrity']['status'] == 'pass':
        counts = collections.Counter(r['event'] for r in rows)
        valid = counts['imu_received'] == imu['received'] and counts['imu_accepted'] == imu['accepted']
        valid = valid and all(counts['imu_reject_' + key] == imu[key] for key in REJECTS)
        checks['imu_trace_counts'] = check('pass' if valid else 'fail', 'trace versus final IMU counters')
    calls, current = [], None
    for row in rows:
        if row['event'] == 'track_begin':
            if current is not None:
                checks['trace_pairing'] = check('insufficient', 'unclosed wrapper interval')
            current = row
        elif row['event'] == 'track_end':
            if current and current['value1'] == row['value1']:
                calls.append(dict(begin_ns=current['steady_ns'], end_ns=row['steady_ns'], sensor_sec=row['value1']))
                current = None
            else:
                checks['trace_pairing'] = check('insufficient', 'unmatched wrapper end')
    result['observations']['wrapper_intervals'] = calls
    result['observations']['unfinished_wrapper_interval'] = current
    checks.setdefault('trace_pairing', check('insufficient' if current else checks['trace_integrity']['status'],
                                            'wrapper pairing is not processed accounting'))
    try:
        process = one(process_rows)
        if process['schema_version'] != '1' or process['first_stop'] not in REASONS or process['first_failure'] not in REASONS:
            raise ValueError('unknown process schema/reason')
        for field in ('exception_present', 'cleanup_failed'):
            if process[field] not in ('true', 'false'):
                raise ValueError('invalid process boolean')
        status = process['logging_finish_status']
        if status not in ('ok', 'failed', 'unavailable'):
            raise ValueError('unknown logging status')
        external = read_optional(folder / 'node_exit_code.txt').strip()
        if not external and (folder / 'result.json').exists():
            external = str(json.loads((folder / 'result.json').read_text()).get('node_exit_code', ''))
        if not re.fullmatch(r'-?\d+', external):
            raise ValueError('missing external node exit code')
        numbers = integers(process, ('backend_starts', 'exit_code', 'logging_dropped_messages'))
        failure = process['first_failure'] != 'none' or process['cleanup_failed'] == 'true' or status == 'failed'
        valid = int(external) == numbers['exit_code'] and numbers['exit_code'] == (1 if failure else 0)
        if account is not None:
            valid = valid and account['processed'] <= numbers['backend_starts'] <= account['processed'] + 1
        checks['stop_outcome'] = check('pass' if valid else 'fail', 'control summary versus actual exit; not replay success', process_rows)
        checks['logging_integrity'] = check('fail' if status == 'failed' else 'insufficient' if status != 'ok' or numbers['logging_dropped_messages'] else 'pass',
                                            'logging finish and omissions', process_rows)
    except (KeyError, ValueError) as error:
        checks['stop_outcome'] = check('insufficient', str(error), process_rows)
    discard_rows = records(text, 'Stereo overload discard: ', project)
    if account is not None:
        try:
            sequences = []
            for row in discard_rows:
                d = integers(row['values'], ('enqueue_sequence', 'timestamp_ns', 'incoming_enqueue_sequence',
                                              'incoming_timestamp_ns', 'overload_discarded'))
                if row['duplicate_keys'] or d['enqueue_sequence'] >= d['incoming_enqueue_sequence']:
                    raise ValueError('ambiguous discard identity')
                sequences.append(d['enqueue_sequence'])
            unfinished = {int(w['values']['enqueue_sequence']) for w in work_rows}
            valid = len(sequences) == account['overload_discarded'] and len(set(sequences)) == len(sequences) and not unfinished.intersection(sequences)
            checks['overload_identity'] = check('pass' if valid else 'insufficient',
                                                'discard count/identity versus unfinished work', discard_rows)
        except (KeyError, ValueError) as error:
            checks['overload_identity'] = check('insufficient', str(error), discard_rows)
    frame_rows = records(text, 'Stereo frame: ', project)
    result['observations']['pipeline_counts'] = dict(
        sync=len(re.findall(r'Stereo sync: left_ns=', console)),
        debug_completed=len(frame_rows), final_accounting=account)
    # Debug records may be disabled or lost after completion. Never infer missing
    # processed work merely from a smaller debug-record count.
    checks['completed_identity'] = check('insufficient', 'exact completed identities require unique source mapping')
    if account is not None and account['processed'] == 0 and not frame_rows:
        checks['completed_identity'] = check('not_applicable', 'no completed frames')
    # Source comparison is occurrence-aware; optional files never prevent other checks.
    source_path = folder / 'clip_timestamps.json'
    if source_path.is_file() and checks['trace_integrity']['status'] == 'pass':
        source = json.loads(source_path.read_text())
        reception = {}
        if account is not None and account['processed'] and len(frame_rows) == account['processed']:
            try:
                expected = source.get('/camera/left_ir/image_raw', [])
                mapped, indices = [], []
                for row in frame_rows:
                    values = row['values']
                    ns = Decimal(values['timestamp']) * 1000000000
                    # Historical frame seconds are printed to 9 decimals. Account
                    # for double precision at this epoch, but require unique mapping.
                    tolerance = max(1, math.ulp(float(values['timestamp'])) * 1e9 + 1)
                    candidates = [i for i, stamp in enumerate(expected) if abs(Decimal(stamp) - ns) <= Decimal(str(tolerance))]
                    if len(candidates) != 1:
                        raise ValueError('completed timestamp mapping is missing/ambiguous')
                    mapped.append(candidates[0])
                    indices.append(int(values['index']))
                valid = indices == list(range(1, account['processed'] + 1)) and all(a < b for a, b in zip(mapped, mapped[1:]))
                excluded = {int(row['values']['timestamp_ns']) for row in work_rows + discard_rows}
                valid = valid and not any(expected[i] in excluded for i in mapped)
                checks['completed_identity'] = check('pass' if valid else 'fail', 'unique ordered source mapping; excludes unfinished/discarded', frame_rows)
            except (KeyError, ValueError, ArithmeticError) as error:
                checks['completed_identity'] = check('insufficient', str(error), frame_rows)
        for event, topic in [('image_left', '/camera/left_ir/image_raw'), ('image_right', '/camera/right_ir/image_raw'),
                             ('imu_received', '/camera/gyro_accel/sample')]:
            observed = [r['sensor_ns'] for r in rows if r['event'] == event]
            expected = source.get(topic)
            if expected is None:
                continue
            used = collections.Counter(observed)
            unknown = used - collections.Counter(expected)
            end = max(observed) if observed else None
            missing = collections.Counter(t for t in expected if end is not None and t <= end) - used
            reception[event] = dict(received=len(observed), unknown_occurrences=sum(unknown.values()),
                                    missing_before_last=sum(missing.values()),
                                    complete=observed == expected,
                                    nondecreasing=all(a <= b for a, b in zip(observed, observed[1:])))
        result['observations']['source_reception'] = reception
        checks['source_coverage'] = check('fail' if any(v['unknown_occurrences'] or v['missing_before_last'] or not v['nondecreasing'] for v in reception.values())
                                          else 'pass' if len(reception) == 3 and all(v['complete'] for v in reception.values())
                                          else 'insufficient', 'occurrence/order comparison; unreceived suffix is not proven played')
    else:
        checks['source_coverage'] = check('insufficient', 'requires complete trace and source timestamps')
    result['passed'] = all(c['status'] in ('pass', 'not_applicable') for c in checks.values())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze_folder(args.folder)
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    return 1 if any(c['status'] == 'fail' for c in result['checks'].values()) else 2 if not result['passed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
