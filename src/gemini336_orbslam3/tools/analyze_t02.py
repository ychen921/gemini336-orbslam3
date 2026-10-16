#!/usr/bin/env python3
"""Compare schema-3 live endpoint candidates from Stereo and Stereo-IMU sessions.

Usage: analyze_t02.py STEREO_SESSION IMU_SESSION --output results/t02_report.json
Defaults select the same first/last three sensor seconds in both runs. Override
--start-window/--end-window with seconds from the shared sensor origin; windows
never move to the first valid pose. --sensor-range-ns can supply original bag
sensor bounds when recorded frames do not cover the complete bag.

Only evaluation artifacts and logs are read; no ROS/bag/hardware is launched.
This is live endpoint consistency, including placement mismatch and jitter,
not absolute accuracy, pure drift or final optimized trajectory consistency.
"""

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

import analyze_t01 as t01


CRITICAL_EVENTS = frozenset((
    'map_created', 'map_switched', 'active_map_reset', 'atlas_reset', 'merge_applied',
))
DIAGNOSTIC_EVENTS = ('LOOP_ACCEPT', 'GBA_START', 'GBA_APPLY_BEGIN', 'GBA_APPLY_END')
DIAGNOSTIC_PATTERN = re.compile(r'\bEVENT=(' + '|'.join(DIAGNOSTIC_EVENTS) + r')\b')
POSE_COLUMNS = ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')
FRAME_COLUMNS = frozenset((
    'sensor_timestamp_ns', 'adapter_start_steady_ns', 'adapter_return_steady_ns',
    'raw_tracking_state', 'core_snapshot_valid', 'core_frame_id', 'tracking_map_valid',
    'tracking_map_id', 'pose_enabled', 'pose_valid', 'pose_capture_steady_ns',
    'map_change_index_valid', 'map_change_index', 'imu_initialized_valid', 'imu_initialized',
) + POSE_COLUMNS)


@dataclass(frozen=True)
class Frame:
    sensor_ns: int
    start_ns: int
    return_ns: int
    capture_ns: int | None
    map_id: int | None
    change_index: int | None
    imu_initialized: bool | None
    translation: tuple | None
    quaternion: tuple | None
    invalid_reason: str | None


def flag(row, name):
    if row[name] not in ('0', '1'):
        raise ValueError(f'{name} must be 0 or 1')
    return row[name] == '1'


def optional_integer(row, name, available=True):
    if not available:
        return None
    if row[name] == 'N/A':
        return None
    value = int(row[name])
    if value < 0:
        raise ValueError(f'{name} must be nonnegative')
    return value


def normalize_quaternion(quaternion):
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError('invalid_rotation')
    return tuple(value / norm for value in quaternion)


def rotation_distance_deg(first, second):
    # Vector/scalar parts of conjugate(first) * second. atan2 is stable for
    # small angles; abs(w) makes antipodal quaternions the same SO(3) rotation.
    x, y, z, w = first
    a, b, c, d = second
    vector = (w*a - x*d - y*c + z*b,
              w*b - y*d - z*a + x*c,
              w*c - z*d - x*b + y*a)
    scalar = w*d + x*a + y*b + z*c
    return math.degrees(2.0 * math.atan2(math.hypot(*vector), abs(scalar)))


def quaternion_mean(quaternions):
    """Sign-invariant SO(3) chordal mean: dominant eigenvector of mean(q q^T)."""
    if not quaternions:
        raise ValueError('no_rotation_samples')
    count = len(quaternions)
    matrix = [[math.fsum(q[i] * q[j] / count for q in quaternions)
               for j in range(4)] for i in range(4)]
    vectors = [[float(i == j) for j in range(4)] for i in range(4)]
    # Jacobi diagonalization of this fixed 4x4 symmetric matrix needs no NumPy.
    # Unlike a seeded power iteration, it also handles rotations near 180 deg
    # and can identify a non-unique mean instead of choosing an arbitrary axis.
    for _ in range(64):
        p, q = max(((i, j) for i in range(4) for j in range(i + 1, 4)),
                   key=lambda pair: abs(matrix[pair[0]][pair[1]]))
        off = matrix[p][q]
        if abs(off) < 1e-14:
            break
        tau = (matrix[q][q] - matrix[p][p]) / (2.0 * off)
        tangent = math.copysign(1.0, tau) / (abs(tau) + math.hypot(1.0, tau))
        cosine = 1.0 / math.hypot(1.0, tangent)
        sine = tangent * cosine
        matrix[p][p] -= tangent * off
        matrix[q][q] += tangent * off
        matrix[p][q] = matrix[q][p] = 0.0
        for index in range(4):
            if index not in (p, q):
                left, right = matrix[index][p], matrix[index][q]
                matrix[index][p] = matrix[p][index] = cosine * left - sine * right
                matrix[index][q] = matrix[q][index] = sine * left + cosine * right
            left, right = vectors[index][p], vectors[index][q]
            vectors[index][p] = cosine * left - sine * right
            vectors[index][q] = sine * left + cosine * right
    if max(abs(matrix[i][j]) for i in range(4) for j in range(i + 1, 4)) > 1e-12:
        raise ValueError('rotation_mean_did_not_converge')
    order = sorted(range(4), key=lambda index: matrix[index][index], reverse=True)
    if matrix[order[0]][order[0]] - matrix[order[1]][order[1]] <= 1e-10:
        raise ValueError('ambiguous_rotation_mean')
    result = normalize_quaternion(tuple(vectors[i][order[0]] for i in range(4)))
    return tuple(-value for value in result) if result[3] < 0 else result


def read_frames(path):
    frames = []
    with path.open(newline='') as source:
        reader = csv.DictReader(source)
        columns = reader.fieldnames or []
        if not FRAME_COLUMNS.issubset(columns) or len(columns) != len(set(columns)):
            raise ValueError('schema-3 frame columns are missing or duplicated')
        for number, row in enumerate(reader, 2):
            try:
                sensor_ns = int(row['sensor_timestamp_ns'])
                start_ns = int(row['adapter_start_steady_ns'])
                return_ns = int(row['adapter_return_steady_ns'])
                if start_ns <= 0 or return_ns < start_ns:
                    raise ValueError('invalid adapter call times')
                core_valid = flag(row, 'core_snapshot_valid')
                pose_enabled = flag(row, 'pose_enabled')
                core_id = optional_integer(row, 'core_frame_id', core_valid)
                map_id = optional_integer(
                    row, 'tracking_map_id', core_valid and flag(row, 'tracking_map_valid'))
                context_available = core_valid and pose_enabled
                change = optional_integer(
                    row, 'map_change_index',
                    context_available and flag(row, 'map_change_index_valid'))
                imu_available = context_available and flag(row, 'imu_initialized_valid')
                imu = flag(row, 'imu_initialized') if imu_available else None
                capture = optional_integer(row, 'pose_capture_steady_ns', context_available)
                reason = None
                translation = quaternion = None
                if not core_valid or core_id is None:
                    reason = 'core_snapshot_unavailable'
                elif not pose_enabled:
                    reason = 'pose_collection_disabled'
                elif not flag(row, 'pose_valid'):
                    reason = 'pose_flag_invalid'
                elif int(row['raw_tracking_state']) not in (2, 5):
                    reason = 'unsuccessful_tracking_state'
                elif map_id is None:
                    reason = 'map_context_unavailable'
                elif capture is None or not start_ns <= capture <= return_ns:
                    reason = 'capture_time_invalid'
                else:
                    try:
                        values = tuple(float(row[name]) for name in POSE_COLUMNS)
                        if not all(math.isfinite(value) for value in values):
                            raise ValueError('nonfinite_pose')
                        quaternion = normalize_quaternion(values[3:])
                        translation = values[:3]
                    except ValueError as error:
                        reason = ('invalid_rotation' if str(error) == 'invalid_rotation'
                                  else 'invalid_numeric_pose')
                frames.append(Frame(sensor_ns, start_ns, return_ns, capture, map_id,
                                    change, imu, translation, quaternion, reason))
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'{path}:{number}: {error}') from error
    return frames


def read_session(path, expected_mode, console_log):
    directory = path / 'evaluation'
    metadata = json.loads((directory / 'metadata.json').read_text())
    if metadata.get('schema_version') != 3:
        raise ValueError(f'{path}: T02 requires evaluation schema_version=3')
    if metadata.get('sensor_mode') != expected_mode:
        raise ValueError(f'{path}: expected sensor_mode={expected_mode}')
    if console_log is not None:
        console_log = Path(console_log)
    if console_log is None and (path / 'console.log').exists():
        console_log = path / 'console.log'
    integrity, events = t01.analyze(path, console_log)
    diagnostics = Counter()
    with (path / 'slam.log').open() as source:
        for line in source:
            diagnostics.update(DIAGNOSTIC_PATTERN.findall(line))
    return dict(path=path, metadata=metadata, integrity=integrity, events=events,
                frames=read_frames(directory / 'frames.csv'), console_log=console_log,
                diagnostics={name: diagnostics[name] for name in DIAGNOSTIC_EVENTS})


def select_windows(runs, window_sec, start_window, end_window, sensor_range_ns):
    if not math.isfinite(window_sec) or window_sec <= 0:
        raise ValueError('window-sec must be finite and positive')
    frames = [frame for run in runs for frame in run['frames']]
    if sensor_range_ns is None:
        if not frames:
            raise ValueError('no frames to define windows; supply sensor-range-ns')
        origin, finish = min(f.sensor_ns for f in frames), max(f.sensor_ns for f in frames)
        source = 'union_of_recorded_sensor_bounds'
    else:
        origin, finish = sensor_range_ns
        source = 'operator_supplied_sensor_bounds'
        if any(not origin <= frame.sensor_ns <= finish for frame in frames):
            raise ValueError('sensor-range-ns does not contain the recorded frames')
    if origin >= finish:
        raise ValueError('sensor range must have positive duration')
    duration_ns = finish - origin
    width_ns = round(window_sec * 1e9)
    windows = {}
    for name, requested, default in (
            ('start', start_window, (origin, origin + width_ns)),
            ('end', end_window, (finish - width_ns, finish))):
        if requested is None:
            bounds = default
        else:
            if not all(math.isfinite(value) for value in requested):
                raise ValueError('window offsets must be finite')
            if not 0 <= requested[0] < requested[1] <= duration_ns * 1e-9:
                raise ValueError('window offsets must lie within the sensor range')
            bounds = tuple(origin + round(value * 1e9) for value in requested)
        if not origin <= bounds[0] < bounds[1] <= finish:
            raise ValueError('windows must have positive duration within the sensor range')
        windows[name] = bounds
    if windows['start'][1] >= windows['end'][0]:
        raise ValueError('start/end windows must be disjoint with start preceding end')
    return windows, dict(source=source, first_sensor_ns=origin, last_sensor_ns=finish,
                         duration_sec=duration_ns * 1e-9)


def window_statistics(frames, bounds, mode, min_samples):
    selected = [frame for frame in frames if bounds[0] <= frame.sensor_ns <= bounds[1]]
    valid = [frame for frame in selected if frame.invalid_reason is None]
    reasons = []
    if len(valid) < min_samples:
        reasons.append('insufficient_valid_poses')
    maps = sorted({frame.map_id for frame in valid if frame.map_id is not None})
    if len(maps) > 1:
        reasons.append('mixed_maps_in_window')
    imu_states = sorted({frame.imu_initialized for frame in valid
                         if frame.imu_initialized is not None})
    if mode == 'stereo_imu':
        if any(frame.imu_initialized is None for frame in valid):
            reasons.append('imu_initialization_context_unavailable')
        if len(imu_states) > 1:
            reasons.append('imu_initialization_changed_in_window')
    result = dict(
        sensor_start_ns=bounds[0], sensor_end_ns=bounds[1], interval='inclusive',
        recorded_samples=len(selected), valid_samples=len(valid),
        invalid_pose_counts=dict(Counter(f.invalid_reason for f in selected if f.invalid_reason)),
        first_valid_sensor_ns=valid[0].sensor_ns if valid else None,
        last_valid_sensor_ns=valid[-1].sensor_ns if valid else None,
        valid_sensor_span_sec=(valid[-1].sensor_ns - valid[0].sensor_ns) * 1e-9 if valid else None,
        tracking_map_ids=maps, imu_initialized_states=imu_states,
        available=False, reasons=reasons, translation_mean_m=None, quaternion_mean_xyzw=None,
        translation_rms_m=None, translation_max_m=None,
        rotation_rms_deg=None, rotation_max_deg=None)
    if reasons:
        return result, valid
    count = len(valid)
    try:
        translation = tuple(math.fsum(f.translation[i] / count for f in valid) for i in range(3))
        rotation = quaternion_mean([frame.quaternion for frame in valid])
        translations = [math.dist(frame.translation, translation) for frame in valid]
        rotations = [rotation_distance_deg(rotation, frame.quaternion) for frame in valid]
        translation_rms = math.hypot(*translations) / math.sqrt(count)
        rotation_rms = math.hypot(*rotations) / math.sqrt(count)
        if not all(math.isfinite(value) for value in (*translation, translation_rms, rotation_rms,
                                                      max(translations), max(rotations))):
            raise ValueError('nonfinite_window_statistics')
        result.update(available=True, translation_mean_m=translation, quaternion_mean_xyzw=rotation,
                      translation_rms_m=translation_rms, translation_max_m=max(translations),
                      rotation_rms_deg=rotation_rms, rotation_max_deg=max(rotations))
    except (ValueError, OverflowError) as error:
        result['reasons'].append(str(error))
    return result, valid


def screen_context(run, first, last):
    reasons = []
    begin, end = first.capture_ns, last.capture_ns
    frames = [f for f in run['frames'] if first.sensor_ns <= f.sensor_ns <= last.sensor_ns]
    maps = sorted({f.map_id for f in frames if f.map_id is not None})
    changes = [f.change_index for f in frames if f.change_index is not None]
    if any(f.map_id is None for f in frames):
        reasons.append('map_context_unavailable')
    if len(maps) != 1:
        reasons.append('map_identity_changed')
    if any(f.change_index is None for f in frames):
        reasons.append('map_change_context_unavailable')
    if any(after < before for before, after in zip(changes, changes[1:])):
        reasons.append('map_change_counter_decreased')
    if any(f.capture_ns is None or not f.start_ns <= f.capture_ns <= f.return_ns for f in frames):
        reasons.append('capture_context_unavailable')
    imu_states = sorted({f.imu_initialized for f in frames if f.imu_initialized is not None})
    if run['metadata']['sensor_mode'] == 'stereo_imu':
        if any(f.imu_initialized is None for f in frames):
            reasons.append('imu_initialization_context_unavailable')
        if len(imu_states) > 1:
            reasons.append('imu_initialization_changed')
    events = [event for event in run['events'] if begin <= event['steady_ns'] <= end]
    critical = [event for event in events if event['type'] in CRITICAL_EVENTS]
    if critical:
        reasons.append('coordinate_change_event')
    if not run['integrity']['event_details_complete']:
        reasons.append('event_history_unconfirmed')
    return dict(screening_passed=not reasons, reasons=reasons,
                coordinate_version_confirmed=False, capture_start_steady_ns=begin,
                capture_end_steady_ns=end, tracking_map_ids=maps,
                map_change_index_first=changes[0] if changes else None,
                map_change_index_last=changes[-1] if changes else None,
                map_change_index_distinct=len(set(changes)), imu_initialized_states=imu_states,
                interval_event_counts=dict(Counter(event['type'] for event in events)),
                critical_events=critical)


def analyze_run(run, windows, min_samples):
    integrity, metadata = run['integrity'], run['metadata']
    frames = run['frames']
    reasons = []
    checks = {name: integrity[name] for name in (
        'export_complete', 'frame_records_complete', 'event_details_complete',
        'process_ok', 'final_counters_consistent', 'collection_complete')}
    checks['input_accounting_valid'] = integrity['inputs'].get('accounting_valid') is True
    for check, reason in (
            ('frame_records_complete', 'frame_records_incomplete'),
            ('event_details_complete', 'event_history_unconfirmed'),
            ('process_ok', 'process_completion_unconfirmed'),
            ('final_counters_consistent', 'backend_totals_unconfirmed'),
            ('input_accounting_valid', 'input_accounting_unconfirmed')):
        if checks[check] is not True:
            reasons.append(reason)
    if metadata.get('pose_enabled') is not True:
        reasons.append('pose_collection_disabled')
    if any(b.sensor_ns <= a.sensor_ns for a, b in zip(frames, frames[1:])):
        reasons.append('sensor_timestamps_not_ordered')
    captured = [f.capture_ns for f in frames if f.capture_ns is not None]
    if any(b <= a for a, b in zip(captured, captured[1:])):
        reasons.append('capture_timestamps_not_ordered')
    start, start_samples = window_statistics(
        frames, windows['start'], metadata['sensor_mode'], min_samples)
    end, end_samples = window_statistics(
        frames, windows['end'], metadata['sensor_mode'], min_samples)
    context = None
    if not start['available']:
        reasons.append('start_window_unavailable')
    if not end['available']:
        reasons.append('end_window_unavailable')
    if start['available'] and end['available']:
        context = screen_context(run, start_samples[0], end_samples[-1])
        reasons.extend(context['reasons'])
    endpoint = dict(available=False, translation_m=None, rotation_deg=None,
                    reasons=list(dict.fromkeys(reasons)))
    if not reasons:
        distance = math.dist(start['translation_mean_m'], end['translation_mean_m'])
        angle = rotation_distance_deg(start['quaternion_mean_xyzw'], end['quaternion_mean_xyzw'])
        if math.isfinite(distance) and math.isfinite(angle):
            endpoint.update(available=True, translation_m=distance, rotation_deg=angle)
        else:
            endpoint['reasons'].append('nonfinite_endpoint_difference')
    return dict(
        session=str(run['path']), sensor_mode=metadata['sensor_mode'],
        console_log=str(run['console_log']) if run['console_log'] else None,
        provenance={name: metadata.get(name) for name in (
            'run_id', 'bag_id', 'repository_commit', 'repository_dirty', 'core_commit',
            'core_dirty', 'settings_id', 'vocabulary_id')},
        collection_checks=checks, recorded_frames=len(frames),
        valid_pose_samples=sum(f.invalid_reason is None for f in frames),
        invalid_pose_counts=dict(Counter(f.invalid_reason for f in frames if f.invalid_reason)),
        windows=dict(start=start, end=end), coordinate_context=context,
        loop_gba_annotations=dict(
            scope='whole session; log delivery order, not source-time alignment',
            counts=run['diagnostics'],
            interpretation=('Acceptance/start is not application; '
                            'absent entries do not prove no correction.')),
        live_endpoint=endpoint,
        final_optimized_endpoint=dict(
            available=False, reason='historical_optimized_poses_not_collected'))


def analyze(stereo_session, imu_session, *, window_sec=3.0, min_samples=3,
            start_window=None, end_window=None, sensor_range_ns=None,
            stereo_console_log=None, imu_console_log=None):
    if not isinstance(min_samples, int) or isinstance(min_samples, bool) or min_samples < 3:
        raise ValueError('min-samples must be at least 3')
    runs = [read_session(Path(stereo_session), 'stereo', stereo_console_log),
            read_session(Path(imu_session), 'stereo_imu', imu_console_log)]
    windows, bounds = select_windows(runs, window_sec, start_window, end_window, sensor_range_ns)
    reports = [analyze_run(run, windows, min_samples) for run in runs]
    bag_ids = [run['metadata'].get('bag_id') for run in runs]
    known = all(value not in (None, '', 'unknown') for value in bag_ids)
    same_bag = bag_ids[0] == bag_ids[1] if known else None
    warnings = [
        'Stationarity is not inferred from pose jitter; review the selected windows.',
        'Endpoint differences include physical placement mismatch and estimation jitter.',
        'Map IDs/counters do not certify coordinate versions. '
        'Live poses may mix loop/GBA optimization states.',
    ]
    if sensor_range_ns is None:
        warnings.append('Recorded bounds do not prove complete bag coverage; '
                        'original sensor bounds can be supplied.')
    if same_bag is None:
        warnings.append('Same-bag provenance is unconfirmed; descriptive comparison '
                        'assumes the operator used the same bag.')
    elif not same_bag:
        warnings.append('Bag identifiers differ; paired comparison is unavailable.')
    if any(report['coordinate_context'] and report['sensor_mode'] == 'stereo_imu' and
           report['coordinate_context']['imu_initialized_states'] == [False] for report in reports):
        warnings.append('Stereo-IMU remained uninitialized over the compared interval.')
    return dict(
        schema_version=1, evaluation_schema_version=3, metric='live_endpoint_consistency',
        unavailable_value='null means N/A; see reasons', min_samples=min_samples,
        sensor_bounds=bounds,
        shared_windows_ns={key: list(value) for key, value in windows.items()},
        window_mean_definition=('Equal-weight translation mean and '
                                'sign-invariant SO(3) chordal quaternion mean.'),
        window_variation_definition=('RMS/max distance or rotation angle '
                                     'from the corresponding window mean.'),
        stationary_windows_verified=False, same_bag_confirmed=same_bag,
        comparison_available=(all(report['live_endpoint']['available'] for report in reports)
                              and same_bag is not False),
        runs=reports, warnings=warnings)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('stereo_session', type=Path)
    parser.add_argument('imu_session', type=Path)
    parser.add_argument('--stereo-console-log', type=Path)
    parser.add_argument('--imu-console-log', type=Path)
    parser.add_argument('--window-sec', type=float, default=3.0,
                        help='Default first/last window length (3 seconds)')
    parser.add_argument('--min-samples', type=int, default=3)
    parser.add_argument('--start-window', type=float, nargs=2, metavar=('START_SEC', 'END_SEC'))
    parser.add_argument('--end-window', type=float, nargs=2, metavar=('START_SEC', 'END_SEC'))
    parser.add_argument('--sensor-range-ns', type=int, nargs=2, metavar=('FIRST_NS', 'LAST_NS'))
    parser.add_argument('--output', type=Path, default=Path('results/t02_report.json'))
    parser.add_argument('--require-complete', action='store_true',
                        help='Exit 2 when comparison is unavailable')
    args = parser.parse_args(argv)
    try:
        report = analyze(args.stereo_session, args.imu_session, window_sec=args.window_sec,
                         min_samples=args.min_samples, start_window=args.start_window,
                         end_window=args.end_window, sensor_range_ns=args.sensor_range_ns,
                         stereo_console_log=args.stereo_console_log,
                         imu_console_log=args.imu_console_log)
        # Refuse an output path that would replace an input artifact.
        inputs = {(session / relative).resolve()
                  for session in (args.stereo_session, args.imu_session)
                  for relative in ('slam.log', 'console.log', 'evaluation/metadata.json',
                                   'evaluation/frames.csv', 'evaluation/summary.csv',
                                   'evaluation/inputs.csv', 'evaluation/backend_summary.csv',
                                   'evaluation/export_complete.txt')}
        inputs.update(path.resolve() for path in (args.stereo_console_log, args.imu_console_log)
                      if path)
        if args.output.resolve() in inputs:
            raise ValueError('output must not overwrite an input artifact')
        text = json.dumps(report, indent=2, allow_nan=False) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
        for run in report['runs']:
            endpoint = run['live_endpoint']
            values = (f"translation={endpoint['translation_m']:.6f} m "
                      f"rotation={endpoint['rotation_deg']:.6f} deg"
                      if endpoint['available'] else 'N/A: ' + ', '.join(endpoint['reasons']))
            print(f"{run['sensor_mode']}: {values}")
        print(f'Report: {args.output}')
        return 2 if args.require_complete and not report['comparison_available'] else 0
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as error:
        parser.exit(1, f'T02 analysis failed: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
