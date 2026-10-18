#!/usr/bin/env python3
"""Analyze post-shutdown trajectories without moving the original T02 windows.

Both modes are reconstructed as left-camera poses in their respective final maps.
First-valid-to-last-pose differences are separate diagnostics, not return-to-origin
measurements. Only existing artifacts are read; no ROS or replay is launched.
"""

import argparse
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path

import analyze_t02 as live


MATCH_TOLERANCE_NS = 1000
FILES = {'frames': 'CameraTrajectory.txt', 'keyframes': 'KeyFrameTrajectory.txt'}


@dataclass(frozen=True)
class Pose:
    sensor_ns: int
    translation: tuple
    quaternion: tuple


def transform(value):
    translation = tuple(float(v) for v in value['translation_m'])
    quaternion = tuple(float(v) for v in value['quaternion_xyzw'])
    if len(translation) != 3 or len(quaternion) != 4 or not all(
            math.isfinite(v) for v in translation + quaternion):
        raise ValueError('invalid trajectory transform')
    return translation, live.normalize_quaternion(quaternion)


def compose(left, right):
    """Compose T_a_b * T_b_c using xyzw quaternions."""
    t, (x, y, z, w) = left
    v, (a, b, c, d) = right
    cross = (2 * (y*v[2] - z*v[1]), 2 * (z*v[0] - x*v[2]),
             2 * (x*v[1] - y*v[0]))
    rotated = (v[0] + w*cross[0] + y*cross[2] - z*cross[1],
               v[1] + w*cross[1] + z*cross[0] - x*cross[2],
               v[2] + w*cross[2] + x*cross[1] - y*cross[0])
    q = (w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
         w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c)
    return tuple(t[i] + rotated[i] for i in range(3)), live.normalize_quaternion(q)


def read_trajectory(path, summary, sensor_times, origin, extrinsic):
    if (summary['path'] != path.name or summary['present'] is not True or
            summary['bytes'] != path.stat().st_size):
        raise ValueError(f'{path}: file manifest mismatch')
    poses, deltas = [], []
    source_rows = 0
    previous_ns = None
    with path.open() as source:
        for number, line in enumerate(source, 1):
            try:
                source_rows += 1
                fields = line.split()
                if len(fields) != 8:
                    raise ValueError('expected timestamp and seven pose fields')
                stamp = Decimal(fields[0])
                if not stamp.is_finite() or stamp < 0 or stamp != stamp.to_integral_value():
                    raise ValueError('timestamp must be finite integral nanoseconds')
                source_ns = int(stamp)
                if previous_ns is not None and source_ns < previous_ns:
                    raise ValueError('trajectory timestamps must not decrease')
                previous_ns = source_ns
                # The exporter rounds Core double seconds. Match to an original
                # sensor timestamp before selecting windows, preserving boundary samples.
                lo = bisect_left(sensor_times, source_ns - MATCH_TOLERANCE_NS)
                hi = bisect_right(sensor_times, source_ns + MATCH_TOLERANCE_NS)
                if hi - lo != 1:
                    raise ValueError('timestamp has no unique recorded sensor match')
                sensor_ns = sensor_times[lo]
                pose = transform(dict(translation_m=fields[1:4], quaternion_xyzw=fields[4:]))
                translation, quaternion = compose(compose(origin, pose), extrinsic)
                if not all(math.isfinite(v) for v in translation):
                    raise ValueError('nonfinite reconstructed camera position')
                deltas.append(abs(sensor_ns - source_ns))
                if poses and sensor_ns == poses[-1].sensor_ns:
                    # History can repeat the same timestamp and pose. Count it
                    # once rather than overweighting that instant; reject conflicts.
                    if translation != poses[-1].translation or live.rotation_distance_deg(
                            quaternion, poses[-1].quaternion) > 1e-9:
                        raise ValueError('conflicting poses at the same sensor timestamp')
                    continue
                poses.append(Pose(sensor_ns, translation, quaternion))
            except (ValueError, ArithmeticError) as error:
                raise ValueError(f'{path}:{number}: {error}') from error
    counters = ('written', 'lost', 'invalid_reference', 'other_map',
                'invalid_pose', 'invalid_timestamp')
    if any(type(summary[name]) is not int or summary[name] < 0
           for name in ('considered', 'bytes') + counters):
        raise ValueError(f'{path}: invalid file counters')
    if summary['written'] != source_rows or summary['considered'] != sum(
            summary[name] for name in counters):
        raise ValueError(f'{path}: row counts disagree with manifest')
    if poses:
        for key, pose in (('first_timestamp_sec', poses[0]), ('last_timestamp_sec', poses[-1])):
            stamp = Decimal(str(summary[key])) * 1_000_000_000
            if not stamp.is_finite() or abs(stamp - pose.sensor_ns) > MATCH_TOLERANCE_NS:
                raise ValueError(f'{path}: timestamp bounds disagree with manifest')
    return poses, dict(matched_rows=source_rows, unique_sensor_samples=len(poses),
                       duplicate_rows_removed=source_rows - len(poses),
                       max_rounding_delta_ns=max(deltas, default=0))


def read_export(run):
    directory = run['path'] / 'evaluation/final_trajectory'
    manifest_path = directory / 'manifest.json'
    if not manifest_path.exists():
        return None
    manifest = json.loads(manifest_path.read_text())
    mode = run['metadata']['sensor_mode']
    expected = dict(schema_version=1, evaluation_schema_version=3, sensor_mode=mode,
                    run_id=run['metadata']['run_id'], backend_shutdown_returned=True,
                    export_complete=True, timestamp_unit='ns', translation_unit='m',
                    quaternion_order='xyzw', frame_pose_convention='T_origin_sensor',
                    keyframe_pose_convention='T_map_sensor',
                    pose_target='imu_body' if mode == 'stereo_imu' else 'left_camera')
    if any(type(manifest.get(k)) is not type(v) or manifest[k] != v for k, v in expected.items()):
        raise ValueError(f'{manifest_path}: incompatible manifest')
    if manifest.get('status') != 'complete' or manifest.get('available') is not True:
        return dict(manifest=manifest, available=False)
    if manifest.get('map_selected') is not True or type(manifest.get('map_id')) is not int:
        raise ValueError(f'{manifest_path}: final map unavailable')
    identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    extrinsic = transform(manifest['body_from_camera']) if mode == 'stereo_imu' else identity
    frame_origin = transform(manifest['frame_origin_in_map'])
    sensor_times = [frame.sensor_ns for frame in run['frames']]
    if any(b <= a for a, b in zip(sensor_times, sensor_times[1:])):
        raise ValueError('recorded sensor timestamps must increase strictly')
    result = dict(manifest=manifest, available=True, timestamp_alignment={})
    for kind, filename in FILES.items():
        result[kind], result['timestamp_alignment'][kind] = read_trajectory(
            directory / filename, manifest['files'][kind], sensor_times,
            frame_origin if kind == 'frames' else identity, extrinsic)
    return result


def window_statistics(poses, bounds, min_samples):
    samples = [p for p in poses if bounds[0] <= p.sensor_ns <= bounds[1]]
    result = dict(sensor_start_ns=bounds[0], sensor_end_ns=bounds[1], samples=len(samples),
                  available=False, reasons=[], translation_mean_m=None, quaternion_mean_xyzw=None,
                  translation_rms_m=None, translation_max_m=None,
                  rotation_rms_deg=None, rotation_max_deg=None,
                  first_sensor_ns=samples[0].sensor_ns if samples else None,
                  last_sensor_ns=samples[-1].sensor_ns if samples else None)
    if len(samples) < min_samples:
        result['reasons'].append('insufficient_final_poses')
        return result
    try:
        mean = tuple(math.fsum(p.translation[i] / len(samples) for p in samples) for i in range(3))
        rotation = live.quaternion_mean([p.quaternion for p in samples])
        distances = [math.dist(p.translation, mean) for p in samples]
        angles = [live.rotation_distance_deg(p.quaternion, rotation) for p in samples]
        result.update(available=True, translation_mean_m=mean, quaternion_mean_xyzw=rotation,
                      translation_rms_m=math.hypot(*distances) / math.sqrt(len(samples)),
                      translation_max_m=max(distances),
                      rotation_rms_deg=math.hypot(*angles) / math.sqrt(len(samples)),
                      rotation_max_deg=max(angles))
    except (ValueError, OverflowError) as error:
        result['reasons'].append(str(error))
    return result


def difference(first, last, reasons=()):
    result = dict(available=False, translation_m=None, rotation_deg=None, reasons=list(reasons))
    if reasons:
        return result
    distance = math.dist(first[0], last[0])
    angle = live.rotation_distance_deg(first[1], last[1])
    if not math.isfinite(distance) or not math.isfinite(angle):
        result['reasons'].append('nonfinite_pose_difference')
    else:
        result.update(available=True, translation_m=distance, rotation_deg=angle)
    return result


def analyze_run(run, windows, bounds, min_samples):
    export = read_export(run)
    checks = {name: run['integrity'][name] for name in (
        'export_complete', 'frame_records_complete', 'event_details_complete',
        'process_ok', 'final_counters_consistent', 'collection_complete')}
    checks['input_accounting_valid'] = run['integrity']['inputs'].get('accounting_valid') is True
    reasons = [name + '_unconfirmed' for name, value in checks.items() if value is not True]
    if export is None or not export['available']:
        reasons.append('final_trajectory_unavailable')
    poses = export['frames'] if export and export['available'] else []
    statistics = {name: window_statistics(poses, window, min_samples)
                  for name, window in windows.items()}
    endpoint_reasons = reasons + [name + '_window_unavailable'
                                  for name, stats in statistics.items() if not stats['available']]
    first, last = statistics['start'], statistics['end']
    endpoint = difference((first['translation_mean_m'], first['quaternion_mean_xyzw']),
                          (last['translation_mean_m'], last['quaternion_mean_xyzw']), endpoint_reasons)
    diagnostic = difference((poses[0].translation, poses[0].quaternion) if poses else None,
                            (poses[-1].translation, poses[-1].quaternion) if poses else None,
                            reasons + ([] if poses else ['no_final_poses']))
    diagnostic.update(
        definition='Single first available final camera pose versus last pose; not return-to-origin.',
        first_sensor_ns=poses[0].sensor_ns if poses else None,
        last_sensor_ns=poses[-1].sensor_ns if poses else None,
        first_offset_sec=(poses[0].sensor_ns - bounds['first_sensor_ns']) * 1e-9 if poses else None)
    agreement = None
    if export and export['available']:
        indexed = {p.sensor_ns: p for p in poses}
        pairs = [(indexed[p.sensor_ns], p) for p in export['keyframes'] if p.sensor_ns in indexed]
        agreement = dict(matched_keyframes=len(pairs),
                         unmatched_keyframes=len(export['keyframes']) - len(pairs),
                         max_translation_m=max((math.dist(a.translation, b.translation)
                                                for a, b in pairs), default=None),
                         max_rotation_deg=max((live.rotation_distance_deg(a.quaternion, b.quaternion)
                                               for a, b in pairs), default=None))
    return dict(session=str(run['path']), sensor_mode=run['metadata']['sensor_mode'],
                collection_checks=checks, manifest=export['manifest'] if export else None,
                pose_target='left_camera', coordinate_frame='selected final map of this run',
                timestamp_alignment=export.get('timestamp_alignment') if export else None,
                windows=statistics, final_optimized_endpoint=endpoint,
                first_valid_pose_to_last_pose=diagnostic, keyframe_frame_agreement=agreement)


def input_hashes(sessions, consoles):
    files = [session / relative for session in sessions for relative in (
        'slam.log', 'console.log', 'evaluation/metadata.json', 'evaluation/frames.csv',
        'evaluation/summary.csv', 'evaluation/inputs.csv', 'evaluation/backend_summary.csv',
        'evaluation/export_complete.txt', 'evaluation/final_trajectory/manifest.json',
        'evaluation/final_trajectory/CameraTrajectory.txt',
        'evaluation/final_trajectory/KeyFrameTrajectory.txt')]
    files.extend(Path(path) for path in consoles if path is not None)
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.exists()}


def analyze(stereo_session, imu_session, *, window_sec=3.0, min_samples=3,
            start_window=None, end_window=None, sensor_range_ns=None,
            stereo_console_log=None, imu_console_log=None):
    if type(min_samples) is not int or min_samples < 3:
        raise ValueError('min-samples must be at least 3')
    sessions = [Path(stereo_session), Path(imu_session)]
    consoles = [stereo_console_log, imu_console_log]
    before = input_hashes(sessions, consoles)
    runs = [live.read_session(path, mode, console) for path, mode, console in
            zip(sessions, ('stereo', 'stereo_imu'), consoles)]
    windows, bounds = live.select_windows(runs, window_sec, start_window, end_window, sensor_range_ns)
    reports = [analyze_run(run, windows, bounds, min_samples) for run in runs]
    bag_ids = [run['metadata'].get('bag_id') for run in runs]
    same_bag = bag_ids[0] == bag_ids[1] if all(v not in (None, '', 'unknown') for v in bag_ids) else None
    if input_hashes(sessions, consoles) != before:
        raise ValueError('input artifacts changed during analysis')
    return dict(schema_version=1, metric='final_optimized_camera_endpoint_consistency',
                sensor_bounds=bounds, shared_windows_ns={k: list(v) for k, v in windows.items()},
                window_mean_definition='Equal-weight translation and sign-invariant SO(3) chordal mean.',
                timestamp_match_tolerance_ns=MATCH_TOLERANCE_NS,
                same_bag_confirmed=same_bag, stationary_windows_verified=False,
                comparison_available=all(r['final_optimized_endpoint']['available'] for r in reports)
                and same_bag is not False, runs=reports, input_sha256=before,
                input_artifacts_unchanged=True, warnings=[
                    'Matching bag_id is not a bag content hash; recorded bounds do not prove full bag coverage.',
                    'Start/end windows are shared and never shifted to the first valid pose.',
                    'Pose variation alone does not verify stationarity. Endpoint differences include placement mismatch.',
                    'Map coordinates differ between runs; compare scalar endpoint differences, not absolute map positions.',
                    'First-valid-to-last-pose differences are diagnostic; no ground truth or pure drift is measured.',
                ])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stereo_session', type=Path)
    parser.add_argument('imu_session', type=Path)
    parser.add_argument('--stereo-console-log', type=Path)
    parser.add_argument('--imu-console-log', type=Path)
    parser.add_argument('--window-sec', type=float, default=3.0)
    parser.add_argument('--min-samples', type=int, default=3)
    parser.add_argument('--start-window', type=float, nargs=2)
    parser.add_argument('--end-window', type=float, nargs=2)
    parser.add_argument('--sensor-range-ns', type=int, nargs=2)
    parser.add_argument('--require-complete', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/t02_final_report.json'))
    args = parser.parse_args(argv)
    try:
        sessions = (args.stereo_session, args.imu_session)
        consoles = (args.stereo_console_log, args.imu_console_log)
        destination = args.output.resolve()
        if any(destination.is_relative_to(path.resolve()) for path in sessions) or any(
                path is not None and (destination == path.resolve() or
                (destination.exists() and destination.samefile(path))) for path in consoles):
            raise ValueError('output must not overwrite input artifacts')
        report = analyze(*sessions, window_sec=args.window_sec, min_samples=args.min_samples,
                         start_window=args.start_window, end_window=args.end_window,
                         sensor_range_ns=args.sensor_range_ns,
                         stereo_console_log=args.stereo_console_log, imu_console_log=args.imu_console_log)
        # Also reject hard links to an input outside its original session path.
        if destination.exists() and any(destination.samefile(path) for path in report['input_sha256']):
            raise ValueError('output must not overwrite input artifacts')
        content = json.dumps(report, indent=2, allow_nan=False) + '\n'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content)
        for run in report['runs']:
            endpoint = run['final_optimized_endpoint']
            print(f"{run['sensor_mode']}: {endpoint}")
        print(f'Report: {args.output}')
        return 2 if args.require_complete and not report['comparison_available'] else 0
    except (OSError, ValueError, TypeError, KeyError, ArithmeticError) as error:
        parser.exit(1, f'Final trajectory analysis failed: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
