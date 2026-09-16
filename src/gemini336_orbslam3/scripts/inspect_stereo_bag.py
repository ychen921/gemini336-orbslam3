#!/usr/bin/env python3
"""Inspect a stereo SQLite ROS bag offline; no DDS, image playback, or SLAM."""

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, deque
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def distribution(values):
    if not len(values):
        return None
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), min=float(a.min()), mean=float(a.mean()),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)), max=float(a.max()))


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def camera_dict(value):
    value = dict(value)
    value['header'] = {'frame_id': value['header']['frame_id']}
    return value


def read_yaml(path):
    with path.open() as stream:
        docs = [doc for doc in yaml.safe_load_all(stream) if doc is not None]
    if len(docs) != 1:
        raise ValueError(f'Expected one YAML document: {path}')
    return docs[0]


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        return [item for key in sorted(a.keys() | b.keys())
                for item in differences(a.get(key), b.get(key), path + '/' + key)]
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            return [path + ': length mismatch']
        return [item for i, (x, y) in enumerate(zip(a, b))
                for item in differences(x, y, path + '/' + str(i))]
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        equal = bool(np.isclose(a, b, rtol=0, atol=1e-8))
    else:
        equal = a == b
    return [] if equal else [f'{path}: {a!r} != {b!r}']


def transform_matrix(value):
    q = value['rotation']
    quat = np.array([q[k] for k in ('x', 'y', 'z', 'w')], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or abs(norm - 1) > 1e-4:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = quat / norm
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [value['translation'][k] for k in ('x', 'y', 'z')]
    return result


def optical_transform(transforms, source, target):
    graph = {}
    for (parent, child), value in transforms.items():
        matrix = transform_matrix(value)
        graph.setdefault(parent, []).append((child, matrix))
        graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
    queue, seen = deque([(source, np.eye(4))]), {source}
    while queue:
        name, matrix = queue.popleft()
        if name == target:
            return matrix
        for child, step in graph.get(name, []):
            if child not in seen:
                seen.add(child)
                queue.append((child, matrix @ step))
    return None


def image_from_row(row):
    with sqlite3.connect(Path(row['database']).as_uri() + '?mode=ro', uri=True) as db:
        blob = db.execute('SELECT data FROM messages WHERE id=?', (row['message_id'],)).fetchone()[0]
    msg = deserialize_message(blob, get_message('sensor_msgs/msg/Image'))
    if msg.encoding != 'mono8' or msg.step < msg.width or len(msg.data) != msg.step * msg.height:
        raise ValueError('Sampling requires valid MONO8 images')
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width].copy()


def check_matches(left, right):
    # Do not filter by vertical disparity before measuring it. General fundamental-matrix
    # RANSAC rejects mismatches without assuming horizontal epipolar lines.
    orb = cv2.ORB_create(nfeatures=2500)
    kl, dl = orb.detectAndCompute(left, None)
    kr, dr = orb.detectAndCompute(right, None)
    if dl is None or dr is None:
        return {'matches': 0, 'inliers': 0}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def ratio(a, b):
        return {pair[0].queryIdx: pair[0].trainIdx for pair in matcher.knnMatch(a, b, k=2)
                if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance}
    forward, reverse = ratio(dl, dr), ratio(dr, dl)
    pairs = [(a, b) for a, b in forward.items() if reverse.get(b) == a]
    if len(pairs) < 8:
        return {'matches': len(pairs), 'inliers': 0}
    pl = np.float32([kl[a].pt for a, _ in pairs])
    pr = np.float32([kr[b].pt for _, b in pairs])
    _, mask = cv2.findFundamentalMat(pl, pr, cv2.FM_RANSAC, 1.0, 0.99)
    valid = mask.ravel().astype(bool) if mask is not None else np.zeros(len(pl), dtype=bool)
    delta = pl - pr
    return {'matches': len(pairs), 'inliers': int(valid.sum()),
            'all_abs_dy_px': distribution(abs(delta[:, 1])),
            'inlier_abs_dy_px': distribution(abs(delta[valid, 1])),
            'inlier_signed_dy_px': distribution(delta[valid, 1]),
            'inlier_disparity_px': distribution(delta[valid, 0]),
            'inlier_positive_disparity_fraction': float(np.mean(delta[valid, 0] > 0)) if valid.any() else None}


def run(args):
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    meta = read_yaml(args.bag / 'metadata.yaml')['rosbag2_bagfile_information']
    if meta['storage_identifier'] != 'sqlite3':
        raise ValueError('This inspector supports SQLite bags only')
    args.output.mkdir(parents=True, exist_ok=False)
    images = {'left': [], 'right': []}
    infos = {'left': [], 'right': []}
    info_stamps = {'left': [], 'right': []}
    transforms, tf_conflicts, actual_counts, invalid = {}, [], Counter(), []
    image_topics = {args.left_topic: 'left', args.right_topic: 'right'}
    info_topics = {args.left_info: 'left', args.right_info: 'right'}
    for relative in meta['relative_file_paths']:
        path = (args.bag / relative).resolve()
        with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
            topics = {row[0]: (row[1], row[2]) for row in db.execute('SELECT id,name,type FROM topics')}
            types = {name: get_message(kind) for name, kind in topics.values()
                     if name in image_topics or name in info_topics or name == '/tf_static'}
            for mid, tid, record, blob in db.execute('SELECT id,topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                topic, _ = topics[tid]
                actual_counts[topic] += 1
                if topic not in types:
                    continue
                msg = deserialize_message(blob, types[topic])
                if topic in image_topics:
                    side = image_topics[topic]
                    row = dict(index=len(images[side]), message_id=mid, database=str(path),
                               record_ns=record, stamp_ns=stamp_ns(msg.header),
                               width=msg.width, height=msg.height, encoding=msg.encoding,
                               step=msg.step, data_length=len(msg.data), is_bigendian=msg.is_bigendian,
                               frame_id=msg.header.frame_id)
                    images[side].append(row)
                    if (msg.encoding != 'mono8' or msg.width == 0 or msg.height == 0
                            or msg.step < msg.width or len(msg.data) != msg.step * msg.height):
                        invalid.append({'side': side, 'index': row['index']})
                elif topic in info_topics:
                    side = info_topics[topic]
                    value = camera_dict(message_to_ordereddict(msg))
                    if value not in infos[side]:
                        infos[side].append(value)
                    info_stamps[side].append(stamp_ns(msg.header))
                else:
                    for tf in message_to_ordereddict(msg)['transforms']:
                        key = (tf['header']['frame_id'], tf['child_frame_id'])
                        if key in transforms and differences(transforms[key], tf['transform']):
                            tf_conflicts.append(list(key))
                        transforms[key] = tf['transform']
        print(f'Scanned {relative}: left={len(images["left"])} right={len(images["right"])}', flush=True)
    summary = {'actual_counts': dict(actual_counts), 'invalid_images': invalid,
               'metadata_count_differences': {}, 'images': {}, 'camera_info': {}, 'tf_conflicts': tf_conflicts}
    for row in meta['topics_with_message_count']:
        topic = row['topic_metadata']['name']
        if actual_counts[topic] != row['message_count']:
            summary['metadata_count_differences'][topic] = [row['message_count'], actual_counts[topic]]
    for side, rows in images.items():
        if not rows or not infos[side]:
            raise ValueError(f'Missing images or CameraInfo for {side}')
        intervals = np.diff([row['stamp_ns'] for row in rows])
        positive = intervals[intervals > 0]
        typical = float(np.median(positive)) if len(positive) else None
        summary['images'][side] = {
            'count': len(rows), 'formats': [dict(zip(('width', 'height', 'encoding', 'step', 'data_length', 'is_bigendian', 'frame_id'), key), count=count)
                for key, count in Counter(tuple(row[k] for k in ('width', 'height', 'encoding', 'step', 'data_length', 'is_bigendian', 'frame_id')) for row in rows).items()],
            'interval_ms': distribution(intervals / 1e6),
            'duplicates': int((intervals == 0).sum()), 'backwards': int((intervals < 0).sum()),
            'header_fps': float(1e9 * (len(rows)-1) / (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])) if rows[-1]['stamp_ns'] > rows[0]['stamp_ns'] else None,
            'gaps_over_1_5_period': [{'after_index': i, 'interval_ns': int(v)} for i, v in enumerate(intervals) if typical and v > typical * 1.5],
            'record_minus_header_ms': distribution([(row['record_ns'] - row['stamp_ns']) / 1e6 for row in rows])}
        raw = camera_dict(read_yaml(args.calibration / f'{side}_ir_camera_info.yaml'))
        summary['camera_info'][side] = {'count': len(info_stamps[side]), 'variants': infos[side],
            'raw_differences': [differences(value, raw) for value in infos[side]],
            'stamp_sequence_equals_images': info_stamps[side] == [row['stamp_ns'] for row in rows]}
        with (args.output / f'{side}_images.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    # This is an offline one-to-one timestamp reference, not a simulation of ApproximateTime.
    left, right = [sorted(images[side], key=lambda r: r['stamp_ns']) for side in ('left', 'right')]
    i, j, pairs, unmatched = 0, 0, [], {'left': [], 'right': []}
    while i < len(left) and j < len(right):
        delta = left[i]['stamp_ns'] - right[j]['stamp_ns']
        if abs(delta) <= args.tolerance_ns:
            pairs.append((left[i], right[j]))
            i, j = i+1, j+1
        elif delta < 0:
            unmatched['left'].append(left[i]['index'])
            i += 1
        else:
            unmatched['right'].append(right[j]['index'])
            j += 1
    unmatched['left'] += [row['index'] for row in left[i:]]
    unmatched['right'] += [row['index'] for row in right[j:]]
    right_stamps = [r['stamp_ns'] for r in right]
    nearest = []
    for l in left:
        pos = bisect_left(right_stamps, l['stamp_ns'])
        candidate = min((k for k in (pos-1, pos) if 0 <= k < len(right)),
                        key=lambda k: abs(right_stamps[k] - l['stamp_ns']))
        nearest.append((l, right[candidate]))
    spans = []
    for index in unmatched['left']:
        if spans and index == spans[-1]['last_index'] + 1:
            spans[-1]['last_index'] = index
        else:
            spans.append(dict(first_index=index, last_index=index))
    for span in spans:
        a, b = span['first_index'], span['last_index']
        span.update(count=b-a+1,
                    start_sec=(images['left'][a]['stamp_ns']-images['left'][0]['stamp_ns'])/1e9,
                    end_sec=(images['left'][b]['stamp_ns']-images['left'][0]['stamp_ns'])/1e9)
    summary['pairing'] = {'tolerance_ns': args.tolerance_ns, 'pairs': len(pairs), 'unmatched': unmatched,
        'left_with_multiple_candidates': sum(bisect_right(right_stamps, r['stamp_ns'] + args.tolerance_ns) - bisect_left(right_stamps, r['stamp_ns'] - args.tolerance_ns) > 1 for r in left),
        'right_minus_left_ms': distribution([(r['stamp_ns']-l['stamp_ns']) / 1e6 for l, r in pairs])}
    summary['nearest_timestamp_reference'] = {
        'right_minus_left_ms': distribution([(r['stamp_ns']-l['stamp_ns']) / 1e6 for l, r in nearest]),
        'absolute_difference_ms': distribution([abs(r['stamp_ns']-l['stamp_ns']) / 1e6 for l, r in nearest]),
        'same_sequence_index_count': sum(l['index'] == r['index'] for l, r in nearest),
        'unmatched_left_spans': spans,
        'note': 'Nearest timestamp is diagnostic only and does not establish simultaneous exposures.'}
    with (args.output / 'nearest_pairs.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['left_index', 'right_index', 'left_stamp_ns', 'right_stamp_ns', 'right_minus_left_ns'])
        writer.writerows((l['index'], r['index'], l['stamp_ns'], r['stamp_ns'], r['stamp_ns']-l['stamp_ns']) for l, r in nearest)
    with (args.output / 'pairs.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['left_index', 'right_index', 'left_stamp_ns', 'right_stamp_ns', 'right_minus_left_ns'])
        writer.writerows((l['index'], r['index'], l['stamp_ns'], r['stamp_ns'], r['stamp_ns']-l['stamp_ns']) for l, r in pairs)

    raw_tf = {(t['header']['frame_id'], t['child_frame_id']): t['transform'] for t in read_yaml(args.calibration / 'tf_static.yaml')['transforms']}
    summary['tf_raw_differences'] = differences({' -> '.join(k): v for k, v in transforms.items()}, {' -> '.join(k): v for k, v in raw_tf.items()})
    lc, rc = infos['left'][0], infos['right'][0]
    matrix = optical_transform(transforms, lc['header']['frame_id'], rc['header']['frame_id'])
    summary['T_left_optical_right_optical'] = matrix.tolist() if matrix is not None else None
    baseline = -rc['p'][3] / rc['p'][0] + lc['p'][3] / lc['p'][0]
    storage = cv2.FileStorage(str(args.settings), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError('Cannot read ORB-SLAM3 settings')
    keys = ('Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy', 'Camera.width', 'Camera.height', 'Camera.fps', 'Stereo.b')
    settings = {key: None if storage.getNode(key).empty() else storage.getNode(key).real() for key in keys}
    settings['Camera.type'] = storage.getNode('Camera.type').string()
    summary['settings_camera_schema_checks'] = {
        'version_1_0': storage.getNode('File.version').string() == '1.0',
        'rectified': settings['Camera.type'] == 'Rectified',
        'required_numeric_types': {key: bool(storage.getNode(key).isInt() if key in ('Camera.width', 'Camera.height', 'Camera.fps') else storage.getNode(key).isReal()) for key in keys}}
    storage.release()
    summary['settings'] = settings
    expected = dict(zip(('Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy', 'Camera.width', 'Camera.height', 'Stereo.b'),
                        (lc['p'][0], lc['p'][5], lc['p'][2], lc['p'][6], lc['width'], lc['height'], baseline)))
    summary['settings_minus_camera_info'] = {key: settings[key] - value if settings[key] is not None else None for key, value in expected.items()}
    summary['projection_baseline_m'] = baseline
    summary['rectified_metadata_checks'] = {
        'constant_camera_info': all(len(infos[s]) == 1 for s in infos),
        'image_geometry_and_frame_ids_match_info': all(all(row['width'] == infos[side][0]['width'] and row['height'] == infos[side][0]['height'] and row['frame_id'] == infos[side][0]['header']['frame_id'] for row in rows) for side, rows in images.items()),
        'zero_distortion': all(np.allclose(c['d'], 0, atol=1e-10) for c in (lc, rc)),
        'identity_R': all(np.allclose(np.reshape(c['r'], (3, 3)), np.eye(3), atol=1e-10) for c in (lc, rc)),
        'same_K': bool(np.allclose(lc['k'], rc['k'], atol=1e-8, rtol=0)),
        'projection_3x3_equals_K': all(np.allclose(np.array(c['p']).reshape(3, 4)[:, :3], np.array(c['k']).reshape(3, 3), atol=1e-8, rtol=0) for c in (lc, rc)),
        'tf_matches_horizontal_baseline': bool(matrix is not None and np.allclose(matrix[:3, :3], np.eye(3), atol=1e-6) and np.allclose(matrix[:3, 3], [baseline, 0, 0], atol=1e-6))}
    samples = []
    (args.output / 'samples').mkdir()
    sample_pairs = [(f'{n:04d}', *pairs[n], 'within_tolerance') for n in
                    (np.unique(np.linspace(0, len(pairs)-1, min(args.samples, len(pairs)), dtype=int)) if pairs else [])]
    # Also inspect the beginning/middle/end of each excluded interval; these are diagnostic
    # nearest pairs, never silently counted as valid synchronization output.
    by_left = {l['index']: (l, r) for l, r in nearest}
    for span in spans:
        for index in sorted({span['first_index'], (span['first_index']+span['last_index'])//2, span['last_index']}):
            sample_pairs.append((f'gap_{index:04d}', *by_left[index], 'outside_tolerance_nearest'))
    for label, l, r, kind in sample_pairs:
        li, ri = image_from_row(l), image_from_row(r)
        result = dict(label=label, pairing_kind=kind, left_index=l['index'], right_index=r['index'], **check_matches(li, ri))
        samples.append(result)
        cv2.imwrite(str(args.output / 'samples' / f'{label}_left.png'), li)
        cv2.imwrite(str(args.output / 'samples' / f'{label}_right.png'), ri)
    summary['feature_samples'] = samples
    summary['input_sha256'] = {str(p): file_hash(p) for p in
        [args.settings, args.bag / 'metadata.yaml', args.calibration / 'left_ir_camera_info.yaml', args.calibration / 'right_ir_camera_info.yaml', args.calibration / 'tf_static.yaml'] + [args.bag / p for p in meta['relative_file_paths']]}
    summary['interpretation'] = 'Offline evidence only; review calibration and feature residuals before tracking. No automatic rectification certification.'
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'image_counts': {side: len(rows) for side, rows in images.items()},
                      'invalid_images': len(invalid), 'pairs_within_tolerance': len(pairs),
                      'nearest_timestamp_reference': summary['nearest_timestamp_reference'],
                      'settings_camera_schema_checks': summary['settings_camera_schema_checks'],
                      'rectified_metadata_checks': summary['rectified_metadata_checks']}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'calibration', 'settings', 'output'):
        parser.add_argument('--' + name, required=True, type=lambda p: Path(p).resolve())
    parser.add_argument('--left-topic', default='/camera/left_ir/image_raw')
    parser.add_argument('--right-topic', default='/camera/right_ir/image_raw')
    parser.add_argument('--left-info', default='/camera/left_ir/camera_info')
    parser.add_argument('--right-info', default='/camera/right_ir/camera_info')
    parser.add_argument('--tolerance-ns', type=int, default=500000)
    parser.add_argument('--samples', type=int, default=20)
    args = parser.parse_args()
    if args.tolerance_ns <= 0 or args.samples <= 0:
        parser.error('Tolerance and sample count must be positive')
    run(args)


if __name__ == '__main__':
    main()
