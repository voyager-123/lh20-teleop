"""Replay saved landmarks against v2 directions and v3 guided thumb roots.

No ROS, CAN or camera access. Direction errors refer to tracked human segments,
not measured robot angles. CAD endpoint samples are not executed-path checks.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from preview_l20 import load_retargeting, model_vectors, check_qpos, COMMIT
from low_latency import AdaptiveJointFilter
from collision_check import CollisionChecker


def compare(config, current, frames, cad_stride):
    previous = config.build()
    previous.joint_limits = current.joint_limits.copy()
    from thumb_refinement import ThumbResponsiveOptimizer, root_gesture_targets
    previous.optimizer = ThumbResponsiveOptimizer(previous.optimizer, previous.joint_limits)
    previous.optimizer.root_guide_weight = 0.
    previous.optimizer.opt.set_lower_bounds(previous.joint_limits[:, 0])
    previous.optimizer.opt.set_upper_bounds(previous.joint_limits[:, 1])
    checker = CollisionChecker(current.lh20_physical_model_path)
    mapping = current.lh20_factory_mapping
    thumb_ids = [current.joint_names.index(n) for n in config.target_joint_names[:4]]
    other_ids = [current.joint_names.index(n) for n in config.target_joint_names[4:]]
    result = {};outputs = {}
    for label, ret in [('previous', previous), ('responsive', current)]:
        ret.reset()
        ret.filter = AdaptiveJointFilter(ret.optimizer.adaptor)
        errors = [];elapsed = [];poses = [];guides = [];cad_clear = 0;cad_count = 0
        for index, frame in enumerate(frames):
            points = np.array(frame['landmarks_local_m'])
            indices = config.target_link_human_indices
            target = points[indices[1]] - points[indices[0]]
            guides.append(root_gesture_targets(points, current.optimizer.thumb_limits))
            ret.filter.set_timestamp(frame['captured_monotonic_ns']/1e9)
            began = time.perf_counter()
            if label=='responsive':ret.optimizer.set_root_reference(points)
            q = ret.retarget(target)
            elapsed.append((time.perf_counter()-began)*1000)
            check_qpos(ret, q);poses.append(q)
            robot_vec = model_vectors(config, ret, q)[[0, 1, 10]]
            human_vec = target[[0, 1, 10]]
            cosines = np.sum(robot_vec*human_vec, axis=1)/np.linalg.norm(robot_vec, axis=1)/np.linalg.norm(human_vec, axis=1)
            errors.append(np.rad2deg(np.arccos(np.clip(cosines, -1, 1))))
            raw = mapping.encode({n: float(q[ret.joint_names.index(n)]) for n in config.target_joint_names})
            if index % cad_stride == 0:
                quantized = checker.expand(mapping.decode(raw))
                cad_clear += not checker.check(quantized) and not checker.violations(quantized)
                cad_count += 1
        outputs[label] = np.array(poses)
        result[label] = {
            'frames': len(frames), 'thumb_direction_error_deg_p50_p95': np.percentile(errors, [50, 95], axis=0).tolist(),
            'solver_ms_p50_p95': np.percentile(elapsed, [50, 95]).tolist(),
            'thumb_joint_p5_p50_p95_rad': np.percentile(outputs[label][:, thumb_ids], [5, 50, 95], axis=0).tolist(),
            'root_below_5deg_fraction': np.mean(outputs[label][:, thumb_ids[:3]]<np.deg2rad(5),axis=0).tolist(),
            'root_guide_abs_error_deg_p50_p95':np.rad2deg(np.percentile(abs(outputs[label][:,thumb_ids[:3]]-np.array(guides)),[50,95],axis=0)).tolist(),
            'sampled_endpoints_clear': int(cad_clear), 'sampled_endpoints': cad_count,
        }
    result['four_finger_difference_abs_rad_p50_p95'] = np.percentile(
        abs(outputs['responsive'][:, other_ids]-outputs['previous'][:, other_ids]), [50, 95]).tolist()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cad-stride', type=int, default=10)
    args = parser.parse_args()
    if args.cad_stride < 1:parser.error('cad-stride must be positive')
    config, current = load_retargeting('left', 'hand', 'swapped')
    try:
        report = {'source_commit': COMMIT, 'model_sha256': current.lh20_factory_mapping.model_hash,
                  'thumb_target': 'anatomical_root_guide_v3', 'baseline':'independent_directions_v2', 'cad_stride': args.cad_stride,
                  'thumb_temporal_weight':current.optimizer.temporal_weight,
                  'root_guide_weight':current.optimizer.root_guide_weight,
                  'root_guide_axis_weights':current.optimizer.root_guide_axis_weights.tolist(),
                  'root_guide_parameters':current.optimizer.root_guide_parameters,
                  'thumb_initialization':'previous_refined_pose',
                  'hardware_measured': False, 'datasets': {}}
        for path in args.recordings:
            frames = [json.loads(line) for line in path.open()]
            frames = [frame for frame in frames if 'landmarks_local_m' in frame]
            if not frames:raise ValueError(f'No landmark frames: {path}')
            report['datasets'][path.name] = compare(config, current, frames, args.cad_stride)
            print(path.name, json.dumps(report['datasets'][path.name]), flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output:json.dump(report, output, indent=2)
    finally:
        current.lh20_model_temp.cleanup()


if __name__ == '__main__':main()
