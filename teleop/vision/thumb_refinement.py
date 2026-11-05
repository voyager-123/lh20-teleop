"""Bounded thumb direction fitting with an optional anatomical root guide.

Human and robot thumb bones have different lengths. Fit unit directions with
equal segment influence rather than letting the longest bone dominate. Camera
observations can guide ambiguous root axes with explicit anatomical commands.
No palm position anchor or pinch latch is used.
"""
import nlopt
import numpy as np

from dex_retargeting.optimizer import VectorOptimizer

ROOT_GUIDE_PARAMETERS = {
    'opposition_gain':1.25, 'opposition_offset_degrees':15.,
    'abduction_gain':1.8, 'abduction_offset_rad':.12, 'mcp_bend_gain':1.3,
}


def root_gesture_targets(points, limits, hand="left"):
    """Palm-frame metacarpal angles and MCP bend, adapted to the LH20 axes.

    This is an anatomical command guide, not measured human/robot calibration.
    The camera detector already expresses +X out of the palm, -Y radially and
    +Z towards the fingers. These gains cover a useful part of the robot range.
    """
    points = np.asarray(points, dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError('Expected 21 finite palm-frame landmarks')
    if hand not in ('left','right'):raise ValueError('Unknown hand side')
    # Detector MANO frames differ by diag(-1,-1,1). Express right root
    # gestures in the left guide convention; the direction objective still
    # uses untouched right-frame landmarks and the independent right model.
    if hand=='right':points=points*np.array([-1.,-1.,1.])
    bones = np.diff(points[1:4], axis=0)
    lengths = np.linalg.norm(bones, axis=1)
    if np.any(lengths < .003):raise ValueError('Collapsed thumb root observation')
    first, second = bones/lengths[:, None]
    opposition = np.arctan2(first[2], -first[1])
    abduction = np.arctan2(first[0], np.hypot(first[1], first[2]))
    bend = np.arccos(np.clip(np.dot(first, second), -1., 1.))
    p=ROOT_GUIDE_PARAMETERS
    angles = np.array([p['opposition_gain']*(opposition-np.deg2rad(p['opposition_offset_degrees'])),
                       p['abduction_offset_rad']+p['abduction_gain']*abduction, p['mcp_bend_gain']*bend])
    return np.clip(angles, limits[:3, 0], limits[:3, 1])


class ThumbResponsiveOptimizer(VectorOptimizer):
    def __init__(self, original, limits, hand="left"):
        super().__init__(
            original.robot, original.target_joint_names,
            original.origin_link_names, original.task_link_names,
            original.target_link_human_indices, huber_delta=original.huber_loss.beta,
            norm_delta=original.norm_delta,
            scaling=original.scaling,
        )
        self.hand=hand
        self.set_kinematic_adaptor(original.adaptor)
        self.set_joint_limit(limits, epsilon=0.)
        self.thumb_joints = np.array([
            i for i, name in enumerate(self.target_joint_names) if name.startswith('thumb_')
        ])
        self.thumb_vectors = np.array([
            i for i, (a, b) in enumerate(zip(self.origin_link_names, self.task_link_names))
            if a.startswith('thumb_') and (b.startswith('thumb_') or b == 'teleop_thumb_tip')
        ])
        if len(self.thumb_joints) != 4 or len(self.thumb_vectors) != 3:
            raise ValueError('Thumb refinement requires four joints and three segments')
        names = list(dict.fromkeys(
            [self.origin_link_names[i] for i in self.thumb_vectors]
            + [self.task_link_names[i] for i in self.thumb_vectors]
        ))
        self.thumb_links = self.get_link_indices(names)
        self.thumb_origins = np.array([names.index(self.origin_link_names[i]) for i in self.thumb_vectors])
        self.thumb_tasks = np.array([names.index(self.task_link_names[i]) for i in self.thumb_vectors])
        self.thumb_opt = nlopt.opt(nlopt.LD_SLSQP, 4)
        self.thumb_limits = limits[self.thumb_joints].copy()
        self.thumb_opt.set_lower_bounds(limits[self.thumb_joints, 0])
        self.thumb_opt.set_upper_bounds(limits[self.thumb_joints, 1])
        self.thumb_opt.set_ftol_abs(1e-8)
        self.thumb_opt.set_xtol_abs(1e-5)
        self.thumb_opt.set_maxeval(60)
        self.temporal_weight = 3e-3
        self.root_guide_weight = .6
        self.root_guide_axis_weights = np.array([1., 1., 2.])
        self.pending_root_guide = None

    @property
    def root_guide_parameters(self):
        return ROOT_GUIDE_PARAMETERS.copy()

    def set_root_reference(self, points):
        self.pending_root_guide = None
        self.pending_root_guide = root_gesture_targets(points, self.thumb_limits, self.hand)
        return self.pending_root_guide.copy()

    def thumb_objective(self, target_vector, fixed_qpos, last_qpos, root_guide=None):
        target = np.asarray(target_vector, dtype=float)[self.thumb_vectors]
        lengths = np.linalg.norm(target, axis=1)
        if target.shape != (3, 3) or not np.isfinite(target).all() or np.any(lengths < .003):
            raise ValueError('Invalid or collapsed thumb segments')
        directions = target / lengths[:, None]
        qpos = np.zeros(self.num_joints)
        qpos[self.idx_pin2fixed] = fixed_qpos
        qpos[self.idx_pin2target] = last_qpos
        previous = last_qpos[self.thumb_joints].copy()

        def objective(x, grad):
            qpos[self.idx_pin2target[self.thumb_joints]] = x
            if self.adaptor is not None:
                self.adaptor.forward_qpos(qpos)
            self.robot.compute_forward_kinematics(qpos)
            poses = [self.robot.get_link_pose(i) for i in self.thumb_links]
            points = np.array([pose[:3, 3] for pose in poses])
            vectors = points[self.thumb_tasks] - points[self.thumb_origins]
            norms = np.linalg.norm(vectors, axis=1)
            unit = vectors / norms[:, None]
            residual = unit - directions
            delta = x - previous
            loss = .5 * np.sum(residual * residual) / 3 + self.temporal_weight * np.dot(delta, delta)
            if root_guide is not None:
                guide_delta = x[:3]-root_guide
                loss += .5*self.root_guide_weight*np.dot(guide_delta*self.root_guide_axis_weights, guide_delta)
            if grad.size:
                jac = np.array([
                    pose[:3, :3] @ self.robot.compute_single_link_local_jacobian(qpos, index)[:3]
                    for pose, index in zip(poses, self.thumb_links)
                ])
                if self.adaptor is not None:
                    jac = self.adaptor.backward_jacobian(jac)
                else:
                    jac = jac[..., self.idx_pin2target]
                jac = (jac[self.thumb_tasks] - jac[self.thumb_origins])[..., self.thumb_joints]
                # d(v/|v|)/dq = (I - uu^T) dv/dq / |v|.
                tangent = residual - unit * np.sum(residual * unit, axis=1)[:, None]
                grad[:] = np.einsum('si,sij->j', tangent / norms[:, None], jac) / 3
                grad[:] += 2 * self.temporal_weight * delta
                if root_guide is not None:
                    grad[:3] += self.root_guide_weight*self.root_guide_axis_weights*guide_delta
            return float(loss)

        return objective

    def retarget(self, ref_value, fixed_qpos, last_qpos):
        # Validate before advancing either solver. A failed observation must not
        # silently become a stale thumb target paired with moving other fingers.
        root_guide, self.pending_root_guide = self.pending_root_guide, None
        objective = self.thumb_objective(ref_value, fixed_qpos, last_qpos, root_guide)
        qpos = super().retarget(ref_value, fixed_qpos, last_qpos)
        if self.opt.last_optimize_result() <= 0:
            raise ValueError('Vector optimizer failed')
        self.thumb_opt.set_min_objective(objective)
        try:
            # Continue from the last refined thumb, rather than the upstream
            # length-fit pose: near straight configurations those can lie on
            # different IK branches despite nearly identical directions.
            start = np.clip(last_qpos[self.thumb_joints], self.thumb_limits[:, 0], self.thumb_limits[:, 1])
            thumb = self.thumb_opt.optimize(start)
        except RuntimeError as error:
            raise ValueError('Thumb direction optimizer failed') from error
        if self.thumb_opt.last_optimize_result() <= 0 or not np.isfinite(thumb).all():
            raise ValueError('Thumb direction optimizer failed')
        qpos[self.thumb_joints] = thumb
        return qpos
