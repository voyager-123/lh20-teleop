"""Independent thumb accuracy, derivatives and reversals; no device access."""
import unittest

import numpy as np

from preview_l20 import load_retargeting, model_vectors, check_qpos
from low_latency import AdaptiveJointFilter


class ResponsiveThumbTests(unittest.TestCase):
    def setUp(self):
        self.config, self.ret = load_retargeting('left', 'hand', 'swapped')
        self.addCleanup(self.ret.lh20_model_temp.cleanup)
        self.ret.filter = None
        self.opt = self.ret.optimizer
        self.ids = self.opt.idx_pin2target[self.opt.thumb_joints]

    def pose(self, values):
        q = np.zeros(len(self.ret.joint_names))
        q[self.ids] = values
        return self.opt.adaptor.forward_qpos(q)

    def test_analytic_gradient_matches_finite_difference_including_thumb_mimic(self):
        reference = model_vectors(self.config, self.ret, self.pose([.9, .4, .5, .6]))
        last = self.pose([.4, .3, .2, .2])[self.opt.idx_pin2target]
        objective = self.opt.thumb_objective(reference, np.array([]), last)
        x = np.array([.6, .5, .4, .3]);grad = np.zeros(4);objective(x, grad)
        numeric = np.zeros(4)
        for i in range(4):
            offset = np.eye(4)[i] * 1e-6
            numeric[i] = (objective(x+offset, np.array([]))-objective(x-offset, np.array([]))) / 2e-6
        np.testing.assert_allclose(grad, numeric, atol=1e-7, rtol=1e-5)
        scaled=reference.copy()
        scaled[self.opt.thumb_vectors] *= np.array([1.9, .65, 1.4])[:, None]
        scaled_objective=self.opt.thumb_objective(scaled, np.array([]), last)
        scaled_grad=np.zeros(4)
        self.assertAlmostEqual(objective(x, grad),scaled_objective(x, scaled_grad),places=12)
        np.testing.assert_allclose(grad,scaled_grad,atol=1e-12)

    def test_single_frame_fit_is_bone_length_invariant_and_reverses_immediately(self):
        poses = [[.2, .2, .15, .1], [1.2, .9, .7, .7], [.2, .2, .15, .1]]
        results = []
        for pose in poses:
            expected = self.pose(pose)
            target = model_vectors(self.config, self.ret, expected)
            # Simulate very different human phalanx lengths, preserving angles.
            target[self.opt.thumb_vectors] *= np.array([1.9, .65, 1.4])[:, None]
            q = self.ret.retarget(target)
            check_qpos(self.ret, q)
            actual = model_vectors(self.config, self.ret, q)[self.opt.thumb_vectors]
            desired = target[self.opt.thumb_vectors]
            cosine = np.sum(actual*desired, axis=1)/np.linalg.norm(actual, axis=1)/np.linalg.norm(desired, axis=1)
            # Near a straight thumb, different root joint combinations can
            # produce the same directions. Test the observed task itself.
            self.assertLess(np.max(np.degrees(np.arccos(np.clip(cosine, -1, 1)))), 2.)
            results.append(q)
        self.assertTrue(np.all(results[1][self.ids] > results[0][self.ids]))
        self.assertTrue(np.all(results[2][self.ids] < results[1][self.ids]))

    def test_refinement_changes_only_thumb_and_respects_factory_limits(self):
        expected = self.pose([1.4, 1.1, .85, 1.1])
        target = model_vectors(self.config, self.ret, expected)
        previous = self.ret.last_qpos.copy()
        from dex_retargeting.optimizer import VectorOptimizer
        baseline = VectorOptimizer.retarget(self.opt, target, np.array([]), previous)
        refined = self.opt.retarget(target, np.array([]), previous)
        nonthumb = [i for i in range(16) if i not in self.opt.thumb_joints]
        np.testing.assert_allclose(refined[nonthumb], baseline[nonthumb], atol=1e-7)
        self.assertTrue(np.all(refined >= self.ret.joint_limits[:, 0]-1e-6))
        self.assertTrue(np.all(refined <= self.ret.joint_limits[:, 1]+1e-6))

    def test_collapsed_thumb_rejects_frame_without_advancing_last_pose(self):
        target = model_vectors(self.config, self.ret, self.pose([.4, .3, .2, .2]))
        before = self.ret.last_qpos.copy()
        for bad in (np.zeros(3), np.ones(3)*1e-4, np.full(3, np.nan)):
            broken = target.copy();broken[self.opt.thumb_vectors[-1]] = bad
            with self.assertRaises(ValueError):self.ret.retarget(broken)
            np.testing.assert_array_equal(self.ret.last_qpos, before)

    def test_rest_noise_is_bounded_and_filter_keeps_exact_mimic(self):
        self.ret.filter = AdaptiveJointFilter(self.opt.adaptor)
        truth = self.pose([.6, .6, .4, .3])
        target = model_vectors(self.config, self.ret, truth)
        self.ret.set_qpos(truth)
        rng = np.random.default_rng(17);outputs = []
        for i in range(90):
            self.ret.filter.set_timestamp(i/30)
            q = self.ret.retarget(target+rng.normal(0, .00015, target.shape))
            check_qpos(self.ret, q);outputs.append(q[self.ids])
            a = self.opt.adaptor
            np.testing.assert_allclose(q[a.idx_pin2mimic], q[a.idx_pin2source]*a.multipliers+a.offsets, atol=1e-12)
        self.assertLess(np.max(np.std(outputs[20:], axis=0)), .025)

    def human_pose(self, opposition=.8, abduction=.2, bend=.25):
        points=np.zeros((21,3));points[1]=[.01,-.025,.025]
        first=np.array([np.sin(abduction),-np.cos(abduction)*np.cos(opposition),
                        np.cos(abduction)*np.sin(opposition)])
        tangent=np.cross([1.,0.,0.],first);tangent/=np.linalg.norm(tangent)
        second=np.cos(bend)*first+np.sin(bend)*tangent
        points[2]=points[1]+.035*first
        points[3]=points[2]+.030*second
        points[4]=points[3]+.020*second
        return points

    def test_root_features_separate_three_gestures_and_ignore_bone_scale(self):
        from thumb_refinement import root_gesture_targets
        limits=self.opt.thumb_limits
        baseline=root_gesture_targets(self.human_pose(),limits)
        for axis,kwargs in enumerate(({'opposition':1.05},{'abduction':.4},{'bend':.45})):
            guide=root_gesture_targets(self.human_pose(**kwargs),limits)
            others=[i for i in range(3) if i!=axis]
            np.testing.assert_allclose(guide[others],baseline[others],atol=1e-12)
            self.assertGreater(guide[axis]-baseline[axis],.2)
        np.testing.assert_allclose(root_gesture_targets(self.human_pose()*1.8+[.1,.2,.3],limits),baseline,atol=1e-12)

    def test_each_root_axis_responds_and_reverses_with_anatomical_guide(self):
        def solve(points):
            target=model_vectors(self.config,self.ret,self.pose([.8,.4,.3,.2]))
            ix=self.config.target_link_human_indices
            target[self.opt.thumb_vectors]=(points[ix[1]]-points[ix[0]])[self.opt.thumb_vectors]
            for _ in range(12):
                guide=self.opt.set_root_reference(points)
                q=self.ret.retarget(target)
                check_qpos(self.ret,q)
            return q[self.ids],guide
        for axis,kwargs in enumerate(({'opposition':1.05},{'abduction':.4},{'bend':.45})):
            self.ret.set_qpos(self.pose([.8,.4,.3,.2]))
            before,guide_before=solve(self.human_pose())
            moved,guide_after=solve(self.human_pose(**kwargs))
            returned,_=solve(self.human_pose())
            requested=guide_after[axis]-guide_before[axis]
            self.assertGreater(moved[axis]-before[axis],.6*requested)
            self.assertLess(returned[axis],moved[axis]-.5*requested)

    def test_guided_gradient_matches_finite_difference(self):
        reference=model_vectors(self.config,self.ret,self.pose([.9,.4,.5,.6]))
        last=self.pose([.4,.3,.2,.2])[self.opt.idx_pin2target]
        guide=self.opt.set_root_reference(self.human_pose())
        objective=self.opt.thumb_objective(reference,np.array([]),last,guide)
        x=np.array([.6,.5,.4,.3]);grad=np.zeros(4);objective(x,grad)
        numeric=np.array([(objective(x+np.eye(4)[i]*1e-6,np.array([]))-
                           objective(x-np.eye(4)[i]*1e-6,np.array([])))/2e-6 for i in range(4)])
        np.testing.assert_allclose(grad,numeric,atol=1e-7,rtol=1e-5)

    def test_root_reference_is_bounded_consumed_once_and_cleared_on_failure(self):
        reference=model_vectors(self.config,self.ret,self.pose([.6,.5,.4,.3]))
        guide=self.opt.set_root_reference(self.human_pose(opposition=3.,abduction=1.,bend=1.5))
        self.assertTrue(np.all(guide>=self.opt.thumb_limits[:3,0]))
        self.assertTrue(np.all(guide<=self.opt.thumb_limits[:3,1]))
        self.ret.retarget(reference)
        self.assertIsNone(self.opt.pending_root_guide)
        self.opt.set_root_reference(self.human_pose())
        with self.assertRaises(ValueError):self.opt.set_root_reference(np.zeros((21,3)))
        self.assertIsNone(self.opt.pending_root_guide)
        self.opt.set_root_reference(self.human_pose())
        reference[self.opt.thumb_vectors[0]]=0
        with self.assertRaises(ValueError):self.ret.retarget(reference)
        self.assertIsNone(self.opt.pending_root_guide)

    def test_directions_option_preserves_v2_without_geometric_guide(self):
        config,ret=load_retargeting('left','hand','swapped','segments','vector','directions')
        self.addCleanup(ret.lh20_model_temp.cleanup)
        self.assertEqual(ret.optimizer.root_guide_weight,0.)
        ret.filter=None
        target=model_vectors(config,ret,self.pose([.6,.5,.4,.3]))
        initial=ret.last_qpos.copy()
        expected=ret.retarget(target)
        ret.last_qpos=initial
        ret.optimizer.set_root_reference(self.human_pose())
        np.testing.assert_array_equal(ret.retarget(target),expected)

    def test_guided_static_landmark_noise_stays_bounded_with_exact_mimic(self):
        self.ret.filter=AdaptiveJointFilter(self.opt.adaptor)
        points=self.human_pose();rng=np.random.default_rng(29);poses=[]
        fixed_target=model_vectors(self.config,self.ret,self.pose([.8,.4,.3,.2]))
        indices=self.config.target_link_human_indices
        for frame in range(90):
            observed=points+rng.normal(0,.00015,points.shape)
            target=fixed_target.copy()
            target[self.opt.thumb_vectors]=(observed[indices[1]]-observed[indices[0]])[self.opt.thumb_vectors]
            self.opt.set_root_reference(observed)
            self.ret.filter.set_timestamp(frame/30)
            q=self.ret.retarget(target);check_qpos(self.ret,q);poses.append(q[self.ids])
            a=self.opt.adaptor
            np.testing.assert_allclose(q[a.idx_pin2mimic],q[a.idx_pin2source]*a.multipliers+a.offsets,atol=1e-12)
        self.assertLess(np.max(np.std(poses[30:],axis=0)),.025)


if __name__ == '__main__':unittest.main()
