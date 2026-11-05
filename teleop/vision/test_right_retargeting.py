"""Right model, mirror guide, exact mimic and real mesh path checks; no device."""
import unittest
import numpy as np
from preview_l20 import load_retargeting,model_vectors,check_qpos
from thumb_refinement import root_gesture_targets
from factory_hand_audit import FactoryAudit


def human_points(opposition=35.,abduction=15.,bend=30.):
    o,a,b=np.deg2rad([opposition,abduction,bend])
    u=np.array([np.sin(a),-np.cos(a)*np.cos(o),np.cos(a)*np.sin(o)])
    tangent=np.array([0.,np.sin(o),np.cos(o)])
    v=np.cos(b)*u+np.sin(b)*tangent
    points=np.zeros((21,3));points[1]=[.01,-.025,.025]
    points[2]=points[1]+.035*u;points[3]=points[2]+.03*v;points[4]=points[3]+.02*v
    return points


class RightRetargetingTests(unittest.TestCase):
    def setUp(self):
        self.config,self.ret=load_retargeting('right','hand')
        self.addCleanup(self.ret.lh20_model_temp.cleanup)
        self.opt=self.ret.optimizer

    def test_side_specific_limits_and_tip_frames_are_used(self):
        self.assertEqual(self.opt.hand,'right')
        self.assertTrue(self.ret.lh20_vector_tip)
        self.assertIn('l20_10_right',str(self.ret.lh20_physical_model_path))
        self.assertEqual(len(self.config.target_joint_names),16)
        self.assertEqual(len(self.ret.lh20_tip_metadata['frames']),5)
        for i,n in enumerate(self.config.target_joint_names):
            np.testing.assert_allclose(self.ret.joint_limits[i],self.ret.lh20_factory_mapping.limits[n])

    def test_guide_mirror_convention_matches_detector_mano_frames(self):
        limits=self.opt.thumb_limits
        for o,a,b in ((20,5,10),(40,20,25),(60,35,50)):
            points=human_points(o,a,b)
            mirrored=points*np.array([-1.,-1.,1.])
            left=root_gesture_targets(points,limits,'left')
            right=root_gesture_targets(mirrored,limits,'right')
            np.testing.assert_allclose(left,right,atol=1e-12)
            np.testing.assert_allclose(self.opt.set_root_reference(mirrored),right)

    def test_right_direction_objective_gradient_includes_mimic(self):
        q=np.zeros(len(self.ret.joint_names))
        ids=self.opt.idx_pin2target[self.opt.thumb_joints]
        q[ids]=[.8,.5,.35,.5];q=self.opt.adaptor.forward_qpos(q)
        target=model_vectors(self.config,self.ret,q)
        obj=self.opt.thumb_objective(target,np.array([]),self.ret.last_qpos.copy())
        x=np.array([.6,.3,.2,.3]);grad=np.zeros(4);obj(x,grad)
        numeric=np.array([(obj(x+np.eye(4)[i]*1e-6,np.array([]))-obj(x-np.eye(4)[i]*1e-6,np.array([])))/2e-6 for i in range(4)])
        np.testing.assert_allclose(grad,numeric,atol=1e-7,rtol=1e-5)

    def test_right_agile_output_is_bounded_and_keeps_exact_mimic(self):
        q=np.zeros(len(self.ret.joint_names));q=self.opt.adaptor.forward_qpos(q)
        target=model_vectors(self.config,self.ret,q)
        for o,a,b in ((20,5,10),(60,35,50),(20,5,10)):
            self.opt.set_root_reference(human_points(o,a,b)*np.array([-1.,-1.,1.]))
            for _ in range(4):
                result=self.ret.retarget(target);check_qpos(self.ret,result)
                raw=self.ret.lh20_factory_mapping.encode(dict(zip(self.config.target_joint_names,result[self.opt.idx_pin2target].tolist())))
                self.assertTrue(all(raw[i]==255 for i in (11,12,13,14)))
                ad=self.opt.adaptor
                np.testing.assert_allclose(result[ad.idx_pin2mimic],result[ad.idx_pin2source]*ad.multipliers+ad.offsets,atol=1e-6)

    def test_right_dexpilot_runtime_keeps_projection_and_hysteresis(self):
        config,ret=load_retargeting('right','hand',algorithm='dexpilot')
        self.addCleanup(ret.lh20_model_temp.cleanup)
        opt=ret.optimizer
        self.assertEqual(opt.project_dist,.03);self.assertEqual(opt.escape_dist,.05)
        q=np.zeros(ret.optimizer.robot.dof)
        target=model_vectors(config,ret,q)
        target[:4]=np.array([.02,0.,0.])
        opt.get_objective_function(target,np.array([]),ret.last_qpos.copy())
        self.assertTrue(np.all(opt.projected[:4]))
        target[:4]=np.array([.04,0.,0.])
        opt.get_objective_function(target,np.array([]),ret.last_qpos.copy())
        self.assertTrue(np.all(opt.projected[:4]))
        target[:4]=np.array([.06,0.,0.])
        opt.get_objective_function(target,np.array([]),ret.last_qpos.copy())
        self.assertFalse(np.any(opt.projected[:4]))

    def test_spread_feedback_first_step_and_closed_contact_regression(self):
        from live_hand import HandController
        audit=FactoryAudit(hand='right')
        raw=[255,223,254,254,252,179,254,168,5,0,254,0,0,0,0,254,249,255,255,253]
        self.assertTrue(audit.check(raw,raw)['ok'])
        engine=HandController(raw,audit.mapping)
        engine.active=True;engine.last_input=100.
        engine.desired=audit.mapping.encode({n:0. for n in audit.mapping.joints})
        target=engine.candidate(100.)
        self.assertIsNotNone(target)
        self.assertLessEqual(audit.mapping.step_interval(engine.last_command,target),1/30+1e-9)
        result=audit.check(raw,target)
        self.assertTrue(result['ok'],result)
        engine.commit(target,100.)
        # The prior closed-finger step must still fail for deepening contact.
        closed=[255,202,200,198,208,136,79,131,178,224,238,0,0,0,0,92,254,255,255,255]
        blocked=HandController(closed,audit.mapping)
        blocked.active=True;blocked.last_input=100.;blocked.desired=engine.desired
        result=audit.check(closed,blocked.candidate(100.))
        self.assertFalse(result['ok'])
        self.assertEqual(result.get('reason'),'existing_collision_deepened')

    def test_right_cad_uses_right_geometry_and_checks_stationary_path(self):
        audit=FactoryAudit(hand='right')
        self.assertEqual(audit.mapping.hand,'right')
        self.assertEqual(audit.mapping.model_hash,self.ret.lh20_factory_mapping.model_hash)
        self.assertGreater(len(audit.checker.geometry.collisionPairs),0)
        baseline=audit.mapping.encode({n:0. for n in audit.mapping.joints})
        checked=audit.check(baseline,baseline)
        self.assertTrue(checked['ok'],checked)
        before=audit.checker.expand(audit.mapping.decode(baseline))
        self.assertFalse(audit.checker.violations(before))


if __name__=='__main__':unittest.main()
