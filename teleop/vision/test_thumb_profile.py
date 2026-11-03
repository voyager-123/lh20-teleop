"""Model-generated thumb tests; no claim of physical angle calibration."""
import unittest
import numpy as np
from preview_l20 import load_retargeting,model_vectors,check_qpos


class ThumbProfileTests(unittest.TestCase):
    def anchor(self,retargeting,q):
        robot=retargeting.optimizer.robot
        robot.compute_forward_kinematics(q)
        return (robot.get_link_pose(robot.get_link_index('thumb_distal'))[:3,3]
                -robot.get_link_pose(robot.get_link_index('index_proximal'))[:3,3])

    def test_palm_anchor_reduces_model_thumb_position_error(self):
        base_config,base=load_retargeting('left','hand','swapped','segments')
        self.addCleanup(base.lh20_model_temp.cleanup)
        palm_config,palm=load_retargeting('left','hand','swapped','palm')
        # The failed palm experiment retains the old ten segments plus a palm
        # anchor. Current segments instead appends the thumb IP-to-tip vector.
        self.assertEqual(palm_config.target_origin_link_names[:-1],base_config.target_origin_link_names[:-1])
        self.assertEqual(palm_config.target_task_link_names[:-1],base_config.target_task_link_names[:-1])
        np.testing.assert_array_equal(palm_config.target_link_human_indices[:,-1],[5,3])
        self.assertEqual(base.lh20_factory_mapping.sha256,palm.lh20_factory_mapping.sha256)
        np.testing.assert_array_equal(base.joint_limits,palm.joint_limits)
        # Preserve this historical palm-vs-original-Vector experiment. The
        # current independent direction refinement is tested separately.
        original=base_config.build()
        original.joint_limits=base.joint_limits.copy()
        original.optimizer.opt.set_lower_bounds(original.joint_limits[:,0])
        original.optimizer.opt.set_upper_bounds(original.joint_limits[:,1])
        base=original
        base.filter=None;palm.filter=None
        errors=[[],[]]
        poses=[(.2,.2,.2,.02),(.5,.8,.4,.02),(1.2,1.,.7,.02),
               (.2,1.,.7,.3),(1.2,.2,.2,.3),(.7,.6,.4,.8)]
        for pose in poses:
            reference=np.zeros_like(base.get_qpos())
            for name,value in zip(('thumb_cmc_yaw','thumb_cmc_roll','thumb_cmc_pitch','thumb_mcp'),pose):
                reference[base.joint_names.index(name)]=value
            reference=base.optimizer.adaptor.forward_qpos(reference)
            expected=self.anchor(base,reference)
            for index,(config,retargeting) in enumerate(((base_config,base),(palm_config,palm))):
                retargeting.reset();target=model_vectors(config,retargeting,reference)
                for _ in range(35):q=retargeting.retarget(target)
                check_qpos(retargeting,q)
                errors[index].append(np.linalg.norm(self.anchor(retargeting,q)-expected))
        self.assertLess(np.mean(errors[1]),.65*np.mean(errors[0]))
        self.assertLess(max(errors[1]),.006)

    def test_palm_profile_is_restricted_to_left_whole_hand(self):
        for side,scope in [('right','hand'),('left','index')]:
            with self.assertRaises(ValueError):load_retargeting(side,scope,'native','palm')


if __name__=='__main__':unittest.main()
