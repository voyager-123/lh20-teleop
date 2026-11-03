"""LH20 DexPilot adaptation: real upstream optimizer, model-only inputs."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from preview_l20 import load_retargeting,model_vectors,check_qpos,run_camera
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
from factory_mapping import FactoryMapping,MODEL_ID
from live_hand import HandController
from live_index import SOURCE_COMMIT
from dexpilot_model import reference_vectors


class DexPilotTests(unittest.TestCase):
    def setUp(self):
        self.mapping=FactoryMapping('swapped')
        self.original=self.mapping.model_path.read_bytes()
        self.config,self.retarget=load_retargeting('left','hand','swapped','segments','dexpilot')

    def tearDown(self):
        self.retarget.lh20_model_temp.cleanup()

    def test_real_optimizer_tip_order_and_original_joint_limits(self):
        from dex_retargeting.optimizer import DexPilotOptimizer
        self.assertIsInstance(self.retarget.optimizer,DexPilotOptimizer)
        self.assertEqual(len(self.retarget.joint_names),21)
        self.assertEqual(set(self.config.target_joint_names),set(self.mapping.joints))
        np.testing.assert_array_equal(self.retarget.optimizer.target_link_human_indices,
                                      np.array(DexPilotOptimizer.generate_link_indices(5))*4)
        for i,name in enumerate(self.config.target_joint_names):
            np.testing.assert_allclose(self.retarget.joint_limits[i],self.mapping.limits[name])
        self.assertEqual(self.retarget.lh20_physical_model_path,self.mapping.model_path)
        self.assertEqual(self.mapping.model_path.read_bytes(),self.original)

    def test_fixed_tip_frames_are_surface_offsets_not_distal_joint_origins(self):
        robot=self.retarget.optimizer.robot
        robot.compute_forward_kinematics(np.zeros(robot.dof))
        for finger,frame in self.retarget.lh20_tip_metadata['frames'].items():
            parent=robot.get_link_pose(robot.get_link_index(frame['parent']))
            tip=robot.get_link_pose(robot.get_link_index(frame['link']))
            offset=np.linalg.inv(parent)@tip
            np.testing.assert_allclose(offset[:3,3],frame['xyz_m'],atol=1e-10)
            self.assertGreater(np.linalg.norm(offset[:3,3]),.02)

    def test_projection_and_escape_hysteresis_uses_upstream_behavior(self):
        optimizer=self.retarget.optimizer
        reference=np.tile([.12,.02,.04],(15,1)).astype(np.float32)
        last=self.retarget.last_qpos
        for distance,expected in ((.02,True),(.04,True),(.06,False),(.04,False)):
            reference[0]=[distance,0,0]
            optimizer.get_objective_function(reference,np.array([]),last)
            self.assertEqual(bool(optimizer.projected[0]),expected)

    def human_points_from_robot(self,q,shorter_palm=.05):
        robot=self.retarget.optimizer.robot
        robot.compute_forward_kinematics(q)
        points=np.zeros((21,3))
        for index,name in zip((4,8,12,16,20),self.config.finger_tip_link_names):
            points[index]=robot.get_link_pose(robot.get_link_index(name))[:3,3]
        for index,finger in zip((5,9,13,17),('index','middle','ring','pinky')):
            points[index]=robot.get_link_pose(robot.get_link_index(finger+'_proximal'))[:3,3]
            points[index+1]=robot.get_link_pose(robot.get_link_index(finger+'_middle'))[:3,3]
            points[index+2]=robot.get_link_pose(robot.get_link_index(finger+'_distal'))[:3,3]
        points[1:,2]-=shorter_palm
        return points

    def test_short_human_palm_does_not_force_all_four_distal_joints_to_bend(self):
        ret=self.retarget;ret.filter=None
        points=self.human_points_from_robot(np.zeros(ret.optimizer.robot.dof))
        indices=ret.optimizer.target_link_human_indices
        original=points[indices[1]]-points[indices[0]]
        corrected=reference_vectors(points,ret)
        np.testing.assert_allclose(corrected[:-5],original[:-5],atol=1e-12)
        np.testing.assert_allclose(corrected,model_vectors(self.config,ret,np.zeros(ret.optimizer.robot.dof)),atol=1e-10)
        ret.reset();ret.optimizer.projected[:]=False
        for _ in range(35):q=ret.retarget(corrected)
        self.assertTrue(all(q[ret.joint_names.index(f+'_pip')]<.05 for f in ('index','middle','ring','pinky')))
        raw=self.mapping.encode({name:float(q[ret.joint_names.index(name)]) for name in self.config.target_joint_names})
        self.assertTrue(all(raw[i]>=246 for i in (16,17,18,19)),raw)

    def test_recorded_palm_offset_does_not_force_four_nearly_maximal_tip_bends(self):
        source=json.loads((Path(__file__).parent/'fixtures/dexpilot_palm_offset_20261025.json').read_text())
        points=np.array(source['landmarks_local_m'])
        ret=self.retarget;ret.filter=None;indices=ret.optimizer.target_link_human_indices
        results=[]
        for target in (points[indices[1]]-points[indices[0]],reference_vectors(points,ret)):
            ret.reset();ret.optimizer.projected[:]=False
            for _ in range(50):q=ret.retarget(target)
            results.append([float(q[ret.joint_names.index(f+'_pip')]) for f in ('index','middle','ring','pinky')])
        self.assertTrue(all(angle>1.2 for angle in results[0]),results)
        self.assertLess(np.median(results[1]),.5,results)
        # This source frame's pinky is visibly bent in the input geometry;
        # the adaptation must not replace every final target by straight zeros.
        self.assertGreater(results[1][3],.7,results)

    def test_palm_alignment_preserves_intertip_targets_and_pinch_projection(self):
        ret=self.retarget
        points=self.human_points_from_robot(np.zeros(ret.optimizer.robot.dof))
        points[4]=points[8]+np.array([.02,0,0])
        indices=ret.optimizer.target_link_human_indices
        original=points[indices[1]]-points[indices[0]]
        corrected=reference_vectors(points,ret)
        np.testing.assert_allclose(original[:-5],corrected[:-5],atol=1e-12)
        ret.optimizer.get_objective_function(corrected,np.array([]),ret.last_qpos)
        self.assertTrue(ret.optimizer.projected[0])
        # Position of the whole tracked hand cannot change relative targets.
        np.testing.assert_allclose(reference_vectors(points+[.1,-.2,.3],ret),corrected,atol=1e-12)

    def test_shorter_human_finger_bones_can_still_reach_straight_targets(self):
        ret=self.retarget;ret.filter=None
        points=self.human_points_from_robot(np.zeros(ret.optimizer.robot.dof))
        for mcp in (5,9,13,17):
            points[mcp+1:mcp+4]=points[mcp]+.7*(points[mcp+1:mcp+4]-points[mcp])
        target=reference_vectors(points,ret)
        for _ in range(35):q=ret.retarget(target)
        self.assertTrue(all(q[ret.joint_names.index(f+'_pip')]<.05 for f in ('index','middle','ring','pinky')))

    def test_palm_alignment_still_allows_bending(self):
        ret=self.retarget;ret.filter=None
        reference=np.zeros(ret.optimizer.robot.dof)
        for finger in ('index','middle','ring','pinky'):
            reference[ret.joint_names.index(finger+'_mcp_pitch')]=.3
            reference[ret.joint_names.index(finger+'_pip')]=.65
        reference=ret.optimizer.adaptor.forward_qpos(reference)
        points=self.human_points_from_robot(reference)
        target=reference_vectors(points,ret)
        for _ in range(35):q=ret.retarget(target)
        check_qpos(ret,q)
        self.assertTrue(all(q[ret.joint_names.index(f+'_pip')]>.4 for f in ('index','middle','ring','pinky')))

    def test_dexpilot_targets_preserve_mimics_raw_mapping_and_bridge_identity(self):
        ret=self.retarget;ret.filter=None
        for fraction in (.05,.45,.85):
            reference=ret.optimizer.robot.joint_limits[:,0]+fraction*np.ptp(ret.optimizer.robot.joint_limits,axis=1)
            reference=ret.optimizer.adaptor.forward_qpos(reference)
            target=model_vectors(self.config,ret,reference)
            ret.reset();ret.optimizer.projected[:]=False
            for _ in range(35):q=ret.retarget(target)
            check_qpos(ret,q)
            raw=self.mapping.encode({name:float(q[ret.joint_names.index(name)]) for name in self.config.target_joint_names})
            self.assertTrue(all(raw[i]==255 for i in (11,12,13,14)))
            adaptor=ret.optimizer.adaptor
            np.testing.assert_allclose(q[adaptor.idx_pin2mimic],q[adaptor.idx_pin2source]*adaptor.multipliers+adaptor.offsets,atol=1e-6)
            engine=HandController(raw,self.mapping,'dexpilot')
            for seq in range(3):
                now=100.+seq*.04
                packet={'schema':'lh20.retarget.v1','hand':'left','scope':'hand','model':MODEL_ID,
                        'model_sha256':hashlib.sha256(self.original).hexdigest(),'source_commit':SOURCE_COMMIT,
                        'thumb_axes':'swapped','retargeting_algorithm':'dexpilot','stream_session':'d'*32,
                        'seq':seq,'calibration_epoch':0,'captured_monotonic_ns':int(now*1e9),
                        'valid':True,'operator_enabled':True,'auto_follow':True,'cad_collision_hits':[],
                        'joint_names':self.config.target_joint_names,
                        'position_rad':[float(q[ret.joint_names.index(name)]) for name in self.config.target_joint_names],
                        'factory_profile_sha256':self.mapping.sha256,'motor_position_raw':raw}
                engine.accept(packet,now)
            self.assertTrue(engine.active,engine.state)

    def test_mock_camera_records_real_dexpilot_with_original_cad_identity(self):
        import cv2
        robot=self.retarget.optimizer.robot
        robot.compute_forward_kinematics(np.zeros(robot.dof))
        landmarks=self.human_points_from_robot(np.zeros(robot.dof))
        observations=iter([np.zeros_like(landmarks)]+[landmarks.copy() for _ in range(15)])
        detector=SimpleNamespace(detect=lambda image:(1,next(observations),None,None),
                                 hand_detector=SimpleNamespace(close=lambda:None))
        class Camera:
            def isOpened(self):return True
            def set(self,*args):return True
            def read(self):return True,np.zeros((480,640,3),dtype=np.uint8)
            def release(self):pass
        with tempfile.TemporaryDirectory() as folder:
            record=Path(folder)/'vision.jsonl'
            args=SimpleNamespace(hand='left',scope='hand',thumb_axes='swapped',thumb_profile='segments',
                                 retargeting='dexpilot',auto_follow=False,stream=False,socket=None,
                                 camera='0',record=record,rotate_deg=[0,0,0])
            with patch.dict(sys.modules,{'single_hand_detector':SimpleNamespace(SingleHandDetector=lambda **kwargs:detector)}), \
                 patch.object(cv2,'VideoCapture',return_value=Camera()),patch.object(cv2,'imshow'), \
                 patch.object(cv2,'waitKey',side_effect=[-1]*14+[ord('q')]),patch.object(cv2,'destroyAllWindows'):
                run_camera(args)
            packets=[json.loads(line) for line in record.read_text().splitlines()]
            self.assertFalse(packets[0]['valid'])  # Bad bone geometry pauses one frame, not the session.
            self.assertTrue(any(packet['valid'] for packet in packets),'Open model-generated input must have a CAD-valid result')
            for packet in packets:
                self.assertEqual(packet['retargeting_algorithm'],'dexpilot')
                self.assertEqual(packet['model'],MODEL_ID)
                self.assertEqual(packet['model_sha256'],self.mapping.model_hash)
                self.assertEqual(packet['reference_alignment'],'mcp_center_bone_lengths_v1')


if __name__=='__main__':unittest.main()
