"""Freshness, tracking/filtering, thumb pose and original CAD identity; offline."""
import copy
import json
from pathlib import Path
import queue
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from low_latency import LatestFrameCapture, AdaptiveJointFilter
from preview_l20 import load_retargeting, model_vectors, check_qpos, run_camera


class FilterTests(unittest.TestCase):
    def test_fast_motion_has_less_lag_without_overshoot_and_rest_noise_stays_bounded(self):
        rng=np.random.default_rng(4)
        for frequency in (30,60):
            timestamps=np.arange(6*frequency)/frequency
            truth=np.where(timestamps<2,0.,.5+.35*np.sin(2*np.pi*1.2*(timestamps-2)))
            noisy=truth+rng.normal(0,.001,len(truth))
            filt=AdaptiveJointFilter();actual=[];ema=[];old=noisy[0]
            for t,x in zip(timestamps,noisy):
                filt.set_timestamp(t);actual.append(filt.next([x])[0])
                old+=.6*(x-old);ema.append(old)
            actual=np.array(actual);ema=np.array(ema)
            moving=timestamps>2.5;rest=(timestamps>.5)&(timestamps<1.8)
            self.assertLess(np.mean(abs(actual[moving]-truth[moving])),.8*np.mean(abs(ema[moving]-truth[moving])))
            self.assertLess(np.std(actual[rest]),1.15*np.std(ema[rest]))
            self.assertGreaterEqual(actual.min(),noisy.min())
            self.assertLessEqual(actual.max(),noisy.max())

    def test_gap_resets_without_prediction_and_bad_timestamp_is_rejected(self):
        filt=AdaptiveJointFilter()
        filt.set_timestamp(1.);filt.next([0.])
        filt.set_timestamp(1.03);self.assertLess(filt.next([1.])[0],1.)
        filt.set_timestamp(2.);np.testing.assert_array_equal(filt.next([.2]),[.2])
        filt.set_timestamp(1.9)
        with self.assertRaises(ValueError):filt.next([.3])
        with self.assertRaises(ValueError):filt.set_timestamp(float('nan'))
        with self.assertRaises(ValueError):filt.next([float('nan')])


class FreshCameraTests(unittest.TestCase):
    class Camera:
        def __init__(self):self.frames=queue.Queue();self.released=False
        def read(self):return self.frames.get(timeout=1)
        def release(self):self.released=True

    def test_slow_consumer_gets_latest_frame_and_cannot_repeat_stale_image(self):
        camera=self.Camera();reader=LatestFrameCapture(camera)
        try:
            camera.frames.put((True,1))
            first=reader.read();self.assertEqual(first[0],1)
            for value in (2,3,4):camera.frames.put((True,value))
            with reader.condition:
                self.assertTrue(reader.condition.wait_for(lambda:reader.sequence==4,.5))
            latest=reader.read();self.assertEqual(latest[0],4)
            self.assertGreater(latest[1],first[1]);self.assertEqual(latest[2],4)
            with self.assertRaisesRegex(RuntimeError,'fresh'):reader.read(timeout=.01)
            camera.frames.put((False,None))
            with reader.condition:
                self.assertTrue(reader.condition.wait_for(lambda:reader.error is not None,.5))
            with self.assertRaisesRegex(RuntimeError,'stopped'):reader.read()
        finally:
            camera.frames.put((False,None));reader.close()
        self.assertTrue(camera.released);self.assertFalse(reader.thread.is_alive())


class VectorThumbTests(unittest.TestCase):
    def setUp(self):
        self.config,self.ret=load_retargeting('left','hand','swapped','segments')
        self.addCleanup(self.ret.lh20_model_temp.cleanup)
        self.ret.filter=None

    def human_points(self,q):
        robot=self.ret.optimizer.robot;robot.compute_forward_kinematics(q)
        points=np.zeros((21,3))
        for index,name in ((1,'thumb_metacarpals'),(2,'thumb_proximal'),(3,'thumb_distal'),(4,'teleop_thumb_tip')):
            points[index]=robot.get_link_pose(robot.get_link_index(name))[:3,3]
        for mcp,finger in zip((5,9,13,17),('index','middle','ring','pinky')):
            for offset,part in enumerate(('proximal','middle','distal')):
                points[mcp+offset]=robot.get_link_pose(robot.get_link_index(finger+'_'+part))[:3,3]
            points[mcp+3]=robot.get_link_pose(robot.get_link_index('teleop_'+finger+'_tip'))[:3,3]
        return points

    def test_real_vector_adds_tip_without_changing_four_finger_targets_or_mapping(self):
        from dex_retargeting.optimizer import VectorOptimizer
        self.assertIsInstance(self.ret.optimizer,VectorOptimizer)
        self.assertEqual(self.config.target_origin_link_names[-1],'thumb_distal')
        self.assertEqual(self.config.target_task_link_names[-1],'teleop_thumb_tip')
        np.testing.assert_array_equal(self.config.target_link_human_indices[:,-1],[3,4])
        self.assertEqual(len(self.ret.joint_names),21)
        self.assertEqual(self.ret.lh20_factory_mapping.model_path,self.ret.lh20_physical_model_path)
        self.assertEqual(self.config.target_link_human_indices[:,:10].tolist(),
                         [[1,2,5,6,9,10,13,14,17,18],[2,3,6,7,10,11,14,15,18,19]])

    def test_thumb_tip_changes_request_and_real_mimic_relation_survives_filtering(self):
        robot=self.ret.optimizer.robot
        q=np.zeros(robot.dof)
        for name,value in zip(self.config.target_joint_names[:4],(.5,.5,.5,.3)):
            q[self.ret.joint_names.index(name)]=value
        q=self.ret.optimizer.adaptor.forward_qpos(q)
        points=self.human_points(q);indices=self.config.target_link_human_indices
        target=points[indices[1]]-points[indices[0]]
        self.ret.set_qpos(q)
        for _ in range(40):initial=self.ret.retarget(target)
        moved=points.copy();v=points[4]-points[3]
        # Alter only the tracker IP->tip direction; the old ten references do
        # not change, but the new request must respond to this observation.
        moved[4]=points[3]+np.array([v[0],v[2],-v[1]])
        changed=moved[indices[1]]-moved[indices[0]]
        np.testing.assert_allclose(changed[:-1],target[:-1])
        for _ in range(40):after=self.ret.retarget(changed)
        active=[self.ret.joint_names.index(n) for n in self.config.target_joint_names[:4]]
        self.assertGreater(np.linalg.norm(after[active]-initial[active]),.03)
        filt=AdaptiveJointFilter(self.ret.optimizer.adaptor)
        for index,pose in enumerate((initial,after,initial)):
            filt.set_timestamp(1+index/30);filtered=filt.next(pose);check_qpos(self.ret,filtered)
            adaptor=self.ret.optimizer.adaptor
            np.testing.assert_allclose(filtered[adaptor.idx_pin2mimic],
                filtered[adaptor.idx_pin2source]*adaptor.multipliers+adaptor.offsets,atol=1e-12)

    def test_mock_camera_sends_before_rendering_with_original_model_identity(self):
        self.run_mock_camera()

    def test_requested_endpoint_contact_is_streamed_for_actual_bridge_path_check(self):
        hits=[{'a':'thumb_distal_0','b':'index_distal_0','kind':'cross_finger'}]
        with patch('collision_check.CollisionChecker.check',return_value=hits):
            self.run_mock_camera(expected_hits=hits)

    def run_mock_camera(self,expected_hits=None):
        import cv2
        from factory_mapping import MODEL_ID
        landmarks=self.human_points(np.zeros(self.ret.optimizer.robot.dof))
        observations=iter([landmarks.copy() for _ in range(9)])
        detector=SimpleNamespace(detect=lambda _: (1,next(observations),None,None),hand_detector=SimpleNamespace(close=lambda:None))
        events=[];packets=[]
        class Sender:
            def send(self,packet):events.append('send');packets.append(copy.deepcopy(packet));return True
            def close(self):pass
        class Camera:
            def isOpened(self):return True
            def set(self,*args):return True
            def read(self):return True,np.zeros((480,640,3),dtype=np.uint8)
            def release(self):pass
        class Reader:
            def __init__(self,capture):self.capture=capture;self.sequence=0
            def read(self):
                self.sequence+=2
                return self.capture.read()[1],time.monotonic_ns(),self.sequence
            def close(self):self.capture.release()
        def panel(*args):events.append('render');return np.zeros((480,640,3),dtype=np.uint8)
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'vision.jsonl'
            args=SimpleNamespace(hand='left',scope='hand',thumb_axes='swapped',thumb_profile='segments',
                retargeting='vector',auto_follow=True,stream=True,socket=None,camera='0',record=output,rotate_deg=[0,0,0])
            with patch.dict(sys.modules,{'single_hand_detector':SimpleNamespace(SingleHandDetector=lambda **_:detector)}), \
                 patch.object(cv2,'VideoCapture',return_value=Camera()),patch.object(cv2,'imshow'), \
                 patch.object(cv2,'waitKey',side_effect=[-1]*7+[ord('q')]),patch.object(cv2,'destroyAllWindows'), \
                 patch('low_latency.LatestFrameCapture',Reader),patch('live_index.VisionSender',return_value=Sender()), \
                 patch('preview_l20.model_panel',side_effect=panel):
                run_camera(args)
            saved=[json.loads(line) for line in output.read_text().splitlines()]
        self.assertTrue(any(p['valid'] for p in saved))
        for packet in saved:
            self.assertEqual(packet['model'],MODEL_ID)
            self.assertEqual(packet['model_sha256'],self.ret.lh20_factory_mapping.model_hash)
            self.assertEqual(packet['vector_thumb_target'],'anatomical_root_guide_v3')
            self.assertEqual(packet['cad_validation'],'bridge_path_v1')
            self.assertEqual(packet['joint_filter'],'speed_adaptive_v1')
            self.assertIn('detection',packet['timing_ms'])
            if packet['valid'] and expected_hits is not None:
                self.assertEqual(packet['cad_collision_hits'],expected_hits)
        for i,event in enumerate(events):
            if event=='render':self.assertEqual(events[i-1],'send')
        self.assertFalse(packets[-1]['operator_enabled'])


if __name__=='__main__':unittest.main()
