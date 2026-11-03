"""Offline data-integrity checks. No camera, ROS or device is opened."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import preview_capture as capture


class CaptureTests(unittest.TestCase):
    def setup_observation(self):
        config = SimpleNamespace(target_joint_names=["joint"])
        optimizer = SimpleNamespace(target_link_human_indices=np.array([[0], [1]]),
                                    opt=SimpleNamespace(last_optimize_result=lambda: 1))
        retargeting = SimpleNamespace(optimizer=optimizer, joint_names=["joint"],
                                     retarget=Mock(return_value=np.array([.25])))
        checker = Mock()
        checker.expand.side_effect = lambda x: x
        checker.check.return_value = []
        checker.violations.return_value = []
        mapping = Mock()
        mapping.encode.return_value = [123] * 20
        mapping.decode.return_value = {"joint": .25}
        return config, retargeting, checker, mapping

    def test_missing_hand_is_distinct_from_rejected_candidate(self):
        config, model, checker, mapping = self.setup_observation()
        result = capture.observation(None, None, config, model, checker, mapping)
        self.assertEqual(result["rejection_reason"], "no_selected_hand")
        model.retarget.assert_not_called()
        with patch.object(capture, "check_qpos"):
            checker.check.return_value = ["mock overlap"]
            result = capture.observation(np.zeros((21, 3)), None, config, model, checker, mapping)
        self.assertFalse(result["valid"])
        self.assertEqual(result["rejection_reason"], "cad_collision")
        self.assertEqual(result["candidate_position_rad"], [.25])
        self.assertIn("landmarks_local_m", result)
        self.assertNotIn("position_rad", result)
        mapping.encode.assert_not_called()

    def test_solver_failure_and_success_keep_separate_statuses(self):
        config, model, checker, mapping = self.setup_observation()
        with patch.object(capture, "check_qpos"):
            result = capture.observation(np.zeros((21, 3)), None, config, model, checker, mapping)
            self.assertTrue(result["valid"])
            self.assertIsNone(result["rejection_reason"])
            model.optimizer.opt.last_optimize_result = lambda: -1
            result = capture.observation(np.zeros((21, 3)), None, config, model, checker, mapping)
            self.assertEqual(result["rejection_reason"], "solver_failed")

    def test_nonfinite_landmarks_do_not_enter_solver(self):
        config, model, checker, mapping = self.setup_observation()
        result = capture.observation(np.full((21, 3), np.nan), None, config, model, checker, mapping)
        self.assertEqual(result["rejection_reason"], "invalid_landmarks")
        model.retarget.assert_not_called()

    def test_lossless_video_roundtrip_and_count_mismatch_detection(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.avi"
            frames = np.random.default_rng(17).integers(0, 256, (5, 48, 64, 3), dtype=np.uint8)
            archive = capture.FrameArchive(path, frames[0].shape)
            for i, frame in enumerate(frames):
                self.assertEqual(archive.write(frame), i)
            with self.assertRaises(ValueError):
                archive.write(frames[0, :20])
            archive.close()
            self.assertEqual(capture.verify_video(path, 5), 5)
            with self.assertRaisesRegex(RuntimeError, "帧数不一致"):
                capture.verify_video(path, 6)
            with self.assertRaises(FileExistsError):
                capture.FrameArchive(path, frames[0].shape)
            reader = cv2.VideoCapture(str(path))
            try:
                for expected in frames:
                    ok, decoded = reader.read()
                    self.assertTrue(ok)
                    np.testing.assert_array_equal(decoded, expected)
            finally:
                reader.release()


if __name__ == "__main__":
    unittest.main()
