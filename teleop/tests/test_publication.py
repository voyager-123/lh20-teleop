"""Public snapshot invariants; no CAN, ROS graph, or camera is opened."""
import hashlib
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'teleop/bridge'))
from factory_mapping import FactoryMapping
from probe_index_joint import feedback_errors
from runtime_paths import vision_python
import auto_vision
import lh20_cli


class PublicSnapshotTests(unittest.TestCase):
    def test_meshes_are_portable_and_pinned(self):
        mapping = FactoryMapping('swapped')
        for node in ET.parse(mapping.model_path).iter('mesh'):
            path = Path(node.get('filename'))
            self.assertFalse(path.is_absolute())
            self.assertTrue((mapping.model_path.parent/path).is_file())
        snapshot = json.loads((ROOT/'docs/snapshot.json').read_text())
        for path, digest in snapshot['file_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(), digest)

    def test_exact_fault_exceptions_retain_transport_and_combined_fault_checks(self):
        now = 100.
        state = {'q':[100]*20, 'state_at':now, 'info_at':now,
                 'info':{'hand_joint':'L20-10', 'fault':[0]*20}}
        for code in (0,64,128):
            state['info']['fault'][5] = code
            self.assertEqual(feedback_errors(state,now,ignore_64=True,ignore_128=True), [])
        for code in (1,32,65,129,192,255):
            state['info']['fault'][5] = code
            self.assertTrue(feedback_errors(state,now,ignore_64=True,ignore_128=True))
        state['info']['fault'][5] = 64
        self.assertTrue(feedback_errors(state,now+.31,ignore_64=True,ignore_128=True))

    def test_vision_interpreter_override(self):
        with patch.dict(os.environ, {'LH20_VISION_PYTHON':sys.executable}):
            self.assertEqual(vision_python(), Path(sys.executable).resolve())

    def test_no_identity_is_embedded_in_factory_profile(self):
        self.assertNotIn('expected_identifier', FactoryMapping().profile)

    def test_automatic_session_requires_local_identity_before_discovery(self):
        from types import SimpleNamespace
        args = SimpleNamespace(hand='left', scope='hand', thumb_axes='swapped')
        with patch.dict(os.environ, {'LH20_EXPECTED_IDENTIFIER':''}), \
             patch.object(lh20_cli,'check_can'), \
             patch.object(auto_vision,'read_identity',return_value='test-device'), \
             patch.object(auto_vision,'discover') as discovery:
            with self.assertRaisesRegex(RuntimeError,'register_hand'):
                auto_vision.run(args)
            discovery.assert_not_called()


if __name__ == '__main__':
    unittest.main()
