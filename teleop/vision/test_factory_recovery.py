"""Field-regression CAD samples; no ROS, camera or device access."""
import unittest
from factory_hand_audit import FactoryAudit


class RealMeshRecoveryTests(unittest.TestCase):
    def setUp(self):
        # Actual last readback from live_hand_1792674467795277854.jsonl.
        self.raw=[216,11,131,17,8,117,92,97,56,94,197,0,0,0,0,177,123,230,6,41]
        self.audit=FactoryAudit('swapped')

    def test_recorded_start_overlap_can_hold_instead_of_latching_at_sample_zero(self):
        result=self.audit.check(self.raw,self.raw)
        self.assertTrue(result['ok'],result)
        self.assertTrue(result['recovery_from_existing_collision'])

    def test_thumb_opening_from_recorded_overlap_is_allowed(self):
        target=self.raw.copy();target[0]+=5
        result=self.audit.check(self.raw,target)
        self.assertTrue(result['ok'],result)
        self.assertTrue(result['recovery_from_existing_collision'])
        q=self.audit.checker.expand(self.audit.mapping.decode(target))
        self.assertEqual(self.audit.checker.check(q),[])

    def test_deeper_model_contact_is_still_blocked(self):
        target=self.raw.copy();target[5]-=5
        result=self.audit.check(self.raw,target)
        self.assertFalse(result['ok'],result)
        self.assertEqual(result['reason'],'existing_collision_deepened')

    def test_clear_pose_normal_path_still_passes(self):
        raw=self.audit.mapping.encode({n:0. for n in self.audit.mapping.joints})
        target=raw.copy();target[1]-=5
        result=self.audit.check(raw,target)
        self.assertTrue(result['ok']);self.assertFalse(result['recovery_from_existing_collision'])


if __name__=='__main__':unittest.main()
