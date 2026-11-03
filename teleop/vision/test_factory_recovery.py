"""Field-regression CAD samples; no ROS, camera or device access."""
import unittest
from unittest.mock import patch
from factory_hand_audit import FactoryAudit
from live_hand import HandController
from factory_mapping import MODEL_ID
from live_index import SOURCE_COMMIT


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

    def test_delegated_contact_request_still_cannot_deepen_actual_thumb_contact(self):
        mapping=self.audit.mapping
        engine=HandController(self.raw,mapping)
        desired=self.raw.copy();desired[5]-=5
        named=mapping.decode(desired)
        desired=mapping.encode(named)
        for i in range(3):
            now=100.+i*.04
            engine.accept({'schema':'lh20.retarget.v1','hand':'left','scope':'hand','model':MODEL_ID,
                'model_sha256':mapping.model_hash,'source_commit':SOURCE_COMMIT,'thumb_axes':'swapped',
                'stream_session':'e'*32,'seq':i,'calibration_epoch':0,'captured_monotonic_ns':int(now*1e9),
                'valid':True,'operator_enabled':True,'auto_follow':True,'retargeting_algorithm':'vector',
                'cad_validation':'bridge_path_v1','cad_collision_hits':[{'a':'thumb_distal_0','b':'index_distal_0'}],
                'joint_names':list(named),'position_rad':list(named.values()),
                'motor_position_raw':desired,'factory_profile_sha256':mapping.sha256},now)
        self.assertTrue(engine.active)
        candidate=engine.candidate(now)
        self.assertIsNotNone(candidate)
        result=self.audit.select_target(self.raw,engine.last_command,candidate)
        self.assertFalse(result['ok'],result)
        self.assertEqual(result['reason'],'existing_collision_deepened')
        self.assertNotIn('target',result)


class PartialFollowTests(unittest.TestCase):
    def setUp(self):
        # 2026-10-25: 55 blocked checks and no commands from this starting pose.
        self.raw=[64,235,185,166,162,246,189,122,178,236,253,255,255,255,255,212,255,255,255,255]
        desired=[112,251,228,218,214,196,174,105,69,0,241,255,255,255,255,199,243,195,218,192]
        self.audit=FactoryAudit('swapped')
        self.engine=HandController(self.raw,self.audit.mapping)
        self.engine.active=True;self.engine.last_input=100.;self.engine.desired=desired
        self.requested=self.engine.candidate(100.)

    def test_recorded_block_allows_checked_thumb_and_index_only(self):
        blocked=self.audit.check(self.raw,self.requested)
        self.assertFalse(blocked['ok']);self.assertEqual(blocked['reason'],'existing_collision_deepened')
        selected=self.audit.select_target(self.raw,self.raw,self.requested)
        self.assertTrue(selected['ok'],selected)
        self.assertEqual(selected['held_fingers'],['middle','pinky','ring'])
        for name,entry in self.audit.mapping.joints.items():
            i=entry['slot']
            self.assertEqual(selected['target'][i],self.raw[i] if name.split('_')[0] in selected['held_fingers'] else self.requested[i])
        self.assertNotEqual(selected['target'],self.raw)
        self.assertTrue(self.audit.check(self.raw,selected['target'])['ok'])
        self.engine.commit(selected['target'],100.)  # Original velocity bound.

    def test_entirely_blocked_motion_still_sends_no_new_target(self):
        target=self.raw.copy();target[7]-=16
        result=self.audit.select_target(self.raw,self.raw,target)
        self.assertFalse(result['ok'],result)
        self.assertNotIn('target',result)

    def test_clear_request_is_not_modified(self):
        raw=self.audit.mapping.encode({name:0. for name in self.audit.mapping.joints})
        target=raw.copy();target[1]-=5
        result=self.audit.select_target(raw,raw,target)
        self.assertTrue(result['ok']);self.assertEqual(result['target'],target)
        self.assertEqual(result['held_fingers'],[])

    def test_recorded_after_two_commands_continues_with_quantized_feedback(self):
        hold=[82,235,185,165,162,236,191,123,177,236,243,255,255,255,255,200,244,255,255,255]
        feedback=[80,234,184,164,161,237,190,123,177,235,243,0,0,0,0,201,245,255,255,255]
        engine=HandController(hold,self.audit.mapping)
        engine.active=True;engine.last_input=100.;engine.last_step=99.95
        engine.desired=[51,11,3,1,3,188,105,21,58,8,187,255,255,255,255,200,127,44,38,62]
        requested=engine.candidate(100.)
        selected=self.audit.select_target(feedback,hold,requested)
        self.assertTrue(selected['ok'],selected)
        self.assertEqual(selected['held_fingers'],['middle','pinky','ring'])
        for name,entry in self.audit.mapping.joints.items():
            i=entry['slot']
            if name.split('_')[0] in selected['held_fingers']:
                self.assertEqual(selected['target'][i],feedback[i])
            else:self.assertEqual(selected['target'][i],requested[i])
        self.assertTrue(self.audit.check(feedback,selected['target'])['ok'])
        engine.commit(selected['target'],100.)

    def test_large_feedback_offset_cannot_rebase_a_held_joint(self):
        hold=self.raw.copy();hold[2]+=20
        with patch.object(self.audit,'check',side_effect=[
            {'ok':False,'reason':'existing_collision_deepened','collisions':[{'a':'middle_middle_0','b':'ring_middle_0'}]},
            {'ok':True,'samples':2}
        ]):
            selected=self.audit.select_target(self.raw,hold,self.requested)
        # A mocked CAD success must not turn the 20-count lag into a rebase.
        self.assertEqual(selected['target'][2],hold[2])

    def test_partial_follow_survives_repeated_feedback_quantization(self):
        baseline=[80,234,184,164,161,237,190,123,177,235,243,255,255,255,255,201,245,255,255,255]
        engine=HandController(baseline,self.audit.mapping)
        engine.active=True
        engine.desired=[51,11,3,1,3,188,105,21,58,8,187,255,255,255,255,200,127,44,38,62]
        held_slots=[e['slot'] for name,e in self.audit.mapping.joints.items() if name.split('_')[0] in ('middle','ring','pinky')]
        commands=[]
        for step in range(20):
            now=100.+step*.05;engine.last_input=now
            feedback=engine.last_command.copy()
            for i in held_slots:feedback[i]=max(0,min(255,baseline[i]+(step%3)-1))
            selected=self.audit.select_target(feedback,engine.last_command,engine.candidate(now))
            self.assertTrue(selected['ok'],(step,selected))
            engine.commit(selected['target'],now)
            commands.append(selected['target'])
        self.assertNotEqual(commands[-1][0],commands[0][0])
        self.assertNotEqual(commands[-1][1],commands[0][1])
        self.assertTrue(all(abs(q[i]-baseline[i])<=1 for q in commands for i in held_slots))

    def test_partial_path_must_pass_its_own_complete_check(self):
        blocked={'ok':False,'reason':'existing_collision_deepened',
                 'collisions':[{'a':'middle_middle_0','b':'ring_middle_0'}]}
        rejected={'ok':False,'reason':'new_collision_pair','collisions':[]}
        with patch.object(self.audit,'check',side_effect=[blocked,rejected]) as check:
            result=self.audit.select_target(self.raw,self.raw,self.requested)
        self.assertEqual(result,rejected)
        self.assertEqual(check.call_count,2)
        self.assertNotIn('target',result)

    def test_unknown_metrics_or_limits_do_not_trigger_partial_motion(self):
        for failure in ({'ok':False,'reason':'incomplete_contact_metrics'},
                        {'ok':False,'limits':[{'joint':'index_pip'}]},
                        {'ok':False,'error':'bad frame'}):
            with self.subTest(failure=failure),patch.object(self.audit,'check',return_value=failure) as check:
                self.assertEqual(self.audit.select_target(self.raw,self.raw,self.requested),failure)
                self.assertEqual(check.call_count,1)

    def test_pair_cache_matches_fresh_mesh_results_across_changed_poses(self):
        # Thumb/index motion reuses static overlapping pairs. Moving a contact
        # finger must invalidate those pairs and reproduce uncached FCL depths.
        for slot in (None,0,1,7,8):
            raw=self.raw.copy()
            if slot is not None:raw[slot]-=3
            q=self.audit.checker.expand(self.audit.mapping.decode(raw))
            cached=self.audit.snapshot(q,True)
            fresh=FactoryAudit('swapped').snapshot(q,True)
            self.assertEqual(cached,fresh)
        self.assertLessEqual(len(self.audit.pair_cache),4096)


if __name__=='__main__':unittest.main()
