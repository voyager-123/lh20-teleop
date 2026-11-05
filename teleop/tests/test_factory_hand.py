"""Official left conversion and full-hand controller invariants; no hardware."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'bridge'))
from factory_mapping import FactoryMapping,MODEL_ID,ACTIVE
from live_hand import HandController
from live_index import SOURCE_COMMIT
import lh20_cli


class FactoryHandTests(unittest.TestCase):
    def setUp(self):
        self.m=FactoryMapping();self.now=100.;self.seq=0
        self.open={n:0. for n in self.m.joints}
        self.baseline=self.m.encode(self.open)
        self.engine=HandController(self.baseline,self.m)

    def packet(self,named=None,**changes):
        named=self.open if named is None else named
        p={'schema':'lh20.retarget.v1','hand':'left','scope':'hand','model':MODEL_ID,
           'model_sha256':self.m.model_hash,'source_commit':SOURCE_COMMIT,'stream_session':'f'*32,
           'seq':self.seq,'calibration_epoch':0,'captured_monotonic_ns':int(self.now*1e9),
           'valid':True,'operator_enabled':True,'auto_follow':True,'cad_collision_hits':[],
           'joint_names':list(named),'position_rad':list(named.values()),
           'motor_position_raw':self.m.encode(named),'factory_profile_sha256':self.m.sha256}
        p.update(changes);return p

    def feed(self,named=None,**changes):
        self.engine.accept(self.packet(named,**changes),self.now)
        target=self.engine.candidate(self.now)
        if target is not None:self.engine.commit(target,self.now)
        self.seq+=1;self.now+=.04
        return target

    def test_all_4096_native_samples_match_official_left_function(self):
        rows=json.loads((ROOT/'reference/linkerhand_cpp_2_2_0/g20_left_raw_to_rad.json').read_text())
        self.assertEqual(rows['side'],'left');self.assertEqual(len(rows['raw_to_rad']),256)
        for raw,expected in enumerate(rows['raw_to_rad']):
            actual=self.m.decode([raw]*20)
            for n,e in self.m.joints.items():self.assertAlmostEqual(actual[n],expected[ACTIVE.index(e['slot'])],places=12)

    def test_all_protocol_counts_round_trip_and_reserved_fields_are_255(self):
        for raw in range(256):
            command=self.m.encode(self.m.decode([raw]*20))
            self.assertTrue(all(command[i]==raw for i in ACTIVE))
            self.assertTrue(all(command[i]==255 for i in (11,12,13,14)))

    def test_factory_ranges_and_roll_axis_match_lhg20_not_old_l20(self):
        self.assertEqual(self.m.limits['thumb_cmc_pitch'],(0.,.91))
        self.assertEqual(self.m.limits['index_mcp_pitch'],(0.,1.3))
        self.assertEqual(self.m.limits['index_pip'],(0.,1.42))
        self.assertEqual(self.m.decode([0]*20)['index_mcp_roll'],.26)
        self.assertEqual(self.m.decode([255]*20)['index_mcp_roll'],-.26)
        self.assertGreater(self.m.velocities['middle_pip'],0)
        self.assertLessEqual(self.m.velocities['middle_pip']*1.028,1.+1e-12)

    def test_bad_angles_and_raw_values_rejected(self):
        for v in (True,float('nan'),float('inf'),-1,3):
            q=self.open.copy();q['thumb_cmc_pitch']=v
            with self.assertRaises(ValueError):self.m.encode(q)
        for v in (True,float('nan'),-1,256,.5):
            q=self.baseline.copy();q[1]=v
            with self.assertRaises(ValueError):self.m.decode(q)

    def test_whole_hand_changes_16_axes_with_model_velocity_bound(self):
        q={n:(.1 if n.endswith('mcp_roll') else .3) for n in self.open}
        previous=self.baseline
        commands=[]
        for _ in range(100):
            target=self.feed(q)
            if target is None:continue
            self.assertLessEqual(self.m.step_interval(previous,target),.04+1e-9)
            previous=target;commands.append(target)
        self.assertEqual(commands[-1],self.m.encode(q))
        self.assertTrue(all(commands[-1][i]!=self.baseline[i] for i in ACTIVE))

    def test_no_ready_or_enable_step_and_loss_reacquires_automatically(self):
        self.assertIsNone(self.feed());self.assertIsNone(self.feed());self.assertIsNotNone(self.feed())
        last=self.engine.last_command.copy()
        self.assertIsNone(self.feed(valid=False));self.assertEqual(self.engine.last_command,last)
        self.assertIsNone(self.feed());self.assertIsNone(self.feed());self.assertIsNotNone(self.feed())

    def test_wrong_model_side_profile_source_or_targets_block_motion(self):
        cases=[{'hand':'right'},{'scope':'index'},{'model':'L20/left/linkerhand_l20_left.urdf'},
               {'model_sha256':'bad'},{'source_commit':'bad'},{'factory_profile_sha256':'bad'},
               {'motor_position_raw':[0]*20},{'operator_enabled':False},{'auto_follow':False},
               {'cad_collision_hits':[{'a':'index','b':'middle'}]},{'captured_monotonic_ns':1}]
        for change in cases:
            with self.subTest(change=change):
                self.engine=HandController(self.baseline,self.m)
                for _ in range(3):self.feed()
                self.assertIsNone(self.feed(**change));self.assertFalse(self.engine.active)

    def test_capture_timeout_and_out_of_order_frames_block_motion(self):
        for _ in range(3):self.feed()
        self.assertIsNone(self.engine.candidate(self.now+.3))
        for _ in range(3):self.feed()
        self.assertIsNone(self.feed(seq=self.seq-1))

    def test_new_vector_contact_request_reaches_bounded_path_audit_without_global_pause(self):
        q=self.open.copy();q['thumb_cmc_pitch']=.6;q['index_mcp_pitch']=.5
        hits=[{'a':'thumb_distal_0','b':'index_distal_0','kind':'cross_finger'}]
        for _ in range(3):
            target=self.feed(q,cad_collision_hits=hits,cad_validation='bridge_path_v1')
        self.assertTrue(self.engine.active)
        self.assertIsNotNone(target)
        self.assertLessEqual(self.m.step_interval(self.baseline,target),.04+1e-9)
        self.assertEqual(self.engine.desired,self.m.encode(q))
        self.assertNotEqual(target,self.engine.desired)
        # Candidates still require the separate worker's full-path approval;
        # accepting a distant endpoint is not authorization to publish it.
        self.assertIsNone(self.feed(q,cad_collision_hits=hits,cad_validation='skip'))

    def test_one_command_full_hand_routes_preview_selftest_and_automatic(self):
        for extra,entry in (([], 'execv'),(['--preview'],'execve'),(['--self-test'],'execve')):
            with patch.object(lh20_cli.os,entry,side_effect=SystemExit(0)) as execute:
                with self.assertRaises(SystemExit):lh20_cli.main(['vision','--hand','left','--scope','hand',*extra])
                cmd=execute.call_args.args[1]
                self.assertEqual(cmd[cmd.index('--scope')+1],'hand')
        with patch.object(lh20_cli.os,'execv',side_effect=SystemExit(0)) as execute:
            with self.assertRaises(SystemExit):lh20_cli.main(['vision','--hand','right','--scope','hand'])
            command=execute.call_args.args[1]
            self.assertEqual(command[command.index('--hand')+1],'right')
        self.assertEqual(lh20_cli.main(['vision','--hand','left','--scope','hand','--stream']),1)

    def test_existing_actual_left_pose_is_in_factory_limits(self):
        q=[1,252,254,238,254,236,239,153,55,76,252,0,0,0,0,183,255,255,255,255]
        engine=HandController(q,self.m)
        self.assertEqual(engine.last_command[0],q[0])
        self.assertGreater(self.m.decode(q)['thumb_cmc_pitch'],.79)


if __name__=='__main__':unittest.main()
