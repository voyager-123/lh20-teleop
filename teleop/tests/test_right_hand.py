"""Right-side isolation, routing and first-use identity tests; no hardware."""
import ast
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'teleop/bridge'))
from factory_mapping import FactoryMapping,ACTIVE
from live_hand import HandController
from live_index import SOURCE_COMMIT
import auto_vision
import lh20_cli


class RightHandTests(unittest.TestCase):
    def setUp(self):
        self.m=FactoryMapping(hand='right')
        self.q={n:0. for n in self.m.joints}
        self.baseline=self.m.encode(self.q)

    def packet(self,now,seq=0,**extra):
        row={'schema':'lh20.retarget.v1','hand':'right','scope':'hand','model':self.m.model_id,
             'model_sha256':self.m.model_hash,'source_commit':SOURCE_COMMIT,'stream_session':'b'*32,
             'seq':seq,'calibration_epoch':0,'captured_monotonic_ns':int(now*1e9),
             'valid':True,'operator_enabled':True,'auto_follow':True,'cad_collision_hits':[],
             'joint_names':list(self.q),'position_rad':list(self.q.values()),
             'motor_position_raw':self.m.encode(self.q),'factory_profile_sha256':self.m.sha256}
        row.update(extra);return row

    def test_mapping_has_own_model_ranges_hash_and_native_thumb_order(self):
        left=FactoryMapping('swapped')
        self.assertEqual(self.m.hand,'right')
        self.assertNotEqual(self.m.model_id,left.model_id)
        self.assertNotEqual(self.m.model_hash,left.model_hash)
        self.assertNotEqual(self.m.sha256,left.sha256)
        self.assertEqual(self.m.joints['thumb_cmc_roll']['slot'],5)
        self.assertEqual(self.m.joints['thumb_cmc_yaw']['slot'],10)
        self.assertEqual(self.m.limits['thumb_cmc_pitch'],(0.,.7))
        self.assertEqual(self.m.limits['index_mcp_roll'],(-.23,.23))
        self.assertEqual(left.joints['thumb_cmc_roll']['slot'],10)
        for speed in self.m.velocities.values():self.assertGreater(speed,0)

    def test_spread_feedback_and_vendor_endpoints(self):
        # Read-only feedback after the operator opened the paired right hand.
        raw=[255,223,254,254,252,179,254,168,5,0,254,0,0,0,0,254,249,255,255,253]
        engine=HandController(raw,self.m)
        for slot in ACTIVE:self.assertEqual(engine.last_command[slot],raw[slot])
        for name in ('index_mcp_roll','middle_mcp_roll','ring_mcp_roll','pinky_mcp_roll'):
            for outside in (-.231,.231):
                q=self.q.copy();q[name]=outside
                with self.assertRaisesRegex(ValueError,'outside factory/model'):self.m.encode(q)
        self.assertEqual(self.m.limits['thumb_cmc_roll'],(0.,1.22))
        self.assertEqual(self.m.limits['thumb_mcp'],(0.,1.05))

    def test_right_swapped_adapter_changes_only_thumb_channels_and_round_trips(self):
        swapped=FactoryMapping('swapped',hand='right')
        q={n:0. for n in self.m.joints}
        q['thumb_cmc_roll']=.3;q['thumb_cmc_yaw']=.8
        native_raw=self.m.encode(q);swapped_raw=swapped.encode(q)
        self.assertEqual(swapped_raw[5],native_raw[10])
        self.assertEqual(swapped_raw[10],native_raw[5])
        for slot in range(20):
            if slot not in (5,10):self.assertEqual(swapped_raw[slot],native_raw[slot])
        for name,value in self.m.decode(native_raw).items():
            self.assertAlmostEqual(swapped.decode(swapped_raw)[name],value)
        self.assertNotEqual(swapped.sha256,self.m.sha256)
        self.assertEqual(swapped.factory_sha256,self.m.factory_sha256)
        engine=HandController(swapped_raw,swapped)
        for k in range(3):
            now=100.+k*.04
            row=self.packet(now,k,thumb_axes='swapped',factory_profile_sha256=swapped.sha256,
                position_rad=list(q.values()),joint_names=list(q),motor_position_raw=swapped_raw)
            engine.accept(row,now)
        self.assertTrue(engine.active)
        engine.accept(self.packet(100.12,3),100.12)
        self.assertFalse(engine.active)  # A native packet cannot cross adapters.

    def test_right_round_trips_valid_raw_counts_and_reserved_channels(self):
        for n,e in self.m.joints.items():
            low,high=self.m.raw_limits[n]
            for raw in range(low,high+1):
                before=self.baseline.copy();before[e['slot']]=raw
                after=self.m.encode(self.m.decode(before))
                self.assertEqual(after[e['slot']],raw)
                self.assertTrue(all(after[i]==255 for i in (11,12,13,14)))

    def test_right_controller_acquires_and_rejects_wrong_side_model_profile(self):
        for wrong in ({'hand':'left'},{'model':FactoryMapping().model_id},{'factory_profile_sha256':FactoryMapping().sha256}):
            e=HandController(self.baseline,self.m)
            for k in range(3):e.accept(self.packet(100+k*.04,k),100+k*.04)
            self.assertTrue(e.active)
            e.accept(self.packet(100.12,3,**wrong),100.12)
            self.assertFalse(e.active)
            self.assertEqual(e.last_command,self.baseline)

    def test_right_slew_loss_and_reverse_keep_existing_bounds(self):
        e=HandController(self.baseline,self.m)
        self.q['index_mcp_pitch']=.7
        previous=self.baseline
        for k in range(20):
            now=100+k*.04;e.accept(self.packet(now,k),now)
            target=e.candidate(now)
            if target is not None:
                self.assertLessEqual(self.m.step_interval(previous,target),.04+1e-9)
                e.commit(target,now);previous=target
        self.assertLess(previous[1],self.baseline[1])
        self.q['index_mcp_pitch']=0.
        e.accept(self.packet(100.8,20),100.8)
        self.assertGreater(e.candidate(100.8)[1],previous[1])
        self.assertIsNone(e.candidate(101.1))

    def test_cli_right_full_hand_routes_auto_preview_and_selftest(self):
        for extra,method in (([],'execv'),(['--preview'],'execve'),(['--self-test'],'execve')):
            with patch.object(lh20_cli.os,method,side_effect=SystemExit(0)) as run:
                with self.assertRaises(SystemExit):
                    lh20_cli.main(['vision','--hand','right','--scope','hand',*extra])
                command=run.call_args.args[1]
                self.assertEqual(command[command.index('--hand')+1],'right')
                self.assertEqual(command[command.index('--scope')+1],'hand')
                self.assertEqual(command[command.index('--thumb-root-mode')+1],'agile')
                self.assertEqual(command[command.index('--thumb-axes')+1],'native')

    def test_expired_frame_after_cad_pauses_the_correct_side(self):
        # Execute the actual post-CAD expiration branch without ROS or CAN.
        tree=ast.parse((ROOT/'teleop/bridge/live_hand.py').read_text())
        run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        branch=next(n for n in ast.walk(run) if isinstance(n,ast.If)
                    and ast.unparse(n.test)=='now - engine.last_input > 0.25')
        for side in ('left','right'):
            paused=[]
            namespace={'now':100.3,'args':SimpleNamespace(hand=side),
                       'engine':SimpleNamespace(last_input=100.,pause=paused.append)}
            exec(compile(ast.Module(body=[branch],type_ignores=[]),'<real post-CAD stale frame branch>','exec'),namespace)
            self.assertEqual(paused,['WAITING_FOR_'+side.upper()+'_HAND'])

    def test_right_fixed_entry_short_vector_dexpilot_and_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);shutil.copy2(ROOT/'lh20-right',folder/'lh20-right')
            stub=folder/'lh20';stub.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n');stub.chmod(0o755)
            for option in ([],['vector'],['dexpilot'],['--thumb-root-mode','directions']):
                result=subprocess.run([str(folder/'lh20-right'),'vision',*option],capture_output=True,text=True,check=True)
                command=json.loads(result.stdout)
                self.assertEqual(command[command.index('--hand')+1],'right')
                self.assertEqual(command[command.index('--scope')+1],'hand')
                self.assertEqual(command[command.index('--thumb-axes')+1],'swapped')
                if option and option[0] in ('vector','dexpilot'):
                    self.assertEqual(command[command.index('--retargeting')+1],option[0])
            override=subprocess.run([str(folder/'lh20-right'),'vision','--thumb-axes','native'],capture_output=True,text=True,check=True)
            # argparse uses the final explicit adapter, after the entry default.
            with patch.object(lh20_cli.os,'execv',side_effect=SystemExit(0)) as run:
                with self.assertRaises(SystemExit):lh20_cli.main(json.loads(override.stdout))
                forwarded=run.call_args.args[1]
                self.assertEqual(forwarded[forwarded.index('--thumb-axes')+1],'native')
            result=subprocess.run([str(folder/'lh20-right'),'vision','--hand','left'],capture_output=True)
            self.assertEqual(result.returncode,2)

    def test_right_authorized_fault_policy_keeps_raw_and_other_guards(self):
        from probe_index_joint import feedback_errors
        for hand in ('left','right'):
            self.assertEqual(lh20_cli.selected_fault_policy(hand),'ignore-64-128')
            self.assertEqual(lh20_cli.selected_fault_policy(hand,'stop'),'stop')
        policy=lh20_cli.selected_fault_policy('right')
        now=100.
        state={'q':[100]*20,'state_at':now,'info_at':now,
               'info':{'hand_joint':'L20-10','fault':[0]*20}}
        flags={'ignore_128':policy in ('ignore-128','ignore-64-128'),
               'ignore_64':policy=='ignore-64-128'}
        for code in (64,128):
            state['info']['fault'][5]=code;state['info']['fault'][10]=code
            self.assertEqual(feedback_errors(state,now,**flags),[])
            self.assertEqual(state['info']['fault'][5],code)
            self.assertTrue(feedback_errors(state,now))  # Explicit stop policy.
            self.assertTrue(feedback_errors(state,now+.31,**flags))
        for code in (1,32,65,129,192,255):
            state['info']['fault'][5]=code
            self.assertTrue(feedback_errors(state,now,**flags))

    def test_identity_enrolls_right_once_and_rejects_left_or_device_swap(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(auto_vision,'ROOT',Path(tmp)),patch.dict(auto_vision.os.environ,{'LH20_RIGHT_EXPECTED_IDENTIFIER':''}):
            uid='LHT20-010-099-R-Z-1-A'
            path=Path(tmp)/'teleop/data/right_device_identity.json'
            with self.assertRaises(RuntimeError):auto_vision.confirm_right_identity(uid.replace('-R-','-L-'))
            self.assertFalse(path.exists())
            self.assertEqual(auto_vision.confirm_right_identity(uid),uid)
            initial=path.read_bytes()
            self.assertEqual(auto_vision.confirm_right_identity(uid),uid)
            with self.assertRaises(RuntimeError):auto_vision.confirm_right_identity(uid.replace('099','098'))
            self.assertEqual(path.read_bytes(),initial)

    def test_no_identity_reply_never_starts_sdk_or_controller(self):
        args=SimpleNamespace(hand='right',scope='hand')
        with patch.object(lh20_cli,'check_can'),patch.object(auto_vision,'read_identity',side_effect=RuntimeError('no reply')),patch.object(auto_vision.subprocess,'Popen') as start:
            with self.assertRaisesRegex(RuntimeError,'no reply'):auto_vision.run(args)
            start.assert_not_called()

    def test_auto_right_uses_only_right_topics_sdk_bridge_and_recordings(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp)/'teleop').mkdir()
            commands=[]
            def popen(command,**kwargs):
                commands.append(command)
                if any('live_hand.py' in part for part in command):
                    Path(command[command.index('--socket')+1]).touch()
                return SimpleNamespace(poll=lambda:0 if command[0]=='fake-vision' else None,returncode=0)
            counts={name:0 for name in ('/cb_right_hand_state','/cb_right_hand_info','/cb_right_hand_index_control_cmd','/cb_right_hand_control_cmd','/cb_hand_setting_cmd')}
            args=SimpleNamespace(hand='right',scope='hand',thumb_axes='native',retargeting='vector',record=None)
            with patch.object(auto_vision,'ROOT',Path(tmp)),patch.object(lh20_cli,'check_can'),patch.object(auto_vision,'read_identity',return_value='LHT20-010-099-R-Z-1-A') as read,patch.object(auto_vision,'confirm_right_identity'),patch.object(auto_vision,'discover',return_value=counts) as discover,patch.object(lh20_cli,'vision_command',return_value=(['fake-vision'],{})),patch.object(auto_vision.subprocess,'Popen',side_effect=popen),patch.object(auto_vision,'stop_owned'):
                self.assertEqual(auto_vision.run(args),0)
            read.assert_called_once_with(hand='right');discover.assert_called_once_with(hand='right')
            self.assertTrue(commands[0][0].endswith('/lh20-right'))
            bridge=commands[1]
            self.assertEqual(bridge[bridge.index('--hand')+1],'right')
            self.assertEqual(bridge[bridge.index('--fault-policy')+1],'ignore-64-128')
            self.assertIn('lh20-auto-right-',bridge[bridge.index('--socket')+1])
            self.assertTrue(args.record.name.startswith('vision_right_vector_'))


if __name__=='__main__':unittest.main()
