"""Thumb A/B wiring and per-axis stall isolation; no hardware access."""
import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'bridge'))
from factory_mapping import FactoryMapping,ACTIVE
from tracking_governor import LatencyAwareGovernor
from live_hand import HandController
import lh20_cli


class ThumbHoldTests(unittest.TestCase):
    def test_manufacturer_ros20_thumb_frame_order_is_horizontal_then_side(self):
        path=ROOT.parent/'src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_g20_can.py'
        tree=ast.parse(path.read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='cmd_range_to_joint_range')
        namespace={};exec(compile(ast.Module(body=[method],type_ignores=[]),'<manufacturer mapping>','exec'),namespace)
        frame=namespace['cmd_range_to_joint_range'](object(),list(range(20)))[0]
        self.assertEqual(frame,[10,5,0,11,12,15])

    def test_native_and_swapped_change_only_the_two_thumb_channels(self):
        native,swapped=FactoryMapping(),FactoryMapping('swapped')
        for raw in range(256):
            q=native.decode([raw]*20);q['thumb_cmc_yaw']=.2;q['thumb_cmc_roll']=.8
            a,b=native.encode(q),swapped.encode(q)
            self.assertEqual(b[5],a[10]);self.assertEqual(b[10],a[5])
            self.assertTrue(all(a[i]==b[i] for i in range(20) if i not in (5,10)))
            for name,value in native.decode(a).items():self.assertAlmostEqual(value,swapped.decode(b)[name])
        self.assertNotEqual(native.sha256,swapped.sha256)
        self.assertEqual(native.factory_sha256,swapped.factory_sha256)

    def test_each_native_thumb_axis_controls_its_documented_slot(self):
        m=FactoryMapping();q={n:0. for n in m.joints};baseline=m.encode(q)
        for name,slot in [('thumb_cmc_roll',5),('thumb_cmc_yaw',10)]:
            a=q.copy();a[name]=.3;target=m.encode(a)
            self.assertEqual([i for i in range(20) if target[i]!=baseline[i]],[slot])

    def test_stationary_position_64_isolates_one_axis_without_exiting(self):
        m=FactoryMapping();g=LatencyAwareGovernor(m,isolate_stalls=True)
        feedback=[100]*20;target=feedback.copy();feedback[8]=64;target[8]=0
        for k in range(160):g.observe(feedback,target,k*.01)
        self.assertEqual(set(g.isolated),{8})
        desired=target.copy();desired[1]=120
        adjusted=g.adapt_desired(desired)
        self.assertEqual(adjusted[8],64);self.assertEqual(adjusted[1],120)
        self.assertTrue(any(e['event']=='held' and e['slot']==8 for e in g.drain_events()))
        g.reset();self.assertIn(8,g.isolated) # Lost vision cannot re-enable the same push.
        desired[8]=100;self.assertEqual(g.adapt_desired(desired)[8],100)
        self.assertNotIn(8,g.isolated)

    def test_isolated_axis_retreat_is_still_model_velocity_limited(self):
        m=FactoryMapping();baseline=m.encode({n:0. for n in m.joints});baseline[8]=0
        e=HandController(baseline,m);e.active=True;e.last_input=1.
        adjusted=e.desired.copy();adjusted[8]=64
        target=e.candidate(1.,adjusted)
        self.assertGreater(target[8],0);self.assertLess(target[8],64)
        self.assertLessEqual(m.step_interval(e.last_command,target),1/30+1e-9)

    def test_swapped_cli_reaches_automatic_session_and_camera_preview(self):
        for extra,method in (([],'execv'),(['--preview'],'execve')):
            with patch.object(lh20_cli.os,method,side_effect=SystemExit(0)) as run:
                with self.assertRaises(SystemExit):lh20_cli.main(['vision','--hand','left','--scope','hand','--thumb-axes','swapped',*extra])
                command=run.call_args.args[1];self.assertEqual(command[command.index('--thumb-axes')+1],'swapped')
        self.assertEqual(lh20_cli.main(['vision','--hand','left','--thumb-axes','swapped']),1)

    def test_palm_profile_reaches_both_camera_and_automatic_session(self):
        for extra,method in (([],'execv'),(['--preview'],'execve'),(['--self-test'],'execve')):
            with patch.object(lh20_cli.os,method,side_effect=SystemExit(0)) as run:
                with self.assertRaises(SystemExit):
                    lh20_cli.main(['vision','--hand','left','--scope','hand','--thumb-axes','swapped','--thumb-profile','palm',*extra])
                command=run.call_args.args[1]
                self.assertEqual(command[command.index('--thumb-profile')+1],'palm')
        self.assertEqual(lh20_cli.main(['vision','--hand','left','--thumb-profile','palm']),1)


if __name__=='__main__':unittest.main()
