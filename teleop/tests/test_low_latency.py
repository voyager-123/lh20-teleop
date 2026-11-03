"""Speed regression, delayed fast motors and obstruction checks; no hardware."""
from collections import deque
from pathlib import Path
import ast
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'bridge'))
from factory_mapping import FactoryMapping,ACTIVE
from tracking_governor import LatencyAwareGovernor
from live_hand import HandController
import test_factory_hand as factory_tests


class LowLatencyTests(unittest.TestCase):
    def setUp(self):
        self.m=FactoryMapping();self.open={n:0. for n in self.m.joints}

    def test_06rad_step_finishes_under_08s_at_25hz_with_model_velocity_limits(self):
        helper=factory_tests.FactoryHandTests();helper.setUp()
        q=helper.open.copy();q['index_mcp_pitch']=.6
        start=helper.now;previous=helper.baseline
        max_delta=0
        for _ in range(20):
            target=helper.feed(q)
            if target is not None:
                max_delta=max(max_delta,abs(target[1]-previous[1]))
                self.assertLessEqual(self.m.step_interval(previous,target),.04+1e-9)
                previous=target
                if target==self.m.encode(q):break
        self.assertEqual(previous,self.m.encode(q))
        self.assertLessEqual(helper.now-start,.8)
        self.assertGreater(max_delta,1)

    def test_reacquisition_after_a_long_pause_has_no_large_jump(self):
        e=HandController(self.m.encode(self.open),self.m);e.active=True;e.last_input=10.;e.last_step=1.
        e.desired=self.m.encode({n:(.2 if n.endswith('mcp_roll') else .8) for n in self.open})
        target=e.candidate(10.)
        self.assertLessEqual(self.m.step_interval(e.last_command,target),.05+1e-9)

    def test_old_target_is_replaced_immediately_on_reverse_gesture(self):
        helper=factory_tests.FactoryHandTests();helper.setUp()
        bent=helper.open.copy();bent['index_mcp_pitch']=1.
        for _ in range(7):helper.feed(bent)
        before=helper.engine.last_command[1]
        target=helper.feed(helper.open)
        self.assertGreater(target[1],before)
        self.assertEqual(helper.engine.desired,self.m.encode(helper.open))

    def test_moving_feedback_retries_without_rearming_valid_vision(self):
        helper=factory_tests.FactoryHandTests();helper.setUp()
        for _ in range(3):helper.feed()
        previous=helper.engine.last_command.copy()
        helper.engine.recheck_feedback()
        self.assertTrue(helper.engine.active)
        self.assertEqual(helper.engine.last_command,previous)
        self.assertEqual(helper.engine.state,'CAD_RECHECKING_MOVING_FEEDBACK')
        # No stale command was committed. A new bounded candidate is available
        # immediately, but real vision loss still requires full reacquisition.
        self.assertIsNotNone(helper.engine.candidate(helper.now))
        self.assertIsNone(helper.feed(valid=False))
        self.assertIsNone(helper.feed());self.assertIsNone(helper.feed())
        self.assertIsNotNone(helper.feed())

    def test_60hz_vision_can_acquire_and_does_not_drop_back_to_detecting(self):
        helper=factory_tests.FactoryHandTests();helper.setUp()
        first_active=None
        for k in range(20):
            helper.now=100.+k/60;helper.seq=k
            helper.engine.accept(helper.packet(),helper.now)
            if helper.engine.active and first_active is None:first_active=k
            if first_active is not None:self.assertTrue(helper.engine.active)
        self.assertIsNotNone(first_active);self.assertLessEqual(first_active,5)
        self.assertIsNone(helper.engine.candidate(helper.now+.3))

    def test_250ms_old_readback_does_not_throttle_fast_model_speed_motion(self):
        g=LatencyAwareGovernor(self.m);command=self.m.encode(self.open);actual=command.copy();reported=command.copy()
        history=deque();q=self.open.copy();q['index_mcp_pitch']=.6;desired=self.m.encode(q)
        steps=0
        for k in range(35):
            now=k*.04
            history.append((now,actual.copy()))
            while len(history)>1 and history[1][0]<=now-.25:history.popleft()
            reported=history[0][1]
            g.observe(reported,command,now)
            candidate=command.copy();candidate[1]+=max(-7,min(7,desired[1]-command[1]))
            command,held=g.constrain(command,candidate,reported)
            actual=command.copy();steps+=1
            self.assertFalse(any(item['slot']==1 for item in held))
            if command==desired:break
        self.assertEqual(command,desired);self.assertLessEqual(steps*.04,.8)

    def test_slow_motor_uses_bounded_angular_lead_and_keeps_making_progress(self):
        g=LatencyAwareGovernor(self.m);command=[200]*20;feedback=[200]*20;held_any=False
        for k in range(300):
            now=k*.02
            if k%5==0:
                for i in ACTIVE:feedback[i]+=max(-1,min(1,command[i]-feedback[i]))
            g.observe(feedback,command,now)
            candidate=command.copy()
            for i in ACTIVE:candidate[i]=max(100,candidate[i]-5)
            command,held=g.constrain(command,candidate,feedback);held_any|=bool(held)
            self.assertTrue(all(abs(command[i]-feedback[i])<=g.lead_limits[i] for i in ACTIVE))
        self.assertTrue(held_any)
        self.assertLess(feedback[1],160)

    def test_motor_that_does_not_move_still_stops_with_joint_diagnostic(self):
        g=LatencyAwareGovernor(self.m);command=[200]*20;feedback=[200]*20
        with self.assertRaisesRegex(RuntimeError,'slot'):
            for k in range(180):
                now=k*.02;g.observe(feedback,command,now)
                candidate=command.copy();candidate[1]=max(100,candidate[1]-5)
                command,_=g.constrain(command,candidate,feedback)

    def test_sdk_full_command_subscription_keeps_only_latest_pose(self):
        p=ROOT.parent/'src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand.py'
        tree=ast.parse(p.read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_subscription']
        cmd=next(n for n in calls if len(n.args)>2 and isinstance(n.args[2],ast.Attribute) and n.args[2].attr=='hand_control_cb')
        self.assertEqual(ast.literal_eval(cmd.args[3]),1)


if __name__=='__main__':unittest.main()
