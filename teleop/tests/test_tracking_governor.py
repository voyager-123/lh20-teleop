"""Delayed hardware feedback and stalls; never imports ROS or opens CAN."""
from collections import deque
import json
from pathlib import Path
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'bridge'))
from factory_mapping import FactoryMapping,ACTIVE
from tracking_governor import FeedbackGovernor


class TrackingGovernorTests(unittest.TestCase):
    def setUp(self):
        self.g=FeedbackGovernor(FactoryMapping())
        self.q=[100]*20

    def test_target_waits_at_four_count_lead_without_disabling_stop_checks(self):
        previous=self.q.copy();candidate=self.q.copy();previous[1]=104;candidate[1]=105
        target,held=self.g.constrain(previous,candidate,self.q)
        self.assertEqual(target[1],104);self.assertEqual(held[0]['slot'],1)
        previous[1]=109;candidate[1]=110
        target,_=self.g.constrain(previous,candidate,self.q)
        self.assertEqual(target[1],109) # No jump of 5 counts back to the sensor.
        self.g.observe(self.q,previous,0.)
        with self.assertRaisesRegex(RuntimeError,'index_mcp_pitch.*slot 1.*8'):
            self.g.observe(self.q,previous,.76)

    def test_lagging_axis_does_not_prevent_other_axes_from_advancing(self):
        previous=self.q.copy();candidate=self.q.copy();previous[1]=104;candidate[1]=105;candidate[2]=101
        target,_=self.g.constrain(previous,candidate,self.q)
        self.assertEqual(target[1],104);self.assertEqual(target[2],101)

    def test_reversing_toward_feedback_remains_a_unit_step(self):
        previous=self.q.copy();candidate=self.q.copy();previous[1]=106;candidate[1]=105
        target,held=self.g.constrain(previous,candidate,self.q)
        self.assertEqual(target[1],105);self.assertFalse(held)

    def test_250ms_feedback_delay_and_slow_motor_do_not_accumulate_runaway_targets(self):
        actual=self.q.copy();reported=self.q.copy();command=self.q.copy();history=deque()
        waits=0
        for step in range(400):
            now=step*.01
            if step%8==0:
                for i in ACTIVE:actual[i]+=max(-1,min(1,command[i]-actual[i]))
            history.append((now,actual.copy()))
            while history and now-history[0][0]>=.25:reported=history.popleft()[1]
            self.g.observe(reported,command,now)
            if step%4==0:
                desired=command.copy()
                for i in ACTIVE:desired[i]+=int(desired[i]<160)
                command,held=self.g.constrain(command,desired,reported)
                waits+=bool(held)
                self.assertTrue(all(abs(command[i]-reported[i])<=4 for i in ACTIVE))
        self.assertGreater(waits,0);self.assertGreater(actual[1],125)

    def test_stalled_joint_error_names_the_axis_and_values(self):
        target=self.q.copy();target[15]=104
        self.g.observe(self.q,target,0.)
        with self.assertRaisesRegex(RuntimeError,'thumb_mcp.*slot 15.*104.*100'):
            self.g.observe(self.q,target,2.51)

    def test_jitter_does_not_count_as_repeated_forward_progress(self):
        target=self.q.copy();target[15]=105
        self.g.observe(self.q,target,0.)
        q=self.q.copy();q[15]=101;self.g.observe(q,target,.1)
        self.g.observe(self.q,target,1.)
        self.g.observe(q,target,2.)
        with self.assertRaisesRegex(RuntimeError,'thumb_mcp'):
            self.g.observe(q,target,2.61)



if __name__=='__main__':unittest.main()
