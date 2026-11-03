"""Offline tests for bounds, causal timestamps, data integrity and stop behavior."""
import argparse
import ast
import copy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from study_core import (Journal, causal_pairs, guard_feedback, plan_rows, positions,
                        pose_matches, read_events, trajectory, validate_trial)
from study_analysis import annotate, apparent_lag, load_session, session_summary
import study
import study_live


class CoreTests(unittest.TestCase):
    def test_trajectory_moves_only_selected_slot_in_single_counts(self):
        base = [200] * 20
        for slot in (1, 16):
            for delta in (-20, -5, 5, 20):
                path = trajectory(base, slot, delta)
                self.assertEqual(path[-1][slot], 200 + delta)
                for a, b in zip(path, path[1:]):
                    self.assertEqual([i for i in range(20) if a[i] != b[i]], [slot])
                    self.assertEqual(abs(a[slot] - b[slot]), 1)
        self.assertEqual(base, [200] * 20)

    def test_invalid_amplitude_bounds_and_nonfinite_values_are_rejected(self):
        for slot, delta in ((0, 5), (1, 0), (1, 21), (16, True)):
            with self.assertRaises(ValueError):
                trajectory([200] * 20, slot, delta)
        with self.assertRaises(ValueError):
            trajectory([0] * 20, 1, -1)
        for value in (float("nan"), float("inf"), True, .5, -1, 256):
            with self.assertRaises(ValueError):
                positions([value] + [200] * 19)
        for speed in (float("nan"), 0, 21):
            with self.assertRaises(ValueError):
                validate_trial(1, -5, speed, 1, .4, 8)

    def test_larger_excursion_is_allowed_only_for_free_motion_and_keeps_other_bounds(self):
        validate_trial(1, -60, 10, 1, .4, 8, condition="free")
        path = trajectory([254] * 20, 1, -60, condition="free")
        self.assertEqual(len(path), 61)
        self.assertEqual(path[-1][1], 194)
        for before, after in zip(path, path[1:]):
            self.assertEqual([i for i in range(20) if before[i] != after[i]], [1])
            self.assertEqual(before[1] - after[1], 1)
        for condition, delta in (("contact", -21), ("contact", -60), ("free", -61), ("unknown", -5)):
            with self.assertRaises(ValueError):
                validate_trial(1, delta, 10, 1, .4, 8, condition=condition)
            with self.assertRaises(ValueError):
                trajectory([254] * 20, 1, delta, condition=condition)
        with self.assertRaises(ValueError):
            trajectory([30] * 20, 1, -60, condition="free")

    def test_fault_exceptions_do_not_bypass_stale_feedback_or_combined_faults(self):
        now = 5_000_000_000
        latest = {"q": [200] * 20, "state_ns": now, "info_ns": now,
                  "info": {"hand_joint": "L20-10", "version": [1, 0, 6], "fault": [0] * 20}}
        for code in (0, 64, 128):
            latest["info"]["fault"][5] = code
            guard_feedback(latest, now, [1, 0, 6], "ignore-64-128")
        for code in (65, 129, 192, 255):
            latest["info"]["fault"][5] = code
            with self.assertRaises(RuntimeError):
                guard_feedback(latest, now, [1, 0, 6], "ignore-64-128")
        latest["info"]["fault"][5] = 64
        with self.assertRaises(RuntimeError):
            guard_feedback(latest, now + 300_000_001, [1, 0, 6], "ignore-64-128")

    def test_fixed_anchor_does_not_allow_cumulative_drift(self):
        original = [200] * 20
        self.assertTrue(pose_matches(original, [202] * 20))
        self.assertFalse(pose_matches(original, [203] * 20))
        self.assertEqual(original, [200] * 20)

    def test_causal_join_never_uses_a_future_command_or_own_loopback_timestamp(self):
        events = [
            {"kind": "state", "mono_ns": 9, "t_s": .009, "valid": True, "position_raw": [200] * 20},
            {"kind": "command_sent", "mono_ns": 10, "t_s": .010, "slot_values": {"1": 198}},
            {"kind": "command_echo", "mono_ns": 11, "t_s": .011, "slot_values": {"1": 198}},
            {"kind": "state", "mono_ns": 12, "t_s": .012, "valid": True, "position_raw": [200] * 20},
            {"kind": "command_sent", "mono_ns": 20, "t_s": .020, "slot_values": {"1": 195}},
        ]
        pairs = causal_pairs(list(reversed(events)), 1)
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0]["command_raw"], 198)
        self.assertEqual(pairs[0]["command_t_s"], .010)
        self.assertEqual(pairs[0]["error_raw"], -2)

    def test_journal_retains_timestamps_and_tolerates_only_a_truncated_tail(self):
        with tempfile.TemporaryDirectory() as temp:
            j = Journal(temp, {"mode": "probe", "trial_id": "a", "slot": 1}, start_ns=100)
            j.write("marker", mono_ns=200, label="test")
            j.close("complete", end_ns=300)
            events, warnings = read_events(j.path)
            self.assertFalse(warnings)
            self.assertEqual([e["mono_ns"] for e in events], [200, 300])
            with (j.path / "events.jsonl").open("a") as stream:
                stream.write('{"interrupted":')
            events, warnings = read_events(j.path)
            self.assertEqual(len(events), 2)
            self.assertTrue(warnings)
            with (j.path / "events.jsonl").open("a") as stream:
                stream.write("\n")
            with self.assertRaises(ValueError):
                read_events(j.path)

    def test_plan_is_balanced_and_changes_pair_order(self):
        rows = plan_rows(5)
        self.assertEqual(len(rows), 30)
        self.assertEqual(len({r["trial_id"] for r in rows}), 30)
        for speed in (5, 10, 20):
            for condition in ("free", "contact"):
                self.assertEqual(sum(r["condition"] == condition and r["speed_raw_s"] == speed for r in rows), 5)
        self.assertEqual(rows[0]["condition"], "free")
        self.assertEqual(rows[6]["condition"], "contact")


class PipelineTests(unittest.TestCase):
    def test_synthetic_pipeline_and_annotations_preserve_raw_recording(self):
        with tempfile.TemporaryDirectory() as temp:
            paths = study.generate_demo(Path(temp) / "demo")
            self.assertEqual(len(paths), 12)
            free = load_session(next(p for p in paths if "free" in p.name))
            contact_path = next(p for p in paths if "contact" in p.name)
            contact = load_session(contact_path)
            self.assertTrue(free["meta"]["synthetic"])
            self.assertIsNotNone(apparent_lag(free))
            self.assertIsNone(apparent_lag(contact))
            self.assertGreater(session_summary(contact)["error_abs_p95_raw"], session_summary(free)["error_abs_p95_raw"])
            raw = (contact_path / "events.jsonl").read_bytes()
            annotate(contact_path, .5, 1, "uncertain", "video", "rough interval")
            self.assertEqual((contact_path / "events.jsonl").read_bytes(), raw)
            self.assertEqual(len(load_session(contact_path)["annotations"]), 1)
            with self.assertRaises(ValueError):
                annotate(contact_path, -1, 999, "contact", "video", "bad")

    def test_batch_requires_explicit_return_and_valid_unique_trials(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "plan.csv"
            study.main(["plan", "--output", str(path), "--repeats", "1"])
            args = study.parser().parse_args(["run-plan", str(path), "--anchor", "anchor.json"])
            with self.assertRaises(ValueError):
                study.read_plan(args)
            args.return_to_start = True
            self.assertEqual(len(study.read_plan(args)), 6)

    def test_passive_record_never_enables_a_motion_publisher(self):
        with tempfile.TemporaryDirectory() as temp:
            fake = Mock()
            fake.bus = None
            args = SimpleNamespace(channel="can0", no_can_log=True, data_dir=Path(temp),
                                   trial_id="read", condition="unknown", object_id="none", slot=1,
                                   note="test", duration=0, fault_policy="stop")
            with patch.object(study_live, "LiveLink", return_value=fake), \
                 patch.object(study_live, "mapping_instance", return_value=object()):
                session = study_live.record(args)
            fake.enable_publisher.assert_not_called()
            fake.publisher.publish.assert_not_called()
            self.assertFalse(json.loads((session / "metadata.json").read_text())["motion_publisher_created"])

    def test_no_direct_can_transmission_in_data_acquisition_module(self):
        tree = ast.parse(Path(study_live.__file__).read_text())
        can_send_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                          and isinstance(n.func, ast.Attribute) and n.func.attr in ("send", "send_periodic")]
        self.assertEqual(can_send_calls, [])

    def test_preflight_failure_never_enables_publisher_and_keeps_failure_log(self):
        with tempfile.TemporaryDirectory() as temp:
            args = SimpleNamespace(slot=1, delta=-5, speed=5, pre=.2, post=.1, max_gap=8,
                                   trial_id="blocked", condition="free", object_id="none",
                                   return_to_start=False, fault_policy="stop", channel="can0",
                                   anchor="anchor.json", note="", data_dir=Path(temp))
            link = Mock()
            link.bus = None
            link.latest = {"q": [200] * 20, "info": {}}
            link.graph.return_value = {}
            link.guard.side_effect = RuntimeError("stale feedback")
            with patch.object(study_live, "audit_plan", return_value=trajectory([200] * 20, 1, -5)):
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    study_live.one_trial(args, link, Mock(), SimpleNamespace(sha256="a", model_hash="b"), {}, "test", Mock())
            link.enable_publisher.assert_not_called()
            meta = json.loads(next(Path(temp).rglob("metadata.json")).read_text())
            self.assertEqual(meta["status"], "aborted")


class FakeClock:
    def __init__(self):
        self.value = 100.

    def monotonic(self):
        return self.value

    def monotonic_ns(self):
        return int(self.value * 1e9)


class FakeLink:
    """In-memory fake, no rclpy initialization, topic graph, or CAN sockets."""
    def __init__(self, clock, follows=True):
        self.clock, self.follows = clock, follows
        self.bus = None
        self.journal = self.publisher = None
        self.sent = []
        self.latest = {"q": [200] * 20, "state_ns": clock.monotonic_ns(), "info_ns": clock.monotonic_ns(),
                       "info": {"hand_joint": "L20-10", "version": [1, 0, 6], "fault": [0] * 20}}
        self.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
        self.JointState = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None), name=[], position=[])

    def graph(self):
        return {}

    def ready(self, *_):
        self.pump()

    def pump(self, *_):
        self.clock.value += .02
        if self.sent and self.follows:
            for i, value in zip((6, 1, 16), self.sent[-1]):
                self.latest["q"][i] = int(value)
        self.latest["state_ns"] = self.latest["info_ns"] = self.clock.monotonic_ns()

    def guard(self, mapping, policy):
        guard_feedback(self.latest, self.clock.monotonic_ns(), mapping.profile["firmware_version"], policy)

    def enable_publisher(self):
        self.publisher = SimpleNamespace(publish=lambda message: self.sent.append(list(message.position)))


class FakeKeys:
    def __init__(self, link, contact=False, abort=False):
        self.link, self.contact, self.abort = link, contact, abort
        self.calls = 0

    def read(self):
        self.calls += 1
        if self.calls == 1:
            return ["\n"]
        if self.abort and self.link.sent:
            return ["q"]
        if self.contact and self.link.sent and self.link.sent[-1][1] <= 198:
            self.contact = False
            return ["c"]
        return []


class TrialLifecycleTests(unittest.TestCase):
    def exercise(self, temp, follows=True, contact=False, abort=False, stale_after_cad=False, gap=8):
        clock = FakeClock()
        link = FakeLink(clock, follows)
        args = SimpleNamespace(slot=1, delta=-5, speed=5, pre=.2, post=.1, max_gap=gap,
                               trial_id="memory", condition="contact" if contact else "free", object_id="test",
                               return_to_start=True, fault_policy="stop", channel="can0", anchor="test.json",
                               note="in-memory test", data_dir=Path(temp))
        mapping = SimpleNamespace(sha256="a", model_hash="b", profile={"firmware_version": [1, 0, 6]})
        def check(before, after, pump=None):
            if pump:
                pump()
            if stale_after_cad:
                link.latest["state_ns"] -= 500_000_000
            return {"ok": True}
        with patch.object(study_live.time, "monotonic", clock.monotonic), \
             patch.object(study_live.time, "monotonic_ns", clock.monotonic_ns), \
             patch.object(study_live, "audit_plan", return_value=trajectory([200] * 20, 1, -5)):
            try:
                study_live.one_trial(args, link, SimpleNamespace(check=check), mapping, {}, "test", FakeKeys(link, contact, abort))
            except (RuntimeError, KeyboardInterrupt) as error:
                failure = error
            else:
                failure = None
        path = next(Path(temp).rglob("metadata.json")).parent
        return link, load_session(path), failure

    def test_successful_round_trip_keeps_other_axes_and_unit_step_bounds(self):
        with tempfile.TemporaryDirectory() as temp:
            link, session, failure = self.exercise(temp)
            self.assertIsNone(failure)
            self.assertEqual(session["meta"]["status"], "complete")
            commands = [e for e in session["events"] if e["kind"] == "command_sent"]
            self.assertTrue(any(e["phase"] == "return" for e in commands))
            self.assertEqual(link.sent[-1], [200, 200, 200])
            for row in link.sent:
                self.assertEqual([row[0], row[2]], [200, 200])
            self.assertTrue(all(abs(a[1] - b[1]) <= 1 for a, b in zip(link.sent, link.sent[1:])))

    def test_contact_mark_stops_advancing_and_only_requested_return_follows(self):
        with tempfile.TemporaryDirectory() as temp:
            link, session, failure = self.exercise(temp, contact=True)
            self.assertIsNone(failure)
            self.assertGreaterEqual(min(row[1] for row in link.sent), 198)
            self.assertEqual(link.sent[-1][1], 200)
            self.assertEqual(session_summary(session)["operator_contact_marks"], 1)

    def test_gap_limit_aborts_without_automatic_return(self):
        with tempfile.TemporaryDirectory() as temp:
            link, session, failure = self.exercise(temp, follows=False, gap=2)
            self.assertIsInstance(failure, RuntimeError)
            self.assertEqual(session["meta"]["status"], "aborted")
            self.assertFalse(any(e.get("phase") == "return" for e in session["events"]))
            self.assertTrue(all(row[1] >= 198 for row in link.sent))

    def test_operator_abort_does_not_trigger_return(self):
        with tempfile.TemporaryDirectory() as temp:
            _, session, failure = self.exercise(temp, abort=True)
            self.assertIsInstance(failure, KeyboardInterrupt)
            self.assertFalse(any(e.get("phase") == "return" for e in session["events"]))

    def test_freshness_rechecked_after_cad_before_first_publish(self):
        with tempfile.TemporaryDirectory() as temp:
            link, session, failure = self.exercise(temp, stale_after_cad=True)
            self.assertIsInstance(failure, RuntimeError)
            self.assertEqual(link.sent, [])
            self.assertEqual(session["meta"]["status"], "aborted")


if __name__ == "__main__":
    unittest.main()
