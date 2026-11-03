"""Driver scheduling and publication regression with a virtual clock; no ROS/CAN."""
import ast
from array import array
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
SDK = ROOT / 'src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk'
spec = importlib.util.spec_from_file_location('g20_io', SDK / 'LinkerHand/utils/g20_io.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
G20IoLoop = module.G20IoLoop


class Clock:
    def __init__(self):
        self.now = 100.
        self.events = []

    def __call__(self):
        return self.now

    def sleep(self, value):
        self.now += value


class API:
    COSTS = {'finger_move': .015, 'get_state': .017, 'get_speed': .017,
             'get_fault': .015, 'get_current': .018, 'get_temperature': .015,
             'get_torque': .018, 'get_joint_speed': .017, 'set_index_positions': .003}

    def __init__(self, clock):
        self.clock = clock
        self.hand = self
        self.fault = [0] * 20

    def get_finger_order(self):
        return ['joint'] * 20

    def __getattr__(self, name):
        if name not in self.COSTS:
            raise AssertionError('Unexpected device operation: ' + name)
        def call(*args, **kwargs):
            self.clock.events.append((self.clock(), name, args, kwargs))
            self.clock.sleep(self.COSTS[name])
            return self.fault.copy() if name == 'get_fault' else [1] * 20
        return call


def fixture():
    clock = Clock()
    node = SimpleNamespace(
        api=API(clock), cmd_lock=False, sdk_v=2, hand_joint='L20-10',
        hand_type='left', is_touch=False, modbus='None', embedded_version=[1, 0, 6],
        touch_type=2, last_hand_info={'fault': [-1] * 10}, last_hand_vel=[-1] * 10,
        last_hand_post_cmd=None, last_index_post_cmd=None, last_hand_vel_cmd=None,
        last_position_received=None, count_publishers=lambda topic: 0,
        hand_state_pub=SimpleNamespace(get_subscription_count=lambda: 1),
        hand_info_pub=SimpleNamespace(get_subscription_count=lambda: 1),
        g20_loop=G20IoLoop(clock), run_count=0)
    return clock, node


def extracted_method(name, namespace):
    tree = ast.parse((SDK / 'linker_hand.py').read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<SDK offline method>', 'exec'), namespace)
    return namespace[name]


class SdkIoTests(unittest.TestCase):
    def test_only_v2_g20_without_touch_uses_new_schedule(self):
        for model in ('L20-10', 'L20-11', 'G20'):
            self.assertTrue(G20IoLoop.supported(model, 2, False, 'None'))
        for args in (('L20', 2, False, 'None'), ('L20-10', 1, False, 'None'),
                     ('L20-10', 2, True, 'None'), ('L20-10', 2, False, '/dev/ttyUSB0')):
            self.assertFalse(G20IoLoop.supported(*args))

    def test_pending_position_goes_first_and_one_telemetry_field_per_cycle(self):
        clock, node = fixture()
        node.last_hand_post_cmd = list(range(20))
        node.last_position_received = 99.998
        node.g20_loop.run(node)
        self.assertEqual([e[1] for e in clock.events], ['finger_move', 'get_state', 'get_fault'])
        self.assertEqual(clock.events[0][0], 100.)
        self.assertAlmostEqual(clock() - 100., .047)
        self.assertEqual(clock.events[0][3]['pose'], list(range(20)))
        self.assertIsNone(node.last_hand_post_cmd)
        timing = node.last_hand_info['sdk_io']
        self.assertEqual(timing['command_received_monotonic'], 99.998)
        self.assertAlmostEqual(timing['position_send_end_monotonic'], 100.015)
        self.assertAlmostEqual(timing['state_query_end_monotonic'], 100.032)

    def test_sustained_load_bounds_cycles_and_keeps_faults_and_telemetry_live(self):
        clock, node = fixture()
        cycles = []
        for _ in range(140):
            node.last_hand_post_cmd = [200] * 20
            node.last_position_received = clock()
            start, count = clock(), len(clock.events)
            node.g20_loop.run(node)
            events = clock.events[count:]
            cycles.append(clock() - start)
            self.assertEqual(events[0][1], 'finger_move')
            self.assertEqual(events[1][1], 'get_state')
            self.assertLessEqual(len(events), 3)
            clock.sleep(.001)  # A small executor gap; no fast timer catch-up assumed.
        self.assertLessEqual(max(cycles), .050000001)
        for name in ('get_speed', 'get_current', 'get_temperature', 'get_torque', 'get_fault'):
            times = [e[0] for e in clock.events if e[1] == name]
            self.assertGreaterEqual(len(times), 3)
            bound = .16 if name == 'get_fault' else .95
            self.assertLessEqual(max(b - a for a, b in zip(times, times[1:])), bound)
        self.assertEqual(node.g20_loop.state_generation, 140)
        self.assertEqual(node.last_hand_vel, [1] * 20)

    def test_settings_lock_does_not_send_or_consume_a_target(self):
        clock, node = fixture()
        node.cmd_lock = True
        node.last_hand_post_cmd = [200] * 20
        node.g20_loop.run(node)
        self.assertFalse(clock.events)
        self.assertEqual(node.last_hand_post_cmd, [200] * 20)
        self.assertEqual(node.g20_loop.state_generation, 0)
        self.assertEqual(node.g20_loop.info_generation, 0)

    def test_real_ros_array_callback_replaces_an_unsent_target_on_reversal(self):
        clock, node = fixture()
        node.last_hand_eff_cmd = None
        callback = extracted_method('hand_control_cb', {
            'time': SimpleNamespace(monotonic=clock), 'G20IoLoop': G20IoLoop})
        # ROS JointState uses array('d'), not list. The old list-only change
        # detector otherwise silently retains the first unsent position.
        node.list_check = lambda values: False
        callback(node, SimpleNamespace(position=array('d', [150] * 20), velocity=[], effort=[]))
        clock.sleep(.002)
        callback(node, SimpleNamespace(position=array('d', [200] * 20), velocity=[], effort=[]))
        node.g20_loop.run(node)
        self.assertEqual(list(clock.events[0][3]['pose']), [200] * 20)
        self.assertAlmostEqual(node.last_position_received, 100.002)

    def test_index_probe_keeps_exclusive_topic_and_single_finger_path(self):
        clock, node = fixture()
        node.last_index_post_cmd = [100, 120, 140]
        node.g20_loop.run(node)
        self.assertEqual(clock.events[0][1], 'set_index_positions')
        self.assertFalse(any(e[1] == 'finger_move' for e in clock.events))
        for full_pending in (False, True):
            clock, node = fixture()
            node.last_index_post_cmd = [100, 120, 140]
            node.count_publishers = lambda topic: 1
            if full_pending:
                node.last_hand_post_cmd = [200] * 20
            node.g20_loop.run(node)
            self.assertNotIn('set_index_positions', [e[1] for e in clock.events])

    def test_no_subscriber_no_queries_or_unsolicited_movement(self):
        clock, node = fixture()
        node.hand_state_pub.get_subscription_count = lambda: 0
        node.hand_info_pub.get_subscription_count = lambda: 0
        node.g20_loop.run(node)
        self.assertFalse(clock.events)
        node.last_hand_post_cmd = [180] * 20
        node.g20_loop.run(node)
        self.assertEqual([e[1] for e in clock.events], ['finger_move'])

    def test_fault_codes_are_retained_and_settings_are_not_written(self):
        clock, node = fixture()
        node.api.fault[5] = 128
        node.last_hand_vel_cmd = [255] * 20
        node.g20_loop.run(node)
        self.assertEqual(node.last_hand_info['fault'][5], 128)
        self.assertIsNone(node.last_hand_vel_cmd)
        snapshot = node.last_hand_info
        saved_timing = snapshot['sdk_io'].copy()
        clock.sleep(.2)
        node.g20_loop.run(node)
        self.assertEqual(snapshot['sdk_io'], saved_timing)
        self.assertIsNot(snapshot, node.last_hand_info)

    def test_failed_fault_query_cannot_mark_info_refreshed(self):
        clock, node = fixture()
        def failure():
            raise RuntimeError('simulated read failure')
        node.api.get_fault = failure
        with self.assertRaisesRegex(RuntimeError, 'simulated'):
            node.g20_loop.run(node)
        self.assertEqual(node.g20_loop.info_generation, 0)
        self.assertEqual(node.last_hand_info, {'fault': [-1] * 10})

    def test_real_run_selects_optimized_schedule_and_preserves_legacy_v1(self):
        clock, node = fixture()
        run = extracted_method('run', {'time': clock, 'G20IoLoop': G20IoLoop})
        node.last_hand_post_cmd = [200] * 20
        run(node)
        self.assertEqual(clock.events[0][1], 'finger_move')
        clock, node = fixture()
        node.sdk_v = 1
        node.last_hand_post_cmd = [200] * 20
        run = extracted_method('run', {'time': clock, 'G20IoLoop': G20IoLoop})
        run(node)
        self.assertEqual([e[1] for e in clock.events], ['get_state', 'get_joint_speed', 'finger_move'])
        self.assertAlmostEqual(clock() - 100., .060)
        self.assertEqual(node.sleep_time, .009)

    def test_publisher_does_not_relabel_repeated_cache_as_new_feedback(self):
        clock, node = fixture()
        state, info = [], []
        node.hand_state_pub.publish = state.append
        node.hand_info_pub.publish = info.append
        node.joint_state_msg = lambda q, v: q
        node.last_hand_state = [123] * 20
        calls = 0
        def sleep(value):
            nonlocal calls
            calls += 1
            if calls == 1:
                node.g20_loop.state_generation = node.g20_loop.info_generation = 1
            elif calls == 4:
                raise EOFError('stop offline publication loop')
        node.hz = 1 / 60
        pub = extracted_method('pub_state', {'time': SimpleNamespace(sleep=sleep),
            'G20IoLoop': G20IoLoop, 'String': SimpleNamespace, 'json': __import__('json')})
        with self.assertRaises(EOFError):
            pub(node)
        self.assertEqual(len(state), 1)
        self.assertEqual(len(info), 1)


if __name__ == '__main__':
    unittest.main()
