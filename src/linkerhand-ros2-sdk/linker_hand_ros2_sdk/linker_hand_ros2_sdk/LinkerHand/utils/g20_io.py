"""Bounded, single-threaded CAN scheduling for SDK v2 G20 without touch.

Keep the driver's CAN pacing intact. Positions precede reads, position feedback
is queried every cycle, and at most ONE other field is queried in that cycle.
No constructor, setting command, fault reset or unsolicited movement lives here.
"""
import time


class G20IoLoop:
    FAULT_INTERVAL = 0.1
    TELEMETRY_INTERVAL = 0.15
    TELEMETRY = (
        ('speed', 'get_speed'),
        ('current', 'get_current'),
        ('motor_temperature', 'get_temperature'),
        ('torque', 'get_torque'),
    )

    @staticmethod
    def supported(hand_joint, sdk_v, is_touch, modbus):
        return (hand_joint in ('L20-10', 'L20-11', 'G20') and sdk_v == 2
                and not is_touch and modbus == 'None')

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.next_fault = 0.0
        self.next_telemetry = 0.0
        self.telemetry_index = 0
        self.state_generation = 0
        self.info_generation = 0
        self.query_completed = {}
        self.timing = {'mode': 'g20_position_first_v1'}

    def run(self, node):
        if node.cmd_lock:
            return
        began = self.clock()
        if node.last_index_post_cmd is not None:
            payload = node.last_index_post_cmd
            node.last_index_post_cmd = None
            if (node.last_hand_post_cmd is None and not node.count_publishers(
                    f'/cb_{node.hand_type}_hand_control_cmd')):
                node.api.hand.set_index_positions(payload)
        if node.last_hand_post_cmd is not None:
            self.timing['command_received_monotonic'] = node.last_position_received
            self.timing['position_send_begin_monotonic'] = self.clock()
            node.api.finger_move(pose=node.last_hand_post_cmd)
            self.timing['position_send_end_monotonic'] = self.clock()
            node.last_hand_post_cmd = None
        # G20 never applies JointState.velocity as a device setting in the
        # legacy loop either. Explicit settings still use hand_setting_cb.
        node.last_hand_vel_cmd = None

        if node.hand_state_pub.get_subscription_count() > 0:
            self.timing['state_query_begin_monotonic'] = self.clock()
            node.last_hand_state = node.api.get_state()
            self.timing['state_query_end_monotonic'] = self.clock()
            self.state_generation += 1

        info = dict(node.last_hand_info)
        fault_queried = False
        if node.hand_info_pub.get_subscription_count() > 0:
            now = self.clock()
            if now >= self.next_fault:
                info['fault'] = node.api.get_fault()
                self.query_completed['fault'] = self.clock()
                self.next_fault = now + self.FAULT_INTERVAL
                fault_queried = True
            elif now >= self.next_telemetry:
                field, method = self.TELEMETRY[self.telemetry_index]
                info[field] = getattr(node.api, method)()
                if field == 'speed':
                    node.last_hand_vel = info[field]
                self.query_completed[field] = self.clock()
                self.telemetry_index = (self.telemetry_index + 1) % len(self.TELEMETRY)
                self.next_telemetry = now + self.TELEMETRY_INTERVAL
            info.update(version=node.embedded_version, hand_joint=node.hand_joint,
                        is_touch=node.is_touch, touch_type=node.touch_type,
                        finger_order=node.api.get_finger_order())
        self.timing['cycle_ms'] = (self.clock() - began) * 1000
        # These are host query/send windows, not device acknowledgements or
        # proof of a newly received CAN response. Replace snapshots atomically
        # because the publication thread also reads this dictionary.
        info['sdk_io'] = dict(self.timing, query_completed_monotonic=dict(self.query_completed))
        node.last_hand_info = info
        if fault_queried:
            self.info_generation += 1
