#!/usr/bin/env python3
"""Bounded live index-root demo. Not absolute angle calibration or full-hand teleop."""
import argparse
from collections import deque
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import stat
import statistics
import time
import xml.etree.ElementTree as ET

SOURCE_COMMIT = '8632b2cab32e1b51ce379940c414a0f78332ff6b'
ROOT = Path(__file__).resolve().parents[1]


def default_socket(hand):
    if hand not in ('left', 'right'):
        raise ValueError('Invalid hand')
    return Path('/tmp') / f'lh20-{os.getuid()}-{hand}-vision.sock'


class VisionSender:
    """Local datagrams only; a missing bridge must never stall the camera."""
    def __init__(self, path):
        self.path = str(path)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def send(self, packet):
        try:
            self.socket.sendto(json.dumps(packet, allow_nan=False).encode(), self.path)
            return True
        except (OSError, ValueError):
            return False

    def close(self):
        self.socket.close()


class VisionReceiver:
    def __init__(self, path):
        self.path = Path(path)
        lock_path = str(self.path) + '.lock'
        self.fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        self.socket = None
        try:
            if os.fstat(self.fd).st_uid != os.getuid():
                raise RuntimeError('IPC lock belongs to another user')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if self.path.exists() or self.path.is_symlink():
                info = self.path.lstat()
                if info.st_uid != os.getuid() or not stat.S_ISSOCK(info.st_mode):
                    raise RuntimeError('Refusing to replace a non-owned/non-socket IPC path')
                self.path.unlink()  # Stale socket, under the exclusive lock.
            self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.socket.bind(str(self.path))
            os.chmod(self.path, 0o600)
            self.socket.setblocking(False)
        except Exception:
            if self.socket is not None:
                self.socket.close()
            os.close(self.fd)
            raise

    def receive(self):
        try:
            data = self.socket.recv(65536)
        except BlockingIOError:
            return None
        return json.loads(data)

    def close(self):
        self.socket.close()
        self.path.unlink(missing_ok=True)
        os.close(self.fd)


class IndexController:
    """Pure state machine: camera angle -> tested relative raw-count window."""
    def __init__(self, hand, baseline, span=20, model_hash=None, limits=None, auto_follow=False, fixed_open_count=None):
        from probe_index_joint import validate_positions
        self.hand = hand
        self.baseline = validate_positions(baseline)
        if type(span) is not int or not 1 <= span <= 20 or self.baseline[1] < span:
            raise ValueError('Need span 1..20 and enough room below the starting index count')
        self.auto_follow = auto_follow
        observed_start = self.baseline[1]
        if fixed_open_count is not None:
            if type(fixed_open_count) is not int or not span <= fixed_open_count <= 255 or not fixed_open_count-span-1 <= observed_start <= fixed_open_count+1:
                raise ValueError('当前食指位置不在已验证窗口附近，请先回到已验证姿态')
            self.baseline[1] = fixed_open_count
        self.observed_start = observed_start
        self.span, self.model_hash, self.limits = span, model_hash, limits
        self.reference = None
        self.samples = deque(maxlen=12)
        self.session = None
        self.epoch = None
        self.seq = -1
        self.last_input = -math.inf
        self.last_step = -math.inf
        self.last_command = min(self.baseline[1], max(self.baseline[1]-span, observed_start))
        self.desired = self.last_command
        self.active = False
        self.state = 'WAITING_FOR_VISION'

    def pause(self, reason):
        self.active = False
        self.reference = None
        self.samples.clear()
        self.state = reason

    def accept(self, packet, now):
        try:
            if not isinstance(packet, dict):
                raise ValueError('Packet must be an object')
            if packet.get('schema') != 'lh20.retarget.v1' or packet.get('hand') != self.hand or \
                    packet.get('source_commit') != SOURCE_COMMIT or \
                    packet.get('model') != f'L20/{self.hand}/linkerhand_l20_{self.hand}.urdf' or \
                    packet.get('model_sha256') != self.model_hash:
                raise ValueError('Wrong hand/model/source')
            session, seq, epoch = packet.get('stream_session'), packet.get('seq'), packet.get('calibration_epoch')
            if not isinstance(session, str) or len(session) != 32 or type(seq) is not int or seq < 0 or \
                    type(epoch) is not int or epoch < 0:
                raise ValueError('Invalid session/sequence/epoch')
            stamp = packet.get('captured_monotonic_ns')
            if type(stamp) is not int or not -0.02 <= now - stamp / 1e9 <= 0.25:
                raise ValueError('Vision timestamp is stale or in the future')
            if self.session != session:
                self.pause('NEW_VISION_SESSION')
                self.session, self.seq, self.epoch = session, -1, epoch
            if seq <= self.seq:
                raise ValueError('Duplicate or out-of-order frame')
            self.seq, self.last_input = seq, stamp / 1e9
            enabled = packet.get('operator_enabled')
            if type(enabled) is not bool or type(packet.get('valid')) is not bool:
                raise ValueError('Invalid enable/valid flags')
            if self.epoch != epoch:
                self.epoch = epoch
                self.pause('RECENTER_REQUESTED')
            if not packet['valid'] or packet.get('cad_collision_hits'):
                self.pause('WAITING_FOR_LEFT_HAND' if self.auto_follow else 'TRACKING_LOST_OR_INVALID: press S, open hand, then SPACE')
                return
            names, values = packet.get('joint_names'), packet.get('position_rad')
            if not isinstance(names, list) or not isinstance(values, list) or len(names) != 16 or \
                    len(values) != 16 or len(set(names)) != 16 or set(names) != set(self.limits):
                raise ValueError('Expected 16 unique model joints')
            named = dict(zip(names, values))
            for name, value in named.items():
                lo, hi = self.limits[name]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                        not math.isfinite(value) or not lo - 1e-6 <= value <= hi + 1e-6:
                    raise ValueError('Invalid/out-of-limit model angle')
            angle = named['index_mcp_pitch']
            if self.auto_follow:
                if packet.get('auto_follow') is not True or not enabled:
                    self.pause('AUTO_PAUSED')
                    return
                self.samples.append((stamp / 1e9, angle))
                if len(self.samples) < 3 or self.samples[-1][0] - self.samples[-3][0] < .06:
                    self.state = 'DETECTING_LEFT_HAND'
                    self.active = False
                    return
                self.active = True
                self.desired = self.baseline[1] - round(self.span * min(1., max(0., angle / .6)))
                self.state = 'AUTO_FOLLOWING_INDEX_ROOT'
                return
            if not enabled:
                self.active = False
                if self.reference is None:
                    self.samples.append((now, angle))
                    self.state = 'CALIBRATING_OPEN_HAND'
                    angles = [q for _, q in self.samples]
                    if len(angles) == 12 and now - self.samples[0][0] >= .35 and max(angles) - min(angles) <= .08:
                        self.reference = statistics.median(angles)
                        self.state = 'READY: SPACE enables index-root following'
                else:
                    self.state = 'READY: SPACE enables index-root following'
                return
            if self.reference is None:
                self.state = 'NEED_OPEN_HAND: press S and hold open until READY'
                return
            self.active = True
            # A gain for a bounded demo, NOT radians-to-motor calibration.
            fraction = min(1., max(0., (angle - self.reference) / .6))
            self.desired = self.baseline[1] - round(self.span * fraction)
            self.state = 'FOLLOWING_INDEX_ROOT'
        except (ValueError, TypeError, KeyError) as error:
            self.pause('REJECTED_FRAME: ' + str(error))

    def step(self, now):
        if now - self.last_input > .25:
            self.pause('WAITING_FOR_LEFT_HAND' if self.auto_follow else 'VISION_TIMEOUT: press S, open hand, then SPACE')
        if not self.active or now - self.last_step < 1 / 30:
            return None
        self.last_step = now
        self.last_command += max(-1, min(1, self.desired - self.last_command))
        target = self.baseline.copy()
        target[1] = self.last_command
        return target


def model_spec(hand):
    from hand_recording import SLOTS
    path = ROOT / f'linkerhand-urdf/L20/{hand}/linkerhand_l20_{hand}.urdf'
    limits = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
              for j in ET.parse(path).getroot().findall('joint') if j.get('name') in SLOTS}
    if set(limits) != set(SLOTS):
        raise ValueError('Incomplete URDF independent-joint limits')
    return hashlib.sha256(path.read_bytes()).hexdigest(), limits


def run(args):
    if args.hand != 'left':
        raise ValueError('Only the physically verified LEFT index-root live demo is enabled')
    if not 1 <= args.span <= 20 or not 1 <= args.duration <= 3600:
        raise ValueError('span must be 1..20; duration must be 1..3600 seconds')
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from probe_index_joint import feedback_errors, validate_positions, topic_names
    rclpy.init()
    node = rclpy.create_node('lh20_left_live_index')
    topics = topic_names(args.hand)
    latest = {'q': None, 'state_at': 0., 'info': None, 'info_at': 0.}
    recent = deque(maxlen=5)
    publisher = receiver = engine = None
    log_path = ROOT / 'data' / f'live_index_{time.time_ns()}.jsonl'
    log_path.parent.mkdir(exist_ok=True)
    log = log_path.open('x')
    last_fault = None

    def record(kind, **values):
        log.write(json.dumps({'kind': kind, 'monotonic': time.monotonic(), **values}, allow_nan=False) + '\n')
        log.flush()

    def state(msg):
        latest['state_at'] = time.monotonic()
        try:
            latest['q'] = validate_positions(msg.position)
            recent.append(latest['q'])
        except ValueError:
            latest['q'] = None
            recent.clear()

    def info(msg):
        nonlocal last_fault
        latest['info_at'] = time.monotonic()
        try:
            latest['info'] = json.loads(msg.data)
            fault = latest['info'].get('fault') if isinstance(latest['info'], dict) else None
            if fault != last_fault:
                record('raw_fault_change', fault=fault)
                last_fault = fault
        except (ValueError, TypeError):
            latest['info'] = None

    def guard(check_publisher=True):
        errors = feedback_errors(latest, time.monotonic(), ignore_128=args.fault_policy in ('ignore-128', 'ignore-64-128'), ignore_64=args.fault_policy == 'ignore-64-128')
        if errors:
            raise RuntimeError('; '.join(errors))
        if node.count_publishers(topics['state']) != 1 or node.count_publishers(topics['info']) != 1:
            raise RuntimeError('Expected exactly one SDK state/info publisher')
        if node.count_publishers(topics['full']) or node.count_publishers('/cb_hand_setting_cmd'):
            raise RuntimeError('Another full-hand/settings controller is present')
        if node.count_publishers(topics['index']) != int(publisher is not None):
            raise RuntimeError('Another index controller is present')
        if node.count_subscribers(topics['index']) != 1:
            raise RuntimeError('Expected exactly one isolated-index SDK subscriber')
        if check_publisher and publisher is not None and publisher.get_subscription_count() != 1:
            raise RuntimeError('Live publisher lost its SDK subscriber')

    subscriptions = [node.create_subscription(JointState, topics['state'], state, 10),
                     node.create_subscription(String, topics['info'], info, 10)]
    try:
        print('(lh20_ros2) Waiting for stable left-hand feedback...', flush=True)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if len(recent) == 5 and all(max(q[i] for q in recent) - min(q[i] for q in recent) <= 2
                                        for i in range(20)) and not feedback_errors(
                                            latest, time.monotonic(), ignore_128=args.fault_policy in ('ignore-128', 'ignore-64-128'), ignore_64=args.fault_policy == 'ignore-64-128'):
                break
        else:
            raise RuntimeError('No stable fresh feedback; keep (lh20_ros2) ./lh20-left sdk running')
        guard(False)
        model_hash, limits = model_spec(args.hand)
        fixed_open = None
        if args.auto_follow:
            config = json.loads((ROOT / 'config/l20_left_live_index.json').read_text())
            if latest['info'].get('version') != config['firmware_version'] or args.span != config['span']:
                raise ValueError('自动随动配置与固件或已验证窗口不匹配')
            fixed_open = config['open_raw']
        engine = IndexController(args.hand, latest['q'], args.span, model_hash, limits,
                                 auto_follow=args.auto_follow, fixed_open_count=fixed_open)
        receiver = VisionReceiver(args.socket or default_socket(args.hand))
        if args.execute:
            publisher = node.create_publisher(JointState, topics['index'], 1)
            end = time.monotonic() + 2
            while publisher.get_subscription_count() == 0 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
        guard()
        record('start', hand=args.hand, execute=args.execute, baseline=engine.baseline,
               fault_policy=args.fault_policy, span=args.span, socket=str(receiver.path),
               mapping='bounded_index_demo_not_absolute_calibration', model_sha256=model_hash,
               automatic_mode=args.auto_follow, observed_start_root=engine.observed_start)
        print(f'(lh20_ros2) {"EXECUTION ENABLED" if args.execute else "PREVIEW ONLY"}: '
              f'index root window [{engine.baseline[1]-args.span}, {engine.baseline[1]}], '
              'max 1 count/update, <=30 Hz. Other fingers are not commanded.', flush=True)
        if args.auto_follow:
            print('(lh20_ros2) Automatic left index following; Q quits vision. No READY or enable key.', flush=True)
        else:
            print('(lh20_ros2) In the vision window: S=pause, hold LEFT hand open until READY; '
                  'SPACE=toggle following, C=recenter while paused, Q=quit camera.', flush=True)
        deadline = time.monotonic() + args.duration
        last_state = None
        next_print = 0.
        following_error_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            guard()
            for _ in range(32):
                try:
                    packet = receiver.receive()
                except (ValueError, UnicodeError):
                    engine.pause('REJECTED_MALFORMED_JSON')
                    break
                if packet is None:
                    break
                engine.accept(packet, time.monotonic())
            now = time.monotonic()
            target = engine.step(now)
            if engine.state != last_state:
                last_state = engine.state
                print('(lh20_ros2) ' + last_state, flush=True)
                record('state', state=last_state)
            if publisher is not None and engine.active:
                if abs(latest['q'][1] - engine.last_command) > 8:
                    following_error_since = now if following_error_since is None else following_error_since
                    if now - following_error_since > .75:
                        raise RuntimeError('Index feedback differs from last target by >8 counts for >0.75s')
                else:
                    following_error_since = None
                if any(abs(latest['q'][i] - engine.baseline[i]) > 3 for i in (6, 16)):
                    raise RuntimeError('Untested index side/tip moved beyond baseline tolerance')
            if target is not None:
                if publisher is not None:
                    message = JointState()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.name = ['index_abduction', 'index_base', 'index_tip']
                    message.position = [float(target[i]) for i in (6, 1, 16)]
                    publisher.publish(message)
                record('command' if publisher is not None else 'preview', root=target[1],
                       position=[target[i] for i in (6, 1, 16)], feedback=latest['q'][1])
                if now >= next_print:
                    print(f'(lh20_ros2) root target={target[1]}, feedback={latest["q"][1]}', flush=True)
                    next_print = now + 1
    finally:
        record('stop', last_feedback=latest['q'], auto_return=False)
        print('(lh20_ros2) Stopped sending new targets; no automatic return. Log: ' + str(log_path), flush=True)
        if receiver is not None:
            receiver.close()
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hand', choices=('left', 'right'), required=True)
    parser.add_argument('--span', type=int, default=20)
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--socket', type=Path)
    parser.add_argument('--auto-follow', action='store_true')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--fault-policy', choices=('stop', 'ignore-128', 'ignore-64-128'), default='stop')
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError) as error:
        print('(lh20_ros2) STOPPED: ' + str(error), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
