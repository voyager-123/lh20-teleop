"""One-command left vision session; manages only its own child processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]


def read_identity(channel='can0'):
    """Only C0,00 identity queries; no movement/configuration messages."""
    import can
    with can.Bus(interface='socketcan', channel=channel,
                 can_filters=[{'can_id': 0x28, 'can_mask': 0x7ff, 'extended': False}]) as bus:
        for _ in range(2):
            bus.send(can.Message(arbitration_id=0x28, data=[0xC0, 0], is_extended_id=False))
            chunks, final = {}, None
            end = time.monotonic() + .8
            while time.monotonic() < end:
                msg = bus.recv(max(0, end - time.monotonic()))
                if msg is None or len(msg.data) < 3 or msg.data[0] != 0xC0:
                    continue
                seq, payload = msg.data[1], bytes(msg.data[2:])
                if seq > 15 or (seq in chunks and chunks[seq] != payload):
                    raise RuntimeError('Unexpected left identity fragments')
                chunks[seq] = payload
                if len(payload) < 6 or b'\0' in payload:
                    final = seq
                if final is not None and all(i in chunks for i in range(final + 1)):
                    return b''.join(chunks[i] for i in range(final + 1)).split(b'\0', 1)[0].decode('ascii')
    raise RuntimeError('Cannot read left-hand identity')


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=4)


def discover():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('lh20_auto_session_discovery')
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        return {topic: node.count_publishers(topic) for topic in (
            '/cb_left_hand_state', '/cb_left_hand_info', '/cb_left_hand_index_control_cmd',
            '/cb_left_hand_control_cmd', '/cb_hand_setting_cmd')}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run(args):
    if args.hand != 'left':
        raise ValueError('Automatic hardware following is configured only for the verified left hand')
    from lh20_cli import check_can, vision_command, selected_fault_policy
    scope = getattr(args, 'scope', 'index')
    if scope not in ('index', 'hand'):
        raise ValueError('Unknown control scope')
    check_can('can0')
    if scope == 'hand':
        from factory_mapping import FactoryMapping
        config = FactoryMapping().profile
    else:
        config = json.loads((ROOT / 'teleop/config/l20_left_live_index.json').read_text())
    identity = read_identity()
    expected = os.environ.get('LH20_EXPECTED_IDENTIFIER', '').strip()
    if not expected:
        raise RuntimeError('请先运行 ./scripts/register_hand.sh 将左手身份登记到本机 .local.env')
    if identity != expected:
        raise RuntimeError('Connected hand differs from the verified automatic-control setup')
    counts = discover()
    if any(counts[t] for t in ('/cb_left_hand_index_control_cmd', '/cb_left_hand_control_cmd', '/cb_hand_setting_cmd')):
        raise RuntimeError('请先在旧 teleop/jog/GUI 控制终端按 Ctrl+C，随后只需运行 vision')
    state, info = counts['/cb_left_hand_state'], counts['/cb_left_hand_info']
    if (state, info) not in ((0, 0), (1, 1)):
        raise RuntimeError('SDK publishers are incomplete or duplicated')
    logs = ROOT / 'teleop/data'
    logs.mkdir(exist_ok=True)
    stamp = str(time.time_ns())
    bridge_extra=['--thumb-axes',getattr(args,'thumb_axes','native')] if scope=='hand' else []
    sdk = bridge = camera = None
    files = []
    try:
        if state == 0:
            handle = (logs / f'auto_sdk_{stamp}.log').open('x')
            files.append(handle)
            sdk = subprocess.Popen([str(ROOT / 'lh20-left'), 'sdk'], cwd=ROOT, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            print('(lh20_vision) 自动启动左手驱动。', flush=True)
        else:
            print('(lh20_vision) 使用已运行的左手驱动。', flush=True)
        with tempfile.TemporaryDirectory(prefix='lh20-auto-left-') as temp:
            ipc = Path(temp) / 'vision.sock'
            bridge = subprocess.Popen([sys.executable, str(ROOT / ('teleop/bridge/live_hand.py' if scope == 'hand' else 'teleop/bridge/live_index.py')),
                                       '--hand', 'left', '--execute', '--auto-follow', '--duration', '3600',
                                       '--socket', str(ipc), '--fault-policy', selected_fault_policy('left'),*bridge_extra],
                                      cwd=ROOT, start_new_session=True)
            end = time.monotonic() + 15
            while time.monotonic() < end and not ipc.exists():
                if bridge.poll() is not None or (sdk is not None and sdk.poll() is not None):
                    raise RuntimeError('驱动或实时桥启动失败；未打开运动视觉流')
                time.sleep(.05)
            if not ipc.exists():
                raise RuntimeError('实时桥启动超时')
            # Vision stays in its separate Python 3.10 environment.
            command, env = vision_command(args)
            command += ['--stream', '--auto-follow', '--socket', str(ipc)]
            camera = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
            print('(lh20_vision) ' + ('厂家映射整手随动' if scope == 'hand' else '食指随动') + '：识别左手后自动跟随，Q 退出。', flush=True)
            while camera.poll() is None:
                if bridge.poll() is not None or (sdk is not None and sdk.poll() is not None):
                    raise RuntimeError('控制连接已退出，正在停止视觉会话')
                time.sleep(.1)
            return camera.returncode
    finally:
        # Never stop a pre-existing user SDK. Stop targets before the camera.
        for child in (bridge, camera, sdk):
            stop_owned(child)
        for handle in files:
            handle.close()
        print('(lh20_vision) 自动会话已退出，停止新目标；不自动回原位。', flush=True)
