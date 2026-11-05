"""One-command side-isolated vision session; manages only its own child processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]


def read_identity(channel='can0',hand='left'):
    """Only C0,00 identity queries; no movement/configuration messages."""
    if hand not in ('left','right'):raise ValueError('Unknown hand side')
    can_id=0x28 if hand=='left' else 0x27
    import can
    with can.Bus(interface='socketcan', channel=channel,
                 can_filters=[{'can_id': can_id, 'can_mask': 0x7ff, 'extended': False}]) as bus:
        for _ in range(2):
            bus.send(can.Message(arbitration_id=can_id, data=[0xC0, 0], is_extended_id=False))
            chunks, final = {}, None
            end = time.monotonic() + .8
            while time.monotonic() < end:
                msg = bus.recv(max(0, end - time.monotonic()))
                if msg is None or len(msg.data) < 3 or msg.data[0] != 0xC0:
                    continue
                seq, payload = msg.data[1], bytes(msg.data[2:])
                if seq > 15 or (seq in chunks and chunks[seq] != payload):
                    raise RuntimeError('Unexpected hand identity fragments')
                chunks[seq] = payload
                if len(payload) < 6 or b'\0' in payload:
                    final = seq
                if final is not None and all(i in chunks for i in range(final + 1)):
                    return b''.join(chunks[i] for i in range(final + 1)).split(b'\0', 1)[0].decode('ascii')
    raise RuntimeError(f'Cannot read {hand}-hand identity on {channel} / {can_id:#x}; check power and CAN')


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


def discover(hand='left'):
    import rclpy
    rclpy.init()
    node = rclpy.create_node(f'lh20_auto_{hand}_session_discovery')
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        return {topic: node.count_publishers(topic) for topic in (
            f'/cb_{hand}_hand_state', f'/cb_{hand}_hand_info', f'/cb_{hand}_hand_index_control_cmd',
            f'/cb_{hand}_hand_control_cmd', '/cb_hand_setting_cmd')}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def confirm_right_identity(identity):
    """Enroll only an identified right L20-10; reject a later device swap.

    First use is authorized by the right-hand setup command. This local file
    is separate from the left identity and never enters the portable profile.
    All checks happen before any SDK/controller process can be started.
    """
    import re
    if not isinstance(identity,str) or not re.fullmatch(r'LHT20-010-[A-Za-z0-9-]+-R-[A-Za-z0-9-]+',identity):
        raise RuntimeError('设备身份不是右手 L20-10，拒绝启动右手控制：'+str(identity))
    path=ROOT/'teleop/data/right_device_identity.json'
    configured=os.environ.get('LH20_RIGHT_EXPECTED_IDENTIFIER','').strip()
    if path.exists():
        row=json.loads(path.read_text())
        if row.get('hand')!='right' or row.get('can_id')!=0x27:
            raise RuntimeError('Invalid local right-hand registration')
        expected=row.get('identifier')
        if configured and configured!=expected:
            raise RuntimeError('Conflicting local right-hand identities')
    else:
        expected=configured or identity
    if identity!=expected:
        raise RuntimeError('Right hand differs from the registered device; refusing automatic control')
    if not path.exists():
        path.parent.mkdir(parents=True,exist_ok=True)
        # Exclusive creation prevents two starters from overwriting enrollment.
        with path.open('x') as stream:
            json.dump({'hand':'right','can_id':0x27,'identifier':identity},stream)
        path.chmod(0o600)
    return identity


def run(args):
    if args.hand not in ('left','right'):
        raise ValueError('Unknown hand side')
    hand=args.hand
    from lh20_cli import check_can, vision_command, selected_fault_policy
    scope = getattr(args, 'scope', 'index')
    if scope not in ('index', 'hand') or hand=='right' and scope!='hand':
        raise ValueError('Unknown control scope')
    check_can('can0')
    if scope == 'hand':
        from factory_mapping import FactoryMapping
        config = FactoryMapping(hand=hand).profile
    else:
        config = json.loads((ROOT / 'teleop/config/l20_left_live_index.json').read_text())
    identity = read_identity() if hand=='left' else read_identity(hand=hand)
    if hand=='right':
        confirm_right_identity(identity)
    else:
        expected = os.environ.get('LH20_EXPECTED_IDENTIFIER', '').strip() or config.get('expected_identifier')
        if not expected:
            raise RuntimeError('请先运行 ./scripts/register_hand.sh 将左手身份登记到本机 .local.env')
        if identity != expected:
            raise RuntimeError('Connected hand differs from the verified automatic-control setup')
    counts = discover() if hand=='left' else discover(hand=hand)
    if any(counts[t] for t in (f'/cb_{hand}_hand_index_control_cmd', f'/cb_{hand}_hand_control_cmd', '/cb_hand_setting_cmd')):
        raise RuntimeError('请先在旧 teleop/jog/GUI 控制终端按 Ctrl+C，随后只需运行 vision')
    state, info = counts[f'/cb_{hand}_hand_state'], counts[f'/cb_{hand}_hand_info']
    if (state, info) not in ((0, 0), (1, 1)):
        raise RuntimeError('SDK publishers are incomplete or duplicated')
    logs = ROOT / 'teleop/data'
    logs.mkdir(exist_ok=True)
    stamp = str(time.time_ns())
    bridge_extra=['--thumb-axes',getattr(args,'thumb_axes','native')] if scope=='hand' else []
    if scope=='hand':bridge_extra+=['--retargeting',getattr(args,'retargeting','vector')]
    sdk = bridge = camera = None
    files = []
    try:
        if state == 0:
            handle = (logs / f"auto_sdk_{'right_' if hand=='right' else ''}{stamp}.log").open('x')
            files.append(handle)
            sdk = subprocess.Popen([str(ROOT / f'lh20-{hand}'), 'sdk'], cwd=ROOT, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            print(f'(lh20_vision) 自动启动 {hand} 驱动。', flush=True)
        else:
            print(f'(lh20_vision) 使用已运行的 {hand} 驱动。', flush=True)
        with tempfile.TemporaryDirectory(prefix=f'lh20-auto-{hand}-') as temp:
            ipc = Path(temp) / 'vision.sock'
            bridge = subprocess.Popen([sys.executable, str(ROOT / ('teleop/bridge/live_hand.py' if scope == 'hand' else 'teleop/bridge/live_index.py')),
                                       '--hand', hand, '--execute', '--auto-follow', '--duration', '3600',
                                       '--socket', str(ipc), '--fault-policy', selected_fault_policy(hand),*bridge_extra],
                                      cwd=ROOT, start_new_session=True)
            end = time.monotonic() + 15
            while time.monotonic() < end and not ipc.exists():
                if bridge.poll() is not None or (sdk is not None and sdk.poll() is not None):
                    raise RuntimeError('驱动或实时桥启动失败；未打开运动视觉流')
                time.sleep(.05)
            if not ipc.exists():
                raise RuntimeError('实时桥启动超时')
            # Vision stays in its separate Python 3.10 environment.
            if scope=='hand' and args.record is None:
                algorithm=getattr(args,'retargeting','vector')
                args.record=logs/f"vision_{'right_' if hand=='right' else ''}{algorithm}_{stamp}.jsonl"
                print('(lh20_vision) '+algorithm+' 关键点、目标与分段耗时记录：'+str(args.record),flush=True)
            command, env = vision_command(args)
            command += ['--stream', '--auto-follow', '--socket', str(ipc)]
            camera = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
            print('(lh20_vision) ' + ('厂家映射整手随动' if scope == 'hand' else '食指随动') + f'：识别 {hand} 后自动跟随，Q 退出。', flush=True)
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
