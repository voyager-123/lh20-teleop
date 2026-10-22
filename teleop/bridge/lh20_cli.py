#!/usr/bin/env python3
from runtime_paths import vision_python as resolve_vision_python
"""Command-line entry for environment check, official SDK and isolated jog."""
import argparse
import importlib
from importlib import metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

WORKSPACE = Path(__file__).resolve().parents[2]
BRIDGE = Path(__file__).resolve().parent


def check_ros_loopback():
    import rclpy
    from std_msgs.msg import String
    token = uuid.uuid4().hex
    received = []
    rclpy.init()
    node = rclpy.create_node('lh20_environment_check_' + token[:8])
    try:
        topic = '/lh20_environment_check/probe_' + token
        subscription = node.create_subscription(String, topic, lambda msg: received.append(msg.data), 10)
        publisher = node.create_publisher(String, topic, 10)
        message = String()
        message.data = token
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and token not in received:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        if token not in received:
            raise RuntimeError('ROS 本机消息收发超时；检查 DDS 和 ROS_DOMAIN_ID 配置')
        print('ROS 本机消息收发：通过（仅使用随机自检话题）', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_environment(run_tests):
    print('Python:', sys.executable, sys.version.split()[0], flush=True)
    print('ROS_DISTRO:', os.environ.get('ROS_DISTRO'), flush=True)
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('需要 ROS2 Jazzy，请通过工作区 ./lh20 入口运行')
    for module, package in [('rclpy', 'rclpy'), ('sensor_msgs.msg', 'sensor_msgs'),
                            ('std_msgs.msg', 'std_msgs'), ('numpy', 'numpy'),
                            ('can', 'python-can'), ('PyQt5.QtCore', 'PyQt5')]:
        loaded = importlib.import_module(module)
        print(f'{module}: {metadata.version(package)}  {loaded.__file__}', flush=True)
    importlib.import_module('linker_hand_ros2_sdk.linker_hand')
    print('官方 SDK 节点模块导入：通过（未构建设备对象）', flush=True)
    check_ros_loopback()
    if run_tests:
        result = subprocess.run([sys.executable, '-m', 'unittest', 'discover',
                                 '-s', 'teleop/tests', '-v'], cwd=WORKSPACE)
        if result.returncode:
            return result.returncode
    print('环境自检通过；未连接 CAN 或发布手部控制指令。', flush=True)
    return 0


def sdk_command(hand, channel):
    return ['ros2', 'run', 'linker_hand_ros2_sdk', 'linker_hand_sdk', '--ros-args',
            '-p', 'hand_type:=' + hand, '-p', 'hand_joint:=L20-10',
            '-p', 'can:=' + channel, '-p', 'is_touch:=false']


def check_can(channel):
    import re
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', channel):
        raise ValueError('无效的 CAN 接口名称')
    result = subprocess.run(['ip', '-j', '-details', 'link', 'show', 'dev', channel],
                            capture_output=True, text=True, check=True, timeout=5)
    item = json.loads(result.stdout)[0]
    info = item.get('linkinfo', {})
    data = info.get('info_data', {})
    if info.get('info_kind') != 'can' or 'UP' not in item.get('flags', []):
        raise RuntimeError('CAN 接口尚未启动；请先在本机终端配置接口')
    if data.get('bittiming', {}).get('bitrate') != 1000000 or data.get('state') != 'ERROR-ACTIVE':
        raise RuntimeError('需要 1000000 bit/s、ERROR-ACTIVE 的 CAN 接口')


def graph_conflicts(node):
    from probe_index_joint import topic_names
    topics = [name for hand in ('left', 'right') for name in topic_names(hand).values()]
    topics.append('/cb_hand_setting_cmd')
    return [topic for topic in topics if node.count_publishers(topic)]


def check_sdk_start():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('lh20_sdk_start_check_' + uuid.uuid4().hex[:8])
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        conflicts = graph_conflicts(node)
        if conflicts:
            raise RuntimeError('请先停止已有 SDK、GUI 或控制程序，检测到话题：' + ', '.join(conflicts))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def vision_command(args):
    python = resolve_vision_python()
    if not python.is_file():
        raise RuntimeError('找不到 lh20_vision Python')
    from probe_index_joint import vision_subprocess_env
    env = vision_subprocess_env(python)
    for key in ('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH'):
        env.pop(key, None)
    env.setdefault('MPLCONFIGDIR', '/tmp/lh20-vision-matplotlib')
    command = [str(python), '-E', '-s', '-u', str(WORKSPACE / 'teleop/vision/preview_l20.py'),
               '--hand', args.hand, '--camera', args.camera,
               '--rotate-deg', *map(str, args.rotate_deg)]
    command += ['--scope', getattr(args, 'scope', 'index'), '--thumb-axes',getattr(args,'thumb_axes','native')]
    if args.self_test:
        command.append('--self-test')
    if args.record:
        command += ['--record', str(args.record.resolve())]
    if getattr(args, 'stream', False):
        command.append('--stream')
    return command, env


def selected_fault_policy(hand, requested=None):
    # User confirmed this left hand's exact raw 64 and 128 codes on 2026-10-22.
    # Keep the separate right-hand entry strict until explicitly overridden.
    return requested or ('ignore-64-128' if hand == 'left' else 'stop')


def main(argv=None):
    parser = argparse.ArgumentParser(description='LH20 V10 命令行：自检、官方驱动、状态、小幅食指测试')
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('check', help='导入检查和 ROS 本机消息收发；不连接硬件')
    check.add_argument('--tests', action='store_true', help='同时运行离线回归测试')
    sdk = sub.add_parser('sdk', help='启动官方 L20-10 SDK，保持此终端运行')
    sdk.add_argument('--hand', choices=('left', 'right'), required=True)
    sdk.add_argument('--channel', default='can0')
    status = sub.add_parser('status', help='从官方 SDK 读取位置和故障，无动作发布器')
    status.add_argument('--hand', choices=('left', 'right'), required=True)
    status.add_argument('--output', type=Path)
    jog = sub.add_parser('jog', help='食指根部/末端小幅测试；默认仅预览')
    jog.add_argument('--hand', choices=('left', 'right'), required=True)
    jog.add_argument('--slot', type=int, choices=(1, 16), default=1, help='1=食指根部；16=食指末端')
    jog.add_argument('--delta', type=int, default=-5, help='相对原始计数，绝对值 1..20，并非角度')
    jog.add_argument('--fault-policy', choices=('stop', 'ignore-128', 'ignore-64-128'),
                     help='默认：左手忽略纯64/128（用户已确认），右手 stop；保留原始故障值')
    jog.add_argument('--execute', action='store_true')
    jog.add_argument('--return', dest='return_to_start', action='store_true')
    vision = sub.add_parser('vision', help='左手默认自动食指随动；--preview 仅视觉预览')
    vision.add_argument('--hand', choices=('left', 'right'), required=True)
    vision.add_argument('--camera', default='0')
    vision.add_argument('--thumb-axes',choices=('native','swapped'),default='native',help='native=厂家定义；swapped=拇指侧摆/横摆交换对比')
    vision.add_argument('--scope', choices=('index', 'hand'), default='index', help='index=已验证食指；hand=厂家左手整手映射')
    vision.add_argument('--self-test', action='store_true', help='仅模型离线自检，不打开摄像头')
    vision.add_argument('--rotate-deg', nargs=3, type=float, default=[0, 0, 0])
    vision.add_argument('--record', type=Path, help='保存新视觉记录，拒绝覆盖')
    vision.add_argument('--stream', action='store_true', help='发送本机视觉流；SPACE 开启，S 暂停，C 重新取参考，Q 退出')
    teleop = sub.add_parser('teleop', help='已验证左手食指根部实时随动；默认仅预览')
    teleop.add_argument('--hand', choices=('left', 'right'), required=True)
    teleop.add_argument('--span', type=int, default=20)
    teleop.add_argument('--duration', type=float, default=300)
    teleop.add_argument('--execute', action='store_true')
    teleop.add_argument('--fault-policy', choices=('stop', 'ignore-128', 'ignore-64-128'))
    vision.add_argument('--preview', action='store_true', help='只预览，不启动实物控制')
    vision.add_argument('--manual', action='store_true', help='保留旧手动开启方式')
    automatic = sub.add_parser('_auto-vision', help=argparse.SUPPRESS)
    automatic.add_argument('--hand', choices=('left',), required=True)
    automatic.add_argument('--camera', default='0')
    automatic.add_argument('--thumb-axes',choices=('native','swapped'),default='native')
    automatic.add_argument('--scope', choices=('index', 'hand'), default='index')
    automatic.add_argument('--rotate-deg', nargs=3, type=float, default=[0, 0, 0])
    automatic.add_argument('--record', type=Path)
    automatic.set_defaults(self_test=False, stream=False)
    args = parser.parse_args(argv)
    try:
        if args.command == 'check':
            return check_environment(args.tests)
        if args.command == '_auto-vision':
            from auto_vision import run
            return run(args)
        if args.command == 'vision':
            if args.thumb_axes!='native' and (args.hand!='left' or args.scope!='hand'):
                raise ValueError('拇指交换对比只支持左手整手模式')
            if args.scope == 'hand' and (args.hand != 'left' or args.manual or args.stream):
                raise ValueError('整手模式仅支持左手；直接用 vision --scope hand 或加 --preview')
            if args.hand == 'left' and not args.self_test and not args.preview and not args.manual:
                command = [str(WORKSPACE / 'lh20'), '_auto-vision', '--hand', 'left', '--camera', args.camera,
                           '--rotate-deg', *map(str, args.rotate_deg), '--scope', args.scope,'--thumb-axes',args.thumb_axes]
                if args.record:
                    command += ['--record', str(args.record.resolve())]
                print('(lh20_vision) 左手自动随动：将启动或复用驱动及控制桥；Q 退出。', flush=True)
                return os.execv(command[0], command)
            command, env = vision_command(args)
            print(f'视觉：{args.hand}，lh20_vision；仅预览模型角度，无电机输出。', flush=True)
            return os.execve(command[0], command, env)
        if args.command == 'teleop':
            command = [sys.executable, str(BRIDGE / 'live_index.py'), '--hand', args.hand,
                       '--span', str(args.span), '--duration', str(args.duration),
                       '--fault-policy', selected_fault_policy(args.hand, args.fault_policy)]
            if args.execute:
                command.append('--execute')
            return os.execv(sys.executable, command)
        if args.command == 'sdk':
            check_can(args.channel)
            check_sdk_start()
            print(f'启动官方 SDK：{args.hand}，L20-10，{args.channel}，无触觉。Ctrl+C 结束。', flush=True)
            os.execvp('ros2', sdk_command(args.hand, args.channel))
        if args.command == 'status':
            command = [sys.executable, str(BRIDGE / 'inspect_hand.py'), '--hand', args.hand]
            if args.output:
                command += ['--output', str(args.output.resolve())]
            os.execv(sys.executable, command)
        if not 1 <= abs(args.delta) <= 20:
            raise ValueError('--delta 必须是绝对值为 1..20 的非零整数')
        policy = selected_fault_policy(args.hand, args.fault_policy)
        command = [sys.executable, str(BRIDGE / 'probe_index_joint.py'), '--hand', args.hand,
                   '--slot', str(args.slot), '--delta', str(args.delta), '--fault-policy', policy]
        if args.execute:
            command.append('--execute')
        if args.return_to_start:
            command.append('--return')
        os.execv(sys.executable, command)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, ValueError, OSError, ImportError, subprocess.SubprocessError) as error:
        print('检查未通过：' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
