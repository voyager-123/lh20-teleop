"""ROS acquisition. Imported only by explicitly requested live commands."""
from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

from study_core import (DATA, INDEX_SLOTS, Journal, ROOT, guard_feedback, pose_matches,
                        positions, trajectory, validate_trial)

sys.path.insert(0, str(ROOT / "teleop/bridge"))


class Keys:
    """Nonblocking operator marks; terminal settings are always restored."""
    def __enter__(self):
        self.saved = None
        if sys.stdin.isatty():
            self.saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def read(self):
        result = []
        while select.select([sys.stdin], [], [], 0)[0]:
            value = os.read(sys.stdin.fileno(), 1).decode("ascii", errors="ignore")
            if not value:
                break
            result.append(value.lower())
        return result

    def __exit__(self, *_):
        if self.saved is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.saved)


class LiveLink:
    def __init__(self, hand="left", channel="can0", can_log=True):
        import rclpy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from probe_index_joint import topic_names
        self.ros, self.JointState = rclpy, JointState
        self.hand, self.channel = hand, channel
        self.topics = topic_names(hand)
        self.journal = self.publisher = self.bus = None
        self.latest = {"q": None, "state_ns": 0, "info": None, "info_ns": 0}
        self.recent = deque(maxlen=5)
        self.last_state = None
        rclpy.init()
        self.node = rclpy.create_node("lh20_study_" + str(time.time_ns()))
        try:
            self.subs = [
                self.node.create_subscription(JointState, self.topics["state"], self.on_state, 100),
                self.node.create_subscription(String, self.topics["info"], self.on_info, 30),
                self.node.create_subscription(JointState, self.topics["index"], lambda m: self.on_command(m, "index"), 100),
                self.node.create_subscription(JointState, self.topics["full"], lambda m: self.on_command(m, "full"), 100),
            ]
            if can_log:
                import can
                self.bus = can.Bus(interface="socketcan", channel=channel, receive_own_messages=False,
                                   can_filters=[{"can_id": 0x28 if hand == "left" else 0x27,
                                                 "can_mask": 0x7FF, "extended": False}])
                # This bus is receive-only by construction of this module: no send calls.
        except BaseException:
            self.close()
            raise

    def log(self, kind, **values):
        if self.journal is not None:
            return self.journal.write(kind, **values)

    def on_state(self, message):
        received = time.monotonic_ns()
        header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        publication_age_ms = (self.node.get_clock().now().nanoseconds - header_ns) / 1e6
        try:
            q = positions(message.position)
            if not -20 <= publication_age_ms <= 300:
                raise ValueError("SDK发布头时间过旧/超前；可能为队列中的旧消息")
        except (ValueError, TypeError) as error:
            self.latest.update(q=None, state_ns=received)
            self.recent.clear()
            self.log("state", mono_ns=received, valid=False, reason=str(error), position_raw=list(message.position))
            return
        same = q == self.last_state
        self.latest.update(q=q, state_ns=received)
        self.recent.append(q)
        self.last_state = q.copy()
        self.log("state", mono_ns=received, valid=True, position_raw=q,
                 ros_header_ns=header_ns, sdk_publication_age_ms=publication_age_ms,
                 identical_to_previous_message=same, velocity_raw=list(message.velocity),
                 effort_raw=list(message.effort), effort_is_force_measurement=False)

    def on_info(self, message):
        received = time.monotonic_ns()
        try:
            info = json.loads(message.data)
        except (ValueError, TypeError):
            info = None
        self.latest.update(info=info, info_ns=received)
        self.log("info", mono_ns=received, data=info, raw=message.data,
                 current_and_torque_semantics="unverified_vendor_fields_not_validated_force")

    def on_command(self, message, scope):
        received = time.monotonic_ns()
        raw = list(message.position)
        slots = INDEX_SLOTS if scope == "index" else tuple(range(20))
        valid = len(raw) == len(slots) and all(isinstance(v, (int, float)) and 0 <= v <= 255 and v == round(v) for v in raw)
        # Own ROS loopback must not replace the actual publication timestamp.
        own = self.publisher is not None and scope == "index"
        self.log("command_echo" if own else "command_observed", mono_ns=received, scope=scope,
                 valid=valid, published_position=raw,
                 slot_values={str(i): int(v) for i, v in zip(slots, raw)} if valid else {},
                 ros_header_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                 velocity=list(message.velocity), effort=list(message.effort))

    def pump(self, timeout=.005):
        self.ros.spin_once(self.node, timeout_sec=timeout)
        for _ in range(3):
            self.ros.spin_once(self.node, timeout_sec=0.)
        if self.bus is not None:
            for _ in range(256):
                message = self.bus.recv(timeout=0.)
                if message is None:
                    break
                self.log("can_frame", arbitration_id=message.arbitration_id, data_hex=bytes(message.data).hex(),
                         kernel_wall_s=message.timestamp, is_rx=message.is_rx,
                         is_error=message.is_error_frame, is_remote=message.is_remote_frame,
                         dlc=message.dlc)

    def graph(self):
        result = {key + "_publishers": self.node.count_publishers(topic) for key, topic in self.topics.items()}
        result["settings_publishers"] = self.node.count_publishers("/cb_hand_setting_cmd")
        result["index_subscribers"] = self.node.count_subscribers(self.topics["index"])
        return result

    def guard(self, mapping, policy, exclusive=True):
        guard_feedback(self.latest, time.monotonic_ns(), mapping.profile["firmware_version"], policy)
        counts = self.graph()
        if counts["state_publishers"] != 1 or counts["info_publishers"] != 1:
            raise RuntimeError("需要且只允许一个对应手的 SDK 状态/信息发布者")
        if exclusive:
            if counts["full_publishers"] or counts["settings_publishers"] or counts["index_publishers"] != int(self.publisher is not None):
                raise RuntimeError("发现其他遥操/GUI/设置发布器；停止实验新目标")
            if counts["index_subscribers"] != 2:  # Vendor SDK plus this passive observer.
                raise RuntimeError("需要恰好一个 SDK 食指订阅者，另一个为本采集器")
            if self.publisher is not None and self.publisher.get_subscription_count() != 2:
                raise RuntimeError("食指连接发生变化；停止新目标")

    def ready(self, mapping, policy, stable=True, exclusive=True):
        deadline = time.monotonic() + 12
        last = "未收到反馈"
        while time.monotonic() < deadline:
            self.pump(.03)
            try:
                self.guard(mapping, policy, exclusive)
                if not stable or (len(self.recent) == 5 and all(pose_matches(self.recent[0], q) for q in self.recent)):
                    return
                last = "等待五个稳定位置消息"
            except RuntimeError as error:
                last = str(error)
        raise RuntimeError(last + "。请先在另一终端启动 ./lh20-left sdk，并关闭其他控制端。")

    def enable_publisher(self):
        if self.publisher is None:
            self.publisher = self.node.create_publisher(self.JointState, self.topics["index"], 1)
            deadline = time.monotonic() + 3
            while self.publisher.get_subscription_count() < 2 and time.monotonic() < deadline:
                self.pump(.03)

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def mapping_instance():
    from factory_mapping import FactoryMapping
    return FactoryMapping("swapped")


def check_anchor_file(path, mapping):
    anchor = json.loads(Path(path).read_text(encoding="utf-8"))
    if anchor.get("schema") != "lh20.study.anchor.v1" or anchor.get("hand") != "left":
        raise ValueError("需要由 anchor 命令保存的左手参考姿态")
    if anchor.get("mapping_sha256") != mapping.sha256:
        raise ValueError("参考姿态的厂家映射不同")
    positions(anchor["position_raw"])
    if mapping.encode(mapping.decode(anchor["position_raw"])) != [255 if i in (11, 12, 13, 14) else v for i, v in enumerate(anchor["position_raw"])]:
        raise ValueError("参考姿态超出厂家模型有效范围")
    return anchor


def save_anchor(args):
    mapping = mapping_instance()
    link = LiveLink(can_log=False)
    try:
        link.ready(mapping, args.fault_policy)
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        value = {"schema": "lh20.study.anchor.v1", "hand": "left", "position_raw": link.latest["q"],
                 "mapping_sha256": mapping.sha256, "model_sha256": mapping.model_hash,
                 "info": link.latest["info"], "received_monotonic_ns": link.latest["state_ns"],
                 "expected_identifier": os.environ.get("LH20_EXPECTED_IDENTIFIER", ""),
                 "hardware_motion_commands_sent": False}
        with path.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(f"已保存参考姿态（未发动作）：{path.resolve()}\nslot 1={value['position_raw'][1]}，slot 16={value['position_raw'][16]}")
    finally:
        link.close()


def audit_plan(anchor, args, mapping, cad, pump=None):
    validate_trial(args.slot, args.delta, args.speed, args.pre, args.post, args.max_gap, condition=args.condition)
    path = trajectory(anchor["position_raw"], args.slot, args.delta, condition=args.condition)
    for pose in path:
        encoded = mapping.encode(mapping.decode(pose))
        if any(encoded[i] != pose[i] for i in INDEX_SLOTS):
            raise ValueError("实验目标超出厂家模型限位")
    name = next(n for n, e in mapping.joints.items() if e["slot"] == args.slot)
    one_count = abs(mapping.decode(path[1])[name] - mapping.decode(path[0])[name])
    if one_count * args.speed > mapping.velocities[name]:
        raise ValueError("请求的命令斜坡超过模型角速度")
    result = cad.check(path[0], path[-1], pump=pump)
    if not result.get("ok"):
        raise RuntimeError("CAD 预检拒绝该食指路径：" + json.dumps(result, ensure_ascii=False))
    if args.return_to_start:
        result = cad.check(path[-1], path[0], pump=pump)
        if not result.get("ok"):
            raise RuntimeError("返回路径 CAD 预检未通过")
    return path


def preview(args):
    from live_hand import CadWorker
    mapping = mapping_instance()
    anchor = check_anchor_file(args.anchor, mapping)
    cad = CadWorker("swapped")
    try:
        path = audit_plan(anchor, args, mapping, cad)
        result = {"mode": "offline_preview", "hardware_commands_sent": False, "slot": args.slot,
                  "start_raw": path[0][args.slot], "end_raw": path[-1][args.slot],
                  "delta_raw": args.delta, "ramp_speed_raw_s": args.speed,
                  "nominal_ramp_s": abs(args.delta) / args.speed,
                  "return_to_start_requested": args.return_to_start, "cad_path_ok": True}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print("离线预览完成，没有初始化 ROS/CAN；加 --execute 才进入实机采集。")
    finally:
        cad.close()


def operator_keys(keys, journal):
    contact = start = False
    for key in keys.read():
        if key in ("q", "\x1b"):
            journal.write("operator_abort", key=repr(key))
            raise KeyboardInterrupt
        if key in ("\n", "\r"):
            start = True
        if key in ("c", "r", "v", "m"):
            label = {"c": "contact_observed", "r": "release_observed", "v": "video_sync", "m": "note"}[key]
            event = journal.write("marker", label=label, source="operator_key", timing_is_ground_truth=False)
            print(f"标记 {label}，t={event['t_s']:.3f}s", flush=True)
            if key == "v":
                print("******** VIDEO SYNC ********", flush=True)
            contact |= key == "c"
    return start, contact


def verify_identity(args, anchor):
    from auto_vision import read_identity
    from lh20_cli import check_can
    expected = os.environ.get("LH20_EXPECTED_IDENTIFIER", "").strip()
    if not expected:
        raise RuntimeError("先执行 ./scripts/register_hand.sh 登记本机左手，再执行实机采集")
    if anchor.get("expected_identifier") != expected:
        raise RuntimeError("参考姿态未登记相同设备身份；登记后请重新保存 anchor")
    check_can(args.channel)
    actual = read_identity(args.channel)  # Whitelisted identity queries only.
    if actual != expected:
        raise RuntimeError("连接设备与登记/参考姿态不一致")
    return actual


def one_trial(args, link, cad, mapping, anchor, identity, keys):
    path = audit_plan(anchor, args, mapping, cad, lambda: link.pump(0.))
    link.ready(mapping, args.fault_policy)
    if not pose_matches(path[0], link.latest["q"]):
        differences = "; ".join(
            f"slot {i}: anchor={path[0][i]}, 当前={link.latest['q'][i]}, "
            f"差值={link.latest['q'][i] - path[0][i]:+d}"
            for i in tuple(range(11)) + tuple(range(15, 20))
            if abs(path[0][i] - link.latest["q"][i]) > 2)
        raise RuntimeError("当前姿态与固定 anchor 相差超过 2 计数（" + differences +
                           "）；请恢复参考姿态，不能累积更换起点；未发送运动目标")
    metadata = {"mode": "probe", "synthetic": False, "hand": "left", "trial_id": args.trial_id,
                "condition": args.condition, "object_id": args.object_id, "slot": args.slot,
                "delta_raw": args.delta, "speed_raw_s": args.speed, "pre_s": args.pre, "post_s": args.post,
                "max_gap_raw": args.max_gap, "return_to_start_requested": args.return_to_start,
                "fault_policy": args.fault_policy, "identifier": identity, "channel": args.channel,
                "anchor_path": str(Path(args.anchor).resolve()), "anchor_raw": path[0],
                "mapping_sha256": mapping.sha256, "model_sha256": mapping.model_hash,
                "note": args.note, "can_logging": link.bus is not None,
                "internal_speed_torque_settings_changed_by_study": False}
    if hasattr(args, "plan_csv"):
        import hashlib
        metadata["plan_csv"] = str(Path(args.plan_csv).resolve())
        metadata["plan_sha256"] = hashlib.sha256(Path(args.plan_csv).read_bytes()).hexdigest()
    journal = Journal(args.data_dir, metadata)
    link.journal = journal
    status, reason = "aborted", "未开始"
    try:
        journal.write("device_snapshot", data=link.latest["info"], graph=link.graph(), measured_raw=link.latest["q"])
        print(f"\n{args.trial_id} | {args.condition} | 物体 {args.object_id} | slot {args.slot} | "
              f"{path[0][args.slot]} → {path[-1][args.slot]} | {args.speed:g} count/s", flush=True)
        print("布置好物体和视频后按 Enter 开始；c=观察到接触并结束继续闭合，r=脱离，v=视频同步，q/Esc=停止新目标。", flush=True)
        print(f"数据：{journal.path}", flush=True)
        while True:
            link.pump()
            link.guard(mapping, args.fault_policy)
            start, _ = operator_keys(keys, journal)
            if start:
                break
        if not pose_matches(path[0], link.latest["q"]):
            differences = "; ".join(
                f"slot {i}: anchor={path[0][i]}, 当前={link.latest['q'][i]}, "
                f"差值={link.latest['q'][i] - path[0][i]:+d}"
                for i in tuple(range(11)) + tuple(range(15, 20))
                if abs(path[0][i] - link.latest["q"][i]) > 2)
            raise RuntimeError("等待期间参考姿态改变（" + differences + "）；没有开始实验")
        link.enable_publisher()
        link.guard(mapping, args.fault_policy)
        journal.write("trial_start", measured_raw=link.latest["q"])
        last = path[0].copy()
        phase, phase_at = "pre", time.monotonic()
        next_tick = phase_at
        contact_seen = False
        settle_states = 0
        last_settle_ns = 0
        total_deadline = phase_at + args.pre + 2 * abs(args.delta) / min(args.speed, 5) + args.post + 15
        journal.write("phase", phase=phase)
        while time.monotonic() < total_deadline:
            link.pump()
            link.guard(mapping, args.fault_policy)
            _, contact = operator_keys(keys, journal)
            if contact:
                contact_seen = True
                if phase in ("pre", "ramp"):
                    phase, phase_at = "post", time.monotonic()
                    journal.write("phase", phase=phase, cause="operator_contact_mark")
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = now + 1 / 30  # No burst of queued targets after a slow check.
            if phase == "pre" and now - phase_at >= args.pre:
                phase, phase_at = "ramp", now
                journal.write("phase", phase=phase)
            if phase == "ramp" and last[args.slot] == path[-1][args.slot]:
                phase, phase_at = "post", now
                journal.write("phase", phase=phase, cause="endpoint_reached")
            if phase == "post" and now - phase_at >= args.post:
                if args.return_to_start:
                    phase, phase_at = "return", now
                    return_start = last[args.slot]
                    journal.write("phase", phase=phase)
                else:
                    status, reason = "complete", "无自动返回请求，留在最后目标"
                    break
            if phase == "return" and last[args.slot] == path[0][args.slot]:
                phase, phase_at = "settle", now
                journal.write("phase", phase=phase)
            if phase == "settle":
                if link.latest["state_ns"] != last_settle_ns:
                    last_settle_ns = link.latest["state_ns"]
                    settled = all(abs(last[i] - link.latest["q"][i]) <= 2 for i in INDEX_SLOTS)
                    settle_states = settle_states + 1 if settled else 0
                if settle_states >= 5:
                    status, reason = "complete", "已按请求缓慢返回参考姿态"
                    break
                if now - phase_at > 3:
                    raise RuntimeError("返回后 3 秒未稳定到参考姿态；停止后续试验")
            desired = last.copy()
            if phase == "ramp":
                offset = min(abs(args.delta), math_floor((now - phase_at) * args.speed))
                desired[args.slot] = path[0][args.slot] + (1 if args.delta > 0 else -1) * offset
            elif phase == "return":
                offset = min(abs(path[0][args.slot] - return_start), math_floor((now - phase_at) * min(args.speed, 5)))
                desired[args.slot] = return_start + (1 if path[0][args.slot] > return_start else -1) * offset
            candidate = last.copy()
            candidate[args.slot] += max(-1, min(1, desired[args.slot] - last[args.slot]))
            gaps = {str(i): abs(candidate[i] - link.latest["q"][i]) for i in INDEX_SLOTS}
            if max(gaps.values()) > args.max_gap:
                journal.write("acquisition_limit", reason="command_feedback_gap", gaps_raw=gaps,
                              attempted_slot_raw=candidate[args.slot], limit_raw=args.max_gap)
                raise RuntimeError("命令与反馈差超过采集上限；本次属于截断数据，未继续闭合，也不自动返回")
            before = link.latest["q"].copy()
            audited = before.copy()
            for i in INDEX_SLOTS:
                audited[i] = candidate[i]
            cad_started = time.monotonic_ns()
            checked = cad.check(before, audited, pump=lambda: link.pump(0.))
            cad_ms = (time.monotonic_ns() - cad_started) / 1e6
            link.guard(mapping, args.fault_policy)
            if not checked.get("ok"):
                journal.write("acquisition_limit", reason="cad", check=checked)
                raise RuntimeError("CAD 拒绝当前食指路径；停止新目标")
            if any(abs(before[i] - link.latest["q"][i]) > 2 for i in tuple(range(11)) + tuple(range(15, 20))):
                raise RuntimeError("CAD 检查期间姿态变化超过容差；停止新目标")
            # Check again after a potentially slow CAD reply; never publish based on stale feedback.
            if any(abs(candidate[i] - link.latest["q"][i]) > args.max_gap for i in INDEX_SLOTS):
                raise RuntimeError("CAD 检查后命令反馈差超过上限")
            message = link.JointState()
            message.header.stamp = link.node.get_clock().now().to_msg()
            message.name = ["index_mcp_roll", "index_mcp_pitch", "index_pip"]
            message.position = [float(candidate[i]) for i in INDEX_SLOTS]
            published_ns = time.monotonic_ns()
            link.publisher.publish(message)
            journal.write("command_sent", mono_ns=published_ns,
                          publish_return_ns=time.monotonic_ns(), phase=phase,
                          slot_values={str(i): candidate[i] for i in INDEX_SLOTS},
                          desired_slot_raw=desired[args.slot], observed_raw=link.latest["q"],
                          state_receive_age_ms=(published_ns - link.latest["state_ns"]) / 1e6,
                          cad_ms=cad_ms, velocity_and_effort_empty=True)
            last = candidate
        else:
            raise RuntimeError("实验超过限定时长；停止新目标")
        if contact_seen and args.condition == "free":
            status, reason = "condition_mismatch", "标为空载但操作者标记了接触；不能作为空载基线"
        if args.condition == "contact" and not contact_seen:
            reason += "；未人工标记接触，不能据此假定实际发生接触"
        journal.write("trial_end", status=status, reason=reason, contact_marked=contact_seen,
                      last_command_raw=last, measured_raw=link.latest["q"])
    except KeyboardInterrupt:
        reason = "操作者中止；没有执行自动返回"
        raise
    except Exception as error:
        reason = str(error)
        journal.write("error", error=reason)
        raise
    finally:
        journal.close(status, reason=reason, last_feedback_raw=link.latest["q"])
        link.journal = None
        print(f"{status}: {reason}\n已保存：{journal.path}", flush=True)
    return journal.path


def math_floor(value):
    import math
    return math.floor(value + 1e-9)


def run_trials(args, specs):
    from live_hand import CadWorker
    mapping = mapping_instance()
    anchor = check_anchor_file(args.anchor, mapping)
    link, cad = None, None
    completed = []
    try:
        link = LiveLink(channel=args.channel, can_log=not args.no_can_log)
        link.ready(mapping, args.fault_policy)
        identity = verify_identity(args, anchor)
        cad = CadWorker("swapped")
        with Keys() as keys:
            for spec in specs:
                completed.append(one_trial(spec, link, cad, mapping, anchor, identity, keys))
    finally:
        if link is not None:
            link.close()
        if cad is not None:
            cad.close()
        print("采集器退出：停止发布新目标；不在异常退出时自动回位。Ctrl+C 不是硬件急停。", flush=True)
        if completed:
            print(f"已完成 {len(completed)} 次。分析命令：./lh20-study analyze '{Path(args.data_dir)}'", flush=True)
    return completed


def record(args):
    mapping = mapping_instance()
    link = LiveLink(channel=args.channel, can_log=not args.no_can_log)
    journal = Journal(args.data_dir, {"mode": "record", "synthetic": False, "hand": "left",
                      "trial_id": args.trial_id, "condition": args.condition, "object_id": args.object_id,
                      "slot": args.slot, "note": args.note, "motion_publisher_created": False,
                      "can_logging": link.bus is not None})
    link.journal = journal
    status = "aborted"
    try:
        link.ready(mapping, args.fault_policy, stable=False, exclusive=False)
        journal.write("trial_start")
        print(f"仅订阅和记录，不创建动作发布器。{args.duration:g} 秒。\n{journal.path}\nc=接触 r=脱离 v=视频同步 q=结束", flush=True)
        deadline = time.monotonic() + args.duration
        with Keys() as keys:
            while time.monotonic() < deadline:
                link.pump()
                operator_keys(keys, journal)
        status = "complete"
    except KeyboardInterrupt:
        status = "operator_stopped"
    except Exception as error:
        journal.write("error", error=str(error))
        raise
    finally:
        journal.close(status)
        link.close()
        print(f"记录已保存：{journal.path}", flush=True)
    return journal.path
