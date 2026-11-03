"""Pure acquisition primitives. No ROS, CAN, camera, or device initialization."""
from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import time
import uuid
from zoneinfo import ZoneInfo

SCHEMA = "lh20.study.v1"
ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "teleop/data/study"
INDEX_SLOTS = (6, 1, 16)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def positions(values):
    values = list(values)
    if len(values) != 20 or any(not finite(v) or not 0 <= v <= 255 or v != round(v) for v in values):
        raise ValueError("需要 20 槽位、0..255 的整数实测位置")
    return [int(v) for v in values]


def amplitude_limit(condition):
    if condition not in ("free", "contact"):
        raise ValueError("动作试验条件必须为 free 或 contact")
    return 60 if condition == "free" else 20


def validate_trial(slot, delta, speed, pre, post, max_gap, condition="contact"):
    if slot not in (1, 16):
        raise ValueError("今天仅支持食指根部 slot 1 或末端 slot 16")
    limit = amplitude_limit(condition)
    if type(delta) is not int or not 1 <= abs(delta) <= limit:
        raise ValueError(f"{condition} 条件下 delta 绝对值需为 1..{limit} 个协议计数")
    for name, value, lo, hi in (("speed", speed, 1, 20), ("pre", pre, .2, 10),
                                ("post", post, .1, 1), ("max_gap", max_gap, 2, 12)):
        if not finite(value) or not lo <= value <= hi:
            raise ValueError(f"{name} 必须在 {lo}..{hi} 之间")


def trajectory(anchor, slot, delta, condition="contact"):
    anchor = positions(anchor)
    limit = amplitude_limit(condition)
    if slot not in (1, 16) or type(delta) is not int or not 1 <= abs(delta) <= limit:
        raise ValueError(f"仅支持食指 1/16 槽位，{condition} 条件幅度 1..{limit}")
    if not 0 <= anchor[slot] + delta <= 255:
        raise ValueError("目标超出协议位置范围；未发送命令")
    direction = 1 if delta > 0 else -1
    result = [anchor.copy()]
    for step in range(1, abs(delta) + 1):
        q = anchor.copy()
        q[slot] += direction * step
        result.append(q)
    return result


def pose_matches(anchor, measured, tolerance=2):
    anchor, measured = positions(anchor), positions(measured)
    return all(abs(anchor[i] - measured[i]) <= tolerance for i in tuple(range(11)) + tuple(range(15, 20)))


def guard_feedback(latest, now_ns, expected_version, fault_policy="stop"):
    if latest.get("q") is None:
        raise RuntimeError("没有有效位置反馈")
    positions(latest["q"])
    if not 0 <= now_ns - latest["state_ns"] <= 300_000_000:
        raise RuntimeError("位置反馈超过 300 ms 未收到；停止新目标")
    info = latest.get("info")
    if not isinstance(info, dict) or not 0 <= now_ns - latest["info_ns"] <= 1_500_000_000:
        raise RuntimeError("设备信息超过 1.5 s 未收到，或格式无效")
    if info.get("hand_joint") != "L20-10" or info.get("version") != expected_version:
        raise RuntimeError("设备型号或固件与本机左手配置不符")
    faults = info.get("fault")
    if not isinstance(faults, list) or len(faults) != 20 or any(type(v) is not int or not 0 <= v <= 255 for v in faults):
        raise RuntimeError("故障反馈格式无效")
    if fault_policy not in ("stop", "ignore-64-128"):
        raise ValueError("未知故障策略")
    ignored = (0, 64, 128) if fault_policy == "ignore-64-128" else (0,)
    bad = [(i, v) for i, v in enumerate(faults) if v not in ignored]
    if bad:
        raise RuntimeError(f"设备故障 {bad}；停止新目标")


def clean_name(value):
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._")[:70]
    return value or "trial"


def code_hashes():
    paths = list(Path(__file__).parent.glob("*.py")) + [
        ROOT / "teleop/bridge/factory_mapping.py",
        ROOT / "teleop/config/l20_10_left_factory.json",
        ROOT / "src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand.py"]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}


class Journal:
    def __init__(self, base, metadata, start_ns=None):
        stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
        self.path = Path(base) / f"{stamp}_{clean_name(metadata.get('trial_id', metadata['mode']))}_{uuid.uuid4().hex[:6]}"
        self.path.mkdir(parents=True, exist_ok=False)
        self.start_ns = time.monotonic_ns() if start_ns is None else start_ns
        self.metadata = {"schema": SCHEMA, "created_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
                         "start_monotonic_ns": self.start_ns, "start_wall_ns": time.time_ns(),
                         "code_sha256": code_hashes(), "protocol_position_unit": "raw_count_not_calibrated_angle",
                         "ros_header_meaning": "SDK publication time; NOT sensor acquisition time",
                         "command_meaning": "external ROS publication; NOT vendor internal servo reference or CAN acknowledgement",
                         "can_timestamp_meaning": "kernel frame arrival time; NOT sensor acquisition time",
                         "manual_mark_meaning": "operator keypress, contains human reaction delay; NOT exact contact ground truth",
                         **metadata}
        self.save_metadata()
        self.stream = (self.path / "events.jsonl").open("x", encoding="utf-8")
        self.count = 0

    def save_metadata(self):
        temp = self.path / "metadata.tmp"
        temp.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temp.replace(self.path / "metadata.json")

    def write(self, kind, mono_ns=None, **values):
        mono_ns = time.monotonic_ns() if mono_ns is None else mono_ns
        event = {"schema": SCHEMA, "seq": self.count, "kind": kind, "mono_ns": mono_ns,
                 "t_s": (mono_ns - self.start_ns) / 1e9, "wall_ns": time.time_ns(), **values}
        self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        self.stream.flush()
        self.count += 1
        return event

    def close(self, status, end_ns=None, **values):
        if self.stream.closed:
            return
        self.write("session_end", mono_ns=end_ns, status=status, **values)
        self.stream.close()
        self.metadata.update(status=status, event_count=self.count, **values)
        self.save_metadata()


def read_events(path):
    """Preserve a crashed recording; only tolerate a truncated final line."""
    source = Path(path) / "events.jsonl"
    events, warnings = [], []
    with source.open(encoding="utf-8") as stream:
        lines = stream.readlines()
    for i, line in enumerate(lines):
        try:
            event = json.loads(line)
            if event.get("schema") != SCHEMA or type(event.get("mono_ns")) is not int:
                raise ValueError("错误的记录版本或时间戳")
            events.append(event)
        except (ValueError, TypeError) as error:
            if i == len(lines) - 1 and not line.endswith("\n"):
                warnings.append("最后一行被中断，已保留此前数据")
            else:
                raise ValueError(f"{source}:{i + 1}: {error}") from error
    return sorted(events, key=lambda e: (e["mono_ns"], e.get("seq", 0))), warnings


def causal_pairs(events, slot):
    """Join ONLY preceding external command receptions to state receptions."""
    latest = None
    result = []
    for event in sorted(events, key=lambda e: (e["mono_ns"], e.get("seq", 0))):
        if event["kind"] in ("command_sent", "command_observed") and event.get("slot_values", {}).get(str(slot)) is not None:
            latest = event
        elif event["kind"] == "state" and event.get("valid") and latest is not None:
            measured = event["position_raw"][slot]
            command = latest["slot_values"][str(slot)]
            result.append({"t_s": event["t_s"], "mono_ns": event["mono_ns"], "command_raw": command,
                           "measured_raw": measured, "error_raw": command - measured,
                           "command_age_ms": (event["mono_ns"] - latest["mono_ns"]) / 1e6,
                           "command_t_s": latest["t_s"], "phase": latest.get("phase", "external_unknown"),
                           "identical_to_previous_message": event.get("identical_to_previous_message", False),
                           "ros_header_ns": event.get("ros_header_ns"), "source": latest["kind"]})
    return result


def write_csv(path, rows, fields):
    with Path(path).open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def plan_rows(repeats=5):
    # Pair free/contact trials to reduce a systematic warm-up/order confound.
    rows = []
    for rep in range(1, repeats + 1):
        speeds = (5, 10, 20) if rep % 2 else (20, 10, 5)
        for speed in speeds:
            for condition in (("free", "contact") if rep % 2 else ("contact", "free")):
                rows.append({"trial_id": f"{condition}_s{speed}_r{rep:02d}", "condition": condition,
                             "speed_raw_s": speed, "slot": 1, "delta_raw": -15,
                             "object_id": "none" if condition == "free" else "block_A",
                             "repeat": rep, "session_path": "", "video_file": "", "notes": ""})
    return rows
