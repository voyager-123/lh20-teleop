"""Offline descriptive analysis, never imports ROS or opens a device."""
from __future__ import annotations

from bisect import bisect_right
from datetime import datetime
import json
from pathlib import Path
import uuid

import numpy as np

from study_core import SCHEMA, causal_pairs, read_events, write_csv


def load_session(path, slot=None):
    path = Path(path)
    meta = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    if meta.get("schema") != SCHEMA:
        raise ValueError(f"不支持的 metadata 版本：{path}")
    events, warnings = read_events(path)
    slot = meta.get("slot", 1) if slot is None else slot
    pairs = causal_pairs(events, slot)
    states = [e for e in events if e["kind"] == "state" and e.get("valid")]
    commands = [e for e in events if e["kind"] in ("command_sent", "command_observed") and str(slot) in e.get("slot_values", {})]
    annotations = []
    if (path / "annotations.jsonl").exists():
        for line in (path / "annotations.jsonl").read_text(encoding="utf-8").splitlines():
            value = json.loads(line)
            if value.get("schema") != "lh20.study.annotation.v1":
                raise ValueError(f"标注格式无效：{path}")
            annotations.append(value)
    if not states:
        warnings.append("没有有效位置反馈")
    if not commands:
        warnings.append("没有该关节的命令；无法计算跟踪误差")
    if not any(e["kind"] == "session_end" for e in events):
        warnings.append("没有正常关闭标记；可能被强制中断")
    if meta.get("condition") == "contact" and not annotations:
        warnings.append("接触条件只有试验级标签；人工按键不是逐帧接触真值")
    if any(e["kind"] == "acquisition_limit" for e in events):
        warnings.append("采集上限触发，本次被截断，不能作为完整成功试验")
    if meta.get("mode") == "probe" and not any(e["kind"] == "phase" and e.get("phase") == "ramp" for e in events):
        warnings.append("没有进入闭合斜坡阶段，不能视为有效闭合试验")
    if meta.get("synthetic"):
        warnings.append("SYNTHETIC：人工生成数据，不能作为实机结果")
    return {"path": path, "meta": meta, "events": events, "slot": slot, "pairs": pairs,
            "states": states, "commands": commands, "warnings": warnings, "annotations": annotations}


def apparent_lag(session):
    """Exploratory free-trial alignment; NOT sensor latency or contact detection."""
    meta = session["meta"]
    if meta.get("mode") != "probe" or meta.get("condition") != "free" or meta.get("status") != "complete":
        return None
    commands = session["commands"]
    if not commands:
        return None
    slot = str(session["slot"])
    ct = np.array([e["t_s"] for e in commands])
    cq = np.array([e["slot_values"][slot] for e in commands])
    # Use a common evaluation window across all candidate shifts; no future commands.
    pairs = [p for p in session["pairs"] if p["phase"] in ("ramp", "post") and p["t_s"] >= ct[0] + .6]
    if len(pairs) < 8 or np.ptp([p["measured_raw"] for p in pairs]) < 3:
        return None
    ts = np.array([p["t_s"] for p in pairs])
    measured = np.array([p["measured_raw"] for p in pairs])
    shifts = np.linspace(0, .6, 61)
    losses = []
    for shift in shifts:
        indices = np.searchsorted(ct, ts - shift, side="right") - 1
        losses.append(float(np.mean((cq[indices] - measured) ** 2)))
    best = int(np.argmin(losses))
    return {"alignment_ms": float(shifts[best] * 1000), "rmse_raw": float(np.sqrt(losses[best])),
            "at_search_boundary": best == len(shifts) - 1,
            "meaning": "apparent external-command to ROS-readback alignment, not physical acquisition latency",
            "fitted_and_evaluated_on_same_trial": True}


def session_summary(session):
    meta, states = session["meta"], session["states"]
    # Return and settling are excluded from primary descriptive error summaries.
    selected = [p for p in session["pairs"] if p["phase"] in ("pre", "ramp", "post", "external_unknown")]
    errors = np.array([abs(p["error_raw"]) for p in selected])
    intervals = np.diff([e["mono_ns"] for e in states]) / 1e6
    lag = apparent_lag(session)
    frames = [e for e in session["events"] if e["kind"] == "can_frame"]
    result = {"trial_id": meta.get("trial_id"), "session_path": str(session["path"].resolve()),
              "mode": meta.get("mode"), "synthetic": bool(meta.get("synthetic")),
              "condition": meta.get("condition"), "object_id": meta.get("object_id"),
              "status": meta.get("status", "unclosed"), "slot": session["slot"],
              "speed_raw_s": meta.get("speed_raw_s"), "delta_raw": meta.get("delta_raw"),
              "state_messages": len(states), "command_messages": len(session["commands"]),
              "paired_state_messages": len(selected), "can_frames": len(frames),
              "can_local_echo_frames": sum(e.get("is_rx") is False for e in frames),
              "can_rx_frames": sum(e.get("is_rx") is True for e in frames),
              "ros_message_hz": float(1000 / np.mean(intervals)) if len(intervals) and np.mean(intervals) > 0 else None,
              "ros_interval_p95_ms": float(np.percentile(intervals, 95)) if len(intervals) else None,
              "identical_position_fraction": float(np.mean([e.get("identical_to_previous_message", False) for e in states])) if states else None,
              "error_abs_p95_raw": float(np.percentile(errors, 95)) if len(errors) else None,
              "error_abs_max_raw": float(np.max(errors)) if len(errors) else None,
              "error_rmse_raw": float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None,
              "command_excursion_raw": float(np.ptp([p["command_raw"] for p in selected])) if selected else None,
              "feedback_excursion_raw": float(np.ptp([p["measured_raw"] for p in selected])) if selected else None,
              "apparent_alignment_ms": lag["alignment_ms"] if lag else None,
              "alignment_at_search_boundary": lag["at_search_boundary"] if lag else None,
              "annotations": len(session["annotations"]),
              "operator_contact_marks": sum(e["kind"] == "marker" and e.get("label") == "contact_observed" for e in session["events"]),
              "warnings": "；".join(session["warnings"])}
    return result


def plot_session(session, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    meta, slot = session["meta"], session["slot"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout="constrained")
    commands = session["commands"]
    if commands:
        axes[0].step([e["t_s"] for e in commands], [e["slot_values"][str(slot)] for e in commands],
                     where="post", label="External command", color="#2457a6")
    if session["states"]:
        axes[0].plot([e["t_s"] for e in session["states"]], [e["position_raw"][slot] for e in session["states"]],
                     ".-", markersize=2, linewidth=.8, label="ROS readback", color="#e67b24")
    pairs = session["pairs"]
    if pairs:
        axes[1].plot([p["t_s"] for p in pairs], [p["error_raw"] for p in pairs], color="#7b3f95", linewidth=1)
    states = session["states"]
    if len(states) > 1:
        intervals_ms = np.diff([e["mono_ns"] for e in states]) / 1e6
        axes[2].plot([e["t_s"] for e in states[1:]], intervals_ms,
                     ".", markersize=3, color="#218474", label="ROS arrival interval")
        axes[2].set_ylim(0, max(50, float(np.max(intervals_ms)) * 1.1))
        axes[2].ticklabel_format(axis="y", style="plain", useOffset=False)
    marks = [e for e in session["events"] if e["kind"] == "marker"]
    for event in marks:
        for axis in axes:
            axis.axvline(event["t_s"], color="#b75b19", alpha=.6, linestyle="--", linewidth=.8)
        axes[0].annotate(event["label"].replace("_", " "), (event["t_s"], .98),
                         xycoords=("data", "axes fraction"), rotation=90, va="top", fontsize=7)
    for label in session["annotations"]:
        color = "#41a66d" if label["state"] == "contact" else "#78a5d2"
        for axis in axes:
            axis.axvspan(label["start_s"], label["end_s"], color=color, alpha=.13)
    phases = [e for e in session["events"] if e["kind"] == "phase"]
    for event in phases:
        axes[1].axvline(event["t_s"], color="gray", linewidth=.6, alpha=.5)
        axes[1].annotate(event["phase"], (event["t_s"], .97), xycoords=("data", "axes fraction"), fontsize=7, va="top")
    axes[0].set_ylabel(f"Slot {slot} (raw counts)")
    axes[1].set_ylabel("Command - readback\n(raw counts)")
    axes[2].set_ylabel("Arrival interval (ms)")
    axes[2].set_xlabel("Time since session start (s)")
    prefix = "SYNTHETIC / NOT HARDWARE | " if meta.get("synthetic") else ""
    axes[0].set_title(prefix + f"{meta.get('trial_id')} | {meta.get('condition')} | status={meta.get('status', 'unclosed')}")
    for axis in axes:
        axis.grid(alpha=.2)
    if commands or states:
        axes[0].legend(loc="best")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def export_session(session, output):
    output.mkdir()
    fields = ["t_s", "mono_ns", "command_raw", "measured_raw", "error_raw", "command_age_ms", "phase",
              "identical_to_previous_message", "ros_header_ns", "source"]
    write_csv(output / "paired.csv", session["pairs"], fields)
    feedback = []
    for event in session["states"]:
        feedback.append({"t_s": event["t_s"], "mono_ns": event["mono_ns"],
                         "ros_header_ns": event.get("ros_header_ns"),
                         "identical_to_previous_message": event.get("identical_to_previous_message"),
                         **{f"q{i}_raw": v for i, v in enumerate(event["position_raw"])}})
    write_csv(output / "feedback.csv", feedback, ["t_s", "mono_ns", "ros_header_ns", "identical_to_previous_message"] + [f"q{i}_raw" for i in range(20)])
    frames = [e for e in session["events"] if e["kind"] == "can_frame"]
    write_csv(output / "can_frames.csv", frames, ["t_s", "mono_ns", "kernel_wall_s", "arbitration_id", "is_rx", "data_hex", "dlc", "is_error", "is_remote"])
    plot_session(session, output / "trajectory.png")


def overview(summaries, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    kinds = sorted(set((s["synthetic"], s["slot"], str(s["delta_raw"])) for s in summaries))
    fig, axes = plt.subplots(1, len(kinds), figsize=(7 * len(kinds), 4.5), squeeze=False, layout="constrained")
    for index, (synthetic, slot, delta) in enumerate(kinds):
        axis = axes[0][index]
        for condition, color, offset in (("free", "#2457a6", -.25), ("contact", "#e67b24", .25)):
            rows = [s for s in summaries if s["synthetic"] == synthetic and s["condition"] == condition
                    and s["slot"] == slot and str(s["delta_raw"]) == delta
                    and s["status"] == "complete" and s["speed_raw_s"] is not None and s["error_abs_p95_raw"] is not None
                    and s["command_excursion_raw"] is not None and s["command_excursion_raw"] >= 2]
            if rows:
                axis.scatter([s["speed_raw_s"] + offset for s in rows], [s["error_abs_p95_raw"] for s in rows],
                             c=color, label=condition, s=38, alpha=.7)
        axis.set_title(("SYNTHETIC / NOT HARDWARE" if synthetic else "HARDWARE") +
                       f" | slot {slot} | delta {delta}\nDescriptive trial errors, not contact accuracy")
        axis.set_xlabel("Requested command ramp speed (raw counts/s)")
        axis.set_ylabel("95th percentile absolute error (raw counts)")
        axis.grid(alpha=.2)
        if axis.get_legend_handles_labels()[0]:
            axis.legend()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def analyze(source, output=None, slot=None):
    source = Path(source).expanduser().resolve()
    if source.is_file():
        source = source.parent
    paths = [source] if (source / "metadata.json").exists() else sorted(p.parent for p in source.rglob("metadata.json"))
    if not paths:
        raise ValueError("没有找到 metadata.json 和 events.jsonl 会话")
    sessions = [load_session(p, slot) for p in paths]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    output = Path(output).expanduser().resolve() if output else source / ("analysis_" + stamp)
    output.mkdir(parents=True, exist_ok=False)
    summaries = []
    for i, session in enumerate(sessions, 1):
        row = session_summary(session)
        subdir = f"trial_{i:03d}"
        export_session(session, output / subdir)
        row["plot"] = subdir + "/trajectory.png"
        summaries.append(row)
    (output / "summary.json").write_text(json.dumps(summaries, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_csv(output / "summary.csv", summaries, list(summaries[0]))
    overview(summaries, output / "overview.png")
    text = ["# LH20 采集质量与初步响应分析", "",
            "本报告只做描述性统计，不训练模型，不报告接触检测准确率。", "",
            "- 协议计数不是标定角度；命令是外部 ROS 命令，不是厂家内部实际参考。",
            "- ROS 头时间为 SDK 发布时刻；到达频率和相同位置比例不能证明传感器刷新率。",
            "- CAN 本地回环不是硬件确认，CAN 收帧时刻也不是传感器采样时刻。",
            "- 表观对齐估计只针对完整空载试验，混合了伺服和反馈效果，且在同条轨迹上拟合；不是视觉端到端延迟。",
            "- 接触条件表示实验布置；c 按键含人工反应延迟，不能直接作为接触时刻真值。",
            "- 中止/达到偏差上限的数据仍保留，标记为截断；主图排除未完成试验。",
            "- 不同关节和请求幅度分开绘图；没有实际命令行程的记录不放入误差对比图。",
            "- 空载/接触整段误差差异会受接触时刻、实际行程和保持时长影响，不能直接解释成分类性能。", ""]
    if any(s["synthetic"] for s in summaries):
        text += ["**本次包含 SYNTHETIC 模拟数据，不能作为实机实验结果。**", ""]
    text += ["![总览](overview.png)", "", "| 试验 | 条件 | 状态 | ROS消息Hz | 误差P95 raw | 备注 |", "|---|---|---|---:|---:|---|"]
    for row in summaries:
        rate = f"{row['ros_message_hz']:.1f}" if row["ros_message_hz"] is not None else "—"
        err = f"{row['error_abs_p95_raw']:.2f}" if row["error_abs_p95_raw"] is not None else "—"
        text.append(f"| [{row['trial_id']}]({row['plot']}) | {row['condition']} | {row['status']} | {rate} | {err} | {row['warnings']} |")
    text += ["", "下一步：先看同速度空载重复性，再确认接触前后响应是否明显不同；如反馈几乎不随接触改变，先检查真实读回和采样链路。", ""]
    (output / "report.md").write_text("\n".join(text), encoding="utf-8")
    return output


def annotate(session_path, start_s, end_s, state, source, note):
    session = load_session(session_path)
    maximum = max((e["t_s"] for e in session["events"]), default=0)
    if not np.isfinite(start_s) or not np.isfinite(end_s) or not 0 <= start_s < end_s <= maximum:
        raise ValueError(f"标注需满足 0 <= start < end <= {maximum:.3f} 秒")
    if state not in ("contact", "free", "uncertain") or source not in ("video", "contact_switch", "operator_review"):
        raise ValueError("未知标注状态或来源")
    value = {"schema": "lh20.study.annotation.v1", "id": uuid.uuid4().hex, "start_s": start_s,
             "end_s": end_s, "state": state, "source": source, "note": note,
             "created_at": datetime.now().astimezone().isoformat()}
    with (Path(session_path) / "annotations.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
    return value
