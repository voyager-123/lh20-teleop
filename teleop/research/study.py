#!/usr/bin/env python3
"""LH20 今日接触/空载验证：计划、采集、离线演示和分析。"""
from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import sys
import time

from study_core import DATA, Journal, ROOT, plan_rows, validate_trial, write_csv


def generate_demo(base):
    """Generate labeled artificial rollouts; opens no hardware or ROS graph."""
    import numpy as np
    rng = np.random.default_rng(20261023)
    base = Path(base)
    base.mkdir(parents=True, exist_ok=False)
    paths = []
    for spec in plan_rows(2):
        speed, contact = spec["speed_raw_s"], spec["condition"] == "contact"
        start = time.monotonic_ns()
        journal = Journal(base, {"mode": "probe", "synthetic": True, **spec,
                          "slot": 1, "delta_raw": -15, "speed_raw_s": speed}, start_ns=start)
        q = np.full(20, 200, dtype=int)
        actual = 200.
        duration = 1 + 15 / speed + 1
        phase = None
        last_readback = 200
        contact_marked = False
        for k in range(int(duration * 30) + 1):
            t = k / 30
            raw = 200 - min(15, max(0, int((t - 1) * speed)))
            current_phase = "pre" if t < 1 else "ramp" if t < 1 + 15 / speed else "post"
            if current_phase != phase:
                phase = current_phase
                journal.write("phase", mono_ns=start + int(t * 1e9), phase=phase)
            journal.write("command_sent", mono_ns=start + int(t * 1e9), phase=phase,
                          slot_values={"6": 200, "1": raw, "16": 200}, desired_slot_raw=raw)
            delayed = 200 - min(15, max(0, (t - 1 - .12) * speed))
            if contact:
                delayed = max(192., delayed)
            actual += .35 * (delayed - actual)
            # Publish at 30 Hz, but keep every second readback cached.
            new_readback = int(round(actual + rng.normal(0, .1))) if k % 2 == 0 else last_readback
            q[1] = new_readback
            journal.write("state", mono_ns=start + int((t + .006) * 1e9), valid=True,
                          position_raw=q.tolist(), ros_header_ns=int((t + .004) * 1e9),
                          identical_to_previous_message=new_readback == last_readback)
            last_readback = new_readback
            if contact and actual < 192.4 and not contact_marked:
                journal.write("marker", mono_ns=start + int((t + .015) * 1e9),
                              label="contact_observed", source="synthetic_operator", timing_is_ground_truth=False)
                contact_marked = True
        journal.close("complete", end_ns=start + int((duration + .05) * 1e9),
                      reason="SYNTHETIC: not hardware")
        paths.append(journal.path)
    return paths


def common(parser):
    parser.add_argument("--data-dir", type=Path, default=DATA, help="会话根目录；每次自动建立不覆盖的子目录")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--fault-policy", choices=("stop", "ignore-64-128"), default="ignore-64-128",
                        help="沿用已验证左手的纯64/128例外，仍保存原值；组合码不忽略")
    parser.add_argument("--no-can-log", action="store_true", help="只记ROS；默认另开被动CAN收帧")
    parser.add_argument("--note", default="")


def motion(parser, batch=False):
    common(parser)
    parser.add_argument("--anchor", type=Path, required=True)
    parser.add_argument("--slot", type=int, choices=(1, 16), default=1)
    parser.add_argument("--delta", type=int, default=-5, help="协议计数，不是角度；空载幅度至多60，接触至多20；负数通常闭合")
    parser.add_argument("--speed", type=float, default=5, help="外部目标斜坡 count/s，不修改厂家速度设置")
    parser.add_argument("--pre", type=float, default=1.)
    parser.add_argument("--post", type=float, default=.4)
    parser.add_argument("--max-gap", type=float, default=8, help="采集保护上限，超出即截断，不是接触检测阈值")
    parser.add_argument("--return-to-start", action="store_true", help="正常完成后以最多5 count/s返回；异常退出不自动返回")
    parser.add_argument("--execute", action="store_true", help="连接SDK并创建食指动作发布器；默认仅离线CAD预览")
    if batch:
        parser.add_argument("plan_csv", type=Path)
    else:
        parser.add_argument("--condition", choices=("free", "contact"), required=True)
        parser.add_argument("--object-id", default="none")
        parser.add_argument("--trial-id", default="pilot")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    plan = sub.add_parser("plan", help="生成空载/接触 × 三档速度 × 重复次数的交错顺序表，无硬件")
    plan.add_argument("--output", type=Path, required=True)
    plan.add_argument("--repeats", type=int, default=5)
    anchor = sub.add_parser("anchor", help="只订阅SDK，保存固定参考姿态，不发动作")
    anchor.add_argument("--output", type=Path, required=True)
    anchor.add_argument("--fault-policy", choices=("stop", "ignore-64-128"), default="ignore-64-128")
    motion(sub.add_parser("probe", help="单次食指小幅试验；默认离线预览"))
    motion(sub.add_parser("run-plan", help="按计划逐次采集，每次Enter开始；默认预览"), batch=True)
    rec = sub.add_parser("record", help="只订阅并保存已有控制器的命令/反馈/CAN，不发动作")
    common(rec)
    rec.add_argument("--duration", type=float, default=60)
    rec.add_argument("--condition", choices=("free", "contact", "unknown"), default="unknown")
    rec.add_argument("--object-id", default="none")
    rec.add_argument("--trial-id", default="passive")
    rec.add_argument("--slot", type=int, choices=(1, 16), default=1)
    analysis = sub.add_parser("analyze", help="离线导出CSV、曲线和报告，不连接硬件")
    analysis.add_argument("source", type=Path)
    analysis.add_argument("--output", type=Path)
    analysis.add_argument("--slot", type=int, choices=(1, 16))
    demo = sub.add_parser("demo", help="生成明确标记的模拟数据并分析；无ROS/CAN")
    demo.add_argument("--output", type=Path, required=True)
    label = sub.add_parser("annotate", help="事后追加接触区间，保留来源，不改原始日志")
    label.add_argument("session", type=Path)
    label.add_argument("--start", type=float, required=True)
    label.add_argument("--end", type=float, required=True)
    label.add_argument("--state", choices=("contact", "free", "uncertain"), required=True)
    label.add_argument("--source", choices=("video", "contact_switch", "operator_review"), required=True)
    label.add_argument("--note", default="")
    return p


def read_plan(args):
    with args.plan_csv.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or len(rows) > 120:
        raise ValueError("计划必须有 1..120 次试验")
    specs, identifiers = [], set()
    for row in rows:
        spec = copy.copy(args)
        spec.trial_id, spec.condition, spec.object_id = row["trial_id"], row["condition"], row["object_id"]
        spec.slot, spec.delta, spec.speed = int(row["slot"]), int(row["delta_raw"]), float(row["speed_raw_s"])
        if not spec.trial_id or spec.trial_id in identifiers or spec.condition not in ("free", "contact"):
            raise ValueError("试验ID必须唯一且非空；condition需为free/contact")
        if spec.condition == "contact" and spec.object_id in ("", "none"):
            raise ValueError("接触试验需要物体编号")
        identifiers.add(spec.trial_id)
        validate_trial(spec.slot, spec.delta, spec.speed, spec.pre, spec.post, spec.max_gap, condition=spec.condition)
        specs.append(spec)
    if not args.return_to_start:
        raise ValueError("批量试验需要显式加 --return-to-start 保持固定起点；异常时仍不返回")
    return specs


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        if args.command == "plan":
            if not 1 <= args.repeats <= 20:
                raise ValueError("重复次数需为 1..20")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if args.output.exists():
                raise ValueError("计划文件已存在，拒绝覆盖")
            rows = plan_rows(args.repeats)
            write_csv(args.output, rows, list(rows[0]))
            print(f"已生成 {len(rows)} 次试验：{args.output.resolve()}。默认 slot1、delta=-15；先完成 -5 计数方向验证再使用。")
        elif args.command == "analyze":
            from study_analysis import analyze
            result = analyze(args.source, args.output, args.slot)
            print(f"分析完成：{result / 'report.md'}")
        elif args.command == "demo":
            from study_analysis import analyze
            paths = generate_demo(args.output)
            result = analyze(args.output)
            print(f"已生成 {len(paths)} 次 SYNTHETIC 模拟试验。不是实机数据。\n{result / 'report.md'}")
        elif args.command == "annotate":
            from study_analysis import annotate
            print(json.dumps(annotate(args.session, args.start, args.end, args.state, args.source, args.note), ensure_ascii=False))
        elif args.command == "anchor":
            from study_live import save_anchor
            save_anchor(args)
        elif args.command == "record":
            if not 1 <= args.duration <= 3600:
                raise ValueError("记录时长需为1..3600秒")
            from study_live import record
            record(args)
        else:
            specs = read_plan(args) if args.command == "run-plan" else [args]
            for spec in specs:
                validate_trial(spec.slot, spec.delta, spec.speed, spec.pre, spec.post, spec.max_gap, condition=spec.condition)
                if spec.condition == "contact" and spec.object_id in ("", "none"):
                    raise ValueError("接触试验请指定 --object-id，如 block_A")
            from study_live import preview, run_trials
            if not args.execute:
                # One trajectory per distinct configuration; no redundant CAD startups.
                unique = {}
                for spec in specs:
                    unique.setdefault((spec.slot, spec.delta, spec.speed), spec)
                for spec in unique.values():
                    preview(spec)
            else:
                if not sys.stdin.isatty():
                    raise ValueError("实机试验需在交互终端运行，以便Enter触发和q/Ctrl+C中止")
                run_trials(args, specs)
        return 0
    except KeyboardInterrupt:
        print("已中止；原始数据保留。", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, ImportError) as error:
        print("未完成：" + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
