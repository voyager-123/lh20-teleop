#!/usr/bin/env python3
from runtime_paths import vision_python as resolve_vision_python
"""Small index-finger probe. Default: subscribe and print, without publishing."""
import argparse
import bisect
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import time
from fault_codes import fault_names

ACTIVE = tuple(range(11)) + tuple(range(15, 20))
ROOT = Path(__file__).resolve().parents[1]
INDEX_TOPIC = "/cb_right_hand_index_control_cmd"
FULL_HAND_TOPIC = "/cb_right_hand_control_cmd"
SLOT_NAMES = {5: "thumb abduction", 10: "thumb horizontal abduction"}
MAX_PROBE_DELTA = 20
PUBLISH_HZ = 30.0  # Match the working ROS2 GUI's approximately 31 Hz stream.
FAULT_PAUSE_TIMEOUT = 3.0
FAULT_CLEAR_STABLE = 0.25
MAX_TOTAL_FAULT_PAUSE = 10.0
RESUME_POSITION_TOLERANCE = 3
PREFLIGHT_FEEDBACK_TOLERANCE = 2  # Same resolution/jitter bound used by stable().


def validate_full_hand_execution(args):
    if getattr(args, "full_hand", False) and args.execute:
        if getattr(args, "mode", "relative") != "absolute" or not getattr(args, "mapping_profile", None):
            raise ValueError("Physical full-hand replay requires --mode absolute --mapping-profile with official G20 or measured "
                             "L20-10 mapping. Legacy L20/relative mapping is offline diagnostic only "
                             "after observed index-middle contact; code 64 is not ignored.")


def vision_subprocess_env(vision_python):
    """Keep ROS's Python/ELF paths out of the separate vision interpreter.

    source install/setup.bash adds the ROS Python 3.11 site-packages to
    PYTHONPATH. Selecting a Python 3.10 executable alone does not undo that.
    This changes only the offline child's environment, never the ROS parent.
    """
    prefix = Path(vision_python).expanduser().absolute().parent.parent
    env = dict(os.environ)
    for key in tuple(env):
        if key.startswith("PYTHON") or key == "LD_PRELOAD":
            env.pop(key)
    env["PATH"] = str(prefix / "bin") + os.pathsep + env.get("PATH", os.defpath)
    env["LD_LIBRARY_PATH"] = str(prefix / "lib")
    env["CONDA_PREFIX"] = str(prefix)
    env["CONDA_DEFAULT_ENV"] = prefix.name
    return env


def measured_preflight(args, baseline):
    vision_python = getattr(args, "vision_python", resolve_vision_python())
    with tempfile.TemporaryDirectory(prefix="lh20_preflight_") as tmp:
        output = Path(tmp) / "plan.json"
        result = subprocess.run([str(vision_python), "-E", "-s", str(ROOT / "vision/prepare_replay.py"),
                                 "--recording", str(args.recording), "--mapping-profile", str(args.mapping_profile),
                                 "--baseline", json.dumps(baseline), "--output", str(output)],
                                capture_output=True, text=True, timeout=60,
                                env=vision_subprocess_env(vision_python))
        if result.returncode:
            raise ValueError("Offline measured CAD preflight refused replay: " + result.stderr[-4000:])
        return json.loads(output.read_text())


def preflight_feedback_check(reference, current, samples=()):
    """Allow bounded readback jitter, while preserving the audited command anchor.

    This is a measurement tolerance, not a CAD clearance margin. The motion
    targets/return baseline remain the exact pose that the CAD audit checked.
    """
    reference, current = validate_positions(reference), validate_positions(current)
    observations = [current] + [validate_positions(q) for q in samples]
    excursions = [{"slot": i, "reference": reference[i], "latest": current[i],
                   "max_difference_units": max(abs(q[i] - reference[i]) for q in observations)}
                  for i in ACTIVE]
    excessive = [item for item in excursions if item["max_difference_units"] > PREFLIGHT_FEEDBACK_TOLERANCE]
    if excessive:
        raise ValueError("Pose moved beyond preflight feedback tolerance (2 protocol units); no target sent: " +
                         json.dumps(excessive))
    return {"tolerance_protocol_units": PREFLIGHT_FEEDBACK_TOLERANCE,
            "max_difference_units": max(item["max_difference_units"] for item in excursions),
            "latest_differences": [{"slot": i, "delta": current[i] - reference[i]}
                                   for i in ACTIVE if current[i] != reference[i]],
            "observed_feedback": current, "command_anchor": reference,
            "command_anchor_changed": False}


def load_index_recording(path):
    """Read named root angles and capture times; never treat radians as protocol units."""
    samples = []
    first_ns = previous_ns = None
    with Path(path).open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            row = json.loads(line)
            if row.get("schema") != "lh20.retarget.v1" or row.get("valid") is not True:
                raise ValueError(f"Recording line {line_number}: invalid tracking/schema; replay refused")
            if row.get("hand") != "right":
                raise ValueError("Only right-hand recordings are supported")
            names, values = row["joint_names"], row["position_rad"]
            if len(names) != len(values) or names.count("index_mcp_pitch") != 1:
                raise ValueError("Missing or ambiguous index_mcp_pitch")
            angle = float(values[names.index("index_mcp_pitch")])
            if not math.isfinite(angle) or not -1e-6 <= angle <= 1.4 + 1e-6:
                raise ValueError("Recorded index root angle is outside the candidate URDF range")
            captured = row["captured_monotonic_ns"]
            if not isinstance(captured, int) or (previous_ns is not None and captured <= previous_ns):
                raise ValueError("Recording capture times must increase strictly")
            first_ns = captured if first_ns is None else first_ns
            samples.append(((captured - first_ns) / 1e9, angle))
            previous_ns = captured
    if len(samples) < 2 or samples[-1][0] <= 0:
        raise ValueError("Need at least two timed recording samples")
    angles = [angle for _, angle in samples]
    low, high = min(angles), max(angles)
    if high - low < 1e-5:
        raise ValueError("Recording has no usable index root motion")
    return {"path": str(Path(path).resolve()), "times": [t for t, _ in samples],
            "angles": angles, "min_rad": low, "max_rad": high, "duration": samples[-1][0]}


def recorded_index_target(recording, seconds, baseline, span):
    """Demo normalization only: recorded angle range -> [baseline-span, baseline]."""
    base = validate_positions(baseline)
    if not 1 <= span <= MAX_PROBE_DELTA or base[1] < span:
        raise ValueError("Replay span is invalid or would exceed protocol bounds")
    times, angles = recording["times"], recording["angles"]
    index = max(0, min(len(times) - 2, bisect.bisect_right(times, seconds) - 1))
    ratio = min(1.0, max(0.0, (seconds - times[index]) / (times[index + 1] - times[index])))
    angle = angles[index] + ratio * (angles[index + 1] - angles[index])
    normalized = (angle - recording["min_rad"]) / (recording["max_rad"] - recording["min_rad"])
    base[1] -= int(round(span * min(1.0, max(0.0, normalized))))
    return base


def feedback_errors(latest, now, include_faults=True, ignore_128=False, ignore_64=False):
    errors = []
    if latest["q"] is None:
        errors.append("missing or invalid 20-slot position feedback")
    elif now - latest["state_at"] > 0.3:
        errors.append(f"position feedback stale by {now - latest['state_at']:.3f}s")
    info = latest["info"]
    if not isinstance(info, dict):
        errors.append("missing or invalid device info")
    else:
        if now - latest["info_at"] > 1.5:
            errors.append(f"device info stale by {now - latest['info_at']:.3f}s")
        if info.get("hand_joint") != "L20-10":
            errors.append(f"unexpected hand model: {info.get('hand_joint')}")
        faults = info.get("fault", [])
        if not isinstance(faults, list) or len(faults) != 20 or any(
                not isinstance(code, int) or isinstance(code, bool) or not 0 <= code <= 255
                for code in faults):
            errors.append("expected 20 integer fault entries in [0,255]")
        elif include_faults:
            for slot, code in enumerate(faults):
                if code != 0 and not (ignore_128 and code == 128) and not (ignore_64 and code == 64):
                    label = SLOT_NAMES.get(slot, f"protocol slot {slot}")
                    errors.append(f"device fault at slot {slot} ({label}): {code}; " + ", ".join(fault_names(code)))
    return errors


def observed_thumb_128_only(info):
    """Observed in the working GUI trace; this does NOT decode or dismiss the code."""
    faults = info["fault"]  # Called only after feedback/schema validation.
    return any(faults) and all(code == 0 or (slot in (5, 10) and code == 128)
                              for slot, code in enumerate(faults))


def validate_positions(values):
    q = list(values)
    if len(q) != 20 or any(not math.isfinite(x) or not 0 <= x <= 255 for x in q):
        raise ValueError("Need 20 finite feedback values in [0,255]; initial -1 messages are ignored")
    if any(abs(x - round(x)) > 1e-6 for x in q):
        raise ValueError("Expected integer-valued protocol feedback")
    return [int(round(x)) for x in q]


def plan_targets(baseline, slot, delta):
    base = validate_positions(baseline)
    if slot not in (1, 16):
        raise ValueError("This first probe only supports index base (1) or index tip (16)")
    if not isinstance(delta, int) or isinstance(delta, bool) or not 1 <= abs(delta) <= MAX_PROBE_DELTA:
        raise ValueError(f"delta must be a nonzero integer with magnitude at most {MAX_PROBE_DELTA}")
    if not 0 <= base[slot] + delta <= 255:
        raise ValueError("Target would exceed protocol bounds; no command sent")
    direction = 1 if delta > 0 else -1
    targets = []
    for offset in range(1, abs(delta) + 1):
        target = base.copy()
        target[slot] += direction * offset
        targets.append(target)
    return targets


def topic_names(hand):
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    prefix = f"/cb_{hand}_hand"
    return {"index": prefix + "_index_control_cmd", "full": prefix + "_control_cmd",
            "state": prefix + "_state", "info": prefix + "_info"}


def run(args):
    hand = getattr(args, "hand", "right")
    topics = topic_names(hand)
    if hand == "left" and (getattr(args, "full_hand", False) or getattr(args, "recording", None)):
        raise ValueError("Left-hand entry supports isolated index jog only; right-hand recordings/mappings cannot be reused")
    validate_full_hand_execution(args)  # Before ROS initialization or creating a publisher.
    full_hand = bool(getattr(args, "full_hand", False))
    absolute = full_hand and getattr(args, "mode", "relative") == "absolute"
    command_topic = topics["full"] if full_hand else topics["index"]
    other_topic = topics["index"] if full_hand else topics["full"]
    controlled_slots = ACTIVE if full_hand else (6, 1, 16)
    max_total_pause = 30.0 if absolute else MAX_TOTAL_FAULT_PAUSE
    fault_policy = getattr(args, "fault_policy", "stop")
    if fault_policy not in ("stop", "pause-thumb-128", "ignore-128", "ignore-64-128"):
        raise ValueError("Unsupported fault policy")
    recording_path = getattr(args, "recording", None)
    if full_hand:
        from hand_recording import (NAMES, absolute_hand_plan, command_pose, hand_plan,
                                    hand_target, load_hand_recording, slew_pose)
        if recording_path is None:
            raise ValueError("Full-hand replay requires a recording")
        recording = load_hand_recording(recording_path)
        if absolute:
            if getattr(args, "mapping_profile", None):
                from measured_mapping import load_profile, measured_plan
                measured_profile = load_profile(args.mapping_profile, recording)
                absolute_targets, replay_times, absolute_mapping = measured_plan(recording, measured_profile)
                if args.execute and not absolute_mapping["model_coordinate_alignment_confirmed"]:
                    raise ValueError("Manufacturer G20 raw/radian conversion is verified, but its four-finger "
                                     "MCP roll signs are not yet aligned to this URDF/GUI reference. "
                                     "Offline preflight only until coordinate alignment is confirmed; no ROS/CAN targets sent.")
            else:
                absolute_targets, absolute_mapping = absolute_hand_plan(
                    recording, middle_tip_min=getattr(args, "middle_tip_min", 10))
                replay_times = recording["times"]
        else:
            span = getattr(args, "span", 20)
            # Validate span before constructing any ROS objects.
            hand_target(recording, 0, [128] * 20, span)
    else:
        recording = load_index_recording(recording_path) if recording_path else None
    if not full_hand and recording is not None and (args.slot != 1 or args.delta >= 0):
        raise ValueError("Recording demo requires --slot 1 and a negative --delta (span <=20)")
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("lh20_hand_recording_replay" if full_hand else "lh20_index_joint_probe")
    latest = {"q": None, "state_at": 0.0, "info": None, "info_at": 0.0,
              "state_count": 0, "info_count": 0, "state_error": None}
    recent = deque(maxlen=5)
    publisher = None
    log = {"hand": hand, "slot": None if full_hand else args.slot, "delta": None if full_hand else args.delta,
           "execute_requested": args.execute,
           "return_requested": args.return_to_start, "command_topic": command_topic,
           "commands": [], "fault_changes": [], "fault_policy": fault_policy,
           "ignored_fault_codes": [64, 128] if fault_policy == "ignore-64-128" else [128] if fault_policy == "ignore-128" else [],
           "fault_pauses": [], "total_fault_pause_seconds": 0.0,
           "max_total_pause_seconds": max_total_pause}

    def active_time():
        return time.monotonic() - log["total_fault_pause_seconds"]

    def on_state(msg):
        latest["state_count"] += 1
        latest["state_at"] = time.monotonic()
        try:
            latest["q"] = validate_positions(msg.position)
            latest["state_error"] = None
            recent.append(latest["q"])
        except ValueError as error:
            latest["q"] = None
            latest["state_error"] = str(error)
            recent.clear()

    def on_info(msg):
        latest["info_count"] += 1
        latest["info_at"] = time.monotonic()
        try:
            latest["info"] = json.loads(msg.data)
            if isinstance(latest["info"], dict):
                faults = latest["info"].get("fault", [])
                if not log["fault_changes"] or log["fault_changes"][-1]["fault"] != faults:
                    log["fault_changes"].append({"time_monotonic": latest["info_at"], "fault": faults})
        except ValueError:
            latest["info"] = None

    def feedback_ok():
        return not feedback_errors(latest, time.monotonic(), ignore_128=fault_policy in ("ignore-128", "ignore-64-128"), ignore_64=fault_policy == "ignore-64-128")

    def stable():
        return len(recent) == recent.maxlen and all(
            max(q[i] for q in recent) - min(q[i] for q in recent) <= PREFLIGHT_FEEDBACK_TOLERANCE for i in ACTIVE
        )

    def diagnostics():
        now = time.monotonic()
        info = latest["info"]
        details = {"state_messages": latest["state_count"], "info_messages": latest["info_count"],
                   "state_error": latest["state_error"],
                   "state_age_seconds": now - latest["state_at"] if latest["state_count"] else None,
                   "info_age_seconds": now - latest["info_at"] if latest["info_count"] else None,
                   "stable_sample_count": len(recent), "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")}
        if isinstance(info, dict):
            details.update(hand_joint=info.get("hand_joint"), fault=info.get("fault"))
        if len(recent) == recent.maxlen:
            details["unstable_slots"] = [i for i in ACTIVE
                                         if max(q[i] for q in recent) - min(q[i] for q in recent) > 2]
        try:
            details["state_publishers"] = node.count_publishers(topics["state"])
            details["info_publishers"] = node.count_publishers(topics["info"])
            details["command_subscribers"] = node.count_subscribers(topics["full"])
            details["isolated_index_subscribers"] = node.count_subscribers(topics["index"])
        except Exception as error:
            details["graph_error"] = str(error)
        return details

    def check_transport():
        errors = feedback_errors(latest, time.monotonic(), include_faults=False)
        if errors:
            raise RuntimeError("; ".join(errors))
        if publisher.get_subscription_count() != 1:
            raise RuntimeError("Expected exactly one SDK command subscriber")
        if node.count_publishers(command_topic) != 1 or node.count_publishers(other_topic):
            raise RuntimeError("Another index or full-hand controller is publishing")

    def check_running():
        check_transport()
        errors = feedback_errors(latest, time.monotonic(), ignore_128=fault_policy in ("ignore-128", "ignore-64-128"), ignore_64=fault_policy == "ignore-64-128")
        if not errors:
            return
        if fault_policy != "pause-thumb-128" or not observed_thumb_128_only(latest["info"]):
            raise RuntimeError("; ".join(errors))
        began = time.monotonic()
        event = {"started_monotonic": began, "fault": latest["info"]["fault"].copy(),
                 "command_count": len(log["commands"]), "result": "waiting",
                 "initial_feedback": latest["q"].copy(), "settling_samples": []}
        held = log["commands"][-1]["target"] if log["commands"] else None

        def residuals():
            if held is None:
                return []
            return [{"slot": i, "target": held[i], "feedback": latest["q"][i],
                     "error_units": latest["q"][i] - held[i]}
                    for i in controlled_slots if abs(latest["q"][i] - held[i]) > RESUME_POSITION_TOLERANCE]

        log["fault_pauses"].append(event)
        cleared_at = None
        last_settling_sample = -math.inf
        print("PAUSED: thumb slot 5/10 code 128; no targets sent. Waiting for stable zero faults...", flush=True)
        try:
            while True:
                check_transport()
                now = time.monotonic()
                duration = now - began
                if duration >= FAULT_PAUSE_TIMEOUT:
                    if not feedback_errors(latest, now) and residuals():
                        raise RuntimeError("Controlled feedback did not settle within 3 units before pause timeout: "
                                           + json.dumps(residuals(), ensure_ascii=False))
                    raise RuntimeError("Thumb-128 pause exceeded 3 seconds; automatic resume refused")
                if log["total_fault_pause_seconds"] + duration >= max_total_pause:
                    raise RuntimeError(f"Cumulative fault pauses exceeded {max_total_pause:g} seconds; automatic resume refused")
                errors = feedback_errors(latest, now)
                if errors:
                    cleared_at = None
                    if not observed_thumb_128_only(latest["info"]):
                        raise RuntimeError("; ".join(errors))
                else:
                    cleared_at = now if cleared_at is None else cleared_at
                    if now - cleared_at >= FAULT_CLEAR_STABLE:
                        pending = residuals()
                        if pending:
                            if not event["settling_samples"]:
                                print("Faults cleared; waiting for feedback to reach the last target (no new targets)...", flush=True)
                            if now - last_settling_sample >= 0.1:
                                event["settling_samples"].append({"time_monotonic": now, "residuals": pending})
                                last_settling_sample = now
                        else:
                            event["result"] = "resumed"
                            print(f"RESUMED after {duration:.2f}s with stable zero faults and feedback within tolerance; "
                                  "timeline continues.", flush=True)
                            return
                rclpy.spin_once(node, timeout_sec=0.02)
        finally:
            event["ended_monotonic"] = time.monotonic()
            event["duration_seconds"] = event["ended_monotonic"] - began
            event["commands_during_pause"] = len(log["commands"]) - event["command_count"]
            event["final_feedback"] = latest["q"].copy() if latest["q"] is not None else None
            event["final_residuals"] = residuals() if latest["q"] is not None else None
            if event["result"] == "waiting":
                event["result"] = "aborted"
            log["total_fault_pause_seconds"] += event["duration_seconds"]

    def wait_checked(duration):
        end = active_time() + duration
        while active_time() < end:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - active_time())))
            check_running()

    def publish_target(target, phase):
        check_running()
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = NAMES if full_hand else ["index_abduction", "index_base", "index_tip"]
        message.position = [float(v) for v in target] if full_hand else [float(target[i]) for i in (6, 1, 16)]
        # Empty velocity/effort: do not change the vendor's speed/torque settings.
        publisher.publish(message)
        log["commands"].append({"phase": phase, "target": target,
                                "published_position": list(message.position),
                                "feedback": latest["q"].copy(),
                                "time_monotonic": time.monotonic()})

    def publish_path(path, phase):
        for target in path:
            publish_target(target, phase)
            wait_checked(1.0 / PUBLISH_HZ)

    def hold_target(target, duration, phase):
        end = active_time() + duration
        while active_time() < end:
            publish_target(target, phase)
            wait_checked(min(1.0 / PUBLISH_HZ, max(0.0, end - active_time())))

    def replay_recording(base):
        if full_hand:
            target = hand_target(recording, 0.0, base, span)
        else:
            target = recorded_index_target(recording, 0.0, base, abs(args.delta))
            offset = target[1] - base[1]
            if offset:
                publish_path(plan_targets(base, 1, offset), "recording_align")
        began = active_time()
        last_print = -1.0
        while True:
            check_running()
            elapsed = min(recording["duration"], active_time() - began)
            desired = (hand_target(recording, elapsed, base, span) if full_hand else
                       recorded_index_target(recording, elapsed, base, abs(args.delta)))
            # Preserve ordering/time, but bound each 30 Hz update to one protocol unit.
            if full_hand:
                target = slew_pose(target, desired)
            else:
                desired[1] = max(target[1] - 1, min(target[1] + 1, desired[1]))
                target = desired
            publish_target(target, "recording")
            log["recording_elapsed_seconds"] = elapsed
            if elapsed - last_print >= 1.0:
                if full_hand:
                    print(f"Replay {elapsed:.1f}/{recording['duration']:.1f}s: "
                          f"five root targets={target[:5]}, feedback={latest['q'][:5]}", flush=True)
                else:
                    print(f"Replay {elapsed:.1f}/{recording['duration']:.1f}s: root target={target[1]}, "
                          f"feedback={latest['q'][1]}", flush=True)
                last_print = elapsed
            wait_checked(1.0 / PUBLISH_HZ)
            if elapsed >= recording["duration"]:
                if full_hand:
                    # Finish the last sample even if a rate limit delayed its target.
                    while target != desired:
                        target = slew_pose(target, desired)
                        publish_target(target, "recording_finish")
                        wait_checked(1.0 / PUBLISH_HZ)
                    hold_target(target, 0.5, "recording_final_hold")
                return target

    def replay_absolute_recording(base):
        target = command_pose(base)
        first = absolute_targets[0]
        print("Aligning smoothly to the first absolute recorded pose...", flush=True)
        while target != first:
            target = slew_pose(target, first)
            publish_target(target, "recording_align")
            wait_checked(absolute_mapping.get("ramp_interval_seconds", 1.0 / PUBLISH_HZ))
        hold_target(first, 0.5, "recording_align_hold")
        began = active_time()
        previous_sent = began
        previous_replay_time = 0.0
        last_print = -1.0
        for frame_index, (source_time, replay_time, target) in enumerate(zip(recording["times"], replay_times, absolute_targets)):
            # Never drop a source frame or burst to catch up after a pause/delay.
            due = previous_sent + (replay_time - previous_replay_time)
            remaining = due - active_time()
            if remaining > 0:
                wait_checked(remaining)
            publish_target(target, "recording")
            previous_sent = active_time()
            previous_replay_time = replay_time
            log["commands"][-1].update(recording_frame_index=frame_index, recording_source_seconds=source_time,
                                         recording_retimed_seconds=replay_time)
            log["recording_elapsed_seconds"] = source_time
            log["recording_frames_published"] = frame_index + 1
            if source_time - last_print >= 1.0 or frame_index == len(absolute_targets) - 1:
                print(f"Absolute replay {source_time:.2f}/{recording['duration']:.2f}s, "
                      f"frame {frame_index+1}/{len(absolute_targets)}: "
                      f"root targets={target[:5]}, feedback={latest['q'][:5]}", flush=True)
                last_print = source_time
        log["recording_all_frames_published"] = True
        log["recording_active_wall_seconds"] = active_time() - began
        hold_target(target, 0.5, "recording_final_hold")
        return target

    subscriptions = [node.create_subscription(JointState, topics["state"], on_state, 10),
                     node.create_subscription(String, topics["info"], on_info, 10)]
    try:
        print("Waiting for stable 20-slot feedback and fresh L20-10 info...")
        if fault_policy in ("ignore-128", "ignore-64-128"):
            print(f"User-confirmed policy: ignore exact raw codes {log['ignored_fault_codes']}; retain raw fault logs. "
                  "Other/combined codes and transport checks remain active.", flush=True)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if feedback_ok() and stable():
                break
        else:
            details = diagnostics()
            log["diagnostics"] = details
            print("Feedback diagnostics:", json.dumps(details, ensure_ascii=False))
            if latest["state_count"] == 0 and latest["info_count"] == 0:
                raise RuntimeError(
                    "No feedback messages received. Keep the vendor SDK running in terminal A, "
                    "and run this probe in terminal B. Check the same ROS_DOMAIN_ID if the SDK is running."
                )
            raise RuntimeError(
                "Feedback did not pass validation within 10 seconds; see diagnostics for missing info, "
                "invalid positions, faults, stale messages or unstable slots"
            )
        base = latest["q"].copy()
        path = [] if full_hand else plan_targets(base, args.slot, args.delta)
        log.update(baseline=base, initial_info=latest["info"])
        if full_hand:
            log["mapping"] = absolute_mapping if absolute else hand_plan(recording, base, span)
            log["recording"] = {k: v for k, v in recording.items() if k not in ("times", "angles")}
            if absolute:
                print(f"FULL-HAND ABSOLUTE REPLAY: {recording['duration']:.2f}s, "
                      f"all {len(recording['times'])} source frames; no amplitude scaling")
                print("Mapping:", absolute_mapping["mode"], "; hardware validation is still required.")
                print(f"Scheduled recording duration: {replay_times[-1]:.2f}s; alignment, pauses and return are additional.")
                print("Limit-clipped frames by joint:", absolute_mapping["clipped_frames_by_joint"])
                print("Protocol soft limits:", absolute_mapping["protocol_soft_limits"],
                      "affected frames:", absolute_mapping["protocol_soft_clipped_frames"])
                print("First absolute target:", absolute_targets[0])
            else:
                print(f"FULL-HAND RELATIVE DEMO: {recording['duration']:.2f}s, {len(recording['times'])} frames; "
                      f"per-slot change <= {span}, physical angle mapping NOT calibrated")
            print("Moving slots:", log["mapping"]["moving_slots"])
            print("Planned slot minima:", log["mapping"]["slot_min"])
            print("Planned slot maxima:", log["mapping"]["slot_max"])
        else:
            log["target"] = path[-1]
        if recording is not None:
            if not full_hand:
                recorded_index_target(recording, 0.0, base, abs(args.delta))
                log["recording"] = {k: v for k, v in recording.items() if k not in ("times", "angles")}
                log["mapping"] = "normalized recorded index_mcp_pitch -> bounded relative protocol window; not angle calibration"
                print(f"Recording: {recording['duration']:.2f}s, {len(recording['times'])} frames; "
                      f"index root window [{base[1]-abs(args.delta)}, {base[1]}]")
        if not full_hand:
            print(f"Slot {args.slot}: {base[args.slot]} -> {path[-1][args.slot]} (protocol units)")
            print("Target:  ", path[-1])
        print("Baseline:", base)
        print(f"Stream: {PUBLISH_HZ:g} Hz; " +
              ("vendor GUI full-hand topic; reserved command slots=255" if full_hand else
               "only index 0x42 frames; unused CAN fields=255"))
        print(f"Fault policy: {fault_policy}; no automatic fault clearing or speed/torque changes")
        print("SDK command subscribers:", node.count_subscribers(command_topic))
        if not args.execute:
            log["result"] = "dry_run_no_publisher_created"
            print("DRY RUN: no command publisher was created. --execute enables real motion.")
            return

        if node.count_publishers(topics["full"]) or node.count_publishers(topics["index"]):
            raise RuntimeError("Stop other command publishers before probing")
        if node.count_subscribers(command_topic) != 1:
            raise RuntimeError("SDK command topic is unavailable. Rebuild and restart the vendor SDK first.")
        if full_hand and getattr(args, "mapping_profile", None):
            if latest["info"].get("version") != measured_profile["firmware_version"]:
                raise ValueError("Hardware firmware version differs from measured mapping profile")
            preflight_base = latest["q"].copy()
            checked = measured_preflight(args, preflight_base)
            log["cad_preflight"] = {k: v for k, v in checked.items() if k not in ("commands", "times")}
            if checked["commands"] != absolute_targets or checked["times"] != replay_times or \
                    checked["mapping"]["profile_sha256"] != absolute_mapping["profile_sha256"]:
                raise ValueError("Preflight/profile changed during preparation")
            # Preflight can take seconds; refresh ROS feedback before authorizing output.
            recent.clear()
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                if stable() and feedback_ok():
                    break
            if not feedback_ok() or not stable():
                raise ValueError("Feedback invalid or unstable after preflight; no target sent")
            event = preflight_feedback_check(preflight_base, latest["q"], recent)
            event["phase"] = "after_preflight"
            log["preflight_feedback_checks"] = [event]
            if latest["info"].get("version") != measured_profile["firmware_version"]:
                raise ValueError("Firmware changed during preflight; no target sent")
        publisher = node.create_publisher(JointState, command_topic, 1)
        end = time.monotonic() + 3
        while time.monotonic() < end and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        check_running()
        # Refresh baseline after discovery; do not reuse feedback that may have moved.
        base = latest["q"].copy()
        if full_hand and getattr(args, "mapping_profile", None):
            event = preflight_feedback_check(preflight_base, base, recent)
            event["phase"] = "before_first_target"
            log["preflight_feedback_checks"].append(event)
            log["observed_start_feedback"] = base.copy()
            # Do not alter the audited alignment/return paths when the sensor
            # readback changes by one or two integer protocol counts.
            base = preflight_base.copy()
            if event["max_difference_units"]:
                print(f"Stable feedback differs by <= {event['max_difference_units']} protocol units; "
                      "using the unchanged CAD-audited starting target.", flush=True)
        if not stable():
            raise RuntimeError("Hand is moving; no command sent")
        log["baseline"] = base
        if full_hand:
            log["mapping"] = absolute_mapping if absolute else hand_plan(recording, base, span)
            print("EXECUTING: all five fingers, " +
                  ("absolute recorded poses" if absolute else "bounded relative recording replay"))
        else:
            path = plan_targets(base, args.slot, args.delta)
            log["target"] = path[-1]
            print(f"EXECUTING slot {args.slot}: {base[args.slot]} -> {path[-1][args.slot]}")
        if recording is not None:
            last_target = replay_absolute_recording(base) if absolute else replay_recording(base)
        else:
            publish_path(path, "outbound")
            hold_target(path[-1], 2.0, "hold")
            last_target = path[-1]
        log["outbound_feedback"] = latest["q"].copy()
        print("Feedback after replay:" if full_hand else "Feedback after small move:", log["outbound_feedback"])
        if args.return_to_start:
            if full_hand:
                print("Returning all controlled slots to the sampled starting targets...")
                start_target = command_pose(base)
                while last_target != start_target:
                    last_target = slew_pose(last_target, start_target)
                    publish_target(last_target, "return")
                    wait_checked(absolute_mapping.get("ramp_interval_seconds", 1.0 / PUBLISH_HZ) if absolute else 1.0 / PUBLISH_HZ)
                hold_target(start_target, 1.0, "return_hold")
            else:
                print("Returning the same slot to the sampled starting target...")
                offset = base[args.slot] - last_target[args.slot]
                if offset:
                    publish_path(plan_targets(last_target, args.slot, offset), "return")
                hold_target(base, 1.0, "return_hold")
            log["return_feedback"] = latest["q"].copy()
            print("Feedback after return:", log["return_feedback"])
        log["result"] = ("completed_absolute_full_hand_recording_replay" if absolute else
                         "completed_bounded_full_hand_recording_replay" if full_hand else
                         "completed_bounded_index_recording_replay" if recording is not None
                         else "completed_command_sequence_not_angle_calibration")
    except (KeyboardInterrupt, Exception) as error:
        log["result"] = "aborted"
        log["error"] = str(error) or type(error).__name__
        print(f"STOPPED: {log['error']}. No further targets will be published.")
        if log["commands"]:
            print("The hand may continue toward its last target; this is not an emergency stop.")
        raise
    finally:
        log.setdefault("diagnostics", diagnostics())
        log["final_feedback"] = latest["q"]
        log["final_info"] = latest["info"]
        prefix = "replay_hand" if full_hand else f"probe_index_{args.slot}"
        log_path = ROOT / "data" / f"{prefix}_{time.time_ns()}.json"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(json.dumps(log, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        print(f"Log: {log_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--slot", type=int, choices=(1, 16), default=1)
    parser.add_argument("--delta", type=int, default=-5,
                        help=f"protocol units, nonzero, magnitude <= {MAX_PROBE_DELTA}")
    parser.add_argument("--execute", action="store_true", help="publish real motor targets")
    parser.add_argument("--recording", type=Path, help="replay recorded index root only, in a bounded relative window")
    parser.add_argument("--fault-policy", choices=("stop", "pause-thumb-128", "ignore-128", "ignore-64-128"), default="stop",
                        help="stop on any fault (default), ignore exact 128 or exact 64/128 by user policy, or pause on thumb-only 128; "
                             "resume after 0.25s zero faults, abort after 3s per pause / 10s cumulative")
    parser.add_argument("--return", dest="return_to_start", action="store_true",
                        help="after 2 seconds, ramp the tested slot back to its sampled start")
    args = parser.parse_args()
    if not 1 <= abs(args.delta) <= MAX_PROBE_DELTA:
        parser.error(f"--delta magnitude must be 1..{MAX_PROBE_DELTA}")
    try:
        run(args)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, ValueError):
        # Expected validation failures have already been printed and logged.
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
