#!/usr/bin/env python3
"""Operator-requested retreat to a saved raw pose; never clears faults or changes torque."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

from hand_recording import NAMES, command_pose, slew_pose
from probe_index_joint import ACTIVE, FULL_HAND_TOPIC, INDEX_TOPIC, ROOT, feedback_errors, validate_positions


def classify_faults(info):
    """Only the already observed thumb 128 and middle-tip 64 can enter recovery."""
    faults = info["fault"]
    unexpected = [(i, code) for i, code in enumerate(faults)
                  if code and not (i in (5, 10) and code == 128) and not (i == 17 and code == 64)]
    if unexpected:
        raise RuntimeError(f"Unexpected faults during restore: {unexpected}")
    return bool(faults[5] or faults[10]), faults[17] == 64


def release_target(current, saved):
    """A bounded retreat only, toward the saved open pose, from the stalled low endpoint."""
    if not current[17] <= 10 or saved[17] - current[17] < 30:
        raise RuntimeError("Middle-tip recovery is only allowed near the low endpoint toward a saved open pose")
    target = command_pose(current)
    target[17] += 30
    return target


def run(args):
    source = json.loads(args.from_log.read_text())
    if source.get("command_topic") != FULL_HAND_TOPIC or source.get("initial_info", {}).get("hand_joint") != "L20-10":
        raise ValueError("Expected a saved right L20-10 full-hand log")
    saved = command_pose(validate_positions(source["baseline"]))
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("lh20_operator_restore_start")
    latest = {"q": None, "state_at": 0., "info": None, "info_at": 0.}
    recent = deque(maxlen=5)
    publisher = None
    began = time.monotonic()
    recovery_64 = False
    pause_total = 0.0
    log = {"source_log": str(args.from_log.resolve()), "saved_target": saved, "commands": [],
           "fault_changes": [], "pauses": [], "execute_requested": args.execute}

    def on_state(message):
        latest["state_at"] = time.monotonic()
        try:
            latest["q"] = validate_positions(message.position)
            recent.append(latest["q"])
        except ValueError:
            latest["q"] = None
            recent.clear()

    def on_info(message):
        latest["info_at"] = time.monotonic()
        try:
            latest["info"] = json.loads(message.data)
            faults = latest["info"].get("fault")
            if not log["fault_changes"] or log["fault_changes"][-1]["fault"] != faults:
                log["fault_changes"].append({"time": time.monotonic(), "fault": faults})
        except (ValueError, AttributeError):
            latest["info"] = None

    subscriptions = [node.create_subscription(JointState, "/cb_right_hand_state", on_state, 1),
                     node.create_subscription(String, "/cb_right_hand_info", on_info, 1)]

    def check_transport():
        if time.monotonic() - began > 45:
            raise RuntimeError("Restore exceeded 45-second session limit")
        errors = feedback_errors(latest, time.monotonic(), include_faults=False)
        if errors:
            raise RuntimeError("; ".join(errors))
        expected = 1 if publisher else 0
        if node.count_publishers(FULL_HAND_TOPIC) != expected or node.count_publishers(INDEX_TOPIC):
            raise RuntimeError("Another command publisher appeared")
        if publisher and publisher.get_subscription_count() != 1:
            raise RuntimeError("Expected exactly one SDK command subscriber")

    def check_faults():
        nonlocal recovery_64
        check_transport()
        thumb, middle = classify_faults(latest["info"])
        if middle and not recovery_64:
            raise RuntimeError("New/recurrent middle-tip 64; restore stopped")
        if recovery_64 and not middle:
            recovery_64 = False
            log["middle_64_cleared_at"] = time.monotonic()
            print("Middle-tip 64 cleared; continuing toward saved start.", flush=True)
        return thumb

    def checked():
        nonlocal pause_total
        if not check_faults():
            return
        start = time.monotonic()
        clear_at = None
        event = {"started": start, "command_count": len(log["commands"])}
        log["pauses"].append(event)
        print("Paused targets for thumb 128; awaiting stable clear...", flush=True)
        try:
            while True:
                thumb = check_faults()
                now = time.monotonic()
                if now - start >= 3 or pause_total + now - start >= 10:
                    raise RuntimeError("Thumb pause timeout during restore")
                if thumb:
                    clear_at = None
                else:
                    clear_at = now if clear_at is None else clear_at
                    if now - clear_at >= 0.25:
                        return
                rclpy.spin_once(node, timeout_sec=0.02)
        finally:
            event["duration"] = time.monotonic() - start
            event["commands_during_pause"] = len(log["commands"]) - event["command_count"]
            pause_total += event["duration"]

    def send(target, phase):
        checked()
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = NAMES
        message.position = [float(x) for x in target]
        publisher.publish(message)
        log["commands"].append({"time": time.monotonic(), "phase": phase,
                                "target": target.copy(), "feedback": latest["q"].copy()})
        end = time.monotonic() + 1 / 30
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0, end - time.monotonic())))
            checked()

    try:
        print("Reading stable position and device info before restore...", flush=True)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not feedback_errors(latest, time.monotonic(), include_faults=False) and len(recent) == 5 and all(
                    max(q[i] for q in recent) - min(q[i] for q in recent) <= 2 for i in ACTIVE):
                break
        else:
            raise RuntimeError("No fresh stable feedback before restore")
        thumb, middle = classify_faults(latest["info"])
        current = latest["q"].copy()
        log.update(initial_position=current, initial_info=latest["info"])
        relief = release_target(current, saved) if middle else None
        print("Current:", current)
        print("Saved start target:", saved)
        if relief:
            print(f"Pre-existing middle-tip 64: retreat slot 17 only, {current[17]} -> {relief[17]}, max 30 units.")
        if not args.execute:
            log["result"] = "dry_run_no_publisher"
            return
        check_transport()
        if node.count_subscribers(FULL_HAND_TOPIC) != 1:
            raise RuntimeError("SDK command subscriber unavailable")
        recovery_64 = middle
        publisher = node.create_publisher(JointState, FULL_HAND_TOPIC, 1)
        end = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        checked()
        if max(abs(latest["q"][i] - current[i]) for i in ACTIVE) > 2:
            raise RuntimeError("Hand moved during discovery; restore refused")
        cursor = command_pose(current)
        if relief:
            while recovery_64 and cursor != relief:
                cursor = slew_pose(cursor, relief)
                send(cursor, "middle_tip_retreat")
            end = time.monotonic() + 2
            while recovery_64 and time.monotonic() < end:
                send(cursor, "middle_tip_wait_for_clear")
            if recovery_64:
                raise RuntimeError("Middle-tip 64 persists after bounded retreat; no full-hand restore sent")
        print("Restoring all slots smoothly to saved start...", flush=True)
        while cursor != saved:
            cursor = slew_pose(cursor, saved)
            send(cursor, "restore")
        end = time.monotonic() + 3
        settled_at = None
        while time.monotonic() < end:
            send(saved, "settle")
            error = max(abs(latest["q"][i] - saved[i]) for i in ACTIVE)
            if error <= 3:
                settled_at = time.monotonic() if settled_at is None else settled_at
                if time.monotonic() - settled_at >= 0.25:
                    break
            else:
                settled_at = None
        else:
            raise RuntimeError("Saved targets sent but feedback did not settle within 3 units")
        log["result"] = "restored_saved_start_pose"
        print("RESTORED. Final feedback:", latest["q"], flush=True)
    except BaseException as error:
        log["result"] = "aborted"
        log["error"] = str(error) or type(error).__name__
        print(f"STOPPED: {log['error']}; no further targets.", flush=True)
        raise
    finally:
        log["final_position"] = latest["q"]
        log["final_info"] = latest["info"]
        log["total_pause_seconds"] = pause_total
        path = ROOT / "data" / f"restore_start_{time.time_ns()}.json"
        path.write_text(json.dumps(log, ensure_ascii=False, indent=2) + "\n")
        print("Log:", path)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--from-log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    try:
        run(args)
    except (RuntimeError, ValueError, KeyboardInterrupt):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
