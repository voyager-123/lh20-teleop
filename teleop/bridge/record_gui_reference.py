#!/usr/bin/env python3
"""Passively record GUI commands/settings, feedback and CAN. Never sends targets."""
import argparse
import collections
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CMD = "/cb_right_hand_control_cmd"
INDEX = "/cb_right_hand_index_control_cmd"
STATE = "/cb_right_hand_state"
INFO = "/cb_right_hand_info"
SETTING = "/cb_hand_setting_cmd"


def run(args):
    import can
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    path = args.output or ROOT / "data" / f"gui_reference_{time.time_ns()}.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    stream = path.open("x", encoding="utf-8")
    bus = None
    node = None
    counts = collections.Counter()
    settings = []
    faults = []
    command_times = []
    command_min = command_max = None
    last_position = None
    start = time.monotonic()

    def write(topic, data):
        counts[topic] += 1
        stream.write(json.dumps({"time_monotonic": time.monotonic(), "topic": topic,
                                 "data": data}, ensure_ascii=False, allow_nan=False) + "\n")

    def joint_callback(topic):
        def callback(msg):
            nonlocal command_min, command_max, last_position
            position = list(msg.position)
            write(topic, {"name": list(msg.name), "position": position,
                          "velocity": list(msg.velocity), "effort": list(msg.effort)})
            if topic == STATE:
                last_position = position
            if topic == CMD and len(position) == 20:
                command_times.append(time.monotonic())
                if command_min is None:
                    command_min, command_max = position.copy(), position.copy()
                command_min = [min(a, b) for a, b in zip(command_min, position)]
                command_max = [max(a, b) for a, b in zip(command_max, position)]
        return callback

    def string_callback(topic):
        def callback(msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                data = {"unparsed": msg.data}
            write(topic, data)
            if topic == SETTING:
                settings.append(data)
            elif topic == INFO and isinstance(data, dict):
                fault = data.get("fault", [])
                if not faults or faults[-1] != fault:
                    faults.append(fault)
        return callback

    try:
        rclpy.init()
        node = rclpy.create_node("lh20_gui_reference_recorder")
        # Subscribe only: this program creates no ROS publishers and never calls CAN send.
        subscriptions = [node.create_subscription(JointState, topic, joint_callback(topic), 100)
                         for topic in (CMD, INDEX, STATE)]
        subscriptions += [node.create_subscription(String, topic, string_callback(topic), 100)
                          for topic in (INFO, SETTING)]
        if args.can != "none":
            bus = can.Bus(channel=args.can, interface="socketcan")
            bus.set_filters([{"can_id": 0x27, "can_mask": 0x7ff, "extended": False}])
        print(f"READ ONLY: recording for {args.seconds:g} seconds. No motor commands or settings will be sent.")
        print(f"Output: {path}")
        print("Use the known-working GUI; keep the probe script stopped. Ctrl+C ends recording early.")
        last_status = 0.0
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.005)
            if bus is not None:
                for _ in range(100):
                    message = bus.recv(timeout=0)
                    if message is None:
                        break
                    write("can0x27", {"arbitration_id": message.arbitration_id,
                                      "bus_rx": bool(message.is_rx), "data_hex": message.data.hex(),
                                      "is_error_frame": message.is_error_frame})
            if time.monotonic() - last_status >= 2.0:
                print(f"t={time.monotonic()-start:.1f}s GUI targets={counts[CMD]} "
                      f"settings={counts[SETTING]} states={counts[STATE]}", flush=True)
                stream.flush()
                last_status = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Recording ended.")
    finally:
        stream.close()
        if bus is not None:
            bus.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        average_hz = None
        if len(command_times) > 1 and command_times[-1] > command_times[0]:
            average_hz = (len(command_times) - 1) / (command_times[-1] - command_times[0])
        summary = {"recording": str(path), "read_only": True, "counts": dict(counts),
                   "gui_command_hz": average_hz, "settings": settings,
                   "fault_changes": faults, "command_min": command_min,
                   "command_max": command_max, "last_position": last_position,
                   "physical_motion_observed": "requires human observation; not inferred from feedback"}
        summary_path = path.with_suffix(".summary.json")
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        print(f"Summary: {summary_path}")
        if not command_times:
            print("No GUI targets captured; open/use the GUI while this recorder is running.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--can", default="can0", help="SocketCAN interface, or none for ROS topics only")
    parser.add_argument("--output", type=Path, help="new JSONL path; existing files are not overwritten")
    args = parser.parse_args()
    if not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0,600]")
    run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
