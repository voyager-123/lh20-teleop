#!/usr/bin/env python3
"""Read-only fresh state/info snapshot, including nonzero raw fault codes."""
import argparse
import json
from pathlib import Path
import time

from probe_index_joint import ROOT, feedback_errors, validate_positions, topic_names
from fault_codes import fault_names, SOURCE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    topics = topic_names(args.hand)
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("lh20_read_only_status")
    latest = {"q": None, "state_at": 0., "info": None, "info_at": 0.}

    def state(message):
        latest["state_at"] = time.monotonic()
        try:
            latest["q"] = validate_positions(message.position)
        except ValueError:
            latest["q"] = None

    def info(message):
        latest["info_at"] = time.monotonic()
        try:
            latest["info"] = json.loads(message.data)
        except ValueError:
            latest["info"] = None

    subscriptions = [node.create_subscription(JointState, topics["state"], state, 10),
                     node.create_subscription(String, topics["info"], info, 10)]
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not feedback_errors(latest, time.monotonic(), include_faults=False):
                break
        errors = feedback_errors(latest, time.monotonic(), include_faults=False)
        if errors:
            raise RuntimeError("; ".join(errors))
        report = {"schema": "lh20.read_only_status.v1", "hand": args.hand, "hardware_commands_sent": False,
                  "fault_definition_source": SOURCE,
                  "position": latest["q"], "info": latest["info"],
                  "faults": [{"slot": i, "raw_code": code, "hex": f"0x{code:02x}", "meaning": fault_names(code)}
                             for i, code in enumerate(latest["info"]["fault"]) if code]}
        path = args.output or ROOT / "data" / f"status_{time.time_ns()}.json"
        with path.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"Saved: {path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
