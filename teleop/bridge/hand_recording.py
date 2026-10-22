"""Named recording -> relative demo or absolute targets using the vendor angle table."""
import ast
import bisect
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "8632b2cab32e1b51ce379940c414a0f78332ff6b"
RESERVED = (11, 12, 13, 14)
# Structural correspondence: vendor GUI slot labels + the recorded URDF chain.
# Thumb base is the third CMC axis; side and horizontal are the preceding axes.
# This candidate correspondence is NOT a measured hardware angle calibration.
SLOTS = {
    "thumb_cmc_pitch": (0, -1), "thumb_cmc_roll": (5, -1),
    "thumb_cmc_yaw": (10, -1), "thumb_mcp": (15, -1),
    "index_mcp_pitch": (1, -1), "index_mcp_roll": (6, 1), "index_pip": (16, -1),
    "middle_mcp_pitch": (2, -1), "middle_mcp_roll": (7, 1), "middle_pip": (17, -1),
    "ring_mcp_pitch": (3, -1), "ring_mcp_roll": (8, 1), "ring_pip": (18, -1),
    "pinky_mcp_pitch": (4, -1), "pinky_mcp_roll": (9, 1), "pinky_pip": (19, -1),
}
NAMES = ["拇指根部", "食指根部", "中指根部", "无名指根部", "小指根部",
         "拇指侧摆", "食指侧摆", "中指侧摆", "无名指侧摆", "小指侧摆",
         "拇指横摆", "预留", "预留", "预留", "预留",
         "拇指尖部", "食指末端", "中指末端", "无名指末端", "小指末端"]


def vendor_right_profile():
    """Read the SDK's L20/G20 right-hand table, without substituting URDF endpoints."""
    path = ROOT.parent / "src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/utils/mapping.py"
    source = path.read_text(encoding="utf-8")
    keys = ("l20_r_min", "l20_r_max", "l20_r_derict")
    values = {}
    for statement in ast.parse(source).body:
        if isinstance(statement, ast.Assign):
            for target in statement.targets:
                if isinstance(target, ast.Name) and target.id in keys:
                    values[target.id] = ast.literal_eval(statement.value)
    if set(values) != set(keys) or any(len(values[k]) != 20 for k in keys):
        raise ValueError("Vendor right-hand angle table is missing or malformed")
    for name, (slot, _) in SLOTS.items():
        lo, hi, direction = (values[k][slot] for k in keys)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (lo, hi)) or \
                hi <= lo or direction not in (-1, 0):
            raise ValueError(f"Invalid vendor table for {name}")
    return {"path": str(path), "sha256": hashlib.sha256(source.encode()).hexdigest(),
            "lower": values[keys[0]], "upper": values[keys[1]], "direction": values[keys[2]],
            "scope": "legacy ROS L20/G20 shared table; differs from dedicated G20 in official C++ SDK 2.2.0; diagnostic only"}


def absolute_hand_plan(recording, *, middle_tip_min=10):
    """Convert every source frame without normalizing extrema or adding pose offsets."""
    if isinstance(middle_tip_min, bool) or not isinstance(middle_tip_min, int) or not 10 <= middle_tip_min <= 64:
        raise ValueError("Middle-tip soft lower bound must be an integer in [10,64]")
    profile = vendor_right_profile()
    commands = []
    soft_clipped_middle_frames = 0
    clipped = {name: 0 for name in recording["names"]}
    joint_mapping = {}
    for name in recording["names"]:
        slot = SLOTS[name][0]
        model_lo, model_hi = recording["limits"][name]
        lo = max(model_lo, profile["lower"][slot])
        hi = min(model_hi, profile["upper"][slot])
        if hi <= lo:
            raise ValueError(f"Model/vendor angle ranges do not overlap for {name}")
        joint_mapping[name] = {"slot": slot, "effective_lower_rad": lo, "effective_upper_rad": hi,
                               "vendor_lower_rad": profile["lower"][slot],
                               "vendor_upper_rad": profile["upper"][slot],
                               "vendor_direction": profile["direction"][slot]}
    for angles in recording["angles"]:
        command = [255] * 20
        for name, angle in zip(recording["names"], angles):
            mapping = joint_mapping[name]
            slot = mapping["slot"]
            bounded = min(mapping["effective_upper_rad"], max(mapping["effective_lower_rad"], angle))
            clipped[name] += abs(bounded - angle) > 1e-6
            fraction = (bounded - profile["lower"][slot]) / (profile["upper"][slot] - profile["lower"][slot])
            if profile["direction"][slot] == -1:
                fraction = 1 - fraction
            command[slot] = min(255, max(0, round(255 * fraction)))
        if command[17] < middle_tip_min:
            command[17] = middle_tip_min
            soft_clipped_middle_frames += 1
        commands.append(command)
    return commands, {"mode": "absolute_vendor_angle_mapping_no_amplitude_scaling",
                      "anchor": "absolute recorded angles; independent of current physical pose",
                      "vendor_profile": profile, "joint_to_slot": joint_mapping,
                      "protocol_soft_limits": {"17": {"lower": middle_tip_min, "upper": 255}},
                      "protocol_soft_clipped_frames": {"17": soft_clipped_middle_frames},
                      "clipped_frames_by_joint": clipped, "source_frame_count": len(commands),
                      "slot_min": [min(q[i] for q in commands) for i in range(20)],
                      "slot_max": [max(q[i] for q in commands) for i in range(20)],
                      "moving_slots": [i for i in range(20) if min(q[i] for q in commands) != max(q[i] for q in commands)],
                      "max_source_frame_step": max(abs(b[i] - a[i]) for a, b in zip(commands, commands[1:])
                                                   for i in range(20)),
                      "physical_mapping_calibrated": False}


def load_hand_recording(path):
    path = Path(path).resolve()
    model = ROOT / "linkerhand-urdf/L20/right/linkerhand_l20_right.urdf"
    tree = ET.parse(model).getroot()
    limits = {j.get("name"): (float(j.find("limit").get("lower")),
                              float(j.find("limit").get("upper")))
              for j in tree.findall("joint") if j.get("name") in SLOTS}
    if set(limits) != set(SLOTS) or any(hi <= lo for lo, hi in limits.values()):
        raise ValueError("Recorded URDF must define all 16 expected independent joint limits")
    names = list(SLOTS)
    times, angles = [], []
    first_ns = previous_ns = None
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        row = json.loads(line)
        if row.get("schema") != "lh20.retarget.v1" or row.get("valid") is not True:
            raise ValueError(f"Recording line {line_number}: invalid tracking/schema")
        if row.get("hand") != "right" or row.get("source_commit") != SOURCE_COMMIT or \
                row.get("model") != "L20/right/linkerhand_l20_right.urdf":
            raise ValueError("Recording hand, model or dex source commit differs from the expected setup")
        row_names, values = row.get("joint_names"), row.get("position_rad")
        if not isinstance(row_names, list) or not isinstance(values, list) or len(row_names) != 16 or \
                len(values) != 16 or len(set(row_names)) != 16 or set(row_names) != set(names):
            raise ValueError("Recording must contain the 16 unique named independent joints")
        named = dict(zip(row_names, values))
        ordered = []
        for name in names:
            value = named[name]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Non-finite/non-numeric angle: {name}")
            lo, hi = limits[name]
            if not lo - 1e-6 <= value <= hi + 1e-6:
                raise ValueError(f"Recorded angle outside model limits: {name}")
            ordered.append(min(hi, max(lo, float(value))))
        captured = row.get("captured_monotonic_ns")
        if isinstance(captured, bool) or not isinstance(captured, int) or \
                (previous_ns is not None and captured <= previous_ns):
            raise ValueError("Recording capture times must increase strictly")
        if previous_ns is not None and captured - previous_ns > 300_000_000:
            raise ValueError("Recording tracking gap exceeds 0.3 seconds")
        first_ns = captured if first_ns is None else first_ns
        times.append((captured - first_ns) / 1e9)
        angles.append(ordered)
        previous_ns = captured
    if len(times) < 2 or not 0 < times[-1] <= 60:
        raise ValueError("Expected a recording of 2+ frames and at most 60 seconds")
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "model": str(model), "source_commit": SOURCE_COMMIT, "names": names,
            "limits": limits, "times": times, "angles": angles, "duration": times[-1]}


def command_pose(feedback):
    target = list(feedback)
    for i in RESERVED:
        target[i] = 255  # Vendor GUI command fill, never the feedback placeholder 0.
    return target


def hand_target(recording, seconds, baseline, span):
    if isinstance(span, bool) or not isinstance(span, int) or not 1 <= span <= 20:
        raise ValueError("Full-hand demo span must be 1..20 protocol units")
    if len(baseline) != 20 or any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 255
                                 for v in baseline):
        raise ValueError("Full-hand baseline must contain 20 integers in [0,255]")
    times, angles = recording["times"], recording["angles"]
    index = max(0, min(len(times) - 2, bisect.bisect_right(times, seconds) - 1))
    ratio = min(1.0, max(0.0, (seconds - times[index]) / (times[index + 1] - times[index])))
    target = command_pose(baseline)
    for j, name in enumerate(recording["names"]):
        q = angles[index][j] + ratio * (angles[index + 1][j] - angles[index][j])
        lo, hi = recording["limits"][name]
        slot, direction = SLOTS[name]
        # Use model range, not recording extrema: small recorded motions stay small.
        offset = round(direction * span * (q - angles[0][j]) / (hi - lo))
        if abs(offset) > span:
            raise ValueError("Relative demo exceeded its per-slot motion bound")
        target[slot] = min(255, max(0, baseline[slot] + offset))
    return target


def slew_pose(previous, desired):
    target = command_pose(previous)
    for slot in range(20):
        if slot not in RESERVED:
            target[slot] = max(previous[slot] - 1, min(previous[slot] + 1, desired[slot]))
    return target


def hand_plan(recording, baseline, span):
    targets = [hand_target(recording, t, baseline, span) for t in recording["times"]]
    return {"mode": "relative_scaled_full_hand_demo_not_angle_calibration",
            "span_bound": span, "anchor": "current feedback = first recorded pose",
            "joint_to_slot": {name: {"slot": slot, "direction_candidate": direction}
                              for name, (slot, direction) in SLOTS.items()},
            "slot_min": [min(q[i] for q in targets) for i in range(20)],
            "slot_max": [max(q[i] for q in targets) for i in range(20)],
            "moving_slots": [i for i in range(20) if min(q[i] for q in targets) != max(q[i] for q in targets)],
            "physical_mapping_calibrated": False}
