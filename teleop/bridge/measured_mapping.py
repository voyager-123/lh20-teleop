"""Measured, named angle/protocol mapping. No ROS or device access.

Calibration evidence must come from physical angle measurements in URDF axes.
GUI presets and manufacturer range endpoints alone are not measurements.
"""
import bisect
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from hand_recording import SLOTS

# Pinned manufacturer G20/L20_10 table exported from SDK 2.2.0. This is a
# factory conversion, not fabricated per-device measurement evidence.
VENDOR_G20_PROFILE_SHA256 = "d3e6330fa415780812bd4b8b355954c8f42556e6f8110c13553b584009d7e7ce"
VENDOR_G20_URDF_ALIGNMENT_CONFIRMED = False
G20_URDF_PROFILE_SHA256 = "8d2bafe037e4dbc6bb5ead9a1148ff0b3a32698e5db317026ee87ca23bcae42a"


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def load_profile(path, recording):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    adapted = data.get("schema") == "lh20.vendor_model_mapping.v1"
    vendor = data.get("schema") in ("lh20.vendor_mapping.v1", "lh20.vendor_model_mapping.v1")
    profile_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    expected_hash = G20_URDF_PROFILE_SHA256 if adapted else VENDOR_G20_PROFILE_SHA256
    if vendor and profile_hash != expected_hash:
        raise ValueError("Manufacturer G20 profile differs from the pinned official SDK conversion evidence")
    model_hash = hashlib.sha256(Path(recording["model"]).read_bytes()).hexdigest()
    if data.get("schema") not in ("lh20.measured_mapping.v1", "lh20.vendor_mapping.v1", "lh20.vendor_model_mapping.v1") or data.get("hand") != "right" or \
            data.get("hand_joint") != "L20-10" or data.get("urdf_sha256") != model_hash:
        raise ValueError("Mapping schema, hand/model or URDF hash mismatch")
    version = data.get("firmware_version")
    if not isinstance(version, list) or len(version) != 3 or any(type(v) is not int or v < 0 for v in version):
        raise ValueError("Mapping requires three integer firmware version components")
    if not vendor and (not data.get("measurement_notes") or data.get("measured_on_hardware") is not True):
        raise ValueError("Mapping needs actual hardware measurements and evidence notes; template is not calibration")
    joints = data.get("joints")
    if not isinstance(joints, dict) or set(joints) != set(SLOTS):
        raise ValueError("Mapping must contain all 16 unique named independent joints")
    slots = []
    for name, entry in joints.items():
        if not isinstance(entry, dict):
            raise ValueError(f"Mapping joint entry must be an object: {name}")
        slot, anchors = entry.get("slot"), entry.get("anchors")
        if type(slot) is not int or slot not in (*range(11), *range(15, 20)):
            raise ValueError(f"Invalid measured slot for {name}")
        slots.append(slot)
        if not isinstance(anchors, list) or len(anchors) < 3:
            raise ValueError(f"Need at least three measured anchors for {name}")
        for anchor in anchors:
            if not isinstance(anchor, list) or len(anchor) != 2 or not all(map(finite, anchor)) or \
                    not 0 <= anchor[1] <= 255 or anchor[1] != round(anchor[1]):
                raise ValueError(f"Invalid [radians, protocol integer] anchor for {name}")
            lo, hi = recording["limits"][name]
            if not vendor and not lo <= anchor[0] <= hi:
                raise ValueError(f"Calibration angle outside URDF limits: {name}")
        dq = [b[0] - a[0] for a, b in zip(anchors, anchors[1:])]
        du = [b[1] - a[1] for a, b in zip(anchors, anchors[1:])]
        if any(v <= 0 for v in dq) or not (all(v > 0 for v in du) or all(v < 0 for v in du)):
            raise ValueError(f"Anchors must have increasing angles and strictly monotonic protocol values: {name}")
    if len(set(slots)) != 16:
        raise ValueError("Measured joints must use 16 different active protocol slots")
    data["path"] = str(path)
    data["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return data


def interpolate(x, points):
    # No extrapolation, clipping, normalization or guessed offsets.
    if not points[0][0] <= x <= points[-1][0]:
        raise ValueError(f"Value {x} outside measured calibration coverage [{points[0][0]}, {points[-1][0]}]")
    i = min(len(points) - 2, max(0, bisect.bisect_right([p[0] for p in points], x) - 1))
    a, b = points[i:i + 2]
    return a[1] + (x - a[0]) * (b[1] - a[1]) / (b[0] - a[0])


def inverse_pose(command, profile):
    named = {}
    for name, entry in profile["joints"].items():
        points = sorted((u, q) for q, u in entry["anchors"])
        named[name] = interpolate(command[entry["slot"]], points)
    return named


def measured_plan(recording, profile):
    vendor = profile["schema"] in ("lh20.vendor_mapping.v1", "lh20.vendor_model_mapping.v1")
    adapted = profile["schema"] == "lh20.vendor_model_mapping.v1"
    clipped = {name: 0 for name in recording["names"]}
    commands = []
    for angles in recording["angles"]:
        command = [255] * 20
        for name, q in zip(recording["names"], angles):
            entry = profile["joints"][name]
            if vendor:
                lo = max(recording["limits"][name][0], entry["anchors"][0][0])
                hi = min(recording["limits"][name][1], entry["anchors"][-1][0])
                if lo >= hi:
                    raise ValueError(f"Manufacturer/URDF limits do not overlap: {name}")
                bounded = min(hi, max(lo, q))
                clipped[name] += abs(bounded - q) > 1e-6
                raw_lo, raw_hi = sorted([interpolate(lo, entry["anchors"]), interpolate(hi, entry["anchors"])])
                # Quantization must stay inside model/factory limits as well.
                command[entry["slot"]] = min(math.floor(raw_hi + 1e-9), max(math.ceil(raw_lo - 1e-9),
                                               round(interpolate(bounded, entry["anchors"]))))
            else:
                command[entry["slot"]] = round(interpolate(q, entry["anchors"]))
        commands.append(command)
    tree = ET.parse(recording["model"]).getroot()
    velocity = {j.get("name"): float(j.find("limit").get("velocity"))
                for j in tree.findall("joint") if j.find("limit") is not None}
    mimics = {j.get("name"): (j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", 1)))
              for j in tree.findall("joint") if j.find("mimic") is not None}
    # Use the stricter independent / coupled distal joint velocity bound.
    speeds = {n: velocity[n] for n in recording["names"]}
    for child, (source, multiplier) in mimics.items():
        if source in speeds and multiplier:
            speeds[source] = min(speeds[source], velocity[child] / abs(multiplier))
    if any(not math.isfinite(v) or v <= 0 for v in speeds.values()):
        raise ValueError("Invalid URDF velocity limits")
    quantized = [inverse_pose(c, profile) for c in commands]
    ramp_interval = max([1 / 30] + [abs((b[0] - a[0]) / (b[1] - a[1])) / speeds[name]
                                   for name, entry in profile["joints"].items()
                                   for a, b in zip(entry["anchors"], entry["anchors"][1:])])
    times = [0.0]
    for i in range(1, len(commands)):
        dt = max(recording["times"][i] - recording["times"][i - 1], 1 / 30)
        for j, name in enumerate(recording["names"]):
            # Preserve all source poses, and include quantization in velocity bound.
            dt = max(dt, abs(recording["angles"][i][j] - recording["angles"][i - 1][j]) / speeds[name],
                     abs(quantized[i][name] - quantized[i - 1][name]) / speeds[name])
        times.append(times[-1] + dt)
    return commands, times, {"mode": "absolute_vendor_g20_urdf_mapping" if adapted else "absolute_vendor_g20_mapping" if vendor else "absolute_measured_mapping",
                            "profile_path": profile["path"], "profile_sha256": profile["sha256"],
                            "physical_mapping_calibrated": not vendor,
                            "model_coordinate_alignment_confirmed": not vendor or adapted or VENDOR_G20_URDF_ALIGNMENT_CONFIRMED,
                            "coordinate_adapter": profile.get("model_coordinate_adapter"),
                            "calibration_claim": "manufacturer G20/L20_10 factory conversion; hardware validation pending" if vendor else
                                                 "user-supplied measurements; not independently verified by software",
                            "source": profile.get("source"), "clipped_frames_by_joint": clipped,
                            "protocol_soft_limits": {}, "protocol_soft_clipped_frames": {},
                            "moving_slots": [i for i in range(20) if len({c[i] for c in commands}) > 1],
                            "slot_min": [min(c[i] for c in commands) for i in range(20)],
                            "slot_max": [max(c[i] for c in commands) for i in range(20)],
                            "source_duration": recording["duration"], "retimed_duration": times[-1],
                            "ramp_interval_seconds": ramp_interval,
                            "velocity_basis": "URDF limits, including mimic; not measured device speed/acceleration limits"}
