#!/usr/bin/env python3
"""Pinned dex-retargeting camera/model preview. No ROS, CAN or motor commands."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "dex-retargeting"
COMMIT = "8632b2cab32e1b51ce379940c414a0f78332ff6b"


def load_retargeting(hand="right", scope="index", thumb_axes="native"):
    # The default preserves existing right-recording audit callers.
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    head = subprocess.check_output(
        ["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != COMMIT:
        raise RuntimeError(f"Expected dex-retargeting {COMMIT}, got {head}")
    sys.path.insert(0, str(SOURCE))
    from dex_retargeting.retargeting_config import RetargetingConfig

    if scope == "hand" and hand != "left":
        raise ValueError("Factory full-hand mode is available only for left L20-10")
    RetargetingConfig.set_default_urdf_dir(ROOT if scope == "hand" else ROOT / "linkerhand-urdf")
    config = RetargetingConfig.load_from_file(ROOT / ("config/l20_10_left_vector.yaml" if scope == "hand" else f"config/l20_{hand}_vector.yaml"))
    retargeting = config.build()
    sys.path.insert(0, str(ROOT / "bridge"))
    retargeting.lh20_limit_profile_sha256 = None
    retargeting.lh20_limit_basis = "urdf_only_unverified_hardware_mapping"
    if scope == "hand":
        from factory_mapping import FactoryMapping
        mapping = FactoryMapping(thumb_axes)
        retargeting.lh20_factory_mapping = mapping
        for index, name in enumerate(config.target_joint_names):
            retargeting.joint_limits[index] = mapping.limits[name]
        retargeting.lh20_limit_profile_sha256 = mapping.sha256
        retargeting.lh20_limit_basis = "manufacturer_left_g20_native_model"
    if hand == "right":
        # The migrated vendor profile is right-specific. Never apply it to left.
        from measured_mapping import load_profile
        robot_limits = retargeting.optimizer.robot.joint_limits
        model_limits = {name: tuple(robot_limits[retargeting.joint_names.index(name)])
                        for name in config.target_joint_names}
        profile = load_profile(ROOT / "config/l20_right_vendor_g20.json",
                               {"model": str(config.urdf_path), "limits": model_limits})
        for index, name in enumerate(config.target_joint_names):
            anchors = profile["joints"][name]["anchors"]
            retargeting.joint_limits[index, 0] = max(retargeting.joint_limits[index, 0], anchors[0][0])
            retargeting.joint_limits[index, 1] = min(retargeting.joint_limits[index, 1], anchors[-1][0])
        retargeting.lh20_limit_profile_sha256 = profile["sha256"]
        retargeting.lh20_limit_basis = "urdf_intersect_right_vendor_g20"
    # The upstream optimizer expands limits by 0.001 rad. Use exact model bounds.
    retargeting.optimizer.opt.set_lower_bounds(retargeting.joint_limits[:, 0])
    retargeting.optimizer.opt.set_upper_bounds(retargeting.joint_limits[:, 1])
    print(f"Using Git source: {SOURCE}; commit={head}")
    print(f"Independent joints: {len(config.target_joint_names)}; model joints: {len(retargeting.joint_names)}")
    print(f"Hand: {hand}; optimizer limits: {retargeting.lh20_limit_basis}")
    return config, retargeting


def model_vectors(config, retargeting, qpos):
    robot = retargeting.optimizer.robot
    robot.compute_forward_kinematics(qpos)
    return np.array([
        robot.get_link_pose(robot.get_link_index(task))[:3, 3]
        - robot.get_link_pose(robot.get_link_index(origin))[:3, 3]
        for origin, task in zip(config.target_origin_link_names, config.target_task_link_names)
    ])


def check_qpos(retargeting, qpos):
    limits = retargeting.optimizer.robot.joint_limits
    if not np.isfinite(qpos).all():
        raise ValueError("Non-finite retargeting output")
    if np.any(qpos < limits[:, 0] - 1e-6) or np.any(qpos > limits[:, 1] + 1e-6):
        raise ValueError("Retargeting output exceeds URDF joint limits")


def self_test(hand="right", scope="index", thumb_axes="native"):
    # Model-generated vectors test the config/solver/mimic pipeline, not vision.
    for fraction in (0.05, 0.45, 0.85):
        config, retargeting = load_retargeting(hand, scope, thumb_axes)
        robot = retargeting.optimizer.robot
        qref = robot.joint_limits[:, 0] + fraction * np.ptp(robot.joint_limits, axis=1)
        qref = retargeting.optimizer.adaptor.forward_qpos(qref)
        target = model_vectors(config, retargeting, qref)
        initial = retargeting.get_qpos()
        before = np.linalg.norm(model_vectors(config, retargeting, initial) - target, axis=1).mean()
        for _ in range(35):
            qpos = retargeting.retarget(target / config.scaling_factor)
            if retargeting.optimizer.opt.last_optimize_result() <= 0:
                raise RuntimeError("Optimizer failed during model self-test")
            check_qpos(retargeting, qpos)
        after = np.linalg.norm(model_vectors(config, retargeting, qpos) - target, axis=1).mean()
        adaptor = retargeting.optimizer.adaptor
        np.testing.assert_allclose(
            qpos[adaptor.idx_pin2mimic],
            qpos[adaptor.idx_pin2source] * adaptor.multipliers + adaptor.offsets,
            atol=1e-6,
        )
        if after > 0.006 or after > before * 0.5:
            raise RuntimeError(f"Insufficient vector fit: {before:.6f} -> {after:.6f} m")
        print(f"PASS pose={fraction}: mean vector error {before * 1000:.2f} -> {after * 1000:.2f} mm")
    print("PASS: model/optimizer/limits/mimic. Camera, coordinate alignment and hardware are NOT validated.")


def rotation_matrix(degrees):
    x, y, z = np.deg2rad(degrees)
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def model_panel(config, retargeting, qpos, tree):
    import cv2

    panel = np.full((480, 640, 3), 24, dtype=np.uint8)
    robot = retargeting.optimizer.robot
    robot.compute_forward_kinematics(qpos)
    points = {
        link.get("name"): robot.get_link_pose(robot.get_link_index(link.get("name")))[:3, 3]
        for link in tree.findall("link")
    }
    edges = [(j.find("parent").get("link"), j.find("child").get("link")) for j in tree.findall("joint")]
    center = np.mean(list(points.values()), axis=0)
    for offset, horizontal, title in [(160, 1, "front: Y/Z"), (480, 0, "side: X/Z")]:
        projected = {
            name: (int(offset + (p[horizontal] - center[horizontal]) * 1800),
                   int(255 - (p[2] - center[2]) * 1800))
            for name, p in points.items()
        }
        cv2.putText(panel, title, (offset - 100, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        for parent, child in edges:
            cv2.line(panel, projected[parent], projected[child], (80, 210, 130), 2)
            cv2.circle(panel, projected[child], 3, (100, 190, 250), -1)
    cv2.putText(panel, "URDF joint origins; distal origins are NOT fingertips", (12, 455),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
    return panel


def run_camera(args):
    import cv2

    scope = getattr(args, "scope", "index")
    config, retargeting = load_retargeting(args.hand, scope, getattr(args,"thumb_axes","native"))
    mapping = getattr(retargeting, "lh20_factory_mapping", None)
    auto_label = "AUTO FULL HAND FOLLOW" if scope == "hand" else "AUTO INDEX FOLLOW"
    from collision_check import CollisionChecker
    collision_checker = CollisionChecker(config.urdf_path)
    from live_index import VisionSender, default_socket
    sender = VisionSender(args.socket or default_socket(args.hand)) if args.stream else None
    operator_enabled = bool(args.auto_follow)
    calibration_epoch = 0
    stream_session = uuid.uuid4().hex
    sys.path.insert(0, str(SOURCE / "example/vector_retargeting"))
    from single_hand_detector import SingleHandDetector

    # Match the upstream example: unflipped input, selfie=False.
    detector = SingleHandDetector(hand_type=args.hand.capitalize(), selfie=False)
    device = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(device)
    output = None
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {args.camera}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if args.record:
            # Do not overwrite an existing recording.
            output = open(args.record, "x", encoding="utf-8")
        tree = ET.parse(config.urdf_path).getroot()
        model_sha256 = hashlib.sha256(Path(config.urdf_path).read_bytes()).hexdigest()
        print(f"Detecting physical {args.hand} hand; model={config.urdf_path}; " +
              (auto_label if args.auto_follow else "LIVE STREAM" if args.stream else "PREVIEW ONLY"), flush=True)
        print("Thumb axes: "+getattr(args,"thumb_axes","native"),flush=True)
        transform = rotation_matrix(args.rotate_deg)
        indices = retargeting.optimizer.target_link_human_indices
        active_indices = [retargeting.joint_names.index(n) for n in config.target_joint_names]
        last_print = 0.0
        count = 0
        while True:
            ok, frame = cap.read()
            # Acquisition time is before detection/optimization, on the local monotonic clock.
            captured_ns = time.monotonic_ns()
            if not ok:
                raise RuntimeError("Camera frame read failed")
            packet = {"schema": "lh20.retarget.v1", "seq": count, "captured_monotonic_ns": captured_ns,
                      "valid": False, "hand": args.hand, "source_commit": COMMIT,
                      "model": str(Path(config.urdf_path).relative_to(ROOT)),
                      "model_sha256": model_sha256,
                      "limit_basis": retargeting.lh20_limit_basis,
                      "hardware_mapping_calibrated": False,
                      "joint_limit_profile_sha256": retargeting.lh20_limit_profile_sha256,
                      "rotate_deg": args.rotate_deg,
                      "stream_session": stream_session,
                      "operator_enabled": operator_enabled,
                      "auto_follow": args.auto_follow,
                      "calibration_epoch": calibration_epoch, "scope": scope, "thumb_axes": getattr(args,"thumb_axes","native")}
            _, joint_pos, keypoint_2d, _ = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            panel = np.full((480, 640, 3), 24, dtype=np.uint8)
            status = f"NO VALID {args.hand.upper()} HAND - no joint target"
            if joint_pos is not None and np.isfinite(joint_pos).all():
                joint_pos = joint_pos @ transform.T
                ref_value = joint_pos[indices[1]] - joint_pos[indices[0]]
                qpos = retargeting.retarget(ref_value)
                solver_ok = retargeting.optimizer.opt.last_optimize_result() > 0
                try:
                    check_qpos(retargeting, qpos)
                    valid = solver_ok
                except ValueError:
                    valid = False
                if valid:
                    hits = collision_checker.check(collision_checker.expand(dict(zip(retargeting.joint_names, qpos))))
                    packet["cad_collision_hits"] = hits
                    if hits:
                        valid = False
                if valid and mapping is not None:
                    try:
                        raw = mapping.encode(dict(zip(config.target_joint_names, qpos[active_indices].tolist())))
                        quantized = collision_checker.expand(mapping.decode(raw))
                        hits = collision_checker.check(quantized)
                        valid = not hits and not collision_checker.violations(quantized)
                        packet.update(motor_position_raw=raw, factory_profile_sha256=mapping.sha256,
                                      cad_collision_hits=hits)
                    except ValueError:
                        valid = False
                if valid:
                    packet.update(valid=True, joint_names=config.target_joint_names,
                                  position_rad=qpos[active_indices].tolist(),
                                  model_joint_names=retargeting.joint_names,
                                  model_position_rad=qpos.tolist(),
                                  landmarks_local_m=joint_pos.tolist())
                    panel = model_panel(config, retargeting, qpos, tree)
                    status = f"{args.hand.upper()} HAND - " + (auto_label if args.auto_follow else
                             "LIVE STREAM" if args.stream else ("16 angles + factory raw, NO MOTOR OUTPUT" if mapping is not None else "16 model angles, NO MOTOR OUTPUT"))
                else:
                    status = "CAD COLLISION - no joint target" if packet.get("cad_collision_hits") else "INVALID SOLVER OUTPUT - no joint target"
            if keypoint_2d is not None:
                detector.draw_skeleton_on_image(frame, keypoint_2d, style="default")
            packet["processed_monotonic_ns"] = time.monotonic_ns()
            if output is not None:
                output.write(json.dumps(packet, allow_nan=False) + "\n")
            if sender is not None:
                online = sender.send(packet)
                stream_status = ("STREAM ONLINE" if online else "BRIDGE OFFLINE") + (" / ENABLED" if operator_enabled else " / PAUSED")
                cv2.putText(frame, stream_status, (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
                cv2.putText(frame, ("AUTO LEFT FULL HAND | Q quit" if mapping is not None else "AUTO LEFT INDEX | Q quit") if args.auto_follow else "SPACE toggle | S pause | C recenter | Q quit", (10, 465),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 220, 255), 1)
            if time.monotonic() - last_print > 1.0:
                print(json.dumps({k: packet[k] for k in ["seq", "valid"]}
                                 | ({"position_rad": packet["position_rad"]} if packet["valid"] else {})
                                 | ({"motor_position_raw": packet["motor_position_raw"]} if packet.get("motor_position_raw") else {})))
                last_print = time.monotonic()
            frame = cv2.resize(frame, (640, 480))
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            window_mode = auto_label if args.auto_follow else "LIVE STREAM" if args.stream else "PREVIEW - NO MOTOR OUTPUT"
            cv2.imshow(f"L20 {args.hand.upper()} {window_mode} (Q: quit)", np.hstack([frame, panel]))
            count += 1
            key = cv2.waitKey(1) & 0xff
            if key in (ord("q"), ord("Q")):
                break
            if sender is not None:
                if key == ord(" "):
                    operator_enabled = not operator_enabled
                elif key in (ord("s"), ord("S")):
                    operator_enabled = False
                elif key in (ord("c"), ord("C")):
                    operator_enabled = False
                    calibration_epoch += 1
    finally:
        if sender is not None:
            if "packet" in locals():
                packet.update(seq=count + 1, valid=False, operator_enabled=False,
                              captured_monotonic_ns=time.monotonic_ns())
                sender.send(packet)
            sender.close()
        cap.release()
        detector.hand_detector.close()
        if output is not None:
            output.close()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--thumb-axes",choices=("native","swapped"),default="native")
    parser.add_argument("--scope", choices=("index", "hand"), default="index")
    parser.add_argument("--auto-follow", action="store_true")
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--stream", action="store_true", help="send local vision stream; SPACE toggle, S pause, C recenter")
    parser.add_argument("--self-test", action="store_true", help="model-only offline test, no camera")
    parser.add_argument("--camera", default="0", help="camera index or /dev/video path")
    parser.add_argument("--rotate-deg", nargs=3, type=float, default=[0, 0, 0], metavar=("X", "Y", "Z"),
                        help="candidate MANO-to-L20 rotation, not yet calibrated")
    parser.add_argument("--record", type=Path, help="write new JSONL recording; refuses overwrite")
    args = parser.parse_args()
    if args.self_test:
        self_test(args.hand, args.scope,args.thumb_axes)
    else:
        run_camera(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
