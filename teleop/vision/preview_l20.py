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


def load_retargeting(hand="right", scope="index", thumb_axes="native", thumb_profile="segments", algorithm="vector", thumb_root_mode="agile"):
    # The default preserves existing right-recording audit callers.
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    if thumb_root_mode not in ('agile','directions'):
        raise ValueError('Unknown thumb root mode')
    head = subprocess.check_output(
        ["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != COMMIT:
        raise RuntimeError(f"Expected dex-retargeting {COMMIT}, got {head}")
    sys.path.insert(0, str(SOURCE))
    from dex_retargeting.retargeting_config import RetargetingConfig

    if algorithm not in ('vector','dexpilot') or algorithm=='dexpilot' and (scope!='hand' or thumb_profile!='segments'):
        raise ValueError('DexPilot requires full-hand mode without palm experiment')
    if thumb_profile not in ('segments','palm') or thumb_profile!='segments' and (hand!='left' or scope!='hand'):
        raise ValueError('Palm thumb profile requires left full-hand mode')
    RetargetingConfig.set_default_urdf_dir(ROOT if scope == "hand" else ROOT / "linkerhand-urdf")
    config_path=ROOT / (f"config/l20_10_{hand}_dexpilot.yaml" if algorithm=='dexpilot' else
                        "config/l20_10_left_vector_palm.yaml" if scope=='hand' and thumb_profile=='palm' else
                        f"config/l20_10_{hand}_vector.yaml" if scope == "hand" else f"config/l20_{hand}_vector.yaml")
    config = RetargetingConfig.load_from_file(config_path)
    physical_model=Path(config.urdf_path)
    model_temp=None;tip_metadata=None
    vector_tip=algorithm=='vector' and scope=='hand' and thumb_profile=='segments'
    if algorithm=='dexpilot' or vector_tip:
        from dexpilot_model import prepare_model
        model_temp,adapted,tip_metadata=prepare_model(physical_model)
        config.urdf_path=str(adapted)
    try:
        retargeting = config.build()
    except BaseException:
        if model_temp is not None:model_temp.cleanup()
        raise
    retargeting.lh20_model_temp=model_temp
    retargeting.lh20_tip_metadata=tip_metadata
    retargeting.lh20_physical_model_path=physical_model
    retargeting.lh20_algorithm=algorithm
    retargeting.lh20_vector_tip=vector_tip
    retargeting.lh20_thumb_root_mode=thumb_root_mode
    if algorithm=='dexpilot':
        from dexpilot_model import palm_center,finger_chain_lengths
        retargeting.lh20_palm_center=palm_center(retargeting.optimizer.robot,config.wrist_link_name)
        retargeting.lh20_finger_lengths=finger_chain_lengths(retargeting.optimizer.robot)
    retargeting.lh20_thumb_profile=thumb_profile
    retargeting.lh20_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest()
    sys.path.insert(0, str(ROOT / "bridge"))
    retargeting.lh20_limit_profile_sha256 = None
    retargeting.lh20_limit_basis = "urdf_only_unverified_hardware_mapping"
    if scope == "hand":
        from factory_mapping import FactoryMapping
        mapping = FactoryMapping(thumb_axes,hand=hand)
        retargeting.lh20_factory_mapping = mapping
        for index, name in enumerate(config.target_joint_names):
            retargeting.joint_limits[index] = mapping.limits[name]
        retargeting.lh20_limit_profile_sha256 = mapping.sha256
        retargeting.lh20_limit_basis = ("manufacturer_left_g20_native_model" if hand=="left" else "right_vendor_g20_model_adapter")
    if hand == "right" and scope != "hand":
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
    if vector_tip:
        from thumb_refinement import ThumbResponsiveOptimizer
        retargeting.optimizer = ThumbResponsiveOptimizer(retargeting.optimizer, retargeting.joint_limits, hand=hand)
        if thumb_root_mode=='directions':retargeting.optimizer.root_guide_weight=0.
    # The upstream optimizer expands limits by 0.001 rad. Use exact model bounds.
    retargeting.optimizer.opt.set_lower_bounds(retargeting.joint_limits[:, 0])
    retargeting.optimizer.opt.set_upper_bounds(retargeting.joint_limits[:, 1])
    print(f"Using Git source: {SOURCE}; commit={head}")
    print(f"Independent joints: {len(config.target_joint_names)}; model joints: {len(retargeting.joint_names)}")
    print(f"Hand: {hand}; optimizer limits: {retargeting.lh20_limit_basis}")
    print(f"Retargeting: {type(retargeting.optimizer).__name__}")
    return config, retargeting


def model_vectors(config, retargeting, qpos):
    robot = retargeting.optimizer.robot
    robot.compute_forward_kinematics(qpos)
    return np.array([
        robot.get_link_pose(robot.get_link_index(task))[:3, 3]
        - robot.get_link_pose(robot.get_link_index(origin))[:3, 3]
        for origin, task in zip(retargeting.optimizer.origin_link_names, retargeting.optimizer.task_link_names)
    ])


def check_qpos(retargeting, qpos):
    limits = retargeting.optimizer.robot.joint_limits
    if not np.isfinite(qpos).all():
        raise ValueError("Non-finite retargeting output")
    if np.any(qpos < limits[:, 0] - 1e-6) or np.any(qpos > limits[:, 1] + 1e-6):
        raise ValueError("Retargeting output exceeds URDF joint limits")


def self_test(hand="right", scope="index", thumb_axes="native", thumb_profile="segments", algorithm="vector", thumb_root_mode="agile"):
    # Model-generated vectors test the config/solver/mimic pipeline, not vision.
    for fraction in (0.05, 0.45, 0.85):
        config, retargeting = load_retargeting(hand, scope, thumb_axes,thumb_profile,algorithm,thumb_root_mode)
        robot = retargeting.optimizer.robot
        if algorithm=='dexpilot':
            # FK-generated references test model fitting, not intentional pinch
            # projection. Projection changes nearby tip targets by design and
            # is covered separately; runtime config and hysteresis stay intact.
            retargeting.optimizer.project_dist=0.
            retargeting.optimizer.escape_dist=0.
            print('Model fit self-test: pinch projection disabled; runtime policy unchanged.')
        qref = robot.joint_limits[:, 0] + fraction * np.ptp(robot.joint_limits, axis=1)
        qref[retargeting.optimizer.idx_pin2target] = (retargeting.joint_limits[:,0]
            + fraction*np.ptp(retargeting.joint_limits,axis=1))
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
        if retargeting.lh20_model_temp is not None:retargeting.lh20_model_temp.cleanup()
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
    note=('Vector: independent thumb direction fit / requested pose' if retargeting.lh20_vector_tip else
          'Mesh-derived fingertip frames; not physically calibrated' if config.type=='dexpilot' else
          'URDF joint origins; distal origins are NOT fingertips')
    cv2.putText(panel, note, (12, 455),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
    return panel


def run_camera(args):
    import cv2

    scope = getattr(args, "scope", "index")
    config, retargeting = load_retargeting(args.hand, scope, getattr(args,"thumb_axes","native"),getattr(args,'thumb_profile','segments'),getattr(args,'retargeting','vector'),getattr(args,'thumb_root_mode','agile'))
    mapping = getattr(retargeting, "lh20_factory_mapping", None)
    from factory_mapping import BRIDGE_PATH_POLICY
    bridge_path_cad = retargeting.lh20_vector_tip and mapping is not None
    if retargeting.lh20_vector_tip:
        from low_latency import AdaptiveJointFilter
        retargeting.filter=AdaptiveJointFilter(retargeting.optimizer.adaptor)
    auto_label = "AUTO FULL HAND FOLLOW" if scope == "hand" else "AUTO INDEX FOLLOW"
    if retargeting.lh20_algorithm=='dexpilot':auto_label+=' / DEXPILOT'
    from collision_check import CollisionChecker
    physical_model=retargeting.lh20_physical_model_path
    collision_checker = CollisionChecker(physical_model)
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
    reader = None
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {args.camera}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if retargeting.lh20_vector_tip and (args.camera.isdigit() or args.camera.startswith('/dev/video')):
            from low_latency import LatestFrameCapture
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            reader=LatestFrameCapture(cap)
        if args.record:
            # Do not overwrite an existing recording.
            output = open(args.record, "x", encoding="utf-8")
        tree = ET.parse(config.urdf_path).getroot()
        model_sha256 = hashlib.sha256(physical_model.read_bytes()).hexdigest()
        print(f"Detecting physical {args.hand} hand; model={physical_model}; " +
              (auto_label if args.auto_follow else "LIVE STREAM" if args.stream else "PREVIEW ONLY"), flush=True)
        print("Thumb axes: "+getattr(args,"thumb_axes","native"),flush=True)
        print("Thumb profile: "+retargeting.lh20_thumb_profile,flush=True)
        transform = rotation_matrix(args.rotate_deg)
        indices = retargeting.optimizer.target_link_human_indices
        active_indices = [retargeting.joint_names.index(n) for n in config.target_joint_names]
        last_print = 0.0
        count = 0
        while True:
            if reader is not None:
                frame,captured_ns,capture_sequence=reader.read()
            else:
                ok, frame = cap.read()
                captured_ns = time.monotonic_ns()
                capture_sequence=count
                if not ok:
                    raise RuntimeError("Camera frame read failed")
            timings={'capture_delivery_age':(time.monotonic_ns()-captured_ns)/1e6}
            packet = {"schema": "lh20.retarget.v1", "seq": count, "captured_monotonic_ns": captured_ns,
                      "valid": False, "hand": args.hand, "source_commit": COMMIT,
                      "model": str(physical_model.relative_to(ROOT)),
                      "model_sha256": model_sha256,
                      "limit_basis": retargeting.lh20_limit_basis,
                      "hardware_mapping_calibrated": False,
                      "joint_limit_profile_sha256": retargeting.lh20_limit_profile_sha256,
                      "rotate_deg": args.rotate_deg,
                      "stream_session": stream_session,
                      "operator_enabled": operator_enabled,
                      "auto_follow": args.auto_follow,
                      "calibration_epoch": calibration_epoch, "scope": scope, "thumb_axes": getattr(args,"thumb_axes","native"),
                      "thumb_profile":retargeting.lh20_thumb_profile,"retargeting_config_sha256":retargeting.lh20_config_sha256,
                      "retargeting_algorithm":retargeting.lh20_algorithm,
                      "camera_frame_seq":capture_sequence,
                      "vector_thumb_target":('anatomical_root_guide_v3' if retargeting.lh20_thumb_root_mode=='agile' else 'independent_directions_v2') if retargeting.lh20_vector_tip else None,
                      "thumb_root_mode":retargeting.lh20_thumb_root_mode,
                      "thumb_refinement":({'temporal_weight':retargeting.optimizer.temporal_weight,
                                           'initialization':'previous_refined_pose', 'segment_weights':[1.,1.,1.],
                                           'root_guide_weight':retargeting.optimizer.root_guide_weight,
                                           'root_guide_axis_weights':retargeting.optimizer.root_guide_axis_weights.tolist(),
                                           'root_guide_parameters':retargeting.optimizer.root_guide_parameters,
                                           'root_reference':'metacarpal_opposition_abduction_mcp_bend'}
                                          if retargeting.lh20_vector_tip else None),
                      "cad_validation":BRIDGE_PATH_POLICY if bridge_path_cad else None,
                      "joint_filter":'speed_adaptive_v1' if retargeting.lh20_vector_tip else 'fixed_ema_0.6',
                      "reference_alignment":'mcp_center_bone_lengths_v1' if retargeting.lh20_algorithm=='dexpilot' else 'segment_vectors'}
            stage=time.monotonic_ns()
            _, joint_pos, keypoint_2d, _ = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            timings['detection']=(time.monotonic_ns()-stage)/1e6
            panel = np.full((480, 640, 3), 24, dtype=np.uint8)
            status = f"NO VALID {args.hand.upper()} HAND - no joint target"
            if joint_pos is not None and np.isfinite(joint_pos).all():
                joint_pos = joint_pos @ transform.T
                stage=time.monotonic_ns()
                try:
                    if retargeting.lh20_algorithm=='dexpilot':
                        from dexpilot_model import reference_vectors
                        ref_value=reference_vectors(joint_pos,retargeting)
                    else:ref_value = joint_pos[indices[1]] - joint_pos[indices[0]]
                    if retargeting.lh20_vector_tip:
                        retargeting.filter.set_timestamp(captured_ns/1e9)
                        if retargeting.lh20_thumb_root_mode=='agile':
                            packet['thumb_root_guide_rad']=retargeting.optimizer.set_root_reference(joint_pos).tolist()
                    qpos = retargeting.retarget(ref_value)
                    solver_ok = retargeting.optimizer.opt.last_optimize_result() > 0
                    check_qpos(retargeting, qpos)
                    valid = solver_ok
                except ValueError:
                    valid = False
                timings['retargeting']=(time.monotonic_ns()-stage)/1e6
                stage=time.monotonic_ns()
                if valid and not bridge_path_cad:
                    hits = collision_checker.check(collision_checker.expand(dict(zip(retargeting.joint_names, qpos))))
                    packet["cad_collision_hits"] = hits
                    if hits:
                        valid = False
                if valid and mapping is not None:
                    try:
                        raw = mapping.encode(dict(zip(config.target_joint_names, qpos[active_indices].tolist())))
                        quantized = collision_checker.expand(mapping.decode(raw))
                        hits = collision_checker.check(quantized)
                        valid = not collision_checker.violations(quantized) and (bridge_path_cad or not hits)
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
                    status = f"{args.hand.upper()} HAND - " + (auto_label if args.auto_follow else
                             "LIVE STREAM" if args.stream else ("16 angles + factory raw, NO MOTOR OUTPUT" if mapping is not None else "16 model angles, NO MOTOR OUTPUT"))
                    if packet.get('cad_collision_hits'):
                        status = 'REQUESTED CONTACT - bridge checks actual path / partial follow'
                else:
                    status = "CAD COLLISION - no joint target" if packet.get("cad_collision_hits") else "INVALID SOLVER OUTPUT - no joint target"
                timings['vision_cad']=(time.monotonic_ns()-stage)/1e6
            packet["processed_monotonic_ns"] = time.monotonic_ns()
            timings['processing']=(packet['processed_monotonic_ns']-captured_ns)/1e6
            packet['timing_ms']=timings
            if sender is not None:
                online = sender.send(packet)
                stream_status = ("STREAM ONLINE" if online else "BRIDGE OFFLINE") + (" / ENABLED" if operator_enabled else " / PAUSED")
                cv2.putText(frame, stream_status, (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
                cv2.putText(frame, (f"AUTO {args.hand.upper()} FULL HAND | Q quit" if mapping is not None else f"AUTO {args.hand.upper()} INDEX | Q quit") if args.auto_follow else "SPACE toggle | S pause | C recenter | Q quit", (10, 465),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 220, 255), 1)
            # The target reaches the bridge before model drawing / disk IO.
            stage=time.monotonic_ns()
            if packet['valid']:
                panel=model_panel(config,retargeting,qpos,tree)
            if keypoint_2d is not None:
                detector.draw_skeleton_on_image(frame,keypoint_2d,style='default')
            timings['rendering']=(time.monotonic_ns()-stage)/1e6
            if output is not None:
                output.write(json.dumps(packet, allow_nan=False) + "\n")
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
        if reader is not None:reader.close()
        else:cap.release()
        detector.hand_detector.close()
        if output is not None:
            output.close()
        cv2.destroyAllWindows()
        if retargeting.lh20_model_temp is not None:retargeting.lh20_model_temp.cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--thumb-axes",choices=("native","swapped"),default="native")
    parser.add_argument("--thumb-profile",choices=('segments','palm'),default='segments')
    parser.add_argument('--thumb-root-mode',choices=('agile','directions'),default='agile')
    parser.add_argument('--retargeting',choices=('vector','dexpilot'),default='vector')
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
        self_test(args.hand, args.scope,args.thumb_axes,args.thumb_profile,args.retargeting,args.thumb_root_mode)
    else:
        run_camera(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
