"""Synchronized camera/model research capture; never creates a motion sender."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "teleop/vision"))
from preview_l20 import load_retargeting, check_qpos


def observation(points, keypoints, config, retargeting, checker, mapping):
    """Keep diagnostics even when the candidate fails the normal preview gate."""
    result = {"valid": False, "rejection_reason": "no_selected_hand",
              "joint_names": config.target_joint_names, "detected_selected_hand": points is not None}
    if keypoints is not None:
        xy = np.array([[p.x, p.y, p.z] for p in keypoints.landmark])
        if np.isfinite(xy).all():
            result["landmarks_image_normalized"] = xy.tolist()
    if points is None:
        return result
    points = np.asarray(points)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        result["rejection_reason"] = "invalid_landmarks"
        return result
    result["landmarks_local_m"] = points.tolist()
    indices = retargeting.optimizer.target_link_human_indices
    qpos = retargeting.retarget(points[indices[1]] - points[indices[0]])
    result["solver_status"] = int(retargeting.optimizer.opt.last_optimize_result())
    active = [retargeting.joint_names.index(n) for n in config.target_joint_names]
    if np.isfinite(qpos).all():
        result["candidate_position_rad"] = qpos[active].tolist()
        result["candidate_model_position_rad"] = qpos.tolist()
    try:
        check_qpos(retargeting, qpos)
    except ValueError as error:
        result.update(rejection_reason="invalid_solver_output", rejection_detail=str(error))
        return result
    if result["solver_status"] <= 0:
        result["rejection_reason"] = "solver_failed"
        return result
    hits = checker.check(checker.expand(dict(zip(retargeting.joint_names, qpos))))
    result["cad_collision_hits"] = hits
    if hits:
        result["rejection_reason"] = "cad_collision"
        return result
    try:
        raw = mapping.encode(dict(zip(config.target_joint_names, qpos[active].tolist())))
        quantized = checker.expand(mapping.decode(raw))
        result["candidate_motor_position_raw"] = raw
        hits = checker.check(quantized)
        result["quantized_cad_collision_hits"] = hits
        violations = checker.violations(quantized)
        if hits or violations:
            result.update(rejection_reason="quantized_cad_or_limits", quantized_limit_violations=violations)
            return result
    except ValueError as error:
        result.update(rejection_reason="mapping_failed", rejection_detail=str(error))
        return result
    result.update(valid=True, rejection_reason=None, position_rad=qpos[active].tolist(), motor_position_raw=raw)
    return result


class FrameArchive:
    """One lossless video frame per record; JSON timestamps define real timing."""
    def __init__(self, path, shape):
        import cv2
        if path.exists():
            raise FileExistsError(path)
        self.shape = tuple(shape)
        self.frames = 0
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"FFV1"), 30.,
                                      (shape[1], shape[0]))
        if not self.writer.isOpened():
            self.writer.release()
            raise RuntimeError("无法创建 FFV1 无损视频；采集未开始")

    def write(self, frame):
        if tuple(frame.shape) != self.shape or frame.dtype != np.uint8:
            raise ValueError("摄像头图像尺寸或类型改变")
        index = self.frames
        self.writer.write(frame)
        self.frames += 1
        return index

    def close(self):
        self.writer.release()


def verify_video(path, expected_frames):
    import cv2
    capture = cv2.VideoCapture(str(path))
    count = 0
    try:
        if not capture.isOpened():
            raise RuntimeError("保存的视频无法解码")
        while capture.read()[0]:
            count += 1
    finally:
        capture.release()
    if count != expected_frames:
        raise RuntimeError(f"视频和数据帧数不一致：video={count}, json={expected_frames}")
    return count


def run(args):
    import cv2
    from collision_check import CollisionChecker
    config, retargeting = load_retargeting("left", "hand", "swapped")
    mapping = retargeting.lh20_factory_mapping
    checker = CollisionChecker(config.urdf_path)
    source = ROOT / "teleop/dex-retargeting"
    sys.path.insert(0, str(source / "example/vector_retargeting"))
    from single_hand_detector import SingleHandDetector

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = args.output_root / f"{stamp}_{uuid.uuid4().hex[:6]}"
    folder.mkdir(parents=True, exist_ok=False)
    metadata = {"schema": "lh20.preview_capture.session.v1", "hardware_commands_sent": False,
                "hand": "left", "scope": "hand", "thumb_axes": "swapped", "camera": args.camera,
                "model_sha256": mapping.model_hash, "mapping_sha256": mapping.sha256,
                "model_joint_names": retargeting.joint_names,
                "code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "existing_low_pass_alpha": config.low_pass_alpha,
                "video": "camera.avi", "video_codec": "FFV1", "video_nominal_fps": 30,
                "timing": "frame_index joins JSON to video; monotonic time immediately after cap.read, not exposure time",
                "markers": "operator intention only, not ground-truth occlusion times",
                "status": "initializing"}
    def save_metadata():
        (folder / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    save_metadata()
    camera = detector = archive = records = markers = None
    count = 0
    phase = "clear"
    try:
        detector = SingleHandDetector(hand_type="Left", selfie=False)
        device = int(args.camera) if args.camera.isdigit() else args.camera
        camera = cv2.VideoCapture(device)
        if not camera.isOpened():
            raise RuntimeError(f"无法打开摄像头 {args.camera}")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        records = (folder / "frames.jsonl").open("x", encoding="utf-8")
        markers = (folder / "markers.jsonl").open("x", encoding="utf-8")
        metadata["status"] = "recording"
        save_metadata()
        print(f"纯视觉采集，不发送电机指令。数据：{folder.resolve()}")
        print("在窗口中：B=准备遮挡，E=遮挡已移开，Q=保存退出。按键只是辅助标记，实际遮挡以视频复核。")
        while True:
            ok, frame = camera.read()
            captured = time.monotonic_ns()
            if not ok:
                raise RuntimeError("摄像头读取失败")
            if archive is None:
                archive = FrameArchive(folder / "camera.avi", frame.shape)
                metadata["video_shape"] = list(frame.shape)
            index = archive.write(frame)  # Save the unannotated camera image.
            archived = time.monotonic_ns()
            _, points, keypoints, _ = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            packet = observation(points, keypoints, config, retargeting, checker, mapping)
            processed = time.monotonic_ns()
            packet.update(schema="lh20.preview_capture.frame.v1", seq=count, video_frame_index=index,
                          captured_monotonic_ns=captured, archived_monotonic_ns=archived,
                          processed_monotonic_ns=processed, operator_phase=phase)
            records.write(json.dumps(packet, allow_nan=False) + "\n")
            records.flush()
            count += 1
            if keypoints is not None:
                detector.draw_skeleton_on_image(frame, keypoints, style="default")
            text = "VALID" if packet["valid"] else packet["rejection_reason"]
            cv2.putText(frame, f"frame {index} | {text}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 255), 1)
            cv2.putText(frame, f"{phase} | B begin / E end / Q save", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 255), 1)
            cv2.imshow("Research capture - camera only", frame)
            key = cv2.waitKey(1) & 0xff
            if key in (ord("b"), ord("B"), ord("e"), ord("E"), ord("q"), ord("Q")):
                label = {ord("b"): "occlusion_intent_begin", ord("e"): "occlusion_intent_end", ord("q"): "quit"}[ord(chr(key).lower())]
                event = {"schema": "lh20.preview_capture.marker.v1", "label": label,
                         "monotonic_ns": time.monotonic_ns(), "last_displayed_video_frame_index": index}
                markers.write(json.dumps(event) + "\n")
                markers.flush()
                phase = "occlusion_intended" if label == "occlusion_intent_begin" else "clear"
                print(f"标记：{label}，对应最近显示帧 {index}")
            if key in (ord("q"), ord("Q")):
                metadata["status"] = "complete"
                break
    except BaseException as error:
        metadata.update(status="aborted", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        if camera is not None:
            camera.release()
        if detector is not None:
            detector.hand_detector.close()
        if archive is not None:
            archive.close()
        for stream in (records, markers):
            if stream is not None:
                stream.close()
        cv2.destroyAllWindows()
        metadata["recorded_frames"] = count
        if archive is not None:
            try:
                metadata["verified_video_frames"] = verify_video(folder / "camera.avi", count)
            except Exception as error:
                metadata.update(status="video_verification_failed", verification_error=str(error))
        save_metadata()
        print(f"已保存 {count} 帧；状态 {metadata['status']}：{folder.resolve()}")
    if metadata["status"] != "complete":
        raise RuntimeError("本次采集未完整通过校验，请保留现场输出")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--output-root", type=Path, default=ROOT / "teleop/data/teleop_study/synced")
    run(parser.parse_args())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("采集被中断；已尽可能关闭并保存文件。")
