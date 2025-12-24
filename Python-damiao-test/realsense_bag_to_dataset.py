#!/usr/bin/env python3
"""
Convert a RealSense D435i .bag recording into a rich dataset folder.

Outputs (all inside one folder):
  - rgbd.mp4      : side-by-side video (left=depth Jet colormap, right=color),
                    depth & color are the SAME width/height (color resolution).
  - ir.mp4        : infrared stream video (if present, first IR stream only).
  - imu.csv       : accelerometer & gyro samples.
  - depth.npz     : raw depth frames + timestamps + depth scale.
  - metadata.json : intrinsics, fps (nominal), frame counts, file paths, etc.

Usage:
    python bag_to_dataset.py input.bag [output_dir]

If output_dir is omitted, it creates "<bag_basename>_dataset" next to the .bag.
"""

import sys
import os
import json
import csv
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def open_realsense_bag(input_path: str):
    """Open a RealSense bag file and return (pipeline, profile, playback)."""
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, input_path, repeat_playback=False)
    profile = pipeline.start(config)

    playback = profile.get_device().as_playback()
    # 再生を「実時間」に縛らない（処理が遅くてもフレームを落とさない）
    playback.set_real_time(False)

    return pipeline, profile, playback


def create_video_writer(path: Path, fps: float, size, is_color: bool = True):
    """
    Try to create an MP4 writer. If that fails, fall back to AVI/MJPG.

    Returns (writer, final_path) or (None, None) on failure.
    """
    w, h = size

    if fps <= 0 or not np.isfinite(fps):
        print(f"[WARN] Invalid FPS ({fps}), using 30.0 instead.")
        fps = 30.0

    path.parent.mkdir(parents=True, exist_ok=True)

    # 1) MP4
    if path.suffix.lower() != ".mp4":
        path = path.with_suffix(".mp4")
    fourcc_mp4 = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc_mp4, fps, (w, h), isColor=is_color)
    if writer.isOpened():
        print(f"[INFO] Using MP4 writer: {path} (fps={fps})")
        return writer, path

    # 2) AVI (MJPG) fallback
    print("[WARN] Could not open MP4 writer. Falling back to AVI (MJPG).")
    avi_path = path.with_suffix(".avi")
    fourcc_avi = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(avi_path), fourcc_avi, fps, (w, h), isColor=is_color)
    if writer.isOpened():
        print(f"[INFO] Using AVI writer: {avi_path} (fps={fps})")
        return writer, avi_path

    print("[ERROR] Could not open any video writer for", path)
    return None, None


def intrinsics_to_dict(vs_profile: rs.video_stream_profile | None):
    if vs_profile is None:
        return None
    try:
        intr = vs_profile.get_intrinsics()
    except Exception:
        return None
    return {
        "width": intr.width,
        "height": intr.height,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "fx": intr.fx,
        "fy": intr.fy,
        "model": int(intr.model),
        "coeffs": list(intr.coeffs),
    }


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------
def main():
    # ---------- Arg parsing ----------
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python bag_to_dataset.py input.bag [output_dir]")
        sys.exit(1)

    input_path = Path(sys.argv[1])
    if not input_path.is_file():
        print(f"[ERROR] File not found: {input_path}")
        sys.exit(1)

    if len(sys.argv) == 3:
        output_dir = Path(sys.argv[2])
    else:
        base = input_path.with_suffix("")
        output_dir = base.parent / (base.name + "_dataset")

    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Input bag : {input_path}")
    print(f"[INFO] Output dir: {output_dir}")

    # ---------- Open bag ----------
    try:
        pipeline, profile, playback = open_realsense_bag(str(input_path))
    except Exception as e:
        print(f"[ERROR] Failed to open RealSense bag: {e}")
        sys.exit(1)

    device = profile.get_device()
    dev_name = device.get_info(rs.camera_info.name)
    dev_sn = device.get_info(rs.camera_info.serial_number)
    dev_fw = device.get_info(rs.camera_info.firmware_version)
    print(f"[INFO] Device: {dev_name} (SN={dev_sn}, FW={dev_fw})")

    # ---------- Get stream profiles ----------
    try:
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    except Exception:
        color_stream = None

    try:
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    except Exception:
        depth_stream = None

    # Infrared: first IR stream we find (IR1 or IR2)
    ir_stream = None
    try:
        for sp in profile.get_streams():
            if sp.stream_type() == rs.stream.infrared:
                ir_stream = sp.as_video_stream_profile()
                break
    except Exception:
        ir_stream = None

    if depth_stream is None and color_stream is None and ir_stream is None:
        print("[ERROR] No color, depth, or infrared stream found in bag file.")
        pipeline.stop()
        sys.exit(1)

    # Depth scale (meters per unit)
    depth_scale = None
    try:
        depth_sensor = device.first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        print(f"[INFO] Depth scale: {depth_scale} meters per unit")
    except Exception:
        print("[WARN] Could not query depth scale.")

    # Resolutions & nominal FPS (same扱いにして mp4 スクリプトと揃える)
    if color_stream is not None:
        w_color, h_color = color_stream.width(), color_stream.height()
        fps_color = float(color_stream.fps())
        print(f"[INFO] Color stream: {w_color}x{h_color} @ {fps_color} FPS")
    else:
        w_color = h_color = 0
        fps_color = 0.0
        print("[INFO] No color stream in this bag.")

    if depth_stream is not None:
        w_depth, h_depth = depth_stream.width(), depth_stream.height()
        fps_depth = float(depth_stream.fps())
        print(f"[INFO] Depth stream: {w_depth}x{h_depth} @ {fps_depth} FPS")
    else:
        w_depth = h_depth = 0
        fps_depth = 0.0
        print("[INFO] No depth stream in this bag.")

    if ir_stream is not None:
        w_ir, h_ir = ir_stream.width(), ir_stream.height()
        fps_ir = float(ir_stream.fps())
        print(f"[INFO] Infrared stream: {w_ir}x{h_ir} @ {fps_ir} FPS")
    else:
        w_ir = h_ir = 0
        fps_ir = 0.0
        print("[INFO] No infrared stream in this bag.")

    has_color = color_stream is not None
    has_depth = depth_stream is not None
    has_ir = ir_stream is not None

    # ---------- Decide RGBD video resolution & FPS ----------
    # 要求: depth と RGB を同じサイズにしたい → カラー解像度に合わせる
    if has_color and has_depth:
        rgbd_height = h_color
        single_width = w_color
        rgbd_width = single_width * 2  # [depth(width=h_color) | color(width=h_color)]
        fps_rgbd = fps_color if fps_color > 0 else (fps_depth if fps_depth > 0 else 30.0)
        print("[INFO] RGBD video: depth (left) + color (right), both at color resolution.")
    elif has_color:
        rgbd_height = h_color
        rgbd_width = w_color
        fps_rgbd = fps_color if fps_color > 0 else 30.0
        single_width = w_color
        print("[INFO] RGBD video: color-only (no depth).")
    elif has_depth:
        rgbd_height = h_depth
        rgbd_width = w_depth
        fps_rgbd = fps_depth if fps_depth > 0 else 30.0
        single_width = w_depth
        print("[INFO] RGBD video: depth-only (no color).")
    else:
        rgbd_height = rgbd_width = 0
        fps_rgbd = 0.0
        single_width = 0

    # ---------- Create writers ----------
    rgbd_writer = None
    rgbd_path = None
    if rgbd_width > 0 and rgbd_height > 0:
        rgbd_writer, rgbd_path = create_video_writer(
            output_dir / "rgbd.mp4", fps_rgbd, (rgbd_width, rgbd_height), is_color=True
        )

    ir_writer = None
    ir_path = None
    fps_ir_final = fps_ir if fps_ir > 0 else (fps_rgbd if fps_rgbd > 0 else 30.0)
    if has_ir:
        ir_writer, ir_path = create_video_writer(
            output_dir / "ir.mp4", fps_ir_final, (w_ir, h_ir), is_color=True
        )

    # ---------- Depth colorizer (Jet, same as mp4 script) ----------
    depth_colorizer = rs.colorizer()
    depth_colorizer.set_option(rs.option.color_scheme, 2)  # Jet-like color scheme

    # ---------- Accumulators ----------
    depth_frames_raw = []
    depth_timestamps_ms = []

    imu_rows = []
    any_accel = False
    any_gyro = False

    rgbd_frames_written = 0
    ir_frames_written = 0

    # ---------- Main loop ----------
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames()
            except RuntimeError:
                print("[INFO] End of bag file reached.")
                break

            # Color & depth: mp4 スクリプトと同じ取り方
            color_frame = frames.get_color_frame() if has_color else None
            depth_frame = frames.get_depth_frame() if has_depth else None

            # IR + IMU: Frameset 内の各 frame を調べる
            ir_frame = None
            for f in frames:
                prof = f.get_profile()
                stype = prof.stream_type()

                if has_ir and stype == rs.stream.infrared and ir_frame is None:
                    ir_frame = f

                if f.is_motion_frame():
                    motion = f.as_motion_frame()
                    md = motion.get_motion_data()
                    ts = motion.get_timestamp()
                    st = motion.get_profile().stream_type()
                    if st == rs.stream.accel:
                        imu_rows.append({
                            "timestamp_ms": ts,
                            "type": "accel",
                            "x": md.x,
                            "y": md.y,
                            "z": md.z,
                        })
                        any_accel = True
                    elif st == rs.stream.gyro:
                        imu_rows.append({
                            "timestamp_ms": ts,
                            "type": "gyro",
                            "x": md.x,
                            "y": md.y,
                            "z": md.z,
                        })
                        any_gyro = True

            # ---------------- RGBD frame ----------------
            rgbd_frame = None
            color_img = None
            depth_colormap = None

            if color_frame is not None:
                color_img = np.asanyarray(color_frame.get_data())  # BGR
            if depth_frame is not None:
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_frames_raw.append(depth_raw)
                depth_timestamps_ms.append(depth_frame.get_timestamp())
                depth_colormap = np.asanyarray(depth_colorizer.colorize(depth_frame).get_data())

            if color_img is None and depth_colormap is None:
                # no visual frame
                pass
            elif color_img is not None and depth_colormap is not None:
                # 両方ともカラー解像度にリサイズ
                color_resized = cv2.resize(
                    color_img, (w_color, h_color), interpolation=cv2.INTER_NEAREST
                )
                depth_resized = cv2.resize(
                    depth_colormap, (w_color, h_color), interpolation=cv2.INTER_NEAREST
                )

                # 2 倍の横幅で side-by-side
                rgbd_frame = np.zeros((rgbd_height, rgbd_width, 3), dtype=np.uint8)
                rgbd_frame[:, :w_color, :] = depth_resized
                rgbd_frame[:, w_color:w_color*2, :] = color_resized
            elif color_img is not None:
                rgbd_frame = cv2.resize(
                    color_img, (rgbd_width, rgbd_height), interpolation=cv2.INTER_NEAREST
                )
            else:
                rgbd_frame = cv2.resize(
                    depth_colormap, (rgbd_width, rgbd_height), interpolation=cv2.INTER_NEAREST
                )

            if rgbd_writer is not None and rgbd_frame is not None:
                rgbd_writer.write(rgbd_frame)
                rgbd_frames_written += 1

            # ---------------- IR frame ----------------
            if ir_writer is not None and ir_frame is not None:
                ir_img = np.asanyarray(ir_frame.get_data())

                # IR は 8bit or 16bit のグレースケール → BGR 3ch に変換
                if ir_img.ndim == 2:
                    if ir_img.dtype == np.uint16:
                        ir_8 = (ir_img / 256).astype(np.uint8)
                    else:
                        ir_8 = ir_img.astype(np.uint8)
                    ir_bgr = cv2.cvtColor(ir_8, cv2.COLOR_GRAY2BGR)
                elif ir_img.ndim == 3 and ir_img.shape[2] == 1:
                    ir_bgr = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2BGR)
                else:
                    ir_bgr = ir_img

                ir_bgr_resized = cv2.resize(
                    ir_bgr, (w_ir, h_ir), interpolation=cv2.INTER_NEAREST
                )
                ir_writer.write(ir_bgr_resized)
                ir_frames_written += 1

    finally:
        pipeline.stop()
        if rgbd_writer is not None:
            rgbd_writer.release()
        if ir_writer is not None:
            ir_writer.release()

    print(f"[INFO] Frames written - RGBD: {rgbd_frames_written}, IR: {ir_frames_written}")
    print(f"[INFO] IMU samples - accel: {any_accel}, gyro: {any_gyro}, total rows: {len(imu_rows)}")

    # -----------------------------------------------------------------
    # Save depth.npz
    # -----------------------------------------------------------------
    depth_npz_path = None
    if depth_frames_raw:
        depth_stack = np.stack(depth_frames_raw, axis=0)
        depth_npz_path = output_dir / "depth.npz"
        np.savez_compressed(
            depth_npz_path,
            depth=depth_stack,
            depth_timestamps_ms=np.array(depth_timestamps_ms, dtype=np.float64),
            depth_scale_m=depth_scale if depth_scale is not None else -1.0,
        )
        print(f"[INFO] Saved depth.npz: {depth_npz_path}")
    else:
        print("[WARN] No depth frames collected; depth.npz not created.")

    # -----------------------------------------------------------------
    # Save imu.csv
    # -----------------------------------------------------------------
    imu_path = output_dir / "imu.csv"
    with imu_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["timestamp_ms", "type", "x", "y", "z"],
        )
        writer.writeheader()
        if imu_rows:
            writer.writerows(imu_rows)
    if imu_rows:
        print(f"[INFO] Saved IMU CSV: {imu_path}")
    else:
        print("[WARN] No IMU samples found; imu.csv only has header.")

    # -----------------------------------------------------------------
    # Save metadata.json
    # -----------------------------------------------------------------
    meta = {
        "bag_file": str(input_path.resolve()),
        "output_dir": str(output_dir.resolve()),
        "device": {
            "name": dev_name,
            "serial_number": dev_sn,
            "firmware_version": dev_fw,
        },
        "depth_scale_m": depth_scale,
        "streams": {
            "color": {
                "present": has_color,
                "fps_nominal": fps_color,
                "intrinsics": intrinsics_to_dict(color_stream),
            },
            "depth": {
                "present": has_depth,
                "fps_nominal": fps_depth,
                "intrinsics": intrinsics_to_dict(depth_stream),
            },
            "infrared": {
                "present": has_ir,
                "fps_nominal": fps_ir if has_ir else 0.0,
                "intrinsics": intrinsics_to_dict(ir_stream),
            },
            "imu": {
                "has_accel": any_accel,
                "has_gyro": any_gyro,
                "sample_count": len(imu_rows),
            },
        },
        "frame_counts": {
            "rgbd": rgbd_frames_written,
            "infrared": ir_frames_written,
            "depth_raw": len(depth_frames_raw),
        },
        "outputs": {
            "rgbd_video": str(rgbd_path) if rgbd_path is not None else None,
            "ir_video": str(ir_path) if ir_path is not None else None,
            "depth_npz": str(depth_npz_path) if depth_npz_path is not None else None,
            "imu_csv": str(imu_path),
        },
    }

    meta_path = output_dir / "metadata.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[INFO] Saved metadata.json: {meta_path}")
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
