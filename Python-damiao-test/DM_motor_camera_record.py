#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import csv, json
from datetime import datetime

import os, sys, time, glob, threading, queue
import serial
import numpy as np
import cv2

# RealSense
import pyrealsense2 as rs  # pip install pyrealsense2

# DM motors
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type

from utils import Rate

# ================= パラメータ =================
maxTorque = 10.0
K_p = 20.0
MOTOR_FPS = 100.0      # 100 Hz
RS_FPS    = 30         # 30 Hz (color/depth)
RS_SIZE   = (640, 480) # 解像度（必要に応じ変更）

# ================= ユーティリティ =================
def default_serial_port():
    env = os.getenv("DM_PORT")
    if env: return env
    if sys.platform.startswith("win"):
        return "COM7"
    if sys.platform == "darwin":
        for pat in ["/dev/cu.usbmodem*","/dev/cu.usbserial*","/dev/cu.SLAB_USBtoUART*","/dev/cu.wchusbserial*"]:
            m = sorted(glob.glob(pat))
            if m: return m[0]
        return "/dev/cu.usbmodem00000000050C1"
    for cand in ("/dev/ttyACM0","/dev/ttyUSB0"):
        if os.path.exists(cand): return cand
    return "/dev/ttyACM0"

# ================= 入力（stdin: s/q） =================
cmd_q: "queue.Queue[str]" = queue.Queue()
shutdown_evt = threading.Event()
def stdin_watcher():
    for line in sys.stdin:
        t=line.strip().lower()
        if not t: continue
        if "q" in t: cmd_q.put("q"); break
        if "s" in t: cmd_q.put("s")
    shutdown_evt.set()
def wait_cmd(target:str):
    while True:
        if shutdown_evt.is_set(): return "q"
        try:
            c=cmd_q.get(timeout=0.05)
            if c in (target,"q"): return c
        except queue.Empty:
            pass

# ================= 制御関数 =================
def MITMaxTorque(MotorObj, target_angle: float, kp: float, target_vel: float, kd: float):
    now_angle = MotorObj.getPosition()
    diff = abs(target_angle - now_angle)
    if diff < 1e-9:
        MotorControl1.controlMIT(MotorObj, kp, kd, target_angle, target_vel, 0); return
    power = diff * kp
    if power > maxTorque:
        MotorControl1.controlMIT(MotorObj, maxTorque/diff, kd, target_angle, target_vel, 0)
    else:
        MotorControl1.controlMIT(MotorObj, kp, kd, target_angle, target_vel, 0)

# ================= RealSense スレッド =================
class RealSenseWorker(threading.Thread):
    """
    color/depthをcolor座標系にalignしつつ撮影。
    各フレームで:
      - frame_idx
      - rs_hw_ts_ms  : RealSenseのタイムスタンプ(ms)
      - rs_ts_domain : 'global_time' 等
      - host_ts_s    : 受信時のホストmonotonic秒
      - パス: color_png, depth_png
    を index CSV に追記する。
    """
    def __init__(self, out_dir: Path, fps=30, size=(640,480)):
        super().__init__(daemon=True)
        self.out_dir = out_dir
        self.fps=fps; self.w,self.h=size
        (out_dir/"rs_color").mkdir(parents=True, exist_ok=True)
        (out_dir/"rs_depth").mkdir(parents=True, exist_ok=True)
        self._stop_evt = threading.Event()
        self.index_rows=[]
        self.started=False
        self.depth_scale=None
        self.intrinsics={}
        self.extrinsics=None
        self._frame_idx=0

    def stop(self): self._stop_evt.set()

    def run(self):
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)
        profile = pipeline.start(cfg)

        # align depth->color
        align = rs.align(rs.stream.color)  # 深度をカラーに合わせる。:contentReference[oaicite:5]{index=5}

        # depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())  # 深度の単位→メートル換算の係数。:contentReference[oaicite:6]{index=6}

        # intrinsics / extrinsics（保存）
        c_prof = profile.get_stream(rs.stream.color).as_video_stream_profile()
        d_prof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        c_intr = c_prof.get_intrinsics()
        self.intrinsics = {
            "color": dict(width=c_intr.width, height=c_intr.height, ppx=c_intr.ppx, ppy=c_intr.ppy,
                          fx=c_intr.fx, fy=c_intr.fy, model=int(c_intr.model), coeffs=list(c_intr.coeffs)),
        }
        # depth->color の外部パラメータ（回転3x3, 並進）
        ext = d_prof.get_extrinsics_to(c_prof)
        self.extrinsics = {"rotation": list(ext.rotation), "translation": list(ext.translation)}

        rate = Rate(self.fps); rate.reset()
        self.started=True
        try:
            while not self._stop_evt.is_set():
                frames = pipeline.wait_for_frames()
                aligned = align.process(frames)  # color座標に整列。:contentReference[oaicite:7]{index=7}

                c = aligned.get_color_frame()
                d = aligned.get_depth_frame()
                if not c or not d:
                    rate.sleep(); continue

                rs_ts_ms = float(c.get_timestamp())         # ハードウェア時刻(ms) :contentReference[oaicite:8]{index=8}
                domain   = str(c.get_frame_timestamp_domain())  # どの時計か（global_time等） :contentReference[oaicite:9]{index=9}
                host_ts  = time.monotonic()                  # 受信時のホスト単調時計

                color = np.asanyarray(c.get_data())
                depth = np.asanyarray(d.get_data())  # z16

                # 保存（連番PNG）
                stem = f"{self._frame_idx:06d}"
                color_path = self.out_dir/"rs_color"/f"{stem}.png"
                depth_path = self.out_dir/"rs_depth"/f"{stem}.png"
                cv2.imwrite(str(color_path), color)                 # BGR8
                cv2.imwrite(str(depth_path), depth)                 # Z16そのまま

                self.index_rows.append([
                    self._frame_idx, rs_ts_ms, domain, host_ts,
                    color_path.as_posix(), depth_path.as_posix()
                ])
                self._frame_idx += 1

                rate.sleep()
        finally:
            pipeline.stop()

# ================= メイン：モーター + RealSense 録画 =================
def main():
    # --- モータ初期化 ---
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM6006, 0x02, 0x15)
    Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x11)
    Motor4 = Motor(DM_Motor_Type.DM6006, 0x04, 0x15)
    Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x11)

    serial_device = serial.Serial(default_serial_port(), 921600, timeout=0.5)
    print("Serial port is open:", getattr(serial_device, "port", "?"))

    global MotorControl1
    MotorControl1 = MotorControl(serial_device)
    for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
        MotorControl1.addMotor(m)
        MotorControl1.switchControlMode(m, Control_Type.MIT)
    for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
        MotorControl1.save_motor_param(m); MotorControl1.enable(m)
    time.sleep(1.5)
    for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
        MotorControl1.set_zero_position(m)
        MotorControl1.controlMIT(m, 3, 0, 0, 0, 0)
    print("motor setup done")

    # 出力ディレクトリ
    ts_dir = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = Path("recordings")/ts_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    # RealSense開始
    rs_worker = RealSenseWorker(out_dir, fps=RS_FPS, size=RS_SIZE)
    rs_worker.start()
    while not rs_worker.started: time.sleep(0.01)

    # stdin監視開始
    th = threading.Thread(target=stdin_watcher, daemon=True); th.start()

    # モーター録画
    print("\n[REC] start  ——  停止: 's'+Enter ／ 終了: 'q'+Enter")
    mot_rows=[]  # [t_s, m1_pos, m1_vel, ..., m5_pos, m5_vel, host_ts]
    rate = Rate(MOTOR_FPS); t0 = rate.reset()
    try:
        while True:
            for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
                MotorControl1.controlMIT(m, 0, 0, 0, 0, 0)

            frame = [
                [Motor1.getPosition(), Motor1.getVelocity()],
                [Motor2.getPosition(), Motor2.getVelocity()],
                [Motor3.getPosition(), Motor3.getVelocity()],
                [Motor4.getPosition(), Motor4.getVelocity()],
                [Motor5.getPosition(), Motor5.getVelocity()],
            ]
            t = time.perf_counter() - t0
            host_ts = time.monotonic()
            row = [t] + [x for pair in frame for x in pair] + [host_ts]
            mot_rows.append(row)

            # 停止確認
            if not cmd_q.empty():
                c = cmd_q.get_nowait()
                if c == "s": break
                if c == "q": raise KeyboardInterrupt

            rate.sleep()
        print("[REC] stopping...")
    except KeyboardInterrupt:
        print("[STOP] requested")
    finally:
        # RealSenseも停止
        rs_worker.stop(); rs_worker.join()

    # 保存（モーターCSV）
    mot_csv = out_dir/"motors.csv"
    hdr = ["t_s"]+[f"m{i}_{k}" for i in range(1,6) for k in ("pos","vel")] + ["host_ts_s"]
    with open(mot_csv, "w", newline="") as f:
        w=csv.writer(f); w.writerow(hdr); w.writerows(mot_rows)

    # 保存（RealSenseインデックス＆メタ）
    rs_idx_csv = out_dir/"realsense_index.csv"
    with open(rs_idx_csv, "w", newline="") as f:
        w=csv.writer(f)
        w.writerow(["frame_index","rs_hw_ts_ms","rs_ts_domain","host_ts_s","color_png","depth_png"])
        w.writerows(rs_worker.index_rows)

    meta = {
        "motors": [{"id":1,"type":"DM4310"},{"id":2,"type":"DM6006"},
                   {"id":3,"type":"DM4310"},{"id":4,"type":"DM6006"},{"id":5,"type":"DM4310"}],
        "motor_fps_nominal": MOTOR_FPS,
        "realsense": {
            "fps": RS_FPS,
            "size": {"width": RS_SIZE[0], "height": RS_SIZE[1]},
            "depth_scale_m": rs_worker.depth_scale,
            "intrinsics": rs_worker.intrinsics,
            "extrinsics_depth_to_color": rs_worker.extrinsics
        },
        "port": getattr(serial_device, "port", None),
        "maxTorque": maxTorque, "K_p": K_p
    }
    with open(out_dir/"meta.json", "w") as f: json.dump(meta, f, indent=2)
    print(f"[SAVED] {mot_csv}\n[SAVED] {rs_idx_csv}\n[SAVED] {out_dir/'meta.json'}")

    # ---- 以下、's'でもう一度押すとモーター再生（任意） ----
    print("\n[PLAY] 再生開始は 's'+Enter ／ 終了は 'q'+Enter")
    if wait_cmd("s") == "q": return
    data = []
    for r in mot_rows:
        # l = [[m1_pos, m1_vel], ...]
        pairs = [r[1+2*i:1+2*i+2] for i in range(5)]
        data.append(pairs)

    print("[PLAY] start")
    play_rate = Rate(50); play_rate.reset()
    try:
        while True:
            for l in data:
                MITMaxTorque(Motor1, l[0][0], K_p, l[0][1]/2.0, 1.0)
                MITMaxTorque(Motor2, l[1][0], K_p, l[1][1]/2.0, 1.0)
                MITMaxTorque(Motor3, l[2][0], K_p, l[2][1]/2.0, 1.0)
                MITMaxTorque(Motor4, l[3][0], K_p, l[3][1]/2.0, 1.0)
                MITMaxTorque(Motor5, l[4][0], K_p, l[4][1]/2.0, 1.0)
                if not cmd_q.empty() and cmd_q.get_nowait()=="q": raise KeyboardInterrupt
                play_rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
                MotorControl1.controlMIT(m, 0, 0, m.getPosition(), 0, 0)
            serial_device.close()
        except Exception:
            pass
        print("done. bye.")

if __name__ == "__main__":
    main()
