#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import csv, json
from datetime import datetime

import os
import sys
import time
import glob
import serial
import threading
from queue import SimpleQueue

from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type
from utils import Rate

# ================= パラメータ =================
maxTorque = 10.0
K_p = 20.0

# ================= ユーティリティ =================
def default_serial_port():
    """環境変数 DM_PORT 優先。未指定なら OS に応じて推定。"""
    env = os.getenv("DM_PORT")
    if env:
        return env

    if sys.platform.startswith("win"):
        return "COM7"

    if sys.platform == "darwin":  # macOS
        patterns = [
            "/dev/cu.usbmodem*",
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
        ]
        for pat in patterns:
            cand = sorted(glob.glob(pat))
            if cand:
                return cand[0]
        # 最後の保険（ユーザ実機に合わせたい場合は環境変数 DM_PORT を使う）
        return "/dev/cu.usbmodem00000000050C1"

    # Linux 一般
    for cand in ("/dev/ttyACM0", "/dev/ttyUSB0"):
        if os.path.exists(cand):
            return cand
    return "/dev/ttyACM0"

# ================= 入力（stdin） =================
# - 別スレッドで sys.stdin を1行ずつ読んで合図をキューに投げる
# - 's' でトグル、'q' で終了
#   （sys.stdin について: 標準入力の読み取りは Text IO で行/反復可能。:contentReference[oaicite:1]{index=1}）
cmd_queue: SimpleQueue[str] = SimpleQueue()
shutdown_evt = threading.Event()

def stdin_watcher():
    # TTYでない場合でも、パイプ/リダイレクトされた入力をそのまま読みます
    for line in sys.stdin:
        txt = line.strip().lower()
        if not txt:
            continue
        if "q" in txt:
            cmd_queue.put("q")
            break
        if "s" in txt:
            cmd_queue.put("s")
    # ここに到達＝EOF
    shutdown_evt.set()

def wait_for_cmd(target: str):
    """cmd_queue から target ('s' or 'q') を待つ（到達までブロック）"""
    while True:
        # shutdown 要求が来たら即終了
        if shutdown_evt.is_set():
            return "q"
        try:
            cmd = cmd_queue.get(timeout=0.05)
            if cmd == target:
                return cmd
            if cmd == "q":
                return "q"
        except Exception:
            pass

# ================= 制御関数 =================
def MITMaxTorque(Motor, target_angle: float, kp: float, target_vel: float, kd: float):
    """maxTorque を超えぬよう KP を自動調整して MIT 制御を送る"""
    now_angle = Motor.getPosition()
    diff = abs(target_angle - now_angle)
    if diff < 1e-9:  # ゼロ割防止
        MotorControl1.controlMIT(Motor, kp, kd, target_angle, target_vel, 0)
        return
    power = diff * kp
    if power > maxTorque:
        MotorControl1.controlMIT(Motor, maxTorque / diff, kd, target_angle, target_vel, 0)
    else:
        MotorControl1.controlMIT(Motor, kp, kd, target_angle, target_vel, 0)

# ================= モータ初期化 =================
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM6006, 0x02, 0x15)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x11)
Motor4 = Motor(DM_Motor_Type.DM6006, 0x04, 0x15)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x11)

serial_device = serial.Serial(default_serial_port(), 921600, timeout=0.5)
print("Serial port is open:", getattr(serial_device, "port", "?"))

MotorControl1 = MotorControl(serial_device)
for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
    MotorControl1.addMotor(m)

if MotorControl1.switchControlMode(Motor1, Control_Type.MIT):
    print("motor1 switch MIT success")
if MotorControl1.switchControlMode(Motor2, Control_Type.MIT):
    print("motor2 switch MIT success")
if MotorControl1.switchControlMode(Motor3, Control_Type.MIT):
    print("motor3 switch MIT success")
if MotorControl1.switchControlMode(Motor4, Control_Type.MIT):
    print("motor4 switch MIT success")
if MotorControl1.switchControlMode(Motor5, Control_Type.MIT):
    print("motor5 switch MIT success")

for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
    MotorControl1.save_motor_param(m)
    MotorControl1.enable(m)

time.sleep(1.5)
print("motor setup done")

MotorControl1.set_zero_position(Motor1)
MotorControl1.set_zero_position(Motor2)
MotorControl1.set_zero_position(Motor3)
MotorControl1.set_zero_position(Motor4)
MotorControl1.set_zero_position(Motor5)

# 一度だけ MIT(3,0,0,0,0) を投げる
MotorControl1.controlMIT(Motor1, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor2, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor3, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor4, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor5, 3, 0, 0, 0, 0)

# ================= 記録＆再生 =================
record_dir = Path("recordings")
record_dir.mkdir(exist_ok=True)

data = []        # 再生用（ネスト構造）
log_rows = []    # CSV用（フラット）

# stdin 監視スレッド起動（Event/Queue を使うのはスレッド連携の基本。:contentReference[oaicite:2]{index=2}）
th = threading.Thread(target=stdin_watcher, daemon=True)
th.start()

print("\nrecording start  ——  停止するには  s + Enter 。（終了は q + Enter）")

scale = 1.0  # 減衰後の“原点寄せ”で使う既定値

try:
    # -------- 記録：'s' が来るまで --------
    record_rate = Rate(frequency_hz=100.0)
    t0 = record_rate.reset()
    while True:
        # 目標0でMIT制御しつつ、各モータの pos/vel を取得
        MotorControl1.controlMIT(Motor1, 0, 0, 0, 0, 0)
        MotorControl1.controlMIT(Motor2, 0, 0, 0, 0, 0)
        MotorControl1.controlMIT(Motor3, 0, 0, 0, 0, 0)
        MotorControl1.controlMIT(Motor4, 0, 0, 0, 0, 0)
        MotorControl1.controlMIT(Motor5, 0, 0, 0, 0, 0)

        frame = [
            [Motor1.getPosition(), Motor1.getVelocity()],
            [Motor2.getPosition(), Motor2.getVelocity()],
            [Motor3.getPosition(), Motor3.getVelocity()],
            [Motor4.getPosition(), Motor4.getVelocity()],
            [Motor5.getPosition(), Motor5.getVelocity()],
        ]
        data.append(frame)

        # CSV 1行ぶん: [t, m1_pos, m1_vel, ..., m5_pos, m5_vel]
        t = record_rate._next_deadline - t0
        row = [t] + [x for pair in frame for x in pair]
        log_rows.append(row)

        overrun = record_rate.sleep()
        if overrun > 0.002:
            print(f"[warn] record loop overrun {overrun * 1e3:.1f} ms", file=sys.stderr)

        # 入力チェック
        if not cmd_queue.empty():
            cmd = cmd_queue.get_nowait()
            if cmd == "s":
                break
            if cmd == "q":
                raise KeyboardInterrupt

    print("recording stopped")

    # -------- 保存（CSV と メタ）--------
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = record_dir / f"cleank1_chunk_{ts}.csv"
    hdr = ["t_s"] + [f"m{i}_{k}" for i in range(1, 6) for k in ("pos", "vel")]
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(log_rows)

    meta = {
        "motors": [
            {"id": 1, "type": "DM4310"}, {"id": 2, "type": "DM6006"},
            {"id": 3, "type": "DM4310"}, {"id": 4, "type": "DM6006"},
            {"id": 5, "type": "DM4310"},
        ],
        "fps_nominal": 100,
        "port": getattr(serial_device, "port", None),
        "maxTorque": maxTorque,
        "K_p": K_p,
    }
    with open(record_dir / f"so101_trace_{ts}.meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(f"saved: {csv_path}")

    print("\n再生を始めるには  s + Enter。終了は q + Enter。")

    # -------- 's' を待って再生開始 --------
    cmd = wait_for_cmd("s")
    if cmd == "q":
        raise KeyboardInterrupt

    # -------- 無限ループで再生＋減衰＋原点戻し＋待ち --------
    while True:
        print("moving start")
        playback_rate = Rate(frequency_hz=50)
        playback_rate.reset()

        # 記録シーケンスの再生（速度は半分）
        for l in data:
            MITMaxTorque(Motor1, l[0][0], K_p,       l[0][1] / 2.0, 1.0)
            MITMaxTorque(Motor2, l[1][0], K_p,       l[1][1] / 2.0, 1.0)
            MITMaxTorque(Motor3, l[2][0], K_p,       l[2][1] / 2.0, 1.0)
            MITMaxTorque(Motor4, l[3][0], K_p,       l[3][1] / 2.0, 1.0)
            MITMaxTorque(Motor5, l[4][0], K_p,       l[4][1] / 2.0, 1.0)
            overrun = playback_rate.sleep()
            if overrun > 0.004:
                print(f"[warn] playback loop overrun {overrun * 1e3:.1f} ms", file=sys.stderr)
            if not cmd_queue.empty() and cmd_queue.get_nowait() == "q":
                raise KeyboardInterrupt

        time.sleep(1.0)

        # 減衰フェーズ（KP, KD を指数で落とす）
        last = data[-1]
        damp_rate = Rate(frequency_hz=10)
        damp_rate.reset()
        for i in range(90):
            scale = (0.9 ** i)
            MITMaxTorque(Motor1, last[0][0], K_p * scale, last[0][1], scale)
            MITMaxTorque(Motor2, last[1][0], K_p * scale, last[1][1], scale)
            MITMaxTorque(Motor3, last[2][0], K_p * scale, last[2][1], scale)
            MITMaxTorque(Motor4, last[3][0], K_p * scale, last[3][1], scale)
            MITMaxTorque(Motor5, last[4][0], K_p * scale, last[4][1], scale)
            overrun = damp_rate.sleep()
            if overrun > 0.004:
                print(f"[warn] damping loop overrun {overrun * 1e3:.1f} ms", file=sys.stderr)
            if not cmd_queue.empty() and cmd_queue.get_nowait() == "q":
                raise KeyboardInterrupt

        # 最後に原点へ軽く寄せる（KP低め）
        MITMaxTorque(Motor1, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor2, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor3, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor4, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor5, 0.0, 0.5, 0.0, scale)

        # 10秒待機中に 'q' 監視
        t_wait = time.time() + 10.0
        while time.time() < t_wait:
            if not cmd_queue.empty() and cmd_queue.get_nowait() == "q":
                raise KeyboardInterrupt
            time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[KeyboardInterrupt] stopping...")

finally:
    # 安全停止 & クリーンアップ
    try:
        for m in (Motor1, Motor2, Motor3, Motor4, Motor5):
            try:
                MotorControl1.controlMIT(m, 0, 0, m.getPosition(), 0, 0)
            except Exception:
                pass
        serial_device.close()
    except Exception:
        pass
    shutdown_evt.set()
    print("done. bye.")
