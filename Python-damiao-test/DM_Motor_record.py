from pathlib import Path
import csv, json
from datetime import datetime

import os
import sys
import time
import serial
import threading

from DM_CAN import *
from pynput import keyboard  # Ubuntu でも sudo 不要で扱いやすいグローバルキーフック

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
    # Linux の一般的な候補
    for cand in ("/dev/ttyACM0", "/dev/ttyUSB0"):
        if os.path.exists(cand):
            return cand
    return "/dev/cu.usbmodem00000000050C1"

# ================= キー入力（pynput） =================
s_pressed_event = threading.Event()   # s が押された
s_released_event = threading.Event()  # s が離された（押下後のリリース検出用）

def on_press(key):
    try:
        ch = getattr(key, "char", None)
        if ch and ch.lower() == "s":
            s_pressed_event.set()
            s_released_event.clear()
    except Exception:
        pass

def on_release(key):
    try:
        ch = getattr(key, "char", None)
        if ch and ch.lower() == "s":
            s_released_event.set()
    except Exception:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()  # 非ブロッキングで開始（メインループは継続）

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

# ここは元コード通り：各軸に一度 MIT(3,0,0,0,0) を投げる
MotorControl1.controlMIT(Motor1, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor2, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor3, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor4, 3, 0, 0, 0, 0)
MotorControl1.controlMIT(Motor5, 3, 0, 0, 0, 0)

print("recording start (press 's' to stop)")

data = []

record_dir = Path("recordings")
record_dir.mkdir(exist_ok=True)
log_rows = []                 # CSV用のフラット化データ
t0 = time.perf_counter()      # 経過時間[s]を出す基準


try:
    # -------- 記録：s が押されるまで --------
    s_pressed_event.clear()
    while not s_pressed_event.is_set():
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

        # 追加 ↓（CSV 1行ぶん: [t, m1_pos, m1_vel, ..., m5_pos, m5_vel]）
        t = time.perf_counter() - t0
        row = [t] + [x for pair in frame for x in pair]
        log_rows.append(row)

        print(frame[3])  # 元コードに合わせ Motor4 の pos/vel を表示
        time.sleep(0.01)

    print("recording stopped")



    # 追加 ↓ 保存（CSV と メタ）
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

    # -------- 元コード相当：s を離し、再度 s が押されるのを待つ --------
    # s リリース待ち
    while not s_released_event.is_set():
        time.sleep(0.005)
    # 次の s 押下待ち
    s_pressed_event.clear()
    while not s_pressed_event.is_set():
        time.sleep(0.005)

    # -------- 無限ループで再生＋減衰＋原点戻し＋待ち --------
    while True:
        print("moving start")
        for l in data:
            MITMaxTorque(Motor1, l[0][0], K_p,       l[0][1] / 2.0, 1.0)
            MITMaxTorque(Motor2, l[1][0], K_p,       l[1][1] / 2.0, 1.0)
            MITMaxTorque(Motor3, l[2][0], K_p,       l[2][1] / 2.0, 1.0)
            MITMaxTorque(Motor4, l[3][0], K_p,       l[3][1] / 2.0, 1.0)
            MITMaxTorque(Motor5, l[4][0], K_p,       l[4][1] / 2.0, 1.0)
            time.sleep(0.02)

        time.sleep(1.0)

        # 減衰フェーズ
        last = data[-1]
        for i in range(90):
            scale = (0.9 ** i)
            print(K_p * scale)
            MITMaxTorque(Motor1, last[0][0], K_p * scale, last[0][1], scale)
            MITMaxTorque(Motor2, last[1][0], K_p * scale, last[1][1], scale)
            MITMaxTorque(Motor3, last[2][0], K_p * scale, last[2][1], scale)
            MITMaxTorque(Motor4, last[3][0], K_p * scale, last[3][1], scale)
            MITMaxTorque(Motor5, last[4][0], K_p * scale, last[4][1], scale)
            time.sleep(0.05)

        # 最後に原点へ軽く寄せる（KP低め）
        MITMaxTorque(Motor1, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor2, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor3, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor4, 0.0, 0.5, 0.0, scale)
        MITMaxTorque(Motor5, 0.0, 0.5, 0.0, scale)

        time.sleep(10.0)

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
    try:
        listener.stop()
    except Exception:
        pass


