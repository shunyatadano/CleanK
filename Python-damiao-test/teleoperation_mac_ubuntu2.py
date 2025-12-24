#!/usr/bin/env python3
"""
Tele-op bridge for DM/STS motors (macOS & Linux friendly)

- Auto-detects serial ports on macOS (/dev/tty.*) and Linux (/dev/ttyUSB*, /dev/ttyACM*).
- Allows explicit override via environment variables:
  DM_FOLLOWER_DM, DM_FOLLOWER_STS, DM_LEADER_DM, DM_LEADER_STS
- Press Ctrl+C to stop cleanly (no keyboard hooks / Input Monitoring needed).
"""

import os
import sys
import time
import math
import signal
import glob
from typing import Optional, List
import motor_stream

import serial  # pyserial
from DM_CAN import *  # your vendor SDK

# -------------------- Config --------------------
MAX_TORQUE = 10.0
K_P = 20.0
CONTROLLER_ID = 0x0F
STS_ID_1 = 1
STS_ID_2 = 2

# Motor topology (unchanged from original)
Motor1 = Motor(DM_Motor_Type.DM6006, 0x01, 0x15)
Motor2 = Motor(DM_Motor_Type.DM6006, 0x02, 0x15)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x11)
Motor4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x11)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x11)

Motor2_1 = Motor(DM_Motor_Type.DM6006, 0x01, 0x15)
Motor2_2 = Motor(DM_Motor_Type.DM6006, 0x02, 0x15)
Motor2_3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x11)
Motor2_4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x11)
Motor2_5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x11)

FOLLOWER_DM_MOTORS = [Motor1, Motor2, Motor3, Motor4, Motor5]
LEADER_DM_MOTORS   = [Motor2_1, Motor2_2, Motor2_3, Motor2_4, Motor2_5]


# -------------------- Helpers --------------------
def sts_value(sts_map, sid, default=0.0):
    # sid: STS servo ID (likely 1..254)
    if isinstance(sts_map, dict):
        return sts_map.get(sid, default)
    if isinstance(sts_map, list):
        # many STS libs store values at index == ID
        return sts_map[sid] if 0 <= sid < len(sts_map) else default
    return default


def list_serial_candidates() -> List[str]:
    """
    Enumerate likely serial device paths on macOS/Linux.
    (On macOS, devices typically appear under /dev/tty.usbserial-* or /dev/tty.usbmodem*.)
    """
    patterns = [
        # macOS
        "/dev/tty.wchusbserial*",
        "/dev/tty.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.SLAB_USBtoUART*",
        # Linux
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ]
    found = []
    for pat in patterns:
        found.extend(glob.glob(pat))
    return sorted(set(found))


def pick_port(env_key: str, fallback_index: int) -> Optional[str]:
    """
    1) Use explicit env var if provided
    2) Otherwise pick by index from candidate list
    """
    env = os.getenv(env_key)
    if env:
        return env

    candidates = list_serial_candidates()
    if len(candidates) > fallback_index:
        return candidates[fallback_index]
    return None


def open_serial_or_fail(path: str, baud: int = 921600, timeout: float = 0.5) -> serial.Serial:
    try:
        return serial.Serial(path, baud, timeout=timeout)
    except Exception as e:
        print(f"[ERROR] Failed to open serial '{path}': {e}")
        raise


def mit_max_torque(mcc: MotorControl, motor: Motor,
                   target_angle: float, kp: float, target_vel: float, kd: float) -> None:
    """Clamp kp to avoid exceeding MAX_TORQUE, guard diff=0."""
    now_angle = motor.getPosition()
    diff = abs(target_angle - now_angle)

    if diff <= 1e-9:  # avoid division by zero and pointless spam
        mcc.controlMIT(motor, 0.0, kd, target_angle, target_vel, 0)
        return

    power = diff * kp
    use_kp = min(kp, MAX_TORQUE / diff) if power > MAX_TORQUE else kp
    mcc.controlMIT(motor, use_kp, kd, target_angle, target_vel, 0)


# -------------------- Main --------------------
def main():
    # Resolve 4 ports (override with env if needed)
    follower_dm_port  = pick_port("DM_FOLLOWER_DM", 0)
    follower_sts_port = pick_port("DM_FOLLOWER_STS", 1)
    leader_dm_port    = pick_port("DM_LEADER_DM",   2)
    leader_sts_port   = pick_port("DM_LEADER_STS",  3)

    missing = [name for name, val in [
        ("DM_FOLLOWER_DM",  follower_dm_port),
        ("DM_FOLLOWER_STS", follower_sts_port),
        ("DM_LEADER_DM",    leader_dm_port),
        ("DM_LEADER_STS",   leader_sts_port),
    ] if not val]

    if missing:
        print("[ERROR] Could not auto-detect all required serial ports.")
        print("        Set them via environment variables, for example:")
        print("        export DM_FOLLOWER_DM=/dev/tty.wchusbserialXXXX")
        print("        export DM_FOLLOWER_STS=/dev/tty.usbserialYYYY")
        print("        export DM_LEADER_DM=/dev/tty.usbmodemZZZZ")
        print("        export DM_LEADER_STS=/dev/tty.usbserialWWWW")
        print("\nDetected candidates:", list_serial_candidates())
        sys.exit(1)

    print("Using serial ports:")
    print("  follower DM : ", follower_dm_port)
    print("  follower STS: ", follower_sts_port)
    print("  leader   DM : ", leader_dm_port)
    print("  leader   STS: ", leader_sts_port)

    # Open serials
    ser_f_dm  = open_serial_or_fail(follower_dm_port)
    ser_f_sts = open_serial_or_fail(follower_sts_port)
    ser_l_dm  = open_serial_or_fail(leader_dm_port)
    ser_l_sts = open_serial_or_fail(leader_sts_port)

    # Create controllers
    MCC_f_dm  = MotorControl(ser_f_dm)   # follower DM motors
    MCC_f_sts = MotorControl(ser_f_sts)  # follower STS motors
    MCC_l_dm  = MotorControl(ser_l_dm)   # leader DM motors
    MCC_l_sts = MotorControl(ser_l_sts)  # leader STS motors

    # Register motors
    for m in FOLLOWER_DM_MOTORS:
        MCC_f_dm.addMotor(m)
    for m in LEADER_DM_MOTORS:
        MCC_l_dm.addMotor(m)

    # Switch to MIT for both arms (fix i+1 bug)
    for i in range(5):
        if MCC_f_dm.switchControlMode(FOLLOWER_DM_MOTORS[i], Control_Type.MIT):
            print(f"Follower DM motor {i+1}: MIT mode OK")
    for i in range(5):
        if MCC_l_dm.switchControlMode(LEADER_DM_MOTORS[i], Control_Type.MIT):
            print(f"Leader   DM motor {i+1}: MIT mode OK")

    # Save/enable/zero (fix wrong list usage)
    for i in range(5):
        MCC_f_dm.save_motor_param(FOLLOWER_DM_MOTORS[i])
        MCC_f_dm.enable(FOLLOWER_DM_MOTORS[i])
        MCC_f_dm.set_zero_position(FOLLOWER_DM_MOTORS[i])
    for i in range(5):
        MCC_l_dm.save_motor_param(LEADER_DM_MOTORS[i])
        MCC_l_dm.enable(LEADER_DM_MOTORS[i])
        MCC_l_dm.set_zero_position(LEADER_DM_MOTORS[i])

    time.sleep(1.5)
    print("Motor setup done")

    # Zero command to follower side initially
    for i in range(5):
        MCC_f_dm.controlMIT(FOLLOWER_DM_MOTORS[i], 0, 0, 0, 0, 0)
    print("tele-operation start (Ctrl+C to stop)")

    # Graceful stop via Ctrl+C
    running = True
    def _sigint(_sig, _frm):
        nonlocal running
        print("\n[INFO] Stopping ...")
        running = False
    signal.signal(signal.SIGINT, _sigint)

    # Main loop
    try:
        while running:
            l = []

            # Leader STS (id=1) read first (avoid overlapping commands)
            sts1_val = motor_stream.read_angle(1)

            # Leader DM request current state (send)
            for i in range(5):
                MCC_l_dm.controlMIT(LEADER_DM_MOTORS[i], 0, 0, 0, 0, 0)

            time.sleep(0.005)

            # Leader DM receive (now values should be available)
            MCC_l_dm.recv()
            for i in range(5):
                l.append(LEADER_DM_MOTORS[i].getPosition())  # 5 entries

            # Leader STS (id=2) read request
            sts2_val = motor_stream.read_angle(2)

            # Append STS id=1 pos (MCC_l_sts.sts_map is assumed to be filled after recv)
            # after requesting STS_ID_1, receive and then read it
            #MCC_l_sts.STSControl_read(CONTROLLER_ID, STS_ID_1)
            #MCC_l_sts.recv()
            l.append(sts1_val)  # index 5
            #MCC_f_sts.STSControl_write(CONTROLLER_ID, STS_ID_1, l[5])
            # Mirror STS id=1 to follower
            MCC_f_sts.STSControl_write(CONTROLLER_ID, STS_ID_1, sts1_val)

            # Follow leader DM -> follower DM
            for i in range(5):
                mit_max_torque(MCC_f_dm, FOLLOWER_DM_MOTORS[i], l[i], K_P, 0.0, 1.0)

            # Receive STS (id=2) and mirror to follower
            # request the second STS, receive and mirror it too
            l.append(sts2_val)  # index 6
            MCC_f_sts.STSControl_write(CONTROLLER_ID, STS_ID_2, sts2_val)

            print([float(q) for q in l])

            time.sleep(0.005)

    finally:
        # Try to stop motors safely
        try:
            for i in range(5):
                MCC_f_dm.controlMIT(FOLLOWER_DM_MOTORS[i], 0, 0, 0, 0, 0)
        except Exception:
            pass

        for s in (ser_f_dm, ser_f_sts, ser_l_dm, ser_l_sts):
            try:
                s.close()
            except Exception:
                pass

        print("[INFO] Done.")


if __name__ == "__main__":
    main()
