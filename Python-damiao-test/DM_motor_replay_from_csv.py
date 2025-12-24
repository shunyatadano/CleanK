#!/usr/bin/env python3
"""Replay a recorded DM motor trajectory stored as CSV."""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import serial

from DM_CAN import Control_Type, DM_Motor_Type, Motor, MotorControl

# Default gains/limits are aligned with DM_motor_record_mac.py
DEFAULT_KP = 20.0
DEFAULT_KD = 1.0
DEFAULT_MAX_TORQUE = 10.0


def default_serial_port() -> str:
    """Return the best-effort serial device path unless DM_PORT overrides."""
    env = os.getenv("DM_PORT")
    if env:
        return env

    if sys.platform.startswith("win"):
        return "COM7"

    if sys.platform == "darwin":
        patterns = [
            "/dev/cu.usbmodem*",
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
        ]
        for pat in patterns:
            matches = sorted(glob.glob(pat))
            if matches:
                return matches[0]
        return "/dev/cu.usbmodem00000000050C1"

    for cand in ("/dev/ttyACM0", "/dev/ttyUSB0"):
        if os.path.exists(cand):
            return cand
    return "/dev/ttyACM0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a multi-motor trajectory recorded with DM_motor_record_mac.py"
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to the recorded CSV file (e.g. recordings/cleank1_chunk_xxx.csv)",
    )
    parser.add_argument(
        "--port",
        default=default_serial_port(),
        help="Serial port for the CAN adapter (default: auto-detect / $DM_PORT)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = realtime, 0.5 = half speed)",
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.5,
        help="Scale applied to recorded velocities during playback.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of loops (0 = infinite until Ctrl+C).",
    )
    parser.add_argument(
        "--loop-delay",
        type=float,
        default=5.0,
        help="Seconds to wait between loops when --repeat != 1.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=DEFAULT_KP,
        help="Nominal KP gain for MIT control.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=DEFAULT_KD,
        help="Nominal KD gain for MIT control.",
    )
    parser.add_argument(
        "--max-torque",
        type=float,
        default=DEFAULT_MAX_TORQUE,
        help="Torque ceiling used to auto-scale KP when the error is large.",
    )
    return parser.parse_args()


@dataclass
class Frame:
    motors: List[Tuple[float, float]]
    dt: float = 0.0


def load_trace(csv_path: Path) -> List[Frame]:
    """Return Frame objects parsed from the CSV."""
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    frames: List[Frame] = []
    prev_row_time = None
    prev_frame = None

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = ["t_s"] + [
            f"m{i}_{field}" for i in range(1, 6) for field in ("pos", "vel")
        ]
        for field in required:
            if field not in reader.fieldnames:
                raise ValueError(f"Missing column '{field}' in {csv_path}")

        for row in reader:
            t_s = float(row["t_s"])
            motors = []
            for i in range(1, 6):
                pos = float(row[f"m{i}_pos"])
                vel = float(row[f"m{i}_vel"])
                motors.append((pos, vel))

            frame = Frame(motors=motors, dt=0.0)
            frames.append(frame)

            if prev_row_time is not None and prev_frame is not None:
                prev_frame.dt = max(t_s - prev_row_time, 0.0)

            prev_row_time = t_s
            prev_frame = frame

    if not frames:
        raise ValueError(f"No samples found in {csv_path}")

    return frames


def mit_with_limit(
    ctl: MotorControl,
    motor: Motor,
    target_angle: float,
    target_velocity: float,
    kp: float,
    kd: float,
    max_torque: float,
) -> None:
    """Send an MIT command while respecting the torque ceiling."""
    now_angle = motor.getPosition()
    diff = abs(target_angle - now_angle)
    eff_kp = kp
    if diff > 1e-9:
        power = diff * kp
        if power > max_torque:
            eff_kp = max_torque / diff
    ctl.controlMIT(motor, eff_kp, kd, target_angle, target_velocity, 0)


def setup_motors(port: str) -> Tuple[serial.Serial, MotorControl, List[Motor]]:
    serial_device = serial.Serial(port, 921600, timeout=0.5)
    print("Serial port is open:", getattr(serial_device, "port", "?"))

    motors = [
        Motor(DM_Motor_Type.DM4310, 0x01, 0x11),
        Motor(DM_Motor_Type.DM6006, 0x02, 0x15),
        Motor(DM_Motor_Type.DM4310, 0x03, 0x11),
        Motor(DM_Motor_Type.DM6006, 0x04, 0x15),
        Motor(DM_Motor_Type.DM4310, 0x05, 0x11),
    ]

    ctl = MotorControl(serial_device)
    for m in motors:
        ctl.addMotor(m)
        if ctl.switchControlMode(m, Control_Type.MIT):
            print(f"{m} switched to MIT mode")
        ctl.save_motor_param(m)
        ctl.enable(m)
        ctl.set_zero_position(m)
        ctl.controlMIT(m, 3, 0, 0, 0, 0)

    time.sleep(1.0)
    print("Motor setup done")
    return serial_device, ctl, motors


def playback_trace(
    frames: Sequence[Frame],
    ctl: MotorControl,
    motors: Sequence[Motor],
    *,
    kp: float,
    kd: float,
    max_torque: float,
    velocity_scale: float,
    speed: float,
) -> None:
    for frame in frames:
        for motor, (pos, vel) in zip(motors, frame.motors):
            mit_with_limit(
                ctl,
                motor,
                pos,
                vel * velocity_scale,
                kp,
                kd,
                max_torque,
            )
        if frame.dt > 0:
            time.sleep(frame.dt / speed)


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise ValueError("--speed must be > 0")
    if args.velocity_scale <= 0:
        raise ValueError("--velocity-scale must be > 0")

    frames = load_trace(args.csv_path)
    print(f"Loaded {len(frames)} samples from {args.csv_path}")

    serial_device = None
    ctl = None
    motors: List[Motor] = []

    try:
        serial_device, ctl, motors = setup_motors(args.port)
        loop = 0
        while True:
            loop += 1
            print(f"Playback loop {loop}")
            playback_trace(
                frames,
                ctl,
                motors,
                kp=args.kp,
                kd=args.kd,
                max_torque=args.max_torque,
                velocity_scale=args.velocity_scale,
                speed=args.speed,
            )
            if args.repeat and loop >= args.repeat:
                break
            if args.loop_delay > 0:
                time.sleep(args.loop_delay)
    except KeyboardInterrupt:
        print("\nInterrupted by user, stopping playback.")
    finally:
        if ctl and motors:
            for m in motors:
                try:
                    ctl.controlMIT(m, 0, 0, m.getPosition(), 0, 0)
                except Exception:
                    pass
        if serial_device:
            try:
                serial_device.close()
            except Exception:
                pass
        print("Done. Bye.")


if __name__ == "__main__":
    main()
