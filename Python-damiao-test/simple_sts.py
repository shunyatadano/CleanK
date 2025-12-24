"""Simple STS3215 reader/writer using only pyserial.

- Uses a minimal implementation of Feetech/Dynamixel-like protocol v0 (0xFF 0xFF header).
- Can read `Present_Position` (addr 56, len 2) and write `Goal_Position` (addr 42, len 2).

WARNING: Hardware commands can move motors. Use with care and at a safe torque/pose.
"""

from __future__ import annotations
import argparse
import serial
import struct
import time
import sys

# Protocol constants (Protocol 1-like, commonly used by Feetech/Dynamixel-compatible devices)
HDR = b"\xff\xff"
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# Common STS register addresses (from feetech tables in this repo)
ADDR_GOAL_POSITION = 42  # 2 bytes
ADDR_PRESENT_POSITION = 56  # 2 bytes
ADDR_OPERATING_MODE = 33
ADDR_HOMING_OFFSET = 31
ADDR_TORQUE_ENABLE = 40  # 1 byte (Torque_Enable)

DEFAULT_BAUD = 1000000
DEFAULT_TIMEOUT = 0.5


def _checksum(packet: bytes) -> int:
    # checksum = ~(ID + LENGTH + INSTRUCTION + SUM(PARAMS)) & 0xFF
    s = sum(packet) & 0xFF
    return (~s) & 0xFF


def build_packet(motor_id: int, instruction: int, params: bytes = b"") -> bytes:
    # LENGTH = len(params) + 2 (INSTRUCTION + CHECKSUM)
    length = len(params) + 2
    header = bytearray(HDR)
    header.append(motor_id & 0xFF)
    header.append(length & 0xFF)
    header.append(instruction & 0xFF)
    header.extend(params)
    chk = _checksum(header[2:])  # ID..end
    header.append(chk)
    return bytes(header)


def parse_status(ser: serial.Serial, timeout: float = DEFAULT_TIMEOUT) -> tuple[int, bytes]:
    # Read until we see 0xFF 0xFF header
    start = time.time()
    # find header
    while True:
        b = ser.read(1)
        if not b:
            if time.time() - start > timeout:
                raise TimeoutError("Timeout waiting for header")
            continue
        if b == b"\xff":
            b2 = ser.read(1)
            if b2 == b"\xff":
                break
            # otherwise continue scanning
    # read id, len, error
    hdr = ser.read(3)
    if len(hdr) < 3:
        raise IOError("Incomplete status header")
    motor_id = hdr[0]
    length = hdr[1]
    error = hdr[2]
    params_len = length - 2
    params = ser.read(params_len) if params_len > 0 else b""
    chk = ser.read(1)
    if len(chk) < 1:
        raise IOError("Missing checksum")
    # validate checksum
    packet = bytes([motor_id, length, error]) + params
    expected = _checksum(packet)
    if chk[0] != expected:
        raise IOError(f"Bad checksum: got {chk[0]:02x}, expected {expected:02x}")
    if error != 0:
        raise IOError(f"Device returned error code {error}")
    return motor_id, params


def int_to_2le(value: int) -> bytes:
    return struct.pack('<H', value & 0xFFFF)


def int_from_2le(b: bytes) -> int:
    return struct.unpack('<H', b)[0]


def encode_sign_magnitude(val: int, sign_bit: int = 15) -> int:
    # sign_bit is the index (0-based) of the sign bit. For 16-bit, sign_bit=15.
    mask = (1 << sign_bit) - 1
    if val < 0:
        return (1 << sign_bit) | (abs(val) & mask)
    else:
        return val & ((1 << (sign_bit + 1)) - 1)


def decode_sign_magnitude(enc: int, sign_bit: int = 15) -> int:
    sign_mask = 1 << sign_bit
    mag_mask = sign_mask - 1
    if enc & sign_mask:
        return - (enc & mag_mask)
    return enc & mag_mask


class SimpleSTS:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = DEFAULT_TIMEOUT, verbose: bool = False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.verbose = verbose
        self.ser: serial.Serial | None = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        # small sleep to let device settle
        time.sleep(0.05)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    def _tx(self, packet: bytes) -> None:
        if not self.ser:
            raise IOError("Serial port not open")
        if self.verbose:
            print("TX:", packet.hex())
        self.ser.write(packet)

    def _rx_params(self) -> bytes:
        if not self.ser:
            raise IOError("Serial port not open")
        # parse_status already returns params, but here we capture raw bytes optionally
        motor_id, params = parse_status(self.ser, timeout=self.timeout)
        if self.verbose:
            print("RX (params):", params.hex())
        return params

    def ping(self, motor_id: int) -> bool:
        packet = build_packet(motor_id, INST_PING)
        self._tx(packet)
        try:
            _id, _params = parse_status(self.ser, timeout=self.timeout)
            return True
        except Exception:
            return False

    def read_reg(self, motor_id: int, addr: int, length: int) -> bytes:
        params = bytes([addr & 0xFF, length & 0xFF])
        packet = build_packet(motor_id, INST_READ, params)
        self._tx(packet)
        return self._rx_params()

    def write_reg(self, motor_id: int, addr: int, data: bytes) -> None:
        params = bytes([addr & 0xFF]) + data
        packet = build_packet(motor_id, INST_WRITE, params)
        self._tx(packet)
        # read ack
        _ = self._rx_params()

    def read_present_position(self, motor_id: int) -> int:
        # params: start_addr (1 byte), length (1 byte)
        params = self.read_reg(motor_id, ADDR_PRESENT_POSITION, 2)
        if len(params) != 2:
            raise IOError("Unexpected response length for Present_Position")
        raw = int_from_2le(params)
        # decode sign magnitude if necessary (sts uses sign-magnitude encoding for positions)
        return decode_sign_magnitude(raw, 15)

    def write_goal_position(self, motor_id: int, position: int, verify: bool = True) -> None:
        enc = encode_sign_magnitude(position, 15)
        data = int_to_2le(enc)
        self.write_reg(motor_id, ADDR_GOAL_POSITION, data)
        # optionally verify by reading back present position (may take a moment to move)
        if verify:
            # small delay to let motor react
            time.sleep(0.05)
            try:
                pos = self.read_present_position(motor_id)
                if self.verbose:
                    print(f"After write, Present_Position (decoded) = {pos}")
            except Exception as e:
                if self.verbose:
                    print("Verification read failed:", e)

    def enable_torque(self, motor_id: int) -> None:
        # Torque_Enable is 1 byte
        self.write_reg(motor_id, ADDR_TORQUE_ENABLE, bytes([1]))

    def disable_torque(self, motor_id: int) -> None:
        self.write_reg(motor_id, ADDR_TORQUE_ENABLE, bytes([0]))


def main():
    p = argparse.ArgumentParser(description="Simple STS reader/writer (no SDK dependency)")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--id", type=int, default=1)
    sp = p.add_subparsers(dest="cmd")

    r = sp.add_parser("read")
    r.set_defaults(cmd="read")

    w = sp.add_parser("write")
    w.add_argument("position", type=int)
    w.set_defaults(cmd="write")

    ping = sp.add_parser("ping")
    ping.set_defaults(cmd="ping")

    rr = sp.add_parser("read-reg", help="Read raw register: addr length")
    rr.add_argument("addr", help="Register address (decimal or 0xhex)")
    rr.add_argument("length", type=int, help="Number of bytes to read")
    rr.set_defaults(cmd="read-reg")

    wr = sp.add_parser("write-reg", help="Write raw register: addr data")
    wr.add_argument("addr", help="Register address (decimal or 0xhex)")
    wr.add_argument("data", help="Data as decimal, 0xhex, or comma-separated bytes (e.g. 0x10 or 16 or 1,2)")
    wr.set_defaults(cmd="write-reg")

    et = sp.add_parser("enable-torque", help="Enable torque on motor")
    et.set_defaults(cmd="enable-torque")

    dt = sp.add_parser("disable-torque", help="Disable torque on motor")
    dt.set_defaults(cmd="disable-torque")

    p.add_argument("--verbose", action="store_true", help="Enable raw TX/RX logging")
    args = p.parse_args()
    if not args.cmd:
        p.print_help()
        sys.exit(1)

    client = SimpleSTS(args.port, baud=args.baud, verbose=bool(getattr(args, 'verbose', False)))
    try:
        client.open()
        if args.cmd == "ping":
            ok = client.ping(args.id)
            print("PING OK" if ok else "PING FAILED")
        elif args.cmd == "read":
            pos = client.read_present_position(args.id)
            print(f"Present_Position (decoded) = {pos}")
        elif args.cmd == "write":
            client.write_goal_position(args.id, args.position)
            print(f"Wrote Goal_Position = {args.position}")
        elif args.cmd == "read-reg":
            addr = int(str(args.addr), 0)
            data = client.read_reg(args.id, addr, args.length)
            print(f"REG 0x{addr:02x}: {data.hex()} ({len(data)} bytes)")
        elif args.cmd == "write-reg":
            addr = int(str(args.addr), 0)
            # parse data: support 0xhex, decimal, or comma-separated bytes
            s = str(args.data)
            if "," in s:
                parts = [int(x.strip(), 0) & 0xFF for x in s.split(",")]
                data_bytes = bytes(parts)
            else:
                val = int(s, 0)
                # pick smallest byte-length (1 or 2)
                if val <= 0xFF:
                    data_bytes = bytes([val & 0xFF])
                else:
                    data_bytes = int_to_2le(val)
            client.write_reg(args.id, addr, data_bytes)
            print(f"Wrote REG 0x{addr:02x}: {data_bytes.hex()}")
        elif args.cmd == "enable-torque":
            client.enable_torque(args.id)
            cur = client.read_reg(args.id, ADDR_TORQUE_ENABLE, 1)
            print(f"Torque enabled, reg=0x{cur[0]:02x}")
        elif args.cmd == "disable-torque":
            client.disable_torque(args.id)
            cur = client.read_reg(args.id, ADDR_TORQUE_ENABLE, 1)
            print(f"Torque disabled, reg=0x{cur[0]:02x}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
