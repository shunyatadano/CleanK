"""Motor torque control utility using SimpleSTS

Defaults to COM3 and id=1.

Usage:
    python motor_torque.py enable         # enable torque
    python motor_torque.py disable        # disable torque
    python motor_torque.py status         # read torque register
    python motor_torque.py --port COM4 --id 2 enable
"""
from __future__ import annotations
import argparse
import sys
from simple_sts import SimpleSTS, ADDR_TORQUE_ENABLE


def main():
    p = argparse.ArgumentParser(description="Enable/disable/read torque on an STS motor")
    p.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    p.add_argument("--id", type=int, default=1, help="Motor ID (default: 1)")
    p.add_argument("--verbose", action="store_true", help="Enable verbose TX/RX logging from SimpleSTS")
    sp = p.add_subparsers(dest="cmd")  # make subcommand optional; default to disable when run without args

    sp.add_parser("enable")
    sp.add_parser("disable")
    sp.add_parser("status")

    args = p.parse_args()

    # If the script is invoked with no arguments, perform a fixed disable on COM3 id=1
    if len(sys.argv) == 1:
        # enforce fixed defaults regardless of any changes to ArgumentParser defaults
        args.port = "COM3"
        args.id = 1
        args.cmd = "disable"
        args.verbose = False
        print("# No arguments given — defaulting to: disable torque on COM3 id=1")

    client = SimpleSTS(args.port, verbose=args.verbose)
    try:
        try:
            client.open()
        except Exception as e:
            print(f"Failed to open port {args.port}: {e}", file=sys.stderr)
            sys.exit(2)

        if args.cmd == "enable":
            client.enable_torque(args.id)
            cur = client.read_reg(args.id, ADDR_TORQUE_ENABLE, 1)
            print(f"Torque enabled, reg=0x{cur[0]:02x}")
        elif args.cmd == "disable":
            client.disable_torque(args.id)
            cur = client.read_reg(args.id, ADDR_TORQUE_ENABLE, 1)
            print(f"Torque disabled, reg=0x{cur[0]:02x}")
        elif args.cmd == "status":
            cur = client.read_reg(args.id, ADDR_TORQUE_ENABLE, 1)
            val = cur[0]
            print(f"Torque register = 0x{val:02x} ({'enabled' if val else 'disabled'})")
    except Exception as e:
        print(f"Operation failed: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
