"""Motor streaming utility using SimpleSTS

Defaults to COM3 and id=1 and prints CSV lines: timestamp,id,position

Usage:
    python motor_stream.py           # uses COM3 and id=1
    python motor_stream.py --port COM4 --id 2 --interval 0.1

"""
from __future__ import annotations
import argparse
import time
import sys
from simple_sts import SimpleSTS, DEFAULT_BAUD, DEFAULT_TIMEOUT


def read_angle(motor_id: int, port: str = "COM3", baud: int | None = None, timeout: float | None = None, verbose: bool = False) -> int:
    """Read and return the Present_Position (decoded int) for a single motor.

    This function can be imported and called from other modules:
        from motor_stream import read_angle
        pos = read_angle(1)  # reads id=1 on COM3

    Parameters:
        motor_id: motor ID to query
        port: serial port (default COM3)
        baud: optional baud rate (default from simple_sts.DEFAULT_BAUD)
        timeout: optional read timeout (default from simple_sts.DEFAULT_TIMEOUT)
        verbose: if True, enables raw TX/RX logging from SimpleSTS

    Returns:
        integer decoded position

    Raises any exceptions from serial operations or protocol parsing so callers can handle them.
    """
    _baud = DEFAULT_BAUD if baud is None else baud
    _timeout = DEFAULT_TIMEOUT if timeout is None else timeout
    client = SimpleSTS(port, baud=_baud, timeout=_timeout, verbose=verbose)
    try:
        client.open()
        return client.read_present_position(motor_id)
    finally:
        client.close()


def main():
    p = argparse.ArgumentParser(description="Continuously read Present_Position from a Simple STS motor and print CSV output")
    p.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    p.add_argument("--id", type=int, default=1, help="Motor ID (default: 1)")
    p.add_argument("--interval", type=float, default=0.05, help="Read interval in seconds (default: 0.05)")
    p.add_argument("--verbose", action="store_true", help="Enable verbose TX/RX logging from SimpleSTS")
    args = p.parse_args()

    client = SimpleSTS(args.port, verbose=args.verbose)
    try:
        client.open()
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}", file=sys.stderr)
        sys.exit(2)

    print("# timestamp,id,position")
    try:
        while True:
            try:
                pos = client.read_present_position(args.id)
                print(f"{time.time():.6f},{args.id},{pos}")
            except Exception as e:
                # print error but keep trying
                print(f"# ERROR: {e}", file=sys.stderr)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nInterrupted by user, exiting...")
    finally:
        client.close()


if __name__ == "__main__":
    main()
