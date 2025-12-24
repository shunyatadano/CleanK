import serial
import time
import feetech

def main():
    port = "COM3"
    baudrate = 1000000

    serial_port = serial.Serial(port, baudrate)
    bus = feetech.Bus(serial_port)

    time.sleep(2)  # Wait for the serial connection to initialize

    cm = feetech.Command()

    bus.send_command(cm.set_position(1, 512))  # Move servo with ID 1 to position 512
    time.sleep(1)
    bus.send_command(cm.set_position(1, 256))  # Move servo with ID 2 to position 256
    time.sleep(1)


if __name__ == "__main__":
    exit(main())