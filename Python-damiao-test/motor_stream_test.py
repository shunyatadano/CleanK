import motor_stream
import time

while True:
    print(motor_stream.read_angle(1))
    time.sleep(0.1)