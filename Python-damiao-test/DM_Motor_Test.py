import math
from DM_CAN import *
import serial
import time
import keyboard

maxTorque = 5.0

def MITMaxTorque(Motor, target_angle:float, kp:float, target_vel:float, kd:float):
    now_angle = Motor.getPosition()
    diff= abs(target_angle-now_angle)
    power = diff*kp
    if(power>maxTorque):
        MotorControl1.controlMIT(Motor, maxTorque/diff, kd, target_angle, target_vel,  0)
    else:
        MotorControl1.controlMIT(Motor, kp, kd, target_angle, target_vel,  0)


Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
Motor2=Motor(DM_Motor_Type.DM6006,0x02,0x15)
Motor3=Motor(DM_Motor_Type.DM4310,0x03,0x11)
Motor5=Motor(DM_Motor_Type.DM4310,0x05,0x11)

serial_device = serial.Serial('COM7', 921600, timeout=0.5)
MotorControl1=MotorControl(serial_device)
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.addMotor(Motor5)

if MotorControl1.switchControlMode(Motor1,Control_Type.MIT):
    print("motor1 switch MIT success")
if MotorControl1.switchControlMode(Motor2,Control_Type.MIT):
    print("motor2 switch MIT success")
if MotorControl1.switchControlMode(Motor3,Control_Type.MIT):
    print("motor3 switch MIT success")
if MotorControl1.switchControlMode(Motor5,Control_Type.MIT):
    print("motor5 switch MIT success")
"""
if MotorControl1.switchControlMode(Motor2,Control_Type.VEL):
    print("switch VEL success")
"""

"""
print("sub_ver:",MotorControl1.read_motor_param(Motor1,DM_variable.sub_ver))
print("Gr:",MotorControl1.read_motor_param(Motor1,DM_variable.Gr))

# if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
#     print("write success")
print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))

print("Motor2:")
print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
"""

# MotorControl1.enable(Motor3)
MotorControl1.save_motor_param(Motor1)
MotorControl1.save_motor_param(Motor2)
MotorControl1.save_motor_param(Motor3)
MotorControl1.save_motor_param(Motor5)
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
MotorControl1.enable(Motor5)
i=0
time.sleep(1.5)

print("motor setup done")
MotorControl1.set_zero_position(Motor1)
MotorControl1.set_zero_position(Motor2)
MotorControl1.set_zero_position(Motor3)
MotorControl1.set_zero_position(Motor5)
time.sleep(3)
print("recording start")
data=[]
idx=0
start = time.time()
#while time.time() - start < 10:
while keyboard.is_pressed('s')==False:
    data.append([])
    MotorControl1.controlMIT(Motor1, 0, 0, 0, 0,  0)
    MotorControl1.controlMIT(Motor2, 0, 0, 0, 0,  0)
    MotorControl1.controlMIT(Motor3, 0, 0, 0, 0,  0)
    MotorControl1.controlMIT(Motor5, 0, 0, 0, 0,  0)
    data[idx].append([Motor1.getPosition(),Motor1.getVelocity()])
    data[idx].append([Motor2.getPosition(),Motor2.getVelocity()])
    data[idx].append([Motor3.getPosition(),Motor3.getVelocity()])
    data[idx].append([Motor5.getPosition(),Motor5.getVelocity()])
    print(data[idx][1])

    idx+=1
    time.sleep(0.01)
print("recording stopped")


time.sleep(3)

while keyboard.is_pressed('s')==False:
    pass
print("moving start")
for l in data:
    print(l)
    MITMaxTorque(Motor1, l[0][0], 15, l[0][1], 1)
    MITMaxTorque(Motor2, l[1][0], 15, l[1][1], 1)
    MITMaxTorque(Motor3, l[2][0], 15, l[2][1], 1)
    MITMaxTorque(Motor5, l[3][0], 15, l[3][1], 1)
    time.sleep(0.01)

time.sleep(5)
for i in range(80):
    print(15*(0.9**i))
    MITMaxTorque(Motor1, data[-1][0][0], 15*(0.9**i), data[-1][0][1], (0.9**i))
    MITMaxTorque(Motor2, data[-1][1][0], 15*(0.9**i), data[-1][1][1], (0.9**i))
    MITMaxTorque(Motor3, data[-1][2][0], 15*(0.9**i), data[-1][2][1], (0.9**i))
    MITMaxTorque(Motor5, data[-1][3][0], 15*(0.9**i), data[-1][3][1], (0.9**i))
    time.sleep(0.05)

"""
print("start")
a=0
b=0
c=0
while True:
    if keyboard.is_pressed('w'):
        a+=1/128
    if keyboard.is_pressed('s'):
        a-=1/128
    if keyboard.is_pressed('a'):
        b+=1/64
    if keyboard.is_pressed('d'):
        b-=1/64
    if keyboard.is_pressed('left'):
        c+=1/64
    if keyboard.is_pressed('right'):
        c-=1/64
    print("a:",a,end="\t")
    print("b:",b,end="\t")
    print("c:",c)
    #MotorControl1.control_Pos_Vel(Motor1, b, 8)
    MITMaxTorque(Motor1, b, 15)
    MITMaxTorque(Motor2, a, 15)
    MITMaxTorque(Motor3, c, 15)
    MotorControl1.controlMIT(Motor1, 5, 1, b, 0,  0)
    MotorControl1.controlMIT(Motor2, 5, 1, a, 0,  0)
    MotorControl1.controlMIT(Motor3, 5, 1, c, 0,  0)
    print(Motor2.getPosition())
    time.sleep(0.01)


#语句结束关闭串口
serial_device.close()
"""