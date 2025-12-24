import math
from DM_CAN import *
import serial
import time
import keyboard


maxTorque = 10.0

def MITMaxTorque(MCC :MotorControl, Motor, target_angle:float, kp:float, target_vel:float, kd:float):
    now_angle = Motor.getPosition()
    diff= abs(target_angle-now_angle)
    power = diff*kp
    if(power>maxTorque):
        MCC.controlMIT(Motor, maxTorque/diff, kd, target_angle, target_vel,  0)
    else:
        MCC.controlMIT(Motor, kp, kd, target_angle, target_vel,  0)


K_p = 20

Motor1=Motor(DM_Motor_Type.DM6006,0x01,0x15)
Motor2=Motor(DM_Motor_Type.DM6006,0x02,0x15)
Motor3=Motor(DM_Motor_Type.DM4310,0x03,0x11)
Motor4=Motor(DM_Motor_Type.DM4310,0x04,0x11)
Motor5=Motor(DM_Motor_Type.DM4310,0x05,0x11)

Motor2_1=Motor(DM_Motor_Type.DM6006,0x01,0x15)
Motor2_2=Motor(DM_Motor_Type.DM6006,0x02,0x15)
Motor2_3=Motor(DM_Motor_Type.DM4310,0x03,0x11)
Motor2_4=Motor(DM_Motor_Type.DM4310,0x04,0x11)
Motor2_5=Motor(DM_Motor_Type.DM4310,0x05,0x11)

motorlist = [Motor1,Motor2,Motor3,Motor4,Motor5]
motorlist2 = [Motor2_1,Motor2_2,Motor2_3,Motor2_4,Motor2_5]

serial_device = serial.Serial('COM6', 921600, timeout=0.5)          #follower arm DM
serial_device2 = serial.Serial('COM7', 921600, timeout=0.5)         #follower arm STS
serial_device3 = serial.Serial('COM5', 921600, timeout=0.5)         #leader arm DM
serial_device4 = serial.Serial('COM9', 921600, timeout=0.5)         #leader arm STS
MotorControl1=MotorControl(serial_device)   #follower DM motors
MotorControl2=MotorControl(serial_device2)  #follower STS motors
MotorControl3=MotorControl(serial_device3)  #leader DM motors
MotorControl4=MotorControl(serial_device4)  #leader STS motors

controllerid = 0x0f

motorid = 1
motorid2 = 2

for i in range(5):
    MotorControl1.addMotor(motorlist[i])

for i in range(5):
    MotorControl3.addMotor(motorlist2[i])


for i in range(5):
    if MotorControl1.switchControlMode(motorlist[i+1],Control_Type.MIT):
        print("1st arm motor"+str(i+1)+" switch MIT success")
    
for i in range(5):
    if MotorControl3.switchControlMode(motorlist2[i],Control_Type.MIT):
        print("2nd arm motor"+str(i+1)+" switch MIT success")



for i in range(5):
    MotorControl1.save_motor_param(motorlist[i])
    MotorControl1.enable(motorlist[i])
    MotorControl1.set_zero_position(motorlist[i])
for i in range(5):
    MotorControl3.save_motor_param(motorlist[i])
    MotorControl3.enable(motorlist[i])
    MotorControl3.set_zero_position(motorlist[i])


i=0
time.sleep(1.5)

print("motor setup done")
#time.sleep(3)
#MotorControl1.controlMIT(Motor, maxTorque/diff, kd, target_angle, target_vel,  0)
for i in range(5):
    MotorControl1.controlMIT(motorlist[i], 0, 0, 0, 0,  0)
print("tele-operation start")

data=[]
idx=0
start = time.time()
#while time.time() - start < 10:
while keyboard.is_pressed('s')==False:
    l=[]
    MotorControl4.STSControl_read(controllerid, motorid)        #leader sts id=1 read(同時に二つやると処理が追い付かなくなる)
    for i in range(5):          #leader DM motors read
        MotorControl3.controlMIT(motorlist[i], 0, 0, 0, 0,  0)

    time.sleep(0.005)
    
    MotorControl3.recv()   #leader DM motors recv (値の取得命令送信後、値が返ってくるまでタイムラグあり)
    for i in range(5):
        l.append(motorlist2[i].getPosition())   #leader DM motors position append
    MotorControl4.STSControl_read(controllerid, motorid2)   #leader sts id=2 read （同時に二つやると処理が追い付かなくなる）
    l.append(MotorControl4.sts_map[motorid])        #leader sts id=1 position append (id=1(１つ目のモーターの値が返ってくるのがこのぐらいと予想))
    MotorControl2.STSControl_write(controllerid, motorid, l[5])   #follower sts id=1 write(値戻ってきたのでfollowerに書く)
    
    for i in range(5):
        MITMaxTorque(MotorControl1, motorlist[i], l[i], K_p, 0, 1)
    MotorControl4.recv()   #leader STS recv(id=2) (値の取得命令送信後、値が返ってくるまでタイムラグあり)
    l.append( MotorControl4.sts_map[motorid2])      #leader sts id=2 position append
    MotorControl2.STSControl_write(controllerid, motorid2, l[6])    #follower sts id=2 write(値戻ってきたのでfollowerに送信)
    
    print([float(q) for q in l])

    idx+=1
    time.sleep(0.005)


    #data.append(l)
    #   この辺にcsv保存処理を書きたい
    #
    #
