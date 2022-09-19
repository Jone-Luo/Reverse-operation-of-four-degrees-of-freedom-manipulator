#!/usr/bin/python3
"""
import serial
import math
import time
"""
"""
ser = serial.Serial('/dev/ttyUSB1', 1000000,bytesize=8, parity='N', stopbits=1, timeout=1)    #设置串口
ser.close()
ser.open()
"""
#机械臂逆解算 末端位置 -> 舵机角度
def Arm(height,radius):
    L1 = 135    #机械臂长度参数
    L2 = 57
    L3 = 135
    L4 = 204
    L5 = 147
    bf_2 = radius*radius + height*height    #逆解算
    a1 = math.atan(height/radius)
    a2 = math.acos((bf_2 + L1*L1 - L5*L5)/(2*math.sqrt(bf_2)*L1))
    b2 = math.acos((L1*L1 + L5*L5 - bf_2)/(2*L1*L5))
    angle1 = a1 + a2
    angle2 = angle1 + b2

    return angle1,angle2    #弧度制

#取反(不含符号位）
def Bit_not_op(v,bit_size):
    bit16_not_val = 0
    for i in range(0,bit_size):
        if ((v >> i)&0x1) == 0 :
            bit16_not_val |= (1 << i)
    return (bit16_not_val)

#设置舵机旋转目标角度
def Set_Angle(dr,angle):

    time.sleep(0.001)

    i = 0
    add = 0x00
    #      0    1    2    3 4    5    6    7    8
    dat = [0xFF,0xFF,0x00,5,0x04,0x1E,0x00,0x00,0xFF]      #定义一个数据包
    #print(dat)

    dat[2] = dr                                    #设置ID号
    dat[6] = int(angle * 1023.0 // 300.0) & 255           #设置目标角度的低字节
    dat[7] = int(angle * 1023.0 // 300.0) >> 8            #设置目标角度的高字节
    #print(dat)

    for i in range(2,8):
        add+=dat[i]                    #计算校验和

    dat[8] = Bit_not_op(add,8)

    for i in range(0,9):

        dat[i] = "%02x" % dat[i]
        d=bytes.fromhex(dat[i])
        ser.write(d)

        #print(dat)


#舵机开始运动
def Start_move():
    time.sleep(0.001)
    d=bytes.fromhex('ff ff fe 02 05 fa')
    ser.write(d)

"""
         Y
         |
         |
         |
    X---------
"""
#执行
def Robot_arm(x,y):

    if x<100:
        x = 100
    elif x>200:
        x = 200
    elif y < -90:
        y = -90
    elif y > 180:
        y = 180

    a = Arm(y,x)
    servo2 = a[0]/3.14 * 180    #实际角度
    servo1 = a[1]/3.14 * 180
    #print(1)   #打印实际角度
    #print(servo1)
    #print(2)
    #print(servo2)
    servo2 = 180 - (servo2 - 15)    #校准
    servo1 = servo1 - 88
    print(1)    #打印舵机角度
    print(servo1)
    print(2)
    print(servo2)
    Set_Angle(1,servo1)
    time.sleep(0.001)
    Set_Angle(2,servo2)
    time.sleep(0.001)
    Start_move()

#-----功能函数-----#


def arm_close():
    Set_Angle(3,80)
    Start_move()

def arm_open():
    Set_Angle(3,120)
    Start_move()

def arm_default_position():
    arm_x = 100
    arm_y = 170
    angle_0 = 90

    Robot_arm(arm_x,arm_y)

    Set_Angle(3,120)
    Set_Angle(0,angle_0)
    Start_move()


"""
while 1:

    x = int(input("x: "))
    y = int(input("y: "))
    Robot_arm(x,y)

    angle1 = int(input("angle1: "))
    angle2 = int(input("angle2: "))

    Set_Angle(1,angle1)
    time.sleep(0.001)
    Set_Angle(2,angle2)
    time.sleep(0.001)
    Start_move()
"""








