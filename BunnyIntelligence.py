# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import serial
import time

import numpy as np
import math

# 小车电机引脚定义
LEFT_ADVANCE = 20  # AIN1
LEFT_RETREAT = 21  # AIN2
RIGHT_ADVANCE = 19  # BIN1
RIGHT_RETREAT = 26  # BIN2
LEFT_SPEED = 16  # PWM A
RIGHT_SPEED = 13  # PWM B

# 超声波舵机引脚定义
ULTRASONIC_STEERING_ENGINE = 23

# 超声波模块引脚定义
ULTRASONIC_RECEIVE = 0
ULTRASONIC_TRANSMIT = 1

# 红外模块引脚定义
LEFT_INFRARED = 12
RIGHT_INFRARED = 17

# LED指示灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 设置BCM编码
GPIO.setmode(GPIO.BCM)

# 警告信息
GPIO.setwarnings(False)


# 初始化操作
def init():
    global PWM_LEFT_SPEED
    global PWM_RIGHT_SPEED
    global PWM_ULTRASONIC_STEERING_ENGINE
    global serialPort
    # 电机引脚初始化
    GPIO.setup(LEFT_SPEED, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LEFT_ADVANCE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LEFT_RETREAT, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RIGHT_SPEED, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(RIGHT_ADVANCE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RIGHT_RETREAT, GPIO.OUT, initial=GPIO.LOW)
    # 舵机引脚初始化
    GPIO.setup(ULTRASONIC_STEERING_ENGINE, GPIO.OUT)
    # 超声波模块引脚初始化
    GPIO.setup(ULTRASONIC_RECEIVE, GPIO.IN)
    GPIO.setup(ULTRASONIC_TRANSMIT, GPIO.OUT)
    # 红外模块引脚初始化
    GPIO.setup(LEFT_INFRARED, GPIO.IN)
    GPIO.setup(RIGHT_INFRARED, GPIO.IN)
    # LED指示灯引脚初始化
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    # 设置PWM电机引脚和频率
    PWM_LEFT_SPEED = GPIO.PWM(LEFT_SPEED, 2000)
    PWM_RIGHT_SPEED = GPIO.PWM(RIGHT_SPEED, 2000)
    PWM_LEFT_SPEED.start(0)
    PWM_RIGHT_SPEED.start(0)
    # 设置PWM超声波舵机引脚和频率
    PWM_ULTRASONIC_STEERING_ENGINE = GPIO.PWM(ULTRASONIC_STEERING_ENGINE, 50)
    # PWM_ULTRASONIC_STEERING_ENGINE.start(0)
    # 设置串口
    serialPort = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    rotateAngle(90)


# 打开红色LED
def turnOnRed():
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)


# 打开绿色LED
def turnOnGreen():
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.HIGH)
    GPIO.output(LED_B, GPIO.LOW)


# 打开蓝色LED
def turnOnBlue():
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)


def showLED():
    turnOnRed()
    time.sleep(0.5)
    turnOnGreen()
    time.sleep(0.5)
    turnOnBlue()
    time.sleep(0.5)
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)


# 行进
def advance(speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)


# 转圈圈
def rotate(speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)


# 原地转圈圈
def rotateNow(speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.HIGH)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)


# 小车前进
def moveForward(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车后退
def moveBackward(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.HIGH)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.HIGH)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车左转
def turnLeft(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车右转
def turnRight(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车原地左转
def rotateLeft(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.HIGH)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车原地右转
def rotateRight(delayTime, speed=10):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.HIGH)
    PWM_LEFT_SPEED.ChangeDutyCycle(speed)
    PWM_RIGHT_SPEED.ChangeDutyCycle(speed)
    time.sleep(delayTime)


# 小车停止
def brake(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 舵机旋转指定角度
def rotateAngle(angle):
    PWM_ULTRASONIC_STEERING_ENGINE.start(0)
    for i in range(15):
        PWM_ULTRASONIC_STEERING_ENGINE.ChangeDutyCycle(2.5 + 10 * angle / 180)


# 发射超声波
def transmitUltrasonic():
    GPIO.output(ULTRASONIC_TRANSMIT, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(ULTRASONIC_TRANSMIT, GPIO.LOW)


# 探测距离
def detectDistance():
    distanceAverage = 0
    # for i in range(3):
    #     transmitUltrasonic()
    #     while not GPIO.input(ULTRASONIC_RECEIVE):
    #         pass
    #     timeFirst = time.time()
    #     while GPIO.input(ULTRASONIC_RECEIVE):
    #         pass
    #     timeSecond = time.time()
    #     distanceAverage += ((timeSecond - timeFirst) * 340 / 2) * 100
    #     time.sleep(0.25)
    # distanceAverage /= 3
    transmitUltrasonic()
    while not GPIO.input(ULTRASONIC_RECEIVE):
        pass
    timeFirst = time.time()
    while GPIO.input(ULTRASONIC_RECEIVE):
        pass
    timeSecond = time.time()
    distanceAverage = ((timeSecond - timeFirst) * 340 / 2) * 100
    return distanceAverage


# 寻找通路
def searchAccess():
    print("检测左侧...")
    rotateAngle(165)
    time.sleep(2)
    leftDistance = detectDistance()
    print("检测右侧...")
    rotateAngle(30)
    time.sleep(2)
    rightDistance = detectDistance()
    print("检测返回中路...")
    rotateAngle(90)
    time.sleep(2)
    middleDistance = detectDistance()
    print("Left", leftDistance, "Middle", middleDistance, "Right", rightDistance)
    distanceList = [leftDistance, middleDistance, rightDistance]
    accessIndex = distanceList.index(max(distanceList))
    if accessIndex == 0:
        print("避障中，原地左转1秒，直行1秒")
        rotateLeft(1.2)
        moveForward(1.8)
        brake(0.1)
    if accessIndex == 2:
        print("避障中，原地右转1秒，直行1秒")
        rotateRight(1.2)
        moveForward(1.8)
        brake(0.1)
    # rotateNow(50)
    # distance = 0
    # while distance < 30:
    #     distance = detectDistance()
    # brake(0.1)
    # advance(10)


# 获得位置信息
def getLocation():
    serialByteData = serialPort.read()
    time.sleep(0.02)
    serialByteDataRemain = serialPort.inWaiting()
    serialByteData += serialPort.read(serialByteDataRemain)
    serialData = serialByteData.decode('UTF-8')
    if serialData != '':
        informationList = serialData.split()
        informationArray = np.array(informationList)
        startIndexTuple = np.where(informationArray == 'mc')
        startIndexList = startIndexTuple[0]
        if len(startIndexList) > 1:
            startIndexList = startIndexList[:-1]
            LineZero = 0
            LineOne = 0
            for index in startIndexList:
                LineZero += int(informationList[index + 2], 16) / 100.0 - 0.6  # 探测距离修正
                LineOne += int(informationList[index + 3], 16) / 100.0  # 探测距离修正
            LineZero /= len(startIndexList)
            LineOne /= len(startIndexList)
            # LineZero = int(informationList[startIndexList[0] + 2], 16) / 100.0 - 0.6  # 探测距离修正
            # LineOne = int(informationList[startIndexList[0] + 3], 16) / 100.0
            if 0 < LineZero < 100 and 0 < LineOne < 100:
                # print(LineZero, LineOne)
                return getMovementTrend(LineZero, LineOne)
            else:
                # print('DATA INVALID\n---------------')
                return 1, 1
        else:
            # print('DATA ABANDON\n---------------')
            return 0, 1
    else:
        # print('NO INFO\n---------------')
        return 0, 0


def getMovementTrend(LineZero, LineOne):
    distance = abs(pow((2 * pow(LineZero, 2) + 2 * pow(LineOne, 2) - pow(3, 2)), 0.5) / 2)
    angle = (pow(distance, 2) + pow(1.5, 2) - pow(LineOne, 2)) / (2 * distance * 1.5)
    return distance, angle


def detectObstacle():
    distanceForward = detectDistance()
    print("前方距离: {}".format(distanceForward))
    if distanceForward < 40:
        brake(0.1)
        searchAccess()
        detectObstacle()


def teachingProgram():
    advance(10)
    while True:
        distanceForward = detectDistance()
        print(distanceForward)
        if distanceForward < 40:
            brake(0.1)
            rotateRight(1.0)
            moveForward(2.0)
            brake(0.1)
            break


if __name__ == '__main__':
    try:
        init()
        print("初始化完成...")
        while True:
            detectObstacle()
            # teachingProgram()
            locationAverage, angleAverage = 0, 0
            for i in range(5):
                distanceToGo, angleToGo = getLocation()
                locationAverage += distanceToGo
                angleAverage += angleToGo
                time.sleep(0.2)
            locationAverage /= 5
            angleAverage /= 5
            print(locationAverage, angleAverage)
            if locationAverage > 17:
                if angleAverage < 0:
                    print("执行操作左转")
                    turnLeft(abs(angleAverage) * 2, 20)
                else:
                    print("执行操作右转")
                    turnRight(angleAverage * 2, 20)
                customSpeed = int(20)
                print("当前速度: {}".format(customSpeed))
                advance(customSpeed)
            else:
                brake(0.1)

        # while True:
        #     advance(5)
        #     distanceForward = detectDistance()
        #     print(distanceForward)
        #     if distanceForward < 20:
        #         brake(0.1)
        #         searchAccess()

        # searchAccess()
        # print(detectDistance())
        # moveForward(1)
        # rotateLeft(4)
        # moveForward(0.5)

        # tag = 0
        # while True:
        #     advance(5)
        #     distanceForward = detectDistance()
        #     leftInfrared = GPIO.input(LEFT_INFRARED)
        #     rightInfrared = GPIO.input(RIGHT_INFRARED)
        #     print(distanceForward, leftInfrared, rightInfrared)
        #     if distanceForward < 30:
        #         brake(0.1)
        #         searchAccess()
        #     if leftInfrared == 0 and tag != 1:
        #         brake(0.1)
        #         rotateRight(1)
        #         tag = 2
        #     if rightInfrared == 0 and tag != 2:
        #         brake(0.1)
        #         rotateLeft(1)
        #         tag = 1
    except KeyboardInterrupt:
        pass
    PWM_LEFT_SPEED.stop()
    PWM_RIGHT_SPEED.stop()
    PWM_ULTRASONIC_STEERING_ENGINE.stop()
    GPIO.cleanup()
