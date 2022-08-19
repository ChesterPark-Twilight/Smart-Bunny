# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

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

# LED指示灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 设置BCM编码
GPIO.setmode(GPIO.BCM)

# 警告信息
GPIO.setwarnings(True)


# 初始化操作
def initPin():
    global PWM_LEFT_SPEED
    global PWM_RIGHT_SPEED
    global PWM_ULTRASONIC_STEERING_ENGINE
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
    PWM_ULTRASONIC_STEERING_ENGINE.start(0)


# 小车前进
def run(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车后退
def back(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.HIGH)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.HIGH)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车左转
def left(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车右转
def right(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车原地左转
def spin_left(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.LOW)
    GPIO.output(LEFT_RETREAT, GPIO.HIGH)
    GPIO.output(RIGHT_ADVANCE, GPIO.HIGH)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车原地右转
def spin_right(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.HIGH)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
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
    for i in range(15):
        PWM_ULTRASONIC_STEERING_ENGINE.ChangeDutyCycle(2.5 + 10 * angle / 180)


# 发射超声波
def transmitUltrasonic():
    GPIO.output(ULTRASONIC_TRANSMIT, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(ULTRASONIC_TRANSMIT, GPIO.LOW)


# 探测距离
def detectDistance():
    transmitUltrasonic()
    while not GPIO.input(ULTRASONIC_RECEIVE):
        pass
    timeFirst = time.time()
    while GPIO.input(ULTRASONIC_RECEIVE):
        pass
    timeSecond = time.time()
    distance = ((timeSecond - timeFirst) * 340 / 2) * 100
    print("Distance is ", distance)
    return distance


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


try:
    initPin()
    print("初始化完成...")
    print("旋转0度")
    rotateAngle(0)
    time.sleep(1)
    print("旋转180度")
    rotateAngle(180)
    time.sleep(1)
    print("旋转90度")
    rotateAngle(89)
    time.sleep(1)
    # print(detectDistance())
except KeyboardInterrupt:
    pass
PWM_LEFT_SPEED.stop()
PWM_RIGHT_SPEED.stop()
GPIO.cleanup()
