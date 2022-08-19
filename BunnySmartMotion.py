# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

# 电机引脚定义
LEFT_ADVANCE = 20  # AIN1
LEFT_RETREAT = 21  # AIN2
RIGHT_ADVANCE = 19  # BIN1
RIGHT_RETREAT = 26  # BIN2
LEFT_SPEED = 16  # PWM A
RIGHT_SPEED = 13  # PWM B

# 红外传感引脚定义
LEFT_INFRARED = 12
RIGHT_INFRARED = 17

# 设置BCM编码
GPIO.setmode(GPIO.BCM)

# 警告信息
GPIO.setwarnings(True)


# 初始化
def initMotorPin():
    global PWM_LEFT_SPEED
    global PWM_RIGHT_SPEED
    # 电机引脚初始化
    GPIO.setup(LEFT_SPEED, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LEFT_ADVANCE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LEFT_RETREAT, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RIGHT_SPEED, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(RIGHT_ADVANCE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RIGHT_RETREAT, GPIO.OUT, initial=GPIO.LOW)
    # 红外传感初始化
    GPIO.setup(LEFT_INFRARED, GPIO.IN)
    GPIO.setup(RIGHT_INFRARED, GPIO.IN)
    # 设置pwm引脚和频率为2000hz
    PWM_LEFT_SPEED = GPIO.PWM(LEFT_SPEED, 2000)
    PWM_RIGHT_SPEED = GPIO.PWM(RIGHT_SPEED, 2000)
    PWM_LEFT_SPEED.start(0)
    PWM_RIGHT_SPEED.start(0)


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
    PWM_LEFT_SPEED.ChangeDutyCycle(0)
    PWM_RIGHT_SPEED.ChangeDutyCycle(80)
    time.sleep(delayTime)


# 小车右转
def right(delayTime):
    GPIO.output(LEFT_ADVANCE, GPIO.HIGH)
    GPIO.output(LEFT_RETREAT, GPIO.LOW)
    GPIO.output(RIGHT_ADVANCE, GPIO.LOW)
    GPIO.output(RIGHT_RETREAT, GPIO.LOW)
    PWM_LEFT_SPEED.ChangeDutyCycle(80)
    PWM_RIGHT_SPEED.ChangeDutyCycle(0)
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


# 延时2s
time.sleep(2)

# try/except语句用来检测try语句块中的错误，
# 从而让except语句捕获异常信息并处理。
# 小车循环前进1s，后退1s，左转2s，右转2s，原地左转3s
# 原地右转3s，停止1s。
try:
    initMotorPin()
    while True:
        leftInfraredValue = GPIO.input(LEFT_INFRARED)  # 左侧红外传感返回结果
        rightInfraredValue = GPIO.input(RIGHT_INFRARED)  # 右侧红外传感返回结果
        print(leftInfraredValue, rightInfraredValue)
        if leftInfraredValue == 1 and rightInfraredValue == 1:
            print("All True")
        elif leftInfraredValue == 1 and rightInfraredValue == 0:
            print("Left True")
        elif leftInfraredValue == 0 and rightInfraredValue == 1:
            print("Right True")
        elif leftInfraredValue == 0 and rightInfraredValue == 0:
            print("All False")
        time.sleep(1)
except KeyboardInterrupt:
    pass
PWM_LEFT_SPEED.stop()
PWM_RIGHT_SPEED.stop()
GPIO.cleanup()
