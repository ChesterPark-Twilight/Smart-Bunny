import time

import serial
from time import sleep


#
# inputDataDemo = b'mc 01 00000663 00000366 ffffffff ffffffff 0001 02 000029a8 a0:0 17c7\r\n$T0,-60.87,-77.19\r\n'
#
# serialPort = serial.Serial('./dev/ttyUSB0', 115200, timeout=0.5)
# while True:
#     serialByteData = serialPort.read()
#     sleep(0.05)
#     serialByteDataRemain = serialPort.inWaiting()
#     serialByteData += serialPort.read(serialByteDataRemain)
#     serialData = serialByteData.decode('UTF-8')
#     if serialData != '':
#         informationList = serialData.split()
#         print(informationList[2], informationList[3])

# serialData = inputDataDemo.decode('UTF-8')
# if serialData != '':
#     informationList = serialData.split()
#     print(informationList[2], informationList[3])
#     print(int(informationList[2], 16), int(informationList[3], 16))

import math


def calcCoordinate(LineZero, LineOne):
    cosAZ = (pow(LineZero, 2) + pow(0.3, 2) - pow(LineOne, 2)) / (2 * LineZero * 0.3)
    sinAZ = pow(1-pow(cosAZ, 2), 0.5)
    x = LineZero * cosAZ - 0.15
    y = LineZero * sinAZ
    return abs(round(x, 2)), round(y, 2)


start = time.time()
print(calcCoordinate(0.8, 0.8))
end = time.time()
print(end-start)
