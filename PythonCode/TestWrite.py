from WriteToArduino import *

initArduino()


def sendAngle(angle):
    angle += 20000
    sendInt(angle)


def sendSpeed(speed):
    speed += 10000
    sendInt(speed)


sendSpeed(140);
sendAngle(45);

closeArduino()



