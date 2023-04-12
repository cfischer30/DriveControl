import serial
import time
from FindArduinoPort import *

# find Arduino Port Number

from FindArduinoPort import *


def sendBytes(td):
    numLoops = len(td)

    n = 0
    while n < numLoops:
        writeChar = td[n]
        sendToArduino(writeChar)
        n += 1


def initArduino():
    arduinoPort = findArduino()
    baudRate = 9600
    global ser
    ser = serial.Serial(arduinoPort, baudRate)
    print("Serial port " + arduinoPort + " opened  Baudrate " + str(baudRate))
    time.sleep(3)  #3 second pause to allow connection to be made

    

def sendInt(value1, value2):
    print('sending ', value1, value2)
    sendData = []
    valueString1 = str(value1)
    valueString2 = str(value2)
    sendString = '<' + valueString1 + '|' + valueString2 + '>'
    arg = bytes(sendString,'ascii')
    sendData.append(arg)
    sendBytes(sendData)
    print('sent')

def closeArduino():
    print('closing connection')
    ser.close()

def sendToArduino(sendStr):
    ser.write(sendStr)

    
