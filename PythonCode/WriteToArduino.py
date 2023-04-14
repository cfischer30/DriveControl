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
    arduinoPort = "/dev/ttyACM0"
    baudRate = 9600
    global ser
    ser = serial.Serial(arduinoPort, baudRate)
    print("Serial port " + arduinoPort + " opened  Baudrate " + str(baudRate))
    time.sleep(3)  #3 second pause to allow connection to be made

    startMarker = 60  # < sign
    endMarker = 62    # > sign

def sendInt(value):
    print('sending ', value)
    sendData = []
    valueString = str(value)
    sendString = '<' + valueString + '>'
    arg = bytes(sendString,'ascii')
    sendData.append(arg)
    sendBytes(sendData)
    print('sent')

def closeArduino():
    print('closing connection')
    ser.close()

def sendToArduino(sendStr):
    ser.write(sendStr)

    
