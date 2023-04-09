from FindArduinoPort import *
import serial


arduinoPort = findArduino()

print('Arduino on ', arduinoPort)

ser = serial.Serial(arduinoPort)
print(ser)

ser.close()

