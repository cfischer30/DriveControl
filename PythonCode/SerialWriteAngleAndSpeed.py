from FindArduinoPort import *
import serial


# Find the arduino port

arduinoPort = findArduino()

print('Arduino on ', arduinoPort)

# write to serial port

ser = serial.Serial(arduinoPort, 9600)
print('writing to ', ser)
ser.write(b'<4000>')
ser.write(b'\n')
print('Write Completed')
ser.close()
