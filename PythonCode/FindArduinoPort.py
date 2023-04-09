# https://stackoverflow.com/questions/44056846/how-to-read-and-write-from-a-com-port-using-pyserial





def findArduino():
    import warnings
    import serial
    import serial.tools.list_ports

    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'Arduino' in p.description  # may need tweaking to match new arduinos
    ]
    if not arduino_ports:
        raise IOError("No Arduino found")
    if len(arduino_ports) > 1:
        warnings.warn('Multiple Arduinos found - using the first')
    else: 
        print('Arduion found')
        print(arduino_ports[0])
        
    ser = serial.Serial(arduino_ports[0])
    #print(ser)

    return arduino_ports[0]


# arduinoPort = findArduino()

# print('Arduino on ', arduinoPort)
