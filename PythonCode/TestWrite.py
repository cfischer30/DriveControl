# tested April 11, 2023.   Works with WriteToArduino.py
#  

from WriteToArduino import *



initArduino()

angle = 5
duration = 5

sendInt(angle, duration)

closeArduino()
