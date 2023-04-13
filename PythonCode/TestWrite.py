from WriteToArduino import *

initArduino()


void sendAngle(angle){
    angle += 1000000
    sendInt(angle)   
    }


void sendSpeed(speed){
    angle += 2000000
    sendInt(speed)
    }


sendSpeed(140);
sendAngle(45);

closeArduino()



