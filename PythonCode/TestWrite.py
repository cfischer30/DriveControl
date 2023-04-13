from WriteToArduino import *

initArduino()


void sendAngle(angle){
    angle += 10000
    sendInt(angle)   
    }


void sendSpeed(speed){
    angle += 20000
    sendInt(speed)
    }


sendSpeed(140);
sendAngle(45);

closeArduino()



