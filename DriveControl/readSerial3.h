
// Example 3 - Receive with start- and end-markers
// from https://forum.arduino.cc/t/serial-input-basics-updated/382007/2


// declare following variables in main:
//const byte numChars = 32;
//char receivedChars[numChars];

//boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    int intVal;
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        // converts character array to an integer
        // any characters after a non-integer are ignored
        
        intVal = atoi(receivedChars);
        Serial.print("As an integer minus 1... ");
        Serial.println(intVal-1);
        newData = false;
    }
}