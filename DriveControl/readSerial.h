
const int MAX_LENGTH = 10;
int messagePos;
char inputString[MAX_LENGTH];



int readSerial() {
  // put your main code here, to run repeatedly:

if (Serial.available() == 0){return(0);}

while (Serial.available() > 0){
   char inByte = Serial.read();
   //Serial.println(inByte);
   if (inByte != '\n' && (messagePos < MAX_LENGTH -1)){
          inputString[messagePos] = inByte;
          messagePos++;

   }
    else{
      inputString[messagePos] = '\0';
      Serial.println(inputString);
      duration = atoi(inputString);

      messagePos = 0;   
    }
   
   }
   

 return(1);
    

}
