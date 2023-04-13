
const int MAX_LENGTH = 10;
int messagePos;
char inputString[MAX_LENGTH];



int readSerial() {
  // put your main code here, to run repeatedly:


if (Serial.available() == 0){return(0);}

pinMode(LED_BUILTIN, OUTPUT);

while (Serial.available() > 0){
   char inByte = Serial.read();
   //Serial.println(inByte);
   if (inByte != '\n' && (messagePos < MAX_LENGTH -1)){
          inputString[messagePos] = inByte;
          for(int i=1; i<=3; i++){
            digitalWrite(LED_BUILTIN, HIGH);
            delay(300);
            digitalWrite(LED_BUILTIN,LOW);
            delay(300);
          }
          delay(1500);
          messagePos++;


   }
    else{
      inputString[messagePos] = '\0';
      Serial.println(inputString);
      maxDuration = atoi(inputString);

      messagePos = 0;   
    }
   
   }
   

 return(1);
    

}
