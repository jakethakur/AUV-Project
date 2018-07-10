void setup(){
  Serial.begin(57600);
  pinMode(13, OUTPUT); //required for digitalWrite to work correctly
}


void loop() { 
//receive data from your app
  involtReceive();

  digitalWrite(13, involtPin[13]);
  involtSend(0, analogRead(A0));
  delay(2);
}
