void setup() {
  Serial.begin(19200);
}

void loop() {
  while (Serial.available() > 0) 
  {
    char incomingByte = Serial.read();
    if (incomingByte != -1) 
    {
      Serial.print(incomingByte);
    }
  }
}
