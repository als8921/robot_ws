void setup() {
  Serial.begin(19200);
}

void loop() {
  while (Serial.available() > 0) 
  {
    byte incomingByte = Serial.read(); // byte형으로 읽기
    if (incomingByte != -1) 
    {
      if (incomingByte == 0xb7)
        Serial.write(0xb8);
      else if (incomingByte == 0xb8)
        Serial.write(0xb7);
      else
        Serial.write(incomingByte);
    }
  }
}
