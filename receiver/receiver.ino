HardwareSerial Serial2(PA3, PA2);   // RX, TX
HardwareSerial Serial3(PB11, PB10); // RX, TX

void setup() {
  Serial.begin(230400);
  Serial2.begin(230400);
  Serial3.begin(31250);
}

void loop() {
  int chars = Serial2.available();

  if (chars > 0) {
    String x = Serial2.readStringUntil('»');
    MIDImessage(x.charAt(0), x.charAt(1), x.charAt(2));
  }
}

void MIDImessage(byte command, byte data1, byte data2) {
  Serial3.write(command);
  Serial3.write(data1);
  Serial3.write(data2);
}