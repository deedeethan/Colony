#include <Wire.h>

void onRequest(void) {
  Wire.write(4);
  Wire.write(11);
  Wire.write(12);
  Wire.write(13);
  Wire.write(14);
        
}

void onReceive(int n) {
  Serial.print("Received ");
  Serial.println(n);
  for (int i = 0; i < n; i++) {
    char c = Wire.read();
    Serial.print("Byte ");
    Serial.println((int)c);
  }
}
  

void setup() {
  Wire.begin(5);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
  Serial.begin(9600);
  Serial.println("Hi Jess");
}

void loop() {
  Serial.println("loop");
  delay(5000);
}
