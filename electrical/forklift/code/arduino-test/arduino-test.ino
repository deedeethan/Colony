#include <stdlib.h>
#include <Wire.h>

// A4: SDA
// A5: SCL

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

int readAddr(byte addr) {
  Wire.beginTransmission(0x41);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(0x41, 1);
  for (int i = 0; !Wire.available(); i++) {
    if (i == 5) return -1;
    delay(1);
  }
  byte ret = Wire.read();
  return ret;
}

byte writeAddr(byte addr, byte data) { 
  Wire.beginTransmission(0x41);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

int readMany(byte addr) {
  int x;
  while ((x = readAddr(addr)) == -1);
  return x;
}

char read_buf[5];
int read_buf_i = 0;

int checkSerial() {
  while (Serial.available()) {
    char in = Serial.read();
    if (in == '\n') {
      if (read_buf_i == 4) {
        read_buf_i = 0;
        return -1; // ignore inputs too long
      }
      read_buf[read_buf_i] = 0;
      read_buf_i = 0;
      int n = atoi(read_buf);
      if (n < 0 || n > 255) return -1;
      return n;
    } else if (read_buf_i < 4) {
      read_buf[read_buf_i] = in;
      read_buf_i++;
    }
  }
  return -1;
}

void loop() {
  int in = checkSerial();
  /*Serial.println(in);
  delay(650);*/
  if (in >= 0)
    writeAddr(3, in);
  Serial.print("\nHeight setpoint: ");
  Serial.print(readMany(3));
  Serial.print("\nHeight: ");
  Serial.print(readMany(2));
  Serial.print("\n--------------------");
  delay(650);
}

