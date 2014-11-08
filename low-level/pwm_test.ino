#define PIN1 9
#define PIN2 8

enum {
 COAST, BRAKE
} mode = COAST;

void setup() {
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void setSpeed(int s) {
  if (s > 0) {
    if (s > 255) s = 255;
    if (mode == COAST) {
      analogWrite(PIN1, s);
      analogWrite(PIN2, 0);
    } else {
      analogWrite(PIN1, 1);
      analogWrite(PIN2, 255-s);
    }
  } else {
    if (s < -255) s = -255;
    if (mode == COAST) {
      analogWrite(PIN1, 0);
      analogWrite(PIN2, -s);
    } else {
      analogWrite(PIN1, 255+s);
      analogWrite(PIN2, 1);
    }
  }
}

void loop() {
  while (Serial.available() == 0);
  int speed = Serial.parseInt();
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'b') mode = BRAKE;
    else if (c == 'c') mode = COAST;
  }
  setSpeed(speed);
}
