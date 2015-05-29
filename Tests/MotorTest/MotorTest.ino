#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 10

Servo motor;
int motorSpeed = 750;
void setup() {
  Serial.begin(9600);
  motor.attach(MOTOR_PIN);
  motor.writeMicroseconds(motorSpeed);
}

char getCharacter() {
	while (Serial.available() == 0);
	return Serial.read();
}

void loop() {
  char cin = getCharacter();
  if (cin == '^') {
    motorSpeed += 10;
    motor.writeMicroseconds(motorSpeed);
    Serial.print("Speed Increased: "); Serial.println(motorSpeed);
  } else if (cin == 'v') {
    motorSpeed -= 10;
    motor.writeMicroseconds(motorSpeed);
    Serial.print("Speed Decreased: "); Serial.println(motorSpeed);
  }
}

