#include <Arduino.h>
#include <SIMU.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

void setup() {
	char incomingByte;
	Serial.begin(9600);
	while (Serial.available() == 0);
	incomingByte = Serial.read();
	Serial.print("Recieved: ");
	if (incomingByte == 's')
		Serial.println("Starting");
	else 
		Serial.println("Not Starting");
}

void loop() {}
