#include <Arduino.h>
#include <SIMU.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

/*
							-
						+		-
							+
*/

SIMU imu;
float toDeg = 57.2957795131;
double oldGyroValues[2] = {0, 0};
unsigned long oldTime = 0;
double pitchZero, rollZero, yawZero = 0;
double pitchSet, rollSet, yawSet = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  imu.initialize();
  calibrateAngles();
}

void calibrateAngles() {
  double p, r = 0;
  oldGyroValues[0] = 0;
  oldGyroValues[1] = 0;
  for (int i = 0; i < 100; i++) {
    calculatePitchAndRoll();
    if (i >= 49) {
      p = p + oldGyroValues[0];
      r = r + oldGyroValues[1];
    }
  }
  pitchZero = p / 50.0d;
  rollZero = r / 50.0d;
  pitchSet = pitchZero;
  rollSet = rollZero;

  //Serial.print("Ini Pitch: "); Serial.print(pitchZero); Serial.print(" Ini Roll: "); Serial.println(rollZero);
  Serial.println("Calibration Complete");
}

void calculatePitchAndRoll() {
  float sixReadings[6];
  float accPitch, accRoll;
  float deltaTime = (millis() - oldTime) / 1000;
  oldTime = millis();
  imu.getAllSixReadings(sixReadings);

  if (deltaTime > .01)
    deltaTime = .01;

  oldGyroValues[0] = oldGyroValues[0] + ((sixReadings[3] / 16.4f) * deltaTime);
  oldGyroValues[1] = oldGyroValues[1] + ((sixReadings[4] / 16.4f) * deltaTime);
  // Serial.print("Gyro Pitch: ");Serial.print(oldGyroValues[0]);Serial.print(" Gyro Roll: ");Serial.println(oldGyroValues[1]);

  if (sixReadings[0] != 0 && sixReadings[1] != 0 && sixReadings[2] != 0) {
    accRoll = atan(sixReadings[0] / sqrt(square(sixReadings[1]) + square(sixReadings[2]))) * toDeg;
    accPitch = atan(sixReadings[1] / sqrt(square(sixReadings[0]) + square(sixReadings[2]))) * toDeg;
    // Serial.print("Accel Pitch: ");Serial.print(accPitch);Serial.print(" Accel Roll: ");Serial.println(accRoll);
    oldGyroValues[0] = ((0.8d) * oldGyroValues[0] + (0.2d) * accPitch);
    oldGyroValues[1] = ((0.8d) * oldGyroValues[1] + (0.2d) * accRoll);
  }

  Serial.print("CmbP:"); Serial.print(oldGyroValues[0] - pitchZero); Serial.print("CmbR:"); Serial.println(oldGyroValues[1] - rollZero);
}

void loop() {
  calculatePitchAndRoll();

}
