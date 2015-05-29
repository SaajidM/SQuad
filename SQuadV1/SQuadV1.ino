#include <Arduino.h>
#include <SIMU.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

SIMU imu;
float toDeg = 57.2957795131;
double oldGyroValues[2] = {0, 0};
unsigned long oldTime = 0;

double pitchZero, rollZero, yawZero = 0;
double pitchCtrl, rollCtrl = 0;
double pitchSet, rollSet, yawSet = 0;
double lowAdjustBound = -1000;
double highAdjustBound = 1000;
PID pitchPID(&oldGyroValues[0], &pitchCtrl, &pitchSet, 1.5, 0, .2, DIRECT);
PID rollPID(&oldGyroValues[1], &rollCtrl, &rollSet, 1.5, 0, .2, DIRECT);

Servo nwMotor, swMotor, seMotor, neMotor;
int motorSpeed = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  while (!Serial);

  Serial.println("Awaiting Initialization");
  char cIn = getCharacter();
  if (cIn == 'S') {
    imu.initialize();
    iniPID();
    cIn = getCharacter();
    if (cIn == 'T') {
      calibrateAngles();
      cIn = getCharacter();
      if (cIn == 'R') {
        attachMotors();
      } else {
        halt();
      }
    } else {
      halt();
    }
  } else {
    halt();
  }
}

void attachMotors() {
  nwMotor.attach(3);
  swMotor.attach(5);
  seMotor.attach(10);
  neMotor.attach(11);
  setAllMotorSpeeds(700);
  delay(3000);

  Serial.println("Motors Attached");
}

void iniPID() {
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(lowAdjustBound, highAdjustBound);
  pitchPID.SetSampleTime(20);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(lowAdjustBound, highAdjustBound);
  rollPID.SetSampleTime(20);
  
  Serial.println("PID Initialized");
}

void halt() {
  Serial.println("Execution Halted");
  while (true);
}

char getCharacter() {
  while (Serial.available() == 0);
  return Serial.read();
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

  Serial.print("Combined Pitch: "); Serial.print(oldGyroValues[0] - pitchZero); Serial.print(" Combined Roll: "); Serial.println(oldGyroValues[1] - rollZero);
}

void setAllMotorSpeeds(int setTo) {
  motorSpeed = setTo;
  adjustMotors();
}

void adjustMotors() {
  calculatePitchAndRoll();
  pitchPID.Compute();
  rollPID.Compute();
  Serial.print("NW: "); Serial.print(motorSpeed + rollCtrl + pitchCtrl + yawZero + yawSet);
  nwMotor.writeMicroseconds(motorSpeed + rollCtrl + pitchCtrl + yawZero + yawSet);
  Serial.print(" SW: "); Serial.print(motorSpeed + rollCtrl - pitchCtrl - yawZero - yawSet);
  swMotor.writeMicroseconds(motorSpeed + rollCtrl - pitchCtrl - yawZero - yawSet);
  Serial.print(" SE: "); Serial.print(motorSpeed - rollCtrl - pitchCtrl + yawZero + yawSet);
  seMotor.writeMicroseconds(motorSpeed - rollCtrl - pitchCtrl + yawZero + yawSet);
  Serial.print(" NE: "); Serial.println(motorSpeed - rollCtrl + pitchCtrl - yawZero - yawSet);
  neMotor.writeMicroseconds(motorSpeed - rollCtrl + pitchCtrl - yawZero - yawSet);
}

void shutdownSequence() {
  for (int i = motorSpeed; i >= 700; i = i - 10) {
    setAllMotorSpeeds(i);
  }
  halt();
}

void getPRT() {
  Serial.println("Req PRT");
  while (Serial.available() != 6);
  int pitchAngle = getInt();
  if (abs(pitchAngle) <= 20)
    pitchSet = pitchAngle + pitchZero;
  int rollAngle = getInt();
  if (abs(rollAngle) <= 20)
    rollSet = rollAngle + rollZero;
  int throttle = getInt();
  setAllMotorSpeeds(throttle);
}

int getInt() {
  return (Serial.read() << 8 | Serial.read());
}

void loop() {

  if (Serial.available() > 0) {
    char cIn = Serial.read();
    switch (cIn) {
      case 'Y':										//Yaw
        yawSet = Serial.parseInt();
        break;
      case 'C':										//Calibrate
        if (motorSpeed <= 700)
          calibrateAngles();
        break;
      case 'I':            							//Yaw Trim +
        yawZero += 10;
        break;
      case 'M':										//Yaw Trim -
        yawZero -= 10;
        break;
      case 'E':										//Emergence Stop
        setAllMotorSpeeds(0);
        halt();
        break;
    }
  }

  getPRT();

  adjustMotors();
  if ((abs(oldGyroValues[0]) >= 50) || (abs(oldGyroValues[1]) >= 50))
    shutdownSequence();

  //Serial.print("Pitch Control Out: ");Serial.print(pitchCtrl);
  //Serial.print(" Roll Control Out: ");Serial.println(rollCtrl);
}
