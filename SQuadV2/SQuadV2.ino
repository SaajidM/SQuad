#include <Arduino.h>
#include <SIMU.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

/*
	SQuadV2
	Quadcopter Flight Controller
*/

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
  swMotor.attach(6);
  swMotor.attach(9);
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

void setAllMotorSpeeds(int setTo) {
  motorSpeed = setTo;
  adjustMotors();
}

void adjustMotors() {
  calculatePitchAndRoll();
  pitchPID.Compute();
  rollPID.Compute();
  double nwSpeed = motorSpeed - rollCtrl + pitchCtrl + yawZero + yawSet;
  double swSpeed = motorSpeed - rollCtrl - pitchCtrl - yawZero - yawSet;
  double seSpeed = motorSpeed + rollCtrl - pitchCtrl + yawZero + yawSet;
  double neSpeed = motorSpeed + rollCtrl + pitchCtrl - yawZero - yawSet;
  Serial.print("NW:"); Serial.print(nwSpeed);
  nwMotor.writeMicroseconds(nwSpeed);
  Serial.print("SW:"); Serial.print(swSpeed);
  swMotor.writeMicroseconds(swSpeed);
  Serial.print("SE:"); Serial.print(seSpeed);
  seMotor.writeMicroseconds(seSpeed);
  Serial.print("NE:"); Serial.println(neSpeed);
  neMotor.writeMicroseconds(neSpeed);
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
	pitchSet = getInt() + pitchZero; //Negative pitch v & Positive pitch ^
	rollSet = getInt() + rollZero; //Negative roll is \ & Positive roll is /
	setAllMotorSpeeds(getInt());
}

int getInt() {
  return ((Serial.read() << 8) | Serial.read());
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
      case 'I':										//Yaw Trim +
        yawZero += 5;
        break;
      case 'M':										//Yaw Trim -
        yawZero -= 5;
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
	
}
