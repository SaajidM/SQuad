/*
	Saajid Mohammed
	5040654
	4P78 - Project
	Inertial Measurement Unit Helper
*/

#include "SIMU.h"

SIMU::SIMU() {
    devAddr = 0x68;
}

/*
	initialize
	initializes the IMU with values needed 
*/
bool SIMU::initialize(){
	//reset();
	setClockSource(PWR_MGMT_CL_SR_XG);
	iniGyros(GYRO_FS_SEL_2000);
	iniAccel(ACCEL_AFS_SEL_4G);
	setSleep(false);
	return true;
}

/*
	reset
	resets the IMU to default settings
*/
void SIMU::reset(){
	writeToRegister(PWR_MGMT_1, 0b1, PWR_MGMT_RESET, 1);
	delay(150);
}

/*
	isDeviceConnected
	checks to see if the IMU is found
*/
bool SIMU::isDeviceConnected(){
	readRegister(WHO_AM_I, buffer);
    return buffer[0] == devAddr;
}

/*
	iniGyros
	receives the desired range you would like the gyro to capture
	and sets this value to the gyro
*/
void SIMU::iniGyros(uint8_t range){
	writeToRegister(GYRO_CONFIG, range, 4, 2);
}

/*
	iniAccel
	receives the desired range you would like the accelerometer to capture
	and sets this value to the accelerometer
*/
void SIMU::iniAccel(uint8_t range){
	writeToRegister(ACCEL_CONFIG, range, 4, 2);
}

/*
	setSleep
	turns the device on and off
*/
void SIMU::setSleep(bool setTo){
	if(setTo){writeToRegister(PWR_MGMT_1, 0b1, PWR_MGMT_SLEEP, 1);}
	else{writeToRegister(PWR_MGMT_1, 0b0, PWR_MGMT_SLEEP, 1);};
}

/*
	setClockSource
	sets the clock the device should use
*/
void SIMU::setClockSource(uint8_t source){
    writeToRegister(PWR_MGMT_1, source, 2, 3);
}

/*
	getGyroReadings
	reads the raw form of the gyro and places it into the passed array
*/
void SIMU::getGyroReadings(float *XYZ){
	readRegisters(GYRO_XOUT_H, 6, buffer);
    XYZ[0] = (float)((((int16_t)buffer[0]) << 8) | buffer[1]);
    XYZ[1] = (float)((((int16_t)buffer[2]) << 8) | buffer[3]);
    XYZ[2] = (float)((((int16_t)buffer[4]) << 8) | buffer[5]);
}

/*
	getAccelReadings
	reads the raw form of the accelerometer and places it into the passed array
*/
void SIMU::getAccelReadings(float *XYZ){
	readRegisters(ACCEL_XOUT_H, 6, buffer);
    XYZ[0] = (float)((((int16_t)buffer[0]) << 8) | buffer[1]);
    XYZ[1] = (float)((((int16_t)buffer[2]) << 8) | buffer[3]);
    XYZ[2] = (float)((((int16_t)buffer[4]) << 8) | buffer[5]);
}

/*
	getAllSixReadings
	reads the raw form of the gyro and accelerometer and places them into the array
*/
void SIMU::getAllSixReadings(float *ar){
	readRegisters(ACCEL_XOUT_H, 14, buffer);
    ar[0] = (float)((((int16_t)buffer[0]) << 8) | buffer[1]); //ACCEL X
    ar[1] = (float)((((int16_t)buffer[2]) << 8) | buffer[3]); //ACCEL Y
    ar[2] = (float)((((int16_t)buffer[4]) << 8) | buffer[5]); //ACCEL Z
	ar[3] = (float)((((int16_t)buffer[8]) << 8) | buffer[9]); //GYRO X --> Pitch
    ar[4] = (float)((((int16_t)buffer[10]) << 8) | buffer[11]); //GYRO Y --> Roll
    ar[5] = (float)((((int16_t)buffer[12]) << 8) | buffer[13]); //GYRO Z
}

/*
	readRegister
	reads the register given by thisRegister and places it in inToThis
*/
void SIMU::readRegister(uint8_t thisRegister, uint8_t *inToThis){
	Wire.beginTransmission(devAddr); //Get line
	Wire.write(thisRegister); //Sends which register it needs
	Wire.endTransmission(false);
	Wire.requestFrom(devAddr, (uint8_t)1); //Receives the byte stored in the desired register
	if(Wire.available()){ *inToThis = (uint8_t)Wire.read();}
	Wire.endTransmission(true);
}

/*
	readRegister
	reads a number of registers given by length starting at startRegister and places it in inToThis
*/
void SIMU::readRegisters(uint8_t startRegister, int8_t length, uint8_t *inToThis){
	for(int8_t i = 0; i < length; i++){
		readRegister(startRegister + i,&inToThis[i]);
	}
}

/*
	writeToRegister
	writes fromThis into thisRegister starting at start and going for length
*/
void SIMU::writeToRegister(uint8_t thisRegister, uint8_t fromThis,uint8_t start,uint8_t length){
	uint8_t temp,mask = 0b00000000;
	uint8_t moveOver = (start-(length-1)); //Number of bits needed to move over
	readRegister(thisRegister,&temp); //reads current register values
	for(uint8_t i = 0; i < length; i++){
		mask = mask | 1;
		if(i+1 != length){mask = mask << 1;} //creates a mask to zero out the bits we are writing to
	}
	mask = ~(mask << moveOver); 
	fromThis = fromThis << moveOver; //moves the value into the right position
	temp = (temp & mask) | fromThis; //clears out bits and writes desired value 
	Wire.beginTransmission(devAddr);
	Wire.write(thisRegister); //specifies register to write to 
	Wire.write(temp); //write byte into register
	Wire.endTransmission();
}

		