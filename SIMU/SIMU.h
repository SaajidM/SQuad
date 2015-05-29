/*
	Saajid Mohammed
	5040654
	4P78 - Project
	Inertial Measurement Unit Helper
*/

#include "Arduino.h"
#include "Wire.h"

//Define Registry Addresses and Indexes Needed

#define CONFIG				0x1A
#define DLPF_CFG_256		0x00
#define	DLPF_CFG_42			0x03
#define DLPF_CFG_5			0x06

#define SMPRT_DIV			0x19

#define GYRO_CONFIG 		0x1B
#define GYRO_FS_SEL_250 	0x00
#define GYRO_FS_SEL_500 	0x01
#define GYRO_FS_SEL_1000 	0x02
#define GYRO_FS_SEL_2000 	0x03	

#define ACCEL_CONFIG 		0x1C
#define ACCEL_AFS_SEL_2G 	0x00
#define ACCEL_AFS_SEL_4G 	0x01
#define ACCEL_AFS_SEL_8G 	0x02
#define ACCEL_AFS_SEL_12G 	0x03

#define PWR_MGMT_1			0x6B
#define PWR_MGMT_SLEEP		6
#define PWR_MGMT_RESET		7
#define PWR_MGMT_CL_SR_IN	0x00
#define PWR_MGMT_CL_SR_XG	0x01	

#define WHO_AM_I			0x75

#define GYRO_XOUT_H			0x43
#define ACCEL_XOUT_H		0x3B


class SIMU {
	public:
		SIMU();
		bool initialize();

		void getGyroReadings(float *);
		void getAccelReadings(float *);
		void getAllSixReadings(float *ar);
		
		bool isDeviceConnected();
		
	private:
		void iniGyros(uint8_t range);
		void iniAccel(uint8_t range);
		void setSleep(bool setTo);
		void setClockSource(uint8_t source);
		void reset();
		void readRegister(uint8_t thisRegister, uint8_t *inToThis);
		void readRegisters(uint8_t startRegister, int8_t length, uint8_t *inToThis);
		void writeToRegister(uint8_t thisRegister, uint8_t fromThis,uint8_t start,uint8_t length);
		uint8_t buffer[16];
		uint8_t devAddr;
};
