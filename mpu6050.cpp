#include "mpu6050.h"
#include "i2cctl.h"
#include <iostream>
#include <cmath>
#include <stdio.h>


void powerSensor(bool on) {
	
	uint8_t dataPwr[3] = {PWR_MGMT_1, 0x00, 0x00};	
	
	printf("\n");

	if (on) {
		dataPwr[1] = 0b10100001; //first byte: registers reset, disable sleep, cycltemp enabled,
								           //temp enabled, clock = PPL x axis gyro (reccommended for stability)
								           //second byte: wake-up frequency set to 1.25 Hz, no standby set
	} else {
		dataPwr[1] = 0b11000000; //first byte: registers reset, enable sleep, internal clock
									   //second byte: wake up frequency set to 1.25 Hz, no standby set
	}

	int powerSuccess = i2c_write(mpuAddress, dataPwr, 2);
	if (powerSuccess == 0) {
		if (on) printf("Power on succeeded\n");	
		if (!on) printf("Power off succeeded\n");
	} else {
		if (on) printf("Power on failure\n");
		if (!on) printf("Power off failure\n");
	}
	
	printf("\n");

}


void gyroTest() {
	//begin by setting full-scale range to 250dps through FS_SEL set at 0
	
	//Self Test Response = gyro output with self-test enabled - gyro output without self-test enabled

	
	uint8_t dataGConfig[2] = { GYRO_CONFIG, 0b00000000 }; //3 most significant bits sets self-test for gyro, next 2 significant set gyro DPS
	int gconfigSuc = i2c_write(mpuAddress, dataGConfig, 1);
	if (gconfigSuc == 0) {
		printf("Gyro successfully set to 250 dps, self-test disabled\n");
	} else {
		printf("Failure to set Gyro to 250 dps, self-test disabled\n");
	}

	

	//Now we must gather Factory Trim value of the self test response, FT[Xg],FT[Yg],FT[Zg]
	uint8_t gTest[3];
	if (i2c_read(mpuAddress, SELF_TEST_X, gTest, 3) == 0 ) {
		printf("Self_Test values (x,y,z) successfully read\n");
		printf("  X:%d  ",extracted(gTest[0],0,5));
		printf("  Y:%d  ",extracted(gTest[1],0,5));
		printf("  Z:%d  \n",extracted(gTest[2],0,5));
	} else {
		printf("Factory Trim values failed to read\n");
	}
	

	double ft[3] = { 25*131*pow(1.046, extracted(gTest[0], 0, 5)-1 ) , 
			-25*131*pow(1.046, extracted(gTest[1], 0, 5)-1 ) ,
			 25*131*pow(1.046, extracted(gTest[2], 0, 5)-1 ) };
	printf("  Factory Trim x: %f  ", ft[0]);
	printf("  Factory Trim y: %f  ", ft[1]);
	printf("  Factory Trim z: %f \n ", ft[2]);	
	


	
} 



unsigned short extracted(unsigned short value, int begin, int end) {
	unsigned short mask = (1 << (end - begin ) ) -1 ;
	return (value >> begin) & mask;
}

