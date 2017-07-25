#include "mpu6050.h"
#include "i2cctl.h"
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <unistd.h>
uint16_t GX, GY, GZ, AX, AY, AZ;



void powerSensor(bool on) {
		
	uint8_t dataPwr[3] = {PWR_MGMT_1, 0x00, 0x00};	
	
	printf("\n\nAttempting to change power\n");

	if (on) {
		dataPwr[1] = 0b00000000; //first byte: registers reset, disable sleep, cycltemp disabled,
								           //temp enabled, clock = internal
								           //second byte: wake-up frequency set to 1.25 Hz, no standby set
	} else {
		dataPwr[1] = 0b01000000; //first byte: registers reset, enable sleep, internal clock
									   //second byte: wake up frequency set to 1.25 Hz, no standby set
	}
	uint8_t data[1];
	i2c_read(mpuAddress, PWR_MGMT_1, data, 1);
	printf("%d\n",data[0]);


	if (i2c_write(mpuAddress, dataPwr, 2) == 0) {
		if (on) printf("Power on succeeded\n");	
		if (!on) printf("Power off succeeded\n");
	} else {
		if (on) printf("Power on failure\n");
		if (!on) printf("Power off failure\n");
	}
	
	if (on) {

		for(int i = 0; i < 10; i++)
		usleep(10000);   //delay to allow sensors to turn on

		dataPwr[1] = 0x01;		//use gyro clock (more stable)
		if (i2c_write(mpuAddress, dataPwr, 2) == 0) {
			printf("Clock successfully set to gyrox ppl\n");
		} else {
			printf("Clock set failure\n");
		}
		
	}
	uint8_t read[1] = {0x00};
	i2c_read(mpuAddress, PWR_MGMT_1, read, 1);
	printf("%d", read[0]);
	printf("\n");

}


void gyroTest() {
	//begin by setting full-scale range to 250dps through FS_SEL set at 0
	
	//Self Test Response = gyro output with self-test enabled - gyro output without self-test enabled
	
	printf("\n\nAttempting to self-test gyroscope\n");

 


	uint8_t dataGConfigE[2] = { GYRO_CONFIG, 0xE0 }; //self-test enabled, still 250 dps
	if (i2c_write(mpuAddress, dataGConfigE, 2) == 0) {
		printf("Gyro successfully set to 250 dps, self-test enabled\n");
	} else { 
		printf("Failure to set Gyro to 250 dps, self-test enabled\n");
	}
	
	uint16_t outputTestOn[3] = { 0x00, 0x00, 0x00 };
	
	gyroRead(outputTestOn);



	

	//Now we must gather Factory Trim value of the self test response, FT[Xg],FT[Yg],FT[Zg]
        uint8_t gTest[3];
	if (i2c_read(mpuAddress, SELF_TEST_X, gTest, 3) == 0 ) {
		printf("Self_Test values (x,y,z) successfully read\n");
		printf("  X:%f  ", gTest[0] & 0x1F);
		printf("  Y:%f  ", gTest[1] & 0x1F);
		printf("  Z:%f  \n",gTest[2] & 0x1F);
	} else {
		printf("Factory Trim values failed to read\n");
	}
	gTest[0] = gTest[0] & 0x1F;
	gTest[1] = gTest[1] & 0x1F;
	gTest[2] = gTest[2] & 0x1F;

	double ft[3] = { 25*131*pow(1.046, (float)gTest[0]-1) , 
			-25*131*pow(1.046, (float)gTest[1]-1 ) ,
			 25*131*pow(1.046, (float)gTest[2]-1 ) };
	printf("  Factory Trim x: %f  ", ft[0]);
	printf("  Factory Trim y: %f  ", ft[1]);
	printf("  Factory Trim z: %f \n ", ft[2]);	
	

	
	uint8_t dataGConfig[2] = { GYRO_CONFIG, 0b00000000 }; //3 most significant bits sets self-test for gyro, next 2 significant set gyro DPS
	if (i2c_write(mpuAddress, dataGConfig, 2) == 0) {
		printf("Gyro successfully set to 250 dps, self-test disabled\n");
	} else {
		printf("Failure to set Gyro to 250 dps, self-test disabled\n");
	}
	
	uint16_t outputTestOff[3] = { GX, GY, GZ };
	
	gyroRead(outputTestOff);



	uint8_t SelfTestResponse[3] = { outputTestOn[0] - outputTestOff[0], outputTestOn[1] - outputTestOff[1], outputTestOn[2] - outputTestOff[2] };
	printf("\n\nSTR %d:%d:%d\n\n", SelfTestResponse[0], SelfTestResponse[1], SelfTestResponse[2]);	
	

	double change[3] = { ((SelfTestResponse[0]-ft[0])/ft[0]),((SelfTestResponse[1]-ft[1])/ft[1]),((SelfTestResponse[2]-ft[2])/ft[2]) };
	
	printf("\n\nCHANGE VALUES:  %f  ::  %f  ::  %f \n\n",change[0]*100+100,change[1]*100+100,change[2]*100+100);	
	

	for(int i = 0; i <= 2; i++) {
		if (std::abs(change[i]*100)-100 <= 14) {
			printf("Self-Test successful for Axis %d\n",i+1);
		} else {
			printf("Self-Test failure for %d Axis %d\n",i+1);
		
		}
	}

	printf("\n");


	
} 

void config(short EXT_SYNC_SET, short DLPF_CFG) {
	
	printf("\n\nAttempting to configure sensor\n");
	
	EXT_SYNC_SET = EXT_SYNC_SET << 3; //bits 3-5 for sync
					  //dec: 1 = temp out
					  //  	 2 = gyro x out, 3 = gyro y out, 4 = gyro z out
					  //     5 = accel x out, 6 = accel y out, 7 = accel z out

					  //dlpf: 1 (largest bandwith for a/g, smallest delay) => 6 (smallest bandwith for a/g, largest delay)
	
	uint8_t dataConfig[2] = {CONFIG, DLPF_CFG+EXT_SYNC_SET};
	
	
	if( i2c_write(mpuAddress, dataConfig, 2) == 0) {
		printf("Configuration successfully set\n");
	} else {
		printf("Configuration failure\n");
	}
	
	printf("\n");

}

void gyroRead(uint16_t *data) {
	
	uint8_t raw[6];
	
	printf("\n\nAttempting to read gyro measurement\n");
	
	if (i2c_read(mpuAddress, GYRO_XOUT_H, raw, 6) == 0 ) {
		printf("Successfully read gyro measurement\n");
	} else {
		printf("Failed to read gyro measurement\n");
	}
	
	data[0] = ((uint16_t)raw[0]<<8) | (uint16_t)raw[1];

	data[1] = ((uint16_t)raw[2]<<8) | (uint16_t)raw[3];

	data[2] = ((uint16_t)raw[4]<<8) | (uint16_t)raw[5];	
	
	printf("%d:%d:%d", data[0], data[1], data[2]);

	printf("\n");
	
}

void accelRead(uint16_t *data){

	uint8_t raw[6];

	if (i2c_read(mpuAddress, ACCEL_XOUT_H, raw,  6) == 0) 
	{
		printf("Successfuly read accelerometer\n");	
	} else {
		printf("Failed to read accelerometer\n");
	}
	
	data[0] = ((uint16_t)raw[0]<<8) | (uint16_t)raw[1];
	
	data[1] = ((uint16_t)raw[2]<<8) | (uint16_t)raw[3];
	
	data[2] = ((uint16_t)raw[4]<<8) | (uint16_t)raw[5];

	printf("%d:%d:%d",data[0], data[1], data[2]);
	
	printf("\n"); 

}

void accelTest() {

	//begin by setting full-scale range to +/- 8g through FS_SEL set at 0
	
	//Self Test Response = accel output with self-test enabled - accel output without self-test enabled
	
	printf("\n\nAttempting to self-test accelscope\n");

 


	uint8_t dataAConfigE[2] = { ACCEL_CONFIG, 0xF0 }; //self-test enabled, still 250 dps
	if (i2c_write(mpuAddress, dataAConfigE, 2) == 0) {
		printf("Accelerometer successfully set to +/- 8g, self-test enabled\n");
	} else { 
		printf("Failure to set accelerometer to +/- 8g, self-test enabled\n");
	}
	
	uint16_t outputTestOn[3] = { 0x00, 0x00, 0x00 };
	
	accelRead(outputTestOn);



	

	//Now we must gather Factory Trim value of the self test response, FT[Xg],FT[Yg],FT[Zg]
        uint8_t aTest[4];
	if (i2c_read(mpuAddress, SELF_TEST_X, aTest, 4) == 0 ) {

		aTest[0] = aTest[0] >> 3  |  (aTest[3] & 0x30) >> 4;
		aTest[1] = aTest[1] >> 3  |  (aTest[3] & 0x0C) >> 2;
		aTest[2] = aTest[2] >> 3  |  (aTest[3] & 0x03) >> 0;


		printf("Self_Test values (x,y,z) successfully read\n");
		printf("  X:%f  ", aTest[0] );
		printf("  Y:%f  ", aTest[1] );
		printf("  Z:%f  \n", aTest[2] );
	} else {
		printf("Factory Trim values failed to read\n");
	}
	


	double ft[3] = { 4096.0f*0.34f*(pow( (0.92f/0.34f), ((aTest[0] - 1.0f)/30.0f))),
		        	4096.0f*0.34f*(pow( (0.92f/0.34f), ((aTest[1] - 1.0f)/30.0f))),
			4096.0f*0.34f*(pow( (0.92f/0.34f), ((aTest[2] - 1.0f)/30.0f))) 
		       };

	printf("  Factory Trim x: %f  ", ft[0]);
	printf("  Factory Trim y: %f  ", ft[1]);
	printf("  Factory Trim z: %f \n ", ft[2]);	
	

	
	uint8_t dataAConfig[2] = { ACCEL_CONFIG, 0b00000000 }; //3 most significant bits sets self-test for accel, next 2 significant set accel g
	if (i2c_write(mpuAddress, dataAConfig, 2) == 0) {
		printf("Accelerometer successfully set to +/- 8g, self-test disabled\n");
	} else {
		printf("Failure to set accelerometer to +/- 8g, self-test disabled\n");
	}
	
	uint16_t outputTestOff[3] = { GX, GY, GZ };
	
	accelRead(outputTestOff);



	uint8_t SelfTestResponse[3] = { outputTestOn[0] - outputTestOff[0], outputTestOn[1] - outputTestOff[1], outputTestOn[2] - outputTestOff[2] };
	printf("\n\nSTR %d:%d:%d\n\n", SelfTestResponse[0], SelfTestResponse[1], SelfTestResponse[2]);	
	

	double change[3] = { ((SelfTestResponse[0]-ft[0])/ft[0]),((SelfTestResponse[1]-ft[1])/ft[1]),((SelfTestResponse[2]-ft[2])/ft[2]) };
	
	printf("\n\nCHANGE VALUES:  %f  ::  %f  ::  %f \n\n",change[0]*100+100,change[1]*100+100,change[2]*100+100);	
	

	for(int i = 0; i <= 2; i++) {
		if (std::abs(change[i]*100)-100 <= 14) {
			printf("Self-Test successful for Axis %d\n",i+1);
		} else {
			printf("Self-Test failure for %d Axis %d\n",i+1);
		}
	}

	printf("\n");
		

}


void calibrate(){

}	
 
