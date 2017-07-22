#include "mpu6050.h"
#include "i2cctl.h"
#include <iostream>
#include <string>
#include <stdio.h>


void printBinary(char c);


int main() {
	std::cout << "|| Welcome to MPU6050 3 Axis Accelerometer/Gyro Software ||\n";
	std::cout << "|     Written by: Jack Wagner (2017)                      |\n";
	
	powerSensor(true);
	
	gyroTest();
	
	powerSensor(false);


}



void readToFile (){
	freopen("ReadOutput.txt","w",stdout);	
	uint8_t data[1];
	for (int i = 0; i < 255; i++){
	i2c_read(mpuAddress, i, data, 1);
	std::cout << "\nI2C Read " << i << " : ";
	printBinary(data[0]);
	std::cout << "\n\n";
	}
	fclose(stdout);
	}


void printBinary(char c) {
	for (int i = 7 ; i >= 0 ; i--) {
		std::cout <<((c & (1 << i))? '1' : '0');
	}
}




