/*
 *  MPU9255.c
 *
 *  Created on: Mar 11, 2018
 *  Author: Logan Johnson
 *  
 *  @description: This is a simple library for the MPU9255 mostly in C but also 
 *   uses aspects from C++. It attempts to abstract most of its functions away 
 *   from a particular microcontroller, though its current form is for the 
 *   STM32F767zi. This code is adapted from a few sources including Baser Kandehir's 
 *   MPU6050 library and examples from STM32CubeMX. 
 *
 * Below is the copyright disclaimer: 
 *
 * COPYRIGHT(c) 2018 STMicroelectronics 
 * Copyright (c) 2015, Baser Kandehir, baser.kandehir@ieee.metu.edu.tr
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************  
 */


/*** Includes ***/
#include "MPU9255.h"

/*** Variables ***/
I2C i2c1(I2C1_SDA, I2C1_SCL);
float accelBiasFinal[3] = {0, 0, 0}; 
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

/*** Functions ***/

void getRPY(double *roll, double *pitch){
	int16_t accelData[3] = {0, 0, 0};
	//get accelData
	accelData[0] = readAccelX();
	accelData[1] = readAccelY();
	accelData[2] = readAccelZ();	
	
	accelX = accelData[0] * getRes(2.0) - accelBiasFinal[0];
	accelY = accelData[1] * getRes(2.0) - accelBiasFinal[1];
	accelZ = accelData[2] * getRes(2.0) - accelBiasFinal[2];
	
	int16_t gyroData[3] = {0, 0, 0};
	//get gyroData
	gyroData[0] = readGyroX();
	gyroData[1] = readGyroY();
	gyroData[2] = readGyroZ();
	
	gyroX = gyroData[0] * getRes(250.0);
	gyroY = gyroData[1] * getRes(250.0);
	gyroZ = gyroData[2] * getRes(250.0);
	
	float pitchAccel, rollAccel;
	
	*pitch += gyroX * (.001);
	*roll  += gyroY * (.001);
	
	
	pitchAccel = atan2(accelY, accelZ) * (180/PI);
	rollAccel = atan2(accelX, accelZ) * (180/PI);
	
	*pitch = *pitch * 0.98 + pitchAccel * 0.02;
    *roll  = *roll  * 0.98 + rollAccel  * 0.02;  

}

void calibrate(){
	uint8_t data[12];
	uint8_t gyroOffset[6];
	uint16_t packetCount, curCount;
	int32_t gyroBias[3] = {0, 0, 0};
	int32_t accelBias[3] = {0, 0, 0};
	double accelRes = getRes(2.0);
	double gyroRes = getRes(250.0);
	
	//From datasheet
	uint16_t accelSensitivity = 16384;
	//uint16_t gyroSensitivity = 131;
	
	//Configure the MPU for Calibration
	//Disable Interrupts
	write8MPURegister(MPU_INT_ENABLE , 0x0);
	
	//Make sure all Sensors are enabled
	write8MPURegister(MPU_PWR_MGMT_2, 0x0);
	
	//Turn off FIFO
	write8MPURegister(MPU_FIFO_EN , 0x0);
	
	//Enable internal 20 MHZ oscillator
	write8MPURegister(MPU_PWR_MGMT_2 , 0x0);
	
	//Disable I2C Master
	write8MPURegister(MPU_I2C_MST_CTRL , 0x0);
	
	//Disable all modes
	write8MPURegister(MPU_USER_CTRL , 0x0);
	
	//Reset FIFO
	write8MPURegister(MPU_USER_CTRL, 0x4);
	wait_ms(15);
	
	//Main MPU config for Low-Pass filter settings
	configureMPU(0x1);
	
	//Set Sample Rate
	write8MPURegister(MPU_SMPLRT_DIV, 0x0);
	
	//Configures the Gyro to the default +-250deg/s fullrange
	configureGyro(0x0);
	
	//Configures the Accel to the default +-2g fullrange
	configureAccel(0x0);
	
	//Enable FIFO Mode in user config
	write8MPURegister(MPU_USER_CTRL, 0x40);
	
	//Enable sensors to output to FIFO 
	write8MPURegister(MPU_FIFO_EN , 0x78);
	
	//Wait for ~40 mili seconds to give us 40 samples of each sensor
	//This will give us around 40 * 12 Bytes or ~480 which is < our 512 buffer 
	//Size
	wait_ms(40);
	
	//Disable FIFO Sensor output
	write8MPURegister(MPU_FIFO_EN , 0x00);
	packetCount = getFIFOCount() / 12;
	
	for(curCount = 0; curCount < packetCount; curCount++){
		uint16_t tempGyro[3] = {0, 0, 0};
	 	uint16_t tempAccel[3] = {0, 0, 0};
	 	readBytesMPU(MPU_FIFO_R_W, data, 12);
	 	
	 	//Remember sensor data is written to the FIFO in order of Register number
	 	//Lower coming first. Also the high byte comes first
	 	tempAccel[0] = (uint16_t)data[0]  << 8 | data[1];
	 	tempAccel[1] = (uint16_t)data[2]  << 8 | data[3];
	    tempAccel[2] = (uint16_t)data[4]  << 8 | data[5];
	    tempGyro[0]  = (uint16_t)data[6]  << 8 | data[7];
	    tempGyro[1]  = (uint16_t)data[8]  << 8 | data[9];
	    tempGyro[2]  = (uint16_t)data[10] << 8 | data[11];
	    
	    
	    //Sum the current tempGyro and tempAccel terms with previous ones
		accelBias[0] += tempAccel[0];
		accelBias[1] += tempAccel[1];
		accelBias[2] += tempAccel[2];
		gyroBias[0] += tempGyro[0];
		gyroBias[1] += tempGyro[1];
		gyroBias[2] += tempGyro[2];
	 	
	}
	
	//Average the biases based on the number of sample packet
	accelBias[0] /= (int32_t)packetCount;
	accelBias[1] /= (int32_t)packetCount;
	accelBias[2] /= (int32_t)packetCount;
	gyroBias[0] /= (int32_t)packetCount;
	gyroBias[1] /= (int32_t)packetCount;
	gyroBias[2] /= (int32_t)packetCount;
	
	/* Remove gravity from the z-axis accelerometer bias calculation */  
    if(accelBias[2] > 0) {
    	accelBias[2] -= (int32_t)accelSensitivity;
    } else {
    	accelBias[2] += (int32_t)accelSensitivity;
    }
	
	//Set acceleration biases to be used for later
	accelBiasFinal[0] = accelBias[0]*accelRes;
    accelBiasFinal[1] = accelBias[1]*accelRes;
    accelBiasFinal[2] = accelBias[2]*accelRes; 
	
	//Calculate offsets by dividing by 4 to get 32.9 LSB per deg/s.
	//This conforms to expected bias input format
	gyroOffset[0] = ((-gyroBias[0] / 4) >> 8) & 0xff;
	gyroOffset[1] = (-gyroBias[0] / 4) & 0xff;
	gyroOffset[2] = ((-gyroBias[1] / 4) >> 8) & 0xff;
	gyroOffset[3] = (-gyroBias[1] / 4) & 0xff;
	gyroOffset[4] = ((-gyroBias[2] / 4) >> 8) & 0xff;
	gyroOffset[5] = (-gyroBias[2] / 4) & 0xff;
	
	//Write offsets to gyro registers
	write8MPURegister(MPU_XG_OFFSET_H, gyroOffset[0]);
	write8MPURegister(MPU_XG_OFFSET_L, gyroOffset[1]);
	write8MPURegister(MPU_YG_OFFSET_H, gyroOffset[2]);
	write8MPURegister(MPU_YG_OFFSET_L, gyroOffset[3]);
	write8MPURegister(MPU_ZG_OFFSET_H, gyroOffset[4]);
	write8MPURegister(MPU_ZG_OFFSET_L, gyroOffset[5]);
	
	configureMPU(0x2);
}

void reset(){
	write8MPURegister(MPU_PWR_MGMT_1, 0x80);	
	
	//Wait at least 100ms for device to reach steady state
	wait_ms(100);	
}

double getRes(double fullRange){
	// Resolution = range/2^(n-1) where n is the width of your sample data
	return(fullRange / 32768.0);	
}

void init(){
								
    //Speedy I2C (400khz)
	i2c1.frequency(400000);
	
	//Wake up the device by writing 0's to the sleep bits
	write8MPURegister(MPU_PWR_MGMT_1, 0x00);	

	//Wait at least 100ms for device to reach steady state
	wait_ms(100);
	
	//MPU chooses PLL clock input if ready else sticks with internal oscillator	
	write8MPURegister(MPU_PWR_MGMT_1, 0x01);
	
	//Main MPU config
	configureMPU(0x3);
	
	//Configure the sample rate divider to ~4
	write8MPURegister(MPU_SMPLRT_DIV, 0x04);	
	
	//Configures the Gyro to the default +-250deg/s fullrange
	configureGyro(0x0);
	
	//Configures the Accel to the default +-2g fullrange
	configureAccel(0x0);
}

uint8_t whoAmI(void){
	// Return the value read from the WHOAMI register
	return(read8MPURegister(MPU_WHO_AM_I));
}

char read8MPURegister(uint8_t regAddr){
	uint8_t addr = MPU_ADDR_LOW;
	char readVal[1];
    char regAddress[1];
    regAddress[0] = regAddr;
    i2c1.write(addr, regAddress, 1, 1);
    i2c1.read(addr, readVal, 1, 0);
    return(readVal[0]);    
}

void write8MPURegister(uint8_t regAddr, uint8_t toWrite){
	uint8_t addr = MPU_ADDR_LOW;
    char write[2];
    write[0] = regAddr;
    write[1] = toWrite;
    i2c1.write(addr, write, 2, 0);
}

void readBytesMPU(uint8_t regAddr, uint8_t *byteArray, uint8_t numBytes){
	//Without any external sensors the number of data[numOfdataEntries]
	//should be at most 20 (9 *2 for each sensor + 2 for Temp Sensor);
	char data[20], toWrite[1];
	uint8_t addr = MPU_ADDR_LOW;
	toWrite[0] = regAddr;
	i2c1.write(addr, toWrite, 1, 1);
	i2c1.read(addr, data, numBytes, 0);
	for(int i = 0; i < numBytes; i++){
		byteArray[i] = data[i];	
	}
	
}

int16_t readGyroX(){
	int8_t xHigh = read8MPURegister(MPU_GYRO_XOUT_H);
	int8_t xLow  = read8MPURegister(MPU_GYRO_XOUT_L);
	int16_t x = (int16_t)xHigh << 8 | xLow;
	return x;
}

int16_t readGyroY(){
	int8_t yHigh = read8MPURegister(MPU_GYRO_YOUT_H);
	int8_t yLow  = read8MPURegister(MPU_GYRO_YOUT_L);
	int16_t y = (int16_t)yHigh << 8 | yLow;
	return y;
}

int16_t readGyroZ(){
	int8_t zHigh = read8MPURegister(MPU_GYRO_ZOUT_H);
	int8_t zLow  = read8MPURegister(MPU_GYRO_ZOUT_L);
	int16_t z = (int16_t)zHigh << 8 | zLow;
	return z;
}

int16_t readAccelX(void){
	int8_t xHigh = read8MPURegister(MPU_ACCEL_XOUT_H);
	int8_t xLow = read8MPURegister(MPU_ACCEL_XOUT_L);
	int16_t x = (int16_t)xHigh << 8 | xLow;
	return x;
}

int16_t readAccelY(void){
	int8_t yHigh = read8MPURegister(MPU_ACCEL_YOUT_H);
	int8_t yLow = read8MPURegister(MPU_ACCEL_YOUT_L);
	int16_t y = (int16_t)yHigh << 8 | yLow;
	return y;
}

int16_t readAccelZ(void){
	int8_t zHigh = read8MPURegister(MPU_ACCEL_ZOUT_H);
	int8_t zLow = read8MPURegister(MPU_ACCEL_ZOUT_L);
	int16_t z = (int16_t)zHigh << 8 | zLow;
	return z;
}


void configureMPU(uint8_t configMPU){

	write8MPURegister(MPU_CONFIG, configMPU);
}

void configureAccel(uint8_t configAccel){
	
	write8MPURegister(MPU_ACCEL_CONFIG, configAccel);	
}

void configureGyro(uint8_t configGyro){

	write8MPURegister(MPU_GYRO_CONFIG, configGyro);
}

void configureInterruptPin(uint8_t configInt){

	write8MPURegister(MPU_INT_PIN_CFG, configInt);
}

void configureUserControl(uint8_t userControlConfig){

	write8MPURegister(MPU_USER_CTRL, userControlConfig);
}

void enableFIFO(uint8_t enFIFO){

	write8MPURegister(MPU_FIFO_EN, enFIFO);
}

void enableInterrupt(uint8_t enInt){

	write8MPURegister(MPU_INT_ENABLE, enInt);
}

uint8_t readInterruptStatus(){

	return(read8MPURegister(MPU_INT_STATUS));
}

uint16_t getFIFOCount(){
	int8_t countLow = read8MPURegister(MPU_FIFO_COUNTL);
	int8_t countHigh = read8MPURegister(MPU_FIFO_COUNTH);
	uint16_t count = (uint16_t)countHigh << 8 | countLow;
	return(count);
}