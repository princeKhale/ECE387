/*
 * MPU9255.h
 *
 *  Created on: Mar 11, 2018
 *      Author: Logan Johnson
 *
 *  @description: Header file for the Mpu9255 library it includes the necessary 
 *  includes, variables, and function prototypes for it.
 *
 */

#ifndef MPU9255_H_
#define MPU9255_H_

/*** Includes  ***/
#include <stdint.h>
#include <math.h>
#include "mbed.h"

/*** Variables ***/ 
#define I2C1_SDA PB_9
#define I2C1_SCL PB_8
#define PI 3.14159265358979323846

/***  MPU Variable Definitions ***/

// MPU Address when Pin AD0 is pulled low
#define MPU_ADDR_LOW           0xD0
/******** True Value is 0x68 but the LL API has a small bug this is a fix 
          for now...
*********/

/***  ------------------------------- ***/
/***  MPU9050/9055 Register Addresses ***/
/***  ------------------------------- ***/

/* LEGEND ---------------------- */
/* (R/W) - Read/Write accessible */
/* (R)   - Read Only             */
/* ----------------------------- */

/*** Gyroscope Self Test Registers  (R/W) ***/
#define MPU_SELF_TEST_X_GYRO   0x00
#define MPU_SELF_TEST_Y_GYRO   0x01
#define MPU_SELF_TEST_Z_GYRO   0x02

/*** Accelerometer Self Test Registers (R/W) ***/
#define MPU_SELF_TEST_X_ACCEL  0x0D
#define MPU_SELF_TEST_Y_ACCEL  0x0E
#define MPU_SELF_TEST_Z_ACCEL  0x0F

/*** Gyroscope Offset Registers (R/W) ***/
#define MPU_XG_OFFSET_H        0x13
#define MPU_XG_OFFSET_L        0x14
#define MPU_YG_OFFSET_H        0x15
#define MPU_YG_OFFSET_L        0x16
#define MPU_ZG_OFFSET_H        0x17
#define MPU_ZG_OFFSET_L        0x18

/*** Sample Rate Divider  (R/W) ***/
#define MPU_SMPLRT_DIV         0x19

/*** Configuration register for MPU (R/W) ***/
#define MPU_CONFIG             0x1A

/*** Gyroscope Configuration  (R/W) ***/
#define MPU_GYRO_CONFIG  	   0x1B

/*** Accelerometer Configuration 1 (R/W) ***/
#define MPU_ACCEL_CONFIG       0x1C

/*** Accelerometer Configuration 2 (R/W) ***/
#define MPU_ACCEL_CONFIG2      0x1D

/*** Low Power Accelerometer Output data Rate Control (R/W) ***/
#define MPU_LP_ACCEL_ODR       0x1E

/*** Wake-on Motion Threshold (R/W) ***/
#define MPU_WOM_THR            0x1F

/*** Enable certain sensors to output to the FIFO buffer (R/W) ***/
#define MPU_FIFO_EN            0x23

/*** Control the Mpu9255 as a I2C master to slave 3rd party sensors (R/W) ***/ 
#define MPU_I2C_MST_CTRL       0x24

/*** Configurion for the first slave device (R/W) ***/
#define MPU_I2C_SLV0_ADDR      0x25
#define MPU_I2C_SLV0_REG       0x26
#define MPU_I2C_SLV0_CTRL      0x27

/*** Configurion for the first slave device (R/W) ***/
#define MPU_I2C_SLV1_ADDR      0x28
#define MPU_I2C_SLV1_REG       0x29
#define MPU_I2C_SLV1_CTRL      0x2A

/*** Configurion for the first slave device (R/W) ***/
#define MPU_I2C_SLV2_ADDR      0x2B
#define MPU_I2C_SLV2_REG       0x2C
#define MPU_I2C_SLV2_CTRL      0x2D

/*** Configurion for the first slave device (R/W) ***/
#define MPU_I2C_SLV3_ADDR      0x2E
#define MPU_I2C_SLV3_REG       0x2F
#define MPU_I2C_SLV3_CTRL      0x30

/*** Configurion for the first slave device (R/W) _DI is (R) ***/
#define MPU_I2C_SLV4_ADDR      0x31
#define MPU_I2C_SLV4_REG       0x32
#define MPU_I2C_SLV4_DO        0x33
#define MPU_I2C_SLV4_CTRL      0x34
#define MPU_I2C_SLV4_DI        0x35

/*** Get the I2C master status of the MPU9255 (R) ***/
#define MPU_I2C_MST_STATUS     0x36

/*** Configures the interrupt pin (R/W) ***/
#define MPU_INT_PIN_CFG        0x37

/***  Enables different interrupts and lets you see which one triggered (R/W)***/
#define MPU_INT_ENABLE         0x38
#define MPU_INT_STATUS         0x3A           //(R)

/***  Accelerometer Data out (R) ***/
#define MPU_ACCEL_XOUT_H       0x3B
#define MPU_ACCEL_XOUT_L       0x3C
#define MPU_ACCEL_YOUT_H       0x3D
#define MPU_ACCEL_YOUT_L       0x3E
#define MPU_ACCEL_ZOUT_H       0x3F
#define MPU_ACCEL_ZOUT_L       0x40

/*** Tempature Data out (R) ***/
#define MPU_TEMP_OUT_H         0x41
#define MPU_TEMP_OUT_L         0x42

/*** Gyroscope data out (R) ***/
#define MPU_GYRO_XOUT_H        0x43
#define MPU_GYRO_XOUT_L        0x44
#define MPU_GYRO_YOUT_H        0x45
#define MPU_GYRO_YOUT_L        0x46
#define MPU_GYRO_ZOUT_H        0x47
#define MPU_GYRO_ZOUT_L        0x48

/*** External Sensor Data (R) ***/
#define MPU_EXT_SENS_DATA_00   0x49
#define MPU_EXT_SENS_DATA_01   0x4A
#define MPU_EXT_SENS_DATA_02   0x4B
#define MPU_EXT_SENS_DATA_03   0x4C
#define MPU_EXT_SENS_DATA_04   0x4D
#define MPU_EXT_SENS_DATA_05   0x4E
#define MPU_EXT_SENS_DATA_06   0x4F
#define MPU_EXT_SENS_DATA_07   0x50
#define MPU_EXT_SENS_DATA_08   0x51
#define MPU_EXT_SENS_DATA_09   0x52
#define MPU_EXT_SENS_DATA_10   0x53
#define MPU_EXT_SENS_DATA_11   0x54
#define MPU_EXT_SENS_DATA_12   0x55
#define MPU_EXT_SENS_DATA_13   0x56
#define MPU_EXT_SENS_DATA_14   0x57
#define MPU_EXT_SENS_DATA_15   0x58
#define MPU_EXT_SENS_DATA_16   0x59
#define MPU_EXT_SENS_DATA_17   0x5A
#define MPU_EXT_SENS_DATA_18   0x5B
#define MPU_EXT_SENS_DATA_19   0x5C
#define MPU_EXT_SENS_DATA_20   0x5D
#define MPU_EXT_SENS_DATA_21   0x5E
#define MPU_EXT_SENS_DATA_22   0x5F
#define MPU_EXT_SENS_DATA_23   0x60

/*** The slave devices data out 0-3 (R/W) ***/
#define MPU_I2C_SLV0_DO        0x63
#define MPU_I2C_SLV1_DO        0x64
#define MPU_I2C_SLV2_DO        0x65
#define MPU_I2C_SLV3_DO        0x66

/*** Configure whether the Mpu9255 waits for external data (R/W) ***/
#define MPU_I2C_MST_DELAY_CTRL 0x67

/*** Resets the data path of the sensors (R/W) ***/
#define MPU_SIGNAL_PATH_RESET  0x68

/*** Configures the Accelerometer interrupts (R/W) ***/
#define MPU_MOT_DETECT_CTRL    0x69

/*** User control register (R/W)***/
#define MPU_USER_CTRL          0x6A

/*** To set different power modes, what sensors are on, and clocks (R/W)***/
#define MPU_PWR_MGMT_1         0x6B
#define MPU_PWR_MGMT_2         0x6C

/*** Returns, or sets the number of bytes to read, or write in the FIFO (R/W)***/
#define MPU_FIFO_COUNTH        0x72
#define MPU_FIFO_COUNTL        0x73

/*** Reads or writes one byte from the FIFO (R/W)***/
#define MPU_FIFO_R_W           0x74

/*** The who am I register used to test the device id should return 0x73(R) ***/
#define MPU_WHO_AM_I           0x75

/*** Used to set the offsets for the accelerometer (R/W) ***/
#define MPU_XA_OFFSET_H        0x77
#define MPU_XA_OFFSET_L        0x78
#define MPU_YA_OFFSET_H        0x7A
#define MPU_YA_OFFSET_L        0x7B
#define MPU_ZA_OFFSET_H        0x7D
#define MPU_ZA_OFFSET_L        0x7E




/*** ------------------- ***/
/*** Function Prototypes ***/
/*** ------------------- ***/

/*** Basic Read/Write Fucntions, these are the only ones that implement
     device specific functions. 
***/
void write8MPURegister(uint8_t reg_addr, uint8_t toWrite);
char read8MPURegister(uint8_t reg_addr);
void readBytesMPU(uint8_t reg_addr, uint8_t *byteArray, uint8_t numBytes);

/*** Specific Read requests from data registers ***/
int16_t readGyroX(void);
int16_t readGyroY(void);
int16_t readGyroZ(void);
int16_t readAccelX(void);
int16_t readAccelY(void);
int16_t readAccelZ(void);

/*** Configuration Register functions ***/
void configureMPU(uint8_t configMPU);
void configureGyro(uint8_t configGyro);
void configureAccel(uint8_t configAccel);
void configureInterruptPin(uint8_t configInt);
void configureUserControl(uint8_t userControlConfig);

/*** Interrupt Configuration ***/
void enableInterrupt(uint8_t enInt);
uint8_t readInterruptStatus(void);

/*** FIFO Control ***/
void enableFIFO(uint8_t enFIFO);
uint16_t getFIFOCount(void);

/*** WHOAMI Test Register ***/
uint8_t whoAmI(void);

/*** Hard Reset ***/
void reset(void);

/*** Initialization ***/
void init(void);

/*** Resolution Calculations ***/
double getRes(double fullRange);

/*** Calibration ***/
void calibrate();

/*** Get Roll, Pitch, and Yaw ***/
void getRPY(double *pitch, double *roll);

#endif /* MPU9255_H_ */