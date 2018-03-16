/*
 *  main.c
 *
 *  Created on: Mar 11, 2018
 *  Author: Logan Johnson
 *  
 *  @description: This is a simple test program for the MPU9055, It can be used 
 *  to test, and calibrate, mostly, the accelorometer and gyroscope aspect of the
 *  chip.
 *
 *
 */

#include "mbed.h"
#include "MPU9255.h"

// Serial object pc uses the connected usb to send and receieve data to the computer
Serial pc(USBTX, USBRX);
double *roll, *pitch;

int main(){   
    //Test for connection 
    if(whoAmI() == 0x73){
        pc.printf("Connected");
    } else{
        pc.printf("Not Connected");
    }
    
    init();
    calibrate();
    
    while(1){
        /* As of right now the only usable data is the gyroscope and Acclerometer
         *data, if the calibration is successful.
         */
        
        pc.printf("=== Gyroscope Data ===\n");
        pc.printf("GyroX: %d\n", readGyroX());   
        pc.printf("GyroY: %d\n", readGyroY());
        pc.printf("GyroZ: %d\n", readGyroZ());

        pc.printf("=== Accelerometer Data ===\n");
        pc.printf("AccelX: %d\n", readAccelX());   
        pc.printf("AccelY: %d\n", readAccelY());
        pc.printf("AccelZ: %d\n", readAccelZ());

        /*
        getRPY(roll, pitch);
        pc.printf("----------------\r\n");
        pc.printf("| Roll: %.3f\r\n", roll);
        pc.printf("| Pitch: %.3f\r\n", pitch);
        pc.printf("----------------\r\n");
        */
        
        //update every 500 miliseconds to poll the next read
        wait_ms(500);    
    }
    
}