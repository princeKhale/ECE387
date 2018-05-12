#include "mbed.h"
//PE_14, PF_7, PA_5, PC_8

void calibrate(void);

PwmOut *motors[4];

float curDutyCycle;

int main() {
    motors[0] = new PwmOut(PE_14);
    motors[1] = new PwmOut(PE_6);
    motors[2] = new PwmOut(PA_5);
    motors[3] = new PwmOut(PC_8);
    
    for(int i = 0; i < 4; i++){
        motors[i]->period_us(21.0);   
    }
    
    calibrate();
   
    while(1){
        for(int i = 0; i < 4; i++){
            *motors[i] = 0.35f;   
        }
    }
    
}


void calibrate(){
    for(int i = 0; i < 4; i++){
        *motors[i] = (0.0f);    
    }    
    
    wait(6.0);
    
    for(int i = 0; i < 4; i++){
        *motors[i] = (1.0f);    
    } 
    
    wait(2.0);
    
}