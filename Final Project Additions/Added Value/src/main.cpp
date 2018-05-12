#include "main.h"

PwmOut *motors[4];

// Serial object pc uses the connected usb to send and receieve data to the computer
Serial pc(USBTX, USBRX);

float maxSpeed = 0.95f;
float minSpeed = 0.25f;

int controlEn = 0;

int main(){
    motors[0] = new PwmOut(PE_14);
    motors[1] = new PwmOut(PE_6);   
    motors[2] = new PwmOut(PA_5);
    motors[3] = new PwmOut(PC_8);
   
   //Give everything time to warm up
   wait_ms(1000);

   
   initQuad();
   
   float *motorSpeeds = (float*)malloc(sizeof(float) * NUM_OF_MOTORS); //int32_t
   
   float desiredDelta = 0.01;
   
   //Sample and other accelerometer data variables
   int16_t curX, curY, curZ;
   int16_t deltaX, deltaY, deltaZ;
   //int16_t *sampAccelX, *sampAccelY, *sampAccelZ;
   int16_t avgAccelX, avgAccelY, avgAccelZ;
   int16_t prevX, prevY, prevZ;
   
   /*
   *sampAccelX = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   *sampAccelY = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   *sampAccelZ = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   */
   
   //Gyro data variables
   int16_t curGyroX, curGyroY, curGyroZ;
   int16_t gyroDeltaX, gyroDeltaY, gyroDeltaZ;
   //int16_t *sampGyroX, *sampGyroY, *sampGyroZ;
   int16_t avgGyroX, avgGyroY, avgGyroZ;
   int16_t prevGyroX, prevGyroY, prevGyroZ;
   
   /*
   *sampGyroX = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   *sampGyroY = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   *sampGyroZ = (int16_t*)malloc(sizeof(int16_t) * SAMPLE_SIZE);
   */
   
   prevX = 0;
   prevY = 0;
   prevZ = 0;
   
   prevGyroX = 0;
   prevGyroY = 0;
   prevGyroZ = 0;
   
   
   /*** Inital Speed ***/
   for(int i = 0; i < NUM_OF_MOTORS; i++){
        *(motorSpeeds + i) = 0.5f;    
   }
   writeToMotors(motorSpeeds);
   
   /*** End Inital Speed ***/
   
   
   /* ============================================================== *
    *                      MAIN CONTROL LOOP                         *    
    * ============================================================== */
   
   while(1) {
    
    //Clamp Speed and print it out
    for(int i = 0; i < NUM_OF_MOTORS; i++){
        *(motorSpeeds + i) = MIN(*(motorSpeeds + i), maxSpeed);
        *(motorSpeeds + i) = MAX(*(motorSpeeds + i), minSpeed);

        pc.printf("%s %d %s %d\n","Motor ",  i, " Speed: ", *(motorSpeeds + i));
    }
    
    writeToMotors(motorSpeeds);
    
    /*
    for(int i = 0; i < SAMPLE_SIZE; i++){
        *(sampAccelX + i) = readAccelX();        
        *(sampAccelY + i) = readAccelY();  
        *(sampAccelZ + i) = readAccelZ();
        
        *(sampGyroX + i) = readGyroX();
        *(sampGyroY + i) = readGyroY();
        *(sampGyroZ + i) = readGyroZ();
        wait_ms(10);   // ~20ms per loop iteration  @ 10 samples ~200ms for total loop
    }
    */
    
    //Average out data
    for(int j = 0; j < SAMPLE_SIZE; j++){
        avgAccelX += readAccelX();
        avgAccelY += readAccelY();
        avgAccelZ += readAccelZ();

        avgGyroX += readGyroX();
        avgGyroY += readGyroY();
        avgGyroZ += readGyroZ();
        
        wait_ms(200);   // ~ms per loop iteration  @ 5 samples ~25ms for total loop
    }
    
    avgAccelX /= SAMPLE_SIZE;
    avgAccelY /= SAMPLE_SIZE;
    avgAccelZ /= SAMPLE_SIZE;
    
    avgGyroX /= SAMPLE_SIZE;
    avgGyroY /= SAMPLE_SIZE;
    avgGyroZ /= SAMPLE_SIZE;
    
    curX = avgAccelX * pow(1, 2); //Accel = m/s^2 so mult by s^2 to get ~m
    curY = avgAccelY * pow(1, 2);
    curZ = avgAccelZ * pow(1, 2);
    
    curGyroX = avgGyroX * (0.2);  //in Deg/s so mult by s to get ~m
    curGyroY = avgGyroY * (0.2);
    curGyroZ = avgGyroZ * (0.2);
    
    deltaX = prevX - curX;
    deltaY = prevY - curY;
    deltaZ = prevZ - curZ;

    gyroDeltaX = prevGyroX - curGyroX;
    gyroDeltaY = prevGyroY - curGyroY;
    gyroDeltaZ = prevGyroZ - curGyroZ;

    if(abs(gyroDeltaX) > desiredDelta){
        if(curGyroX < prevGyroX){
            *motorSpeeds  += 0.05f;                 //Front Left Motor
            *(motorSpeeds + 1) -= 0.05f;            //Front Right Motor
            *(motorSpeeds + 2) += 0.05f;            //Back Left Motor
            *(motorSpeeds + 3) -= 0.05f;            //Back Right Motor
        }else if(curGyroX > prevGyroX){
            *motorSpeeds  -= 0.01f;                 //Front Left Motor
            *(motorSpeeds + 1) += 0.05f;            //Front Right Motor
            *(motorSpeeds + 2) -= 0.05f;            //Back Left Motor
            *(motorSpeeds + 3) += 0.05f;            //Back Right Motor
        }
    }
    
    if(abs(gyroDeltaY) > desiredDelta){
        if(curGyroY < prevGyroY){
            *motorSpeeds  -= 0.05f;                 //Front Left Motor
            *(motorSpeeds + 1) -= 0.05f;            //Front Right Motor
            *(motorSpeeds + 2) += 0.05f;            //Back Left Motor
            *(motorSpeeds + 3) += 0.05f;            //Back Right Motor
        }else if(curGyroY > prevGyroY){
            *motorSpeeds  += 0.05f;                 //Front Left Motor
            *(motorSpeeds + 1) += 0.05f;            //Front Right Motor
            *(motorSpeeds + 2) -= 0.05f;            //Back Left Motor
            *(motorSpeeds + 3) -= 0.05f;            //Back Right Motor
        }
    }
    
   //wait_ms(200);
  }
} 

void initQuad(){ 
   /*** Initialize MPU device  ***/
   
   //Test for connection 
    if(whoAmI() == 0x73){
        pc.printf("Connected");
    } else{
        pc.printf("Not Connected");
    }
    
    initMPU();
    calibrateMPU();
   
   
   /*** End MPU Init ***/
   
   
   /*** Quadcopter Init ***/
   
    calibrate();
   
   /*** End Quadcopter Init ***/
   
   wait_ms(500);
   controlEn = 1;
   return;
}

void writeToMotors(float *speed){
    for(int i = 0; i < NUM_OF_MOTORS; i++){
        *motors[i] = *(speed + i);    
    }
}

void calibrate(){
    for(int i = 0; i < NUM_OF_MOTORS; i++){
        *motors[i] = (0.0f);    
    }    
    
    wait(6.0);
    
    for(int i = 0; i < NUM_OF_MOTORS; i++){
        *motors[i] = (1.0f);    
    } 
    
    wait(2.0);
    
}