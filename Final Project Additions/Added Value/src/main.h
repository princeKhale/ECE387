
/*** Includes  ***/
#include <stdint.h>
#include <math.h>
#include "mbed.h"
#include "MPU9255.h"

/*** Defines ***/
#define NUM_OF_MOTORS 4
#define SAMPLE_SIZE 5

#define MIN(x, y) (((x) < (y)) ? (x) : (y)) 
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

/*** ProtoTypes ***/
void initQuad(void);
void writeToMotors(float *speeds);
void calibrate(void);