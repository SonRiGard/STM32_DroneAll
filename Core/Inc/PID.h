#include "stm32f1xx_hal.h"
#include <math.h>

typedef struct
{
	float ampha , betal, gamma, delta;
	int16_t u , u1;
	float e , e1, e2;//error
	float KP, KI, KD;		
} PID_t;

void parameter_calculation (PID_t *DataStruct);
void setK (double KP, double KI, double KD, PID_t *DataStruct);
void get_out (PID_t *DataStruct, double set_Angle, double current_Angle);