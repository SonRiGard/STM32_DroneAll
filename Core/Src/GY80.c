#include "GY80.h"
#include "math.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#define RAD_TO_DEG 57.2957795;

extern double time;
extern UART_HandleTypeDef huart2;
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

extern I2C_HandleTypeDef hi2c1;
//configuration for Digital Accelerometer ADXL345
//funtion for Digital Accelerometer ADXL345
uint8_t ADXL_Init (I2C_HandleTypeDef *I2Cx){
		uint8_t Data;
//	uint8_t check;
//	HAL_I2C_Mem_Read(I2Cx, ADXL345_ADDRESS , WHO_I_AM_ADXL , 1, &check , 1,100);
//	if (check == 0xE5){		
		Data = 0;// reset all bits
		HAL_I2C_Mem_Write(I2Cx, ADXL345_ADDRESS , POWER_CTL , 1, &Data , 1,100);	
		Data = 0x08;// measure and wake up 8hz
		HAL_I2C_Mem_Write(I2Cx, ADXL345_ADDRESS , POWER_CTL , 1, &Data , 1,100);
		Data = 0x00; // data_format range= +- 2g
		HAL_I2C_Mem_Write(I2Cx, ADXL345_ADDRESS , DATA_FORMAT , 1, &Data , 1,100);
		return 1;
//	}
//	return 0;
}

void read_ACC (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct){
	uint8_t ACC[6];
	HAL_I2C_Mem_Read (I2Cx, ADXL345_ADDRESS, DATAX0 , 1 ,(uint8_t *)ACC , 6 ,100 );
	DataStruct->Accel_X_RAW = ((ACC[1]<<8)|ACC[0]) - DataStruct->Accel_X_RAW_0;
	DataStruct->Accel_Y_RAW = ((ACC[3]<<8)|ACC[2]) - DataStruct->Accel_Y_RAW_0;
	DataStruct->Accel_Z_RAW = ((ACC[5]<<8)|ACC[4]) - DataStruct->Accel_Z_RAW_0;
	
	DataStruct->Ax = DataStruct->Accel_X_RAW / 256.0;
  DataStruct->Az = DataStruct->Accel_Y_RAW / 256.0;
  DataStruct->Ay = DataStruct->Accel_Z_RAW / 256.0;
}


//configuration for Digital gyroscope L3G4200D

//funtion for Digital gyroscope L3G4200D
uint8_t L3G4_Init (I2C_HandleTypeDef *I2Cx){
	uint8_t Data;
	Data = 0x0F;
	HAL_I2C_Mem_Write(I2Cx, L3G4200D_ADDRESS , CTRL_REG1 , 1, &Data , 1,100);
	Data = 0x00;//250 dps
	HAL_I2C_Mem_Write(I2Cx, L3G4200D_ADDRESS , CTRL_REG4 , 1, &Data , 1,100);
	Data = 0x00;// Disable FIFO
	HAL_I2C_Mem_Write(I2Cx, L3G4200D_ADDRESS , CTRL_REG5 , 1, &Data , 1,100);
	// Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode

	return 1;
}
void read_GYRO (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct){
	uint8_t GYRO[6];
	uint8_t Status=0;
	while (Status != 1){
		HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, L3G4200D_STATUS , 1 ,&Status , 1 ,100 );
		Status = (Status&0x08)>>3; 
	}
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_X_L , 1 ,&GYRO[0] , 1 ,100 );
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_X_H , 1 ,&GYRO[1] , 1 ,100 );
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_Y_L , 1 ,&GYRO[2] , 1 ,100 );
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_Y_H , 1 ,&GYRO[3] , 1 ,100 );
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_Z_L , 1 ,&GYRO[4] , 1 ,100 );
	HAL_I2C_Mem_Read (I2Cx, L3G4200D_ADDRESS, OUT_Z_H , 1 ,&GYRO[5] , 1 ,100 );
	// xyz follow register in IMU
	DataStruct->Gyro_X_RAW = ((GYRO[1]<<8)|GYRO[0]) - DataStruct->Gyro_X_RAW_0;
	DataStruct->Gyro_Y_RAW = ((GYRO[3]<<8)|GYRO[2]) - DataStruct->Gyro_Y_RAW_0;
	DataStruct->Gyro_Z_RAW = ((GYRO[5]<<8)|GYRO[4]) - DataStruct->Gyro_Z_RAW_0;
	//xyz follow project
	DataStruct->Gz = DataStruct->Gyro_X_RAW * 0.00875 ;
	DataStruct->Gx = -DataStruct->Gyro_Y_RAW * 0.00875 ;
	DataStruct->Gy = DataStruct->Gyro_Z_RAW * 0.00875 ;	
}
//Function caculate Angle________________________________________________
void Angle_GYRO (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct){
	read_GYRO(&hi2c1, DataStruct);
	DataStruct->Pitch_GYRO += DataStruct->Gx*time;
	DataStruct->Roll_GYRO += DataStruct->Gz*time;
	DataStruct->Yaw_GYRO += DataStruct->Gy*time;
}
//configuration for 3-Axis Digital Compass IC
uint16_t COMP_X_Value;
uint16_t COMP_Y_Value;
uint16_t COMP_Z_Value;
//funtion for Digital gyroscope L3G4200D
uint8_t HMC5883L_Init (I2C_HandleTypeDef *I2Cx){
	uint8_t Data;
		
	Data = 0x70;//0 (8-average, 15 Hz default, normal measurement)
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS , REGISTER_CONFIG_A , 1, &Data , 1,100);
	
	Data = 0xA0;//0 (8-average, 15 Hz default, normal measurement)
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS , REGISTER_CONFIG_B , 1, &Data , 1,100);
	
	//Continuous-Measurement Mode
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS , REGISTER_MODE , 1, &Data , 1,100);
	return 1;
}
void read_COMP (I2C_HandleTypeDef *I2Cx){
	uint8_t COMP[6];
	HAL_I2C_Mem_Read (I2Cx, HMC5883L_ADDRESS, X_MSB , 1 ,(uint8_t *)COMP , 6 ,100 );
	COMP_X_Value = ((COMP[0]<<8)|COMP[1]);
	COMP_Z_Value = ((COMP[2]<<8)|COMP[3]);
	COMP_Y_Value = ((COMP[4]<<8)|COMP[5]);
}


void Angle_ACC (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct){
	read_ACC(&hi2c1, DataStruct);
	DataStruct->Pitch_ACC = -atan2((DataStruct->Az),DataStruct->Ay)*RAD_TO_DEG;
	DataStruct->Roll_ACC =-atan2(-(DataStruct->Ax), sqrt((DataStruct->Az)*(DataStruct->Az)+(DataStruct->Ay)*(DataStruct->Ay)))*RAD_TO_DEG; 
}
void Kalman_angle_solve (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct){
	double dt = time;
	double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = -pitch;
    } else {
        DataStruct->KalmanAngleY = -Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = -Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);
	
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};


//END FUNTION_____________________________________________________________
//configuration for Digital pressure sensor BMP085 
// Defines according to the datsheet
short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;

/********************/
long UT = 0;
short oss = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
long B5 = 0;
unsigned long B4 = 0;
long B6 = 0;
unsigned long B7 = 0;

/*******************/
long Press = 0;
long Temp = 0;

void read_calliberation_data (I2C_HandleTypeDef *I2Cx)
{
	uint8_t Callib_Data[22] = {0};
	HAL_I2C_Mem_Read(I2Cx, BMP085_ADDRESS, CALIB_START, 1, Callib_Data,22, 1000);
	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}
uint16_t Get_UTemp (I2C_HandleTypeDef *I2Cx)
{
	uint8_t datatowrite = 0x2E;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(I2Cx, BMP085_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // wait 4.5 ms
	HAL_I2C_Mem_Read(I2Cx, BMP085_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0]<<8) + Temp_RAW[1]);
}

float BMP180_GetTemp (void)
{
	UT = Get_UTemp(&hi2c1);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temp = (B5+8)/(pow(2,4));
	return Temp/10.0;
}
// Get uncompensated Pressure
uint32_t Get_UPress (I2C_HandleTypeDef *I2Cx, int oss)   // oversampling setting 0,1,2,3
{
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(I2Cx, BMP085_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(I2Cx, BMP085_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}


float BMP180_GetPress (int oss)
{
	UP = Get_UPress(&hi2c1,oss);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) Press = (B7*2)/B4;
	else Press = (B7/B4)*2;
	X1 = (Press/(pow(2,8)))*(Press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*Press)/(pow(2,16));
	Press = Press + (X1+X2+3791)/(pow(2,4));
	return Press;
}

#define atmPress 101325 //Pa

float BMP180_GetAlt (int oss)
{
	BMP180_GetPress (oss);
	return 44330*(1-(pow(((float)Press/(float)atmPress), 0.19029495718)));
}

void BMP180_Start (void)
{
	read_calliberation_data(&hi2c1);
}

void Calib (GY80_t *DataStruct){
	int32_t SumACC_X = 0 ,SumACC_Y=0 ,SumACC_Z=0 , sumGyro_X = 0, sumGyro_Y= 0 , sumGyro_Z = 0;
	for(int16_t i = 0; i < 100 ; i++){
		uint8_t ACC[6];
		HAL_I2C_Mem_Read (&hi2c1, ADXL345_ADDRESS, DATAX0 , 1 ,(uint8_t *)ACC , 6 ,100 );
		DataStruct->Accel_X_RAW = (ACC[1]<<8)|ACC[0];
		DataStruct->Accel_Y_RAW = (ACC[3]<<8)|ACC[2];
		DataStruct->Accel_Z_RAW = (ACC[5]<<8)|ACC[4];
		
		uint8_t GYRO[6];
		static uint8_t Status = 0;
		while (Status != 1){
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, L3G4200D_STATUS , 1 ,&Status , 1 ,100 );
		Status = (Status&0x08)>>3;   
		}
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_X_L , 1 ,&GYRO[0] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_X_H , 1 ,&GYRO[1] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Y_L , 1 ,&GYRO[2] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Y_H , 1 ,&GYRO[3] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Z_L , 1 ,&GYRO[4] , 1 ,100 );
		HAL_I2C_Mem_Read (&hi2c1, L3G4200D_ADDRESS, OUT_Z_H , 1 ,&GYRO[5] , 1 ,100 );
			
		DataStruct->Gyro_X_RAW = ((GYRO[1]<<8)|GYRO[0]);
		DataStruct->Gyro_Y_RAW = ((GYRO[3]<<8)|GYRO[2]);
		DataStruct->Gyro_Z_RAW = ((GYRO[5]<<8)|GYRO[4]);
		
		SumACC_X +=   DataStruct->Accel_X_RAW;
		SumACC_Y += 	DataStruct->Accel_Y_RAW;
		SumACC_Z += 	DataStruct->Accel_Z_RAW;
		
		sumGyro_X += DataStruct->Gyro_X_RAW;
		sumGyro_Y += DataStruct->Gyro_Y_RAW;
		sumGyro_Z += DataStruct->Gyro_Z_RAW;
	}
	DataStruct->Accel_X_RAW_0 = SumACC_X/100;
	DataStruct->Accel_Y_RAW_0 = SumACC_Y/100;
	DataStruct->Accel_Z_RAW_0 = SumACC_Z/100 - 256;
	
	DataStruct->Gyro_X_RAW_0 = sumGyro_X/100;
	DataStruct->Gyro_Y_RAW_0 = sumGyro_Y/100;
	DataStruct->Gyro_Z_RAW_0 = sumGyro_Z/100;
}



 