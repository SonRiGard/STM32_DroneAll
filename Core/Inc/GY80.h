#include "stm32f1xx_hal.h"

//Accelerometer ADXL345
#define ADXL345_ADDRESS 0xA6
#define WHO_I_AM_ADXL 0x00
#define DATAX0  0x32
#define DATAX1  0x33
#define DATAY0  0x34
#define DATAY1  0x35
#define DATAZ0  0x36
#define DATAZ1  0x37
#define POWER_CTL  0x2D
#define DATA_FORMAT  0x31

//for Digital gyroscope L3G4200D
#define L3G4200D_ADDRESS 0xD3 
#define WHO_AM_I_L3G4200D  0x0F
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define	CTRL_REG1 0x20
#define	CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define L3G4200D_STATUS 0x27
//for Digital HMC5883L
#define HMC5883L_ADDRESS 0x3D
#define REGISTER_MODE 0x02
#define REGISTER_CONFIG_B 0x01//in configuration register b, we need to configure the scale
#define REGISTER_CONFIG_A 0x00
#define X_MSB 0x03

//for BMP085 Digital pressure sensor
#define BMP085_ADDRESS 0xEE
#define PRESSURE_0 0x34
#define CALIB_START 0xAA

//GY-80 structure
typedef struct
{
		int16_t Accel_X_RAW_0;
		int16_t Accel_Y_RAW_0;
		int16_t Accel_Z_RAW_0;
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;
		
		int16_t Gyro_X_RAW_0;
		int16_t Gyro_Y_RAW_0;
		int16_t Gyro_Z_RAW_0;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
	
		uint16_t COMP_X_RAW;
		uint16_t COMP_Y_RAW;
		uint16_t COMP_Z_RAW;
		double Cx;
    double Cy;
    double Cz;
	
    float Temperature;
		
		double Pitch_ACC;
    double Roll_ACC;
		double Yaw_ACC;
		
		double Pitch_GYRO;
    double Roll_GYRO;
		double Yaw_GYRO;
		
    double KalmanAngleX;
    double KalmanAngleY;
	
} GY80_t;



// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t ADXL_Init (I2C_HandleTypeDef *I2Cx);
void read_ACC (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct);
uint8_t L3G4_Init (I2C_HandleTypeDef *I2Cx);
void read_GYRO (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct);
uint8_t HMC5883L_Init (I2C_HandleTypeDef *I2Cx);
void read_COMP (I2C_HandleTypeDef *I2Cx);


void read_calliberation_data (I2C_HandleTypeDef *I2Cx);
uint16_t Get_UTemp (I2C_HandleTypeDef *I2Cx);
float BMP180_GetTemp (void);
uint32_t Get_UPress (I2C_HandleTypeDef *I2Cx, int oss) ;
float BMP180_GetPress (int oss);
float BMP180_GetAlt (int oss);
void BMP180_Start (void);

void Angle_GYRO (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct);
void Angle_ACC (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void Kalman_angle_solve (I2C_HandleTypeDef *I2Cx, GY80_t *DataStruct);
void Calib (GY80_t *DataStruct);