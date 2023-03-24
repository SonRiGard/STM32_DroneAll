#include "main.h"
#include "TIMER.h"
#include "SPI.h"
#include "UART.h"
#include "I2C.h"
#include "GPIO.h"
#include "RCC.h"
//#include "GY80.h"
#include "ADC.h"
#include "PID.h"
#include "stdio.h"
#include "string.h"
#include "NRF24L01.h"
#include "mpu6050.h"


extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

static void PWM_caculation (int8_t pitch, int8_t roll,int8_t yaw, int8_t altitude);
static void setup_motor (void);
double DeltaT = 0.05;
double time;

/*GY80_t GY80;*/
MPU6050_t MPU6050;

uint32_t count=0;
uint16_t Thurst =1800 ;//force for drone start move
PID_t pid_pitch, pid_roll, pid_yaw, pid_altitude;
//
double L = 0.16;
double KmT = 0.02;

uint16_t test=0 ;

uint32_t t1=0,t=0,t2=0;

double set_pitch =0, set_roll=0;

uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];
uint8_t data[50];
uint16_t ADC_Value[2];
uint16_t pwm_value;
uint16_t pwm=2000;
uint16_t pwm_time=0 ;
double T1=0,T2=0,T3=0,T4=0;


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  //MX_SPI1_Init();
  //MX_TIM1_Init();
  //MX_TIM2_Init();
  //MX_TIM3_Init();
	//MX_ADC1_Init();
  MX_USART1_UART_Init();
  /*          	
	ADXL_Init(&hi2c1);
	L3G4_Init(&hi2c1);
	HMC5883L_Init(&hi2c1);
	BMP180_Start();
	HAL_Delay(200);
	*/
	MPU6050_Init(&hi2c1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	
	//NRF init
	//NRF24_Init();
  //NRF24_RxMode(RxAddress, 10);
	//PID value set KP, KI, KD and caculation ampha betal gamma
	//setK(10,0.21,-0.18,&pid_pitch);
	//setK(10,-0.04,-0.7,&pid_roll);
	//setK(1,1,1,&pid_yaw);
	//setK(1,1,1,&pid_altitude);
	//parameter_calculation(&pid_pitch);
	//parameter_calculation(&pid_roll);
	//parameter_calculation(&pid_yaw);
	//parameter_calculation(&pid_altitude);
////TIM2 PWM controller motors
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	//Set up motor automatically
	//setup_motor();
	
	/*	Calib(&GY80);*/
	Calib(&MPU6050);
	//HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim1);
//	TIM2->CCR1 =2000;	
//	TIM2->CCR3 =2000;
//	TIM2->CCR2 =2000;	
//	TIM2->CCR4 =2000;
	HAL_Delay(200);
	
  while (1)
		{
			MPU6050_Read_All(&hi2c1,&MPU6050);
			//Receiver_NRF();
			
//			while(isDataAvailable(2) == 1)
//			{
//			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		  NRF24_Receive(RxData);
//			ADC_Value[0] = RxData[0]|((uint16_t)RxData[1]<<8);
//			ADC_Value[1] = RxData[2]|((uint16_t)RxData[3]<<8);
//			pwm_value = ADC_Value[0]/2.1 + 2000;//159;
//			count++;
//				if (ADC_Value[0]>4090)ADC_Value[0]=4090;					
//			}
			//TIM2->CCR1 =2300;	
			//TIM2->CCR3 =2300;
			//TIM2->CCR2 =2300;	
			//TIM2->CCR4 =2300;
			
			
			//HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x100);
//			if (isDataAvailable(2) == 1)
//			{
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		  NRF24_Receive(RxData);
//			ADC_Value[0] = RxData[0]|((uint16_t)RxData[1]<<8);
//			ADC_Value[1] = RxData[2]|((uint16_t)RxData[3]<<8);
//			pwm_value = ADC_Value[0]/2.1 + 2000;//159;					
//			//TIM2->CCR1 =pwm_value;	
//			//TIM2->CCR2 =pwm_value;
//			//TIM2->CCR3 =pwm_value;
//			//TIM2->CCR4 =pwm_value;			
//			}

//		double X = MPU6050.KalmanAngleX;
//		double Y = MPU6050.KalmanAngleY;
//			char str[20];
//			sprintf(str,"%f,%f\r\n",X,Y);
//			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x100);
			
			
//		char str[170]={0};	
//		sprintf(str,"%f:%f:%f:%f:%f:%f:%f:%f:%f:%f\r\n",
//		GY80.Pitch_ACC,GY80.Roll_ACC,GY80.KalmanAngleX,GY80.KalmanAngleY,GY80.Ax,GY80.Ay,GY80.Az,GY80.Gx,GY80.Gy,GY80.Gz);
//		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x100);

		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance==TIM1)
    {
				t+=1;			
  	}
    if(htim->Instance==TIM3)
  	{
//			get_out(&pid_pitch,set_pitch,GY80.KalmanAngleX);
//			get_out(&pid_roll,set_roll,GY80.KalmanAngleY);
//			PWM_caculation(pid_pitch.u,pid_roll.u,0,0); 
			//HAL_Delay(500);
//			if (pwm_value < 3000)
//			{
//				if (pwm_time == 0){
//					pwm_time = 1;
//					pwm=2500;
//				}else {
//					pwm_time = 0;
//					pwm=2000;
//				}
//			}
//			else{
//				pwm=2000;
//			}				
		}
}

static void Receiver_NRF (void){
	if(isDataAvailable(2) == 1)
		{
			count++;//test receiver data or no ?	
		  NRF24_Receive(RxData);
			uint16_t pre[2] = {ADC_Value[0],ADC_Value[1]};
			
			uint16_t result[2];
			result[0] = RxData[0]|((uint16_t)RxData[1]<<8);
			result[1] = RxData[2]|((uint16_t)RxData[3]<<8);
			
			if ((result[0]- pre[0] > 2500 )&& (pre[0]- result[0] > 2500)
				&&(result[1]- pre[1] > 2500 )&& (pre[1]- result[1] > 2500)){
				ADC_Value[0] = result[0];
				ADC_Value[1] = result[1];
			}
		}
}

static void PWM_caculation (int8_t pitch, int8_t roll,int8_t yaw, int8_t altitude){
	T1 = pwm_value+(+ pitch/(4*L) - roll/(4*L) - yaw/(4*KmT) + altitude/4)*0.063;
	T2 = pwm_value+(+ pitch/(4*L) + roll/(4*L) + yaw/(4*KmT) + altitude/4)*0.063;
	T3 = pwm_value+(- pitch/(4*L) + roll/(4*L) - yaw/(4*KmT) + altitude/4)*0.063;
	T4 = pwm_value+(- pitch/(4*L) - roll/(4*L) + yaw/(4*KmT) + altitude/4)*0.063;
	
	TIM2->CCR1 = T1;//Thurst - pitch/(4*L) - roll/(4*L) - yaw/(4*KmT) + altitude/4;
	TIM2->CCR2 = T2;//Thurst + pitch/(4*L) - roll/(4*L) + yaw/(4*KmT) + altitude/4;
	TIM2->CCR3 = T3;//Thurst + pitch/(4*L) + roll/(4*L) - yaw/(4*KmT) + altitude/4;
	TIM2->CCR4 = T4;//Thurst - pitch/(4*L) + roll/(4*L) + yaw/(4*KmT) + altitude/4;
}

double get_time (void){
		t2=t1;
		t1=t;
		return (t1-t2)/1000.000;
}
 
static void setup_motor (void)
{
		uint16_t pwm_value;
		uint16_t ADC_Value[1];
		
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		//Set Max value of PWM
		ADC_Value[0] = 4042;//Maximun of ADC Read from NRF
		pwm_value = ADC_Value[0]/2.1 +2000;
		TIM2->CCR1 = pwm_value;
		TIM2->CCR2 = pwm_value;
		TIM2->CCR3 = pwm_value;
		TIM2->CCR4 = pwm_value;
										
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
										
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		ADC_Value[0] = 25;//Minimun of ADC Read from NRF
		pwm_value = ADC_Value[0]/1.5 +2000;
		TIM2->CCR1 = pwm_value;
		TIM2->CCR2 = pwm_value;
		TIM2->CCR3 = pwm_value;
		TIM2->CCR4 = pwm_value;
		HAL_Delay(5000);        	
}
 
 
void Error_Handler(void){
  __disable_irq();
  while (1)
  {
  }
 
}
 
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *     	where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
 	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
