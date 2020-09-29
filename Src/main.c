/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sd_hal_mpu6050.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int a1,a2,a3;
int MainAngleDelay = 100;

//Offsets need to be placed into PWMs
int MinLegSpace = 40;
int MaxLegSpace = 180;
int HipFL_Offset = 	-5;
int HipFR_Offset = 	-5;
int HipBL_Offset = 	-3;
int HipBR_Offset =  -1;
int ThighFL_Offset =0;
int ThighFR_Offset =-2;
int ThighBL_Offset =5;
int ThighBR_Offset =-8;
int KneeFL_Offset = 0;
int KneeFR_Offset = 0;
int KneeBL_Offset = 0;
int KneeBR_Offset = 0;

//IMU Variables
SD_MPU6050 mpu1, mpu2;
static float a1_x, a1_y, a1_z, a2_x, a2_y, a2_z;
static float g1_x, g1_y, g1_z, g2_x, g2_y, g2_z;
static float Filtereda1_x, Filtereda1_y, Filtereda1_z, Filtereda2_x, Filtereda2_y, Filtereda2_z;
static float Filteredg1_x, Filteredg1_y, Filteredg1_z, Filteredg2_x, Filteredg2_y, Filteredg2_z;
static float temperature1, temperature2;
static float *accel[3], *gyro[3], *temperature;
static uint8_t data[14];
extern I2C_HandleTypeDef *Handle;
extern uint8_t reg, address;
extern ReadingDevice numberofBytes;
extern SD_MPU6050 *devicePointer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void setMeasurementPointers(SD_MPU6050 *Handle);
static void setMeasurements(SD_MPU6050 *Handle);
static void KalmanFilter(I2C_HandleTypeDef hi2c1, SD_MPU6050 mpu);
/* USER CODE BEGIN PFP */
//UMU Functions
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_Master_Receive_DMA(Handle, (uint16_t)address, data, numberofBytes);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (numberofBytes == Accelerometer && reg == 0x3B)
  {
    /* Format */
    devicePointer->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
    devicePointer->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
    devicePointer->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
  }
  else if (numberofBytes == Gyroscope && reg == 0x43)
  {
    /* Format */
    devicePointer->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
    devicePointer->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
    devicePointer->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);
  }
  else if (numberofBytes == Temperature && reg == 0x41)
  {
    devicePointer->Temperature = (float)((int16_t)(data[0] << 8 | data[1]) / (float)340.0 + (float)36.53);
  }
  else
  {
    /* Format */
    devicePointer->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
    devicePointer->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
    devicePointer->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

    devicePointer->Temperature = (float)((int16_t)(data[0] << 8 | data[1]) / (float)340.0 + (float)36.53);

    /* Format */
    devicePointer->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
    devicePointer->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
    devicePointer->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);
  }
  setMeasurementPointers(devicePointer);
  setMeasurements(devicePointer);
}
static void setMeasurementPointers(SD_MPU6050 *Handle)
{
  if (Handle->DeviceNumber == 0)
  {
    accel[0] = &a1_x;
    accel[1] = &a1_y;
    accel[2] = &a1_z;
    gyro[0] = &g1_x;
    gyro[1] = &g1_y;
    gyro[2] = &g1_z;
    temperature = &temperature1;
  }
  else
  {
    accel[0] = &a2_x;
    accel[1] = &a2_y;
    accel[2] = &a2_z;
    gyro[0] = &g2_x;
    gyro[1] = &g2_y;
    gyro[2] = &g2_z;
    temperature = &temperature2;
  }
}
static void setMeasurements(SD_MPU6050 *Handle)
{
  *accel[0] = Handle->Accelerometer_X / 16384.0f;
  *accel[1] = Handle->Accelerometer_Y / 16384.0f;
  *accel[2] = Handle->Accelerometer_Z / 16384.0f;
  *gyro[0] = Handle->Gyroscope_X / 131.0f;
  *gyro[1] = Handle->Gyroscope_Y / 131.0f;
  *gyro[2] = Handle->Gyroscope_Z / 131.0f;
  *temperature = Handle->Temperature;
}
static void KalmanFilter(I2C_HandleTypeDef hi2c1, SD_MPU6050 mpu)
{
  int average[6];
  int i;
  //Take 1500 samples of the IMU Raw Readings
  for(i = 0, i < 1500; i++)
  {
    SD_MPU6050_ReadAll(&hi2c1, &mpu1);
    
  }

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*FRONT LEFT LEG******************************************************/
int HipFL(int a){
	htim1.Instance->CCR2 = map(a,-90,90,20,125);// + HipFL_Offset;
	return 0;
}
int ThighFL(int a){
	htim1.Instance->CCR3 = map(a,90,-90,20,125);// + ThighFL_Offset;
	return 0;
}
int KneeFL(int a){
	htim1.Instance->CCR1 = map(a,90,-90,20,125);///possibly change to timer 1 channel 1
	return 0;
}
/*FRONT RIGHT LEG*****************************************************/
int HipFR(int a){
	htim2.Instance->CCR4 = map(a,90,-90,20,125);// + HipFR_Offset;
	return 0;
}
int ThighFR(int a){
	htim2.Instance->CCR2 = map(a,-90,90,20,125);// + ThighFR_Offset;
	return 0;
}
int KneeFR(int a){
	htim2.Instance->CCR1 = map(a,-90,90,20,125);
	return 0;
}
/*BACK LEFT LEG*******************************************************/
int HipBL(int a){
	htim2.Instance->CCR3 = map(a,90,-90,20,125);// + HipBL_Offset;
	return 0;
}
int ThighBL(int a){
	htim3.Instance->CCR1 = map(a,90,-90,20,125);// + ThighBL_Offset;
	return 0;
}
int KneeBL(int a){
	htim3.Instance->CCR2 = map(a,90,-90,20,125);
	return 0;
}
/*BACK RIGHT LEG******************************************************/
int HipBR(int a){
	//htim3.Instance->CCR3 = map(a,-90,90,20,125);// + HipBR_Offset;
	htim1.Instance->CCR4 = map(a,-90,90,20,125);
	return 0;
}
int ThighBR(int a){
	htim3.Instance->CCR4 = map(a,-90,90,20,125);// + ThighBR_Offset;
	return 0;
}
int KneeBR(int a){
	htim3.Instance->CCR3 = map(a,-90,90,20,125);// + HipBR_Offset;
	return 0;
}
/*COMBINED************************************************************/
int FL_leg(int a1, int a2, int a3){
	HipFL(a1);ThighFL(a2);KneeFL(a3);
	return 0;
}
int FR_leg(int a1, int a2, int a3){
	HipFR(a1);ThighFR(a2);KneeFR(a3);
	return 0;
}
int BL_leg(int a1, int a2, int a3){
	HipBL(a1);ThighBL(a2);KneeBL(a3);
	return 0;
}
int BR_leg(int a1, int a2, int a3){
	HipBR(a1);ThighBR(a2);KneeBR(a3);
	return 0;
}
/*DIRECT ANGLE CONTROL************************************************/
int ToAngle_AllJoints(int a, int delay)
{
	HipFL(a);
	ThighFL(a);
	KneeFL(a);
	HipFR(a);
	ThighFR(a);
	KneeFR(a);
	HipBL(a);
	ThighBL(a);
	KneeBL(a);
	HipBR(a);
	ThighBR(a);
	KneeBR(a);
	HAL_Delay(delay);
	return 0;
}
int HipAngle(int a)
{
	int angletopwm = map(a,-90,90,20,125);
	HipFL(angletopwm);
	HipFR(angletopwm);
	HipBL(angletopwm);
	HipBR(angletopwm);
	return 0;
}
/*INVERSE KINEMATICS**************************************************/
float Position3D(float x, float y, float z)
{
	float link1 = 42.45; 		//mm
	float link2 = 92.64; 		//mm
	float link3 = 88.90; 		//mm
	float theta1 = atan2(y,x); 	// HipAngle
	float A = -z;
	float B = link1 - x*cos(theta1) + y*sin(theta1);
	float D = (2*link1*(x*cos(theta1)+y*sin(theta1))+pow(link3,2)-pow(link2,2)-pow(link1,2)-pow(z,2)-pow(x*cos(theta1)+y*sin(theta1),2))/(2*link2);
	float theta2 = -atan2(B,A)+atan2(D,sqrt(pow(A,2)+pow(B,2)-pow(D,2))); // ThighAngle
	float theta3 = atan2(z-link2*sin(theta2),x*cos(theta1) + y*sin(theta1)-link2*cos(theta2)-link1)-theta2; // KneeAngle
	a1 = theta1*(180/3.1415);
	a2 = 90 + theta2*(180/3.1415);
	a3 = 90 + theta3*(180/3.1415);
	return 0;
}
double Position2D(double x, double y, int i){ //x == forward
	//2D workspace inverse kinematics
	double a1_offset = 0;
	double l1 = 72.35; //mm
	double l2 = 88.90; //mm
	double theta2 = acos((pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2));
	double theta1 = atan2(x,y)-atan2((l2*sin(theta2)),(l1+l2*cos(theta2)));
	double a1 = (180/3.1415)*theta1 + a1_offset;
	double a2 = (180/3.1415)*theta2;
	//select which leg to send angles to
	if(i == 0){ //ALL LEGS
		FL_leg(0,a1,a2);
		FR_leg(0,a1,a2);
		BL_leg(0,a1,a2);
		BR_leg(0,a1,a2);}
	if(i == 1){ //FRONT LEFT LEG
		FL_leg(0,a1,a2);}
	if(i == 2){ //FRONT RIGHT LEG
		FR_leg(0,a1,a2);}
	if(i == 3){ //BACK LEFT LEG
		BL_leg(0,a1,a2);}
	if(i == 4){ //BACK RIGHT LEG
		BR_leg(0,a1,a2);}
	if(i == 14){ //FRONT LEFT & BACK RIGHT
		FL_leg(0,a1,a2);
		BR_leg(0,a1,a2);}
	if(i == 23){ //FRONT RIGHT & BACK LEFT
		FR_leg(0,a1,a2);
		BL_leg(0,a1,a2);}
	HAL_Delay(MainAngleDelay);
	return 0;
}
/*XY-2D TESTING*******************************************************/
double VerticalPointToPoint2D(double x, double yStart, double yFinal, double stepsize, int legNumber){
	//Moves from one Y coordinate to another with constant X
	//Stepsize adjusts for loop speed
	if(yStart>yFinal){
        for(int i=yStart;i>=yFinal;i=i-stepsize){Position2D(x,i,legNumber);}
    }
    if(yStart<yFinal){
        for(int i=yStart;i<=yFinal;i=i+stepsize){Position2D(x,i,legNumber);}
    }
    return 0;
}
double HorizontalPointToPoint2D(double xStart, double xFinal, double y, double stepsize, int legNumber){
	//Moves from one X coordinate to another with constant y
	//Stepsize adjusts for loop speed
	if(xStart>xFinal){
        for(int i=xStart;i>=xFinal;i=i-stepsize){Position2D(i,y,0);}
    }
    if(xStart<xFinal){
        for(int i=xStart;i<=xFinal;i=i+stepsize){Position2D(i,y,0);}
    }
    return 0;
}
double VerticleTest2D(double x){
    //test for verticle limits
    for(int i=115;i<=161;i++){Position2D(x,i,0);}
    for(int i=161;i>=115;i--){Position2D(x,i,0);}
    return 0;
}
double PositiveHorizontalTest2D(double y){
    //test for forward horizontal limit
    for(int i=115;i<=144;i++){Position2D(i,y,0);}
    for(int i=144;i>=115;i--){Position2D(i,y,0);}
    return 0;
}
double NegativeHorizontalTest2D(double y){
    //test for backwards horizontal limit
    for(int i=-115;i>=-144;i--){Position2D(i,y,0);}
    for(int i=-144;i<=-115;i++){Position2D(i,y,0);}
    return 0;
}
/*GAIT EQUATIONS******************************************************/
double EllipseGait2D(double h, double k, double a, double b, int stepsize){
	//{centered at h,k; horizontal rad a; vertical rad b}
	//phaseshift from right foot to left foot motion
	int phaseshift = 180;
    for(double i = 0; i<=360;i=i+stepsize){
        double exr = a*cos(i*(3.1415/180))+h;
        double eyr = b*sin(i*(3.1415/180))+k;
        double exl = a*cos((i+phaseshift)*(3.1415/180))+h;
        double eyl = b*sin((i+phaseshift)*(3.1415/180))+k;
        Position2D(exl,eyl,14);
        Position2D(exr,eyr,23);
    }
    return 0;
}
double HalfEllipseGait2D(double h, double k, double a, double b, int stepsize){
    //{centered at h,k; horizontal rad a; vertical rad b}
    //phaseshift from right foot to left foot motion
    int phaseshift = 180;
    for(double i = 0; i<=360;i=i+stepsize){
        double exr = a*cos(i*(3.1415/180))+h;
        double eyr = b*sin(i*(3.1415/180))+k;
        double exl = a*cos((i+phaseshift)*(3.1415/180))+h;
        double eyl = b*sin((i+phaseshift)*(3.1415/180))+k;
        if(eyr>140){eyr=140;}//if passed ground plane set height to ground and move x component only
        if(eyl>140){eyl=140;}//same for other foot
        Position2D(exl,eyl,14);
        Position2D(exr,eyr,23);
    }
    return 0;
}
double SquareGait2D(){
    //Steps from point to point in a
	//2 ARRAYS FOR X AND Y, STEPS FROM POINT TO POINT FROM EACH IN THE FOR LOOP
	//WILL STILL REQUIRE A VARIABLE TIME DELAY
    double xA[7] = {0,-40,-40,0,40,40,0};
    double yA[7] = {140,140,120,120,120,140,140};
    //xA_PS && yA_PS == phase shifted for the opposite leg
    double xA_PS[7] = {0,40,40,0,-40,-40,0};
    double yA_PS[7] = {120,120,140,140,140,120,120};
    for(int i = 0; i<=5; i++){
    	Position2D(xA[i],yA[i],0);
//    	Position2D(xA_PS[i],yA_PS[i],14);
//    	HAL_Delay();
    }
    return 0;
}
double Leap2D(){

	return 0;
}
/*STARTING************************************************************/
int StartingPose()
{
	Position2D(0,140,0); //(forward,height)
	return 0;
}
int StationaryTrot(){
	EllipseGait2D(0,150,0,20,25);
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 SD_MPU6050_Result result;
	 uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
	 uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
	 mpu1.DeviceNumber = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(500);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(500);
  result = SD_MPU6050_Init(&hi2c1, &mpu1, SD_MPU6050_Device_0, SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);
  HAL_Delay(1000);
  KalmanFilter(&hi2c1, &mpu1);
  StartingPose();
  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  SD_MPU6050_ReadAll(&hi2c1, &mpu1);
//	  ToAngle_AllJoints(0,1000);
//	  StationaryTrot();
//	  EllipseGait2D(0,140,65,20,30);
//	  HalfEllipseGait2D(0,140,40,20,30);
//	  SquareGait2D();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 90-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
