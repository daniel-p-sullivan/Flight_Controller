/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "./actuators/actuators.hpp"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "./comms/communications.hpp"
#include "./controllers/controllers.hpp"
#include "./sensors/sensors.hpp"
#include "./threads/actuatorthread.hpp"
#include "./threads/controllerthread.hpp"
#include "./threads/sensorthread.hpp"

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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
enum System_State {
	INIT,
	IMU_CALIB_INIT,
	IMU_CALIB_DONE,
	MOTOR_INIT,
	MOTOR_INIT_DONE,
	COMMS_INIT,
	COMMS_INIT_DONE,
	RTOS
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */




  //system state variable
  enum System_State sys_state = INIT;

  //calibration flags
  bool imu_config_flag, imu_calib_flag = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //required devices
	  sensors::BNO055 imu(hi2c1);
	  actuators::Motors motors;
	  communications::Communicator comms;

	  //thread arguments
	  threads::controllerThreadArgs controllerArgs;
	  threads::actuatorThreadArgs actuatorArgs;
	  threads::sensorThreadArgs sensorArgs;

	  //shared state and controller variables
	  state::QuadStateVector sharedState;
	  state::QuadControlActions sharedOutput;

	  sensorArgs.state = &sharedState;
	  controllerArgs.state = &sharedState;
	  controllerArgs.output = &sharedOutput;
	  actuatorArgs.output = &sharedOutput;

	  switch(sys_state){

	  	  case INIT:
	  	  	  {
	  	  		  //time buffer to allow BNO055 to self test?
	  	  		  HAL_Delay(10);



	  	  		  imu_config_flag = imu.configSensor();
	  	  		  sensorArgs.imu = &imu;

	  	  		  if(imu_config_flag){
	  	  			  sys_state = IMU_CALIB_INIT;
	  	  		  }
	  	  	  }
	  		  break;

	  	  case IMU_CALIB_INIT:
	  	  	  {
			  	  //CODE FOR CALIBRATING THE SENSOR
	  		  	  imu_calib_flag = imu.Read_IMU_Calib_Status();
	  		  	  bool test = imu.Read_Calib_Params();
	  		  	  if(imu_calib_flag){

	  			  	  sys_state = IMU_CALIB_DONE;
	  		  	  }
	  	  	  }
	  		  break;

	  		  //known calibration parameters
//	  		  imu_calib_flag = Write_IMU_Calib_Params();
//	  		  if(imu_calib_flag){
//
//	  			  sys_state = IMU_CALIB_DONE;
//
//	  		  }
//	  		  break;

	  	  case IMU_CALIB_DONE:
	  	  	  {
	  		  	  //blink an LED, transistion to RTOS
			  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  	  HAL_Delay(1000);
			  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  	  HAL_Delay(1000);
	  		  	  sys_state = MOTOR_INIT;
	  	  	  }
	  		  break;

	  	  case MOTOR_INIT:
	  	  	  {
	  	  		  HAL_Delay(1000);
	  	  		  actuators::BLHelis motors(htim8);
	  	  		  actuatorArgs.motors = &motors;
	  	  		  //BLHeli_Start(); ?
	  	  		  //BLHeli_Arm(); ?
	  	  		  sys_state = MOTOR_INIT_DONE;
	  	  	  }
	  		  break;

	  	  case MOTOR_INIT_DONE:
	  	  	  {
	  		  	  sys_state = COMMS_INIT;
	  	  	  }
	  	  	  break;

	  	  case COMMS_INIT:
	  	  	  {
	  	  		  //communications::NRF24 comms(hspi2);
	  	  		  sys_state = COMMS_INIT_DONE;
	  	  	  }
	  	  	  break;


	  	  case COMMS_INIT_DONE:
	  	  	  {
//	  	  		  comms.Enable_Receive();
////	  	  		  uint8_t fifo_status, status, config, rf_setup, rf_ch = 0xff;
////	  	  		  uint8_t rx_addr[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
////	  	  		  comms.Clear_FIFO_Interrupt();
////	  	  		  comms.Read_Register(FIFO_STATUS, &fifo_status);
////	  	  		  comms.Read_Register(STATUS, &status);
////	  	  		  comms.Read_Register(CONFIG, &config);
////	  	  		  comms.Read_Register(RF_SETUP, &rf_setup);
////	  	  		  comms.Read_Register(RF_CH, &rf_ch);
////
////
////	  	  		  while(1){
////	  	  			  comms.Read_Register(FIFO_STATUS, &fifo_status);
////	  	  			  comms.Read_Register(STATUS, &status);
////	  	  			  comms.Read_Register(CONFIG, &config);
////	  	  			  comms.Read_Register(RF_SETUP, &rf_setup);
////	  	  			  comms.Read_Register(RF_CH, &rf_ch);
////	  	  			  comms.Flush_RX();
////	  	  			  comms.Enable_Receive();
////	  	  			  HAL_Delay(100);
////	  	  			  if(comms.Payload_Available()){
////	  	  				  comms.Read_Payload();
////	  	  			  }
////	  	  		  }
	  	  		  sys_state = RTOS;
	  	  	  }
	  	  	  break;

	  	  case RTOS:
	  	  	  {

	  	  		  osKernelInitialize();

	  	  		  //create the mutexes
	  	  		  SemaphoreHandle_t xSharedStateMutex = xSemaphoreCreateBinary();
	  	  		  SemaphoreHandle_t xSharedOutputMutex = xSemaphoreCreateBinary();

	  	  		  //open them
	  	  		  xSemaphoreGive(xSharedStateMutex);
	  	  		  xSemaphoreGive(xSharedOutputMutex);

	  	  		  controllerArgs.pxSharedOutputMutex = &xSharedOutputMutex;
	  	  		  controllerArgs.pxSharedStateMutex = &xSharedStateMutex;
	  	  		  sensorArgs.pxSharedStateMutex = &xSharedStateMutex;
	  	  		  actuatorArgs.pxSharedOutputMutex = &xSharedOutputMutex;

	  	  		  TaskHandle_t xSensorThreadHandle;
	  	  		  TaskHandle_t xActuatorThreadHandle;
	  	  		  TaskHandle_t xControllerThreadHandle;

	  	  		  BaseType_t xRet;

	  	  		  xRet = xTaskCreate(threads::sensorThread,
	  	  			  	  			"sensorThread",
	  	  							256,
	  	  							(void*)&sensorArgs,
	  	  							configMAX_PRIORITIES-1,
	  	  							&xSensorThreadHandle);

	  	  		  xRet = xTaskCreate(threads::controllerThread,
	  	  				  	  	  	  "controllerThread",
									  256,
									  (void*)&controllerArgs,
									  configMAX_PRIORITIES-1,
									  &xControllerThreadHandle);

	  	  		 xRet = xTaskCreate(threads::actuatorThread,
	  	  							"actuatorThread",
									256,
	  	  							(void*)&actuatorArgs,
									configMAX_PRIORITIES-1,
	  	  							&xActuatorThreadHandle);

	  	  		  vTaskStartScheduler();





//	  	  		  uint8_t dt = 1;
//
//				  IMU_Sample* sample_i = malloc(sizeof(IMU_Sample));
//				  TRPY_Controller* trpy_c = Construct_TRPY_Controller(dt);
//
//				  TRPY_Setpoint* hover = malloc(sizeof(TRPY_Setpoint));
//				  hover->pitch_setpoint = 0;
//				  hover->roll_setpoint = 0;
//				  hover->yaw_setpoint = 0;
//				  hover->z_setpoint = 0;
//
//				  Update_Setpoint(trpy_c, hover);
//
//				  while(1){
//
//
//
//					  Update_IMU_Sample(sample_i);
//					  Iter_TRPY_Controller(trpy_c, sample_i);
//					  Motor_Mixing_Algorithm(trpy_c->trpy_o);
//
//					  float p_thrust = 0.3;
//					  BLHeli_Set_Thrust_Percent(&p_thrust);
//
//		  			  for(int i = 0; i < 10; i++){
//		  				  	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		  				  	HAL_Delay(500);
//		  			  }

				  }
	  	  	  }
	  		  break;


	  		  //Kernel_Initialize();

	  }
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 26;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 62418;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, NRF24_CE_Pin|NRF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_RX_DR_Pin */
  GPIO_InitStruct.Pin = NRF24_RX_DR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24_RX_DR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24_CE_Pin NRF24_CSN_Pin */
//  GPIO_InitStruct.Pin = NRF24_CE_Pin|NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
