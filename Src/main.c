/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "hx_IMU_MTix.h"
#include "hx_AbsEncoder.h"
#include "hx_CANDrv.h"
#include "hx_TorqueLoad.h"
#include "hx_ICD.h"
//#include "hx_qav.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

//static TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint8_t Rx_data;//[4];
//static uint32_t Tim2_tick;	//1ms
//static uint32_t Tim2_tick2;	//1ms
static uint32_t LED_tick;	//1ms
uint32_t IMU_tick;	//1ms
static uint16_t LED_BlinkCnt; //pjg++190628
static int8_t uart1_msg[2] = {"12"};
//uint8_t SPI_wTransferState[SPI_NUM_MAX];
//static uint8_t mainWhileCtrl;
//struct Test {
//	uint8_t quit;
//};
//static struct Test test = {
//	.quit = 0,
//};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if (huart->Instance == USART3) {
		MTixDrvInfo.pack.buf[MTixDrvInfo.pack.head] = Rx_data;//[2];
		MTixDrvInfo.pack.head++;
		if (MTixDrvInfo.pack.head >= (uint16_t)MTI_RX_BUF_SIZE) {
		  MTixDrvInfo.pack.head = 0;
		}
		pfnMTixDrv_Process();//MTixDrv_Process(); //try to move to main
		(void)HAL_UART_Receive_DMA(huart, (uint8_t *)&Rx_data, 1U);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	float_t2 ftemp;

	if (hspi->Instance == SPI1) {
		AEncDrvInfo[AEDCH_HIP].data = AEncDrvInfo[AEDCH_HIP].read&(uint16_t)ABS_ENC_RES;
		ADCICDrv_Process();
	}
	else if (hspi->Instance == SPI2) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //ADC_CS
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //ADC_CONVERT
		//sensor error check
		TorqDrvInfo[TCDL_LINE1].data = ADCICBuf[ADCIC_BUF_TORQ1]; //torque1
		TorqDrvInfo[TCDL_LINE2].data = ADCICBuf[ADCIC_BUF_TORQ2]; //torque1
		//load cell
		LoadDrvInfo.data = ADCICBuf[ADCIC_BUF_LOAD];
		ftemp = ((float_t2)ADCICBuf[ADCIC_BUF_LOAD]*(float_t2)ADC_INPUT_VOLTAGE_LOAD)/(float_t2)ADC_RESOLUTION;
		LoadDrvInfo.weight = (ftemp-LoadDrvInfo.b) / LoadDrvInfo.a;
	 	AbsEncDrv_Process((uint8_t)AEDCH_KNEE);
	}
	else {
		if (hspi->Instance == SPI3) {
			AEncDrvInfo[AEDCH_KNEE].data = AEncDrvInfo[AEDCH_KNEE].read&(uint16_t)ABS_ENC_RES;
			AbsEncDrv_Process((uint8_t)AEDCH_HIP);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {	//1ms
		pfnSendData();
		if (LED_tick > 0U) {
			LED_tick--;
		}
		if (IMU_tick > 0U) {
			IMU_tick--;
		}
	}
}

static void InitVar(void)
{
	Rx_data = 0;//[4];
	//Tim2_tick = 0; //1ms
	//Tim2_tick2 = 0;	//1ms
	LED_tick = 0;	//1ms
	IMU_tick = 0;	//1ms
	//mainWhileCtrl = 0;
	LED_BlinkCnt = 249;
}
static void DeviceDrv_Init(void)
{
	InitVar(); //pjg++191125
	CANDrv_Init();
	ICD_Init();
	MTixDrv_Init();
	AbsEncDrv_Init((uint8_t)AEDCH_HIP);
	AbsEncDrv_Init((uint8_t)AEDCH_KNEE);
	TorqDrv_Init();
	LoadDrv_Init();
	(void)HAL_UART_Transmit(&huart1, (uint8_t *)&uart1_msg[0], 1U, 1);
}

static void DeviceDrv_Start(void)
{
  	(void)HAL_UART_Receive_DMA(&huart3, (uint8_t *)&Rx_data, 1);

	HAL_GPIO_WritePin(LED_GRN_GPIO_Port, LED_GRN_Pin, GPIO_PIN_SET);
	(void)HAL_UART_Transmit(&huart1, (uint8_t *)&uart1_msg[1], 1U, 1);

 	AbsEncDrv_Process((uint8_t)AEDCH_HIP);
	(void)HAL_TIM_Base_Start_IT(&htim2);	//1ms
	pfnMTixDrv_Start();
	ICD_Start();
}

static void MX_Peri_Init1(void)
{
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
}

static void MX_Peri_Init2(void)
{
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
}

/* USER CODE END 0 */

int main(void)
{	
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  (void)HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  //MX_DMA_Init();
  //MX_CAN1_Init();
  ///MX_SPI1_Init();
  //MX_SPI2_Init();
  //MX_SPI3_Init();
  //MX_TIM1_Init();
  //MX_USART1_UART_Init();
  //MX_USART3_UART_Init();
  //MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  	MX_Peri_Init1(); //for qav
  	MX_Peri_Init2(); //for qav
  	DeviceDrv_Init();
	DeviceDrv_Start();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)//!mainWhileCtrl)
  {
  /* USER CODE END WHILE */
       
  /* USER CODE BEGIN 3 */
	//fnNOP();
	  if (LED_tick == 0U) {
		  LED_tick = LED_BlinkCnt;
		  HAL_GPIO_TogglePin(GPIOC, LED_GRN_Pin);
	  }
  }
  //return (0);
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
static void SystemClock_Config(void)
{	
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = 0x00010000;//RCC_HSE_ON; //for qav
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = 0x00400000;//RCC_PLLSOURCE_HSE; //for qav
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 144;//168;//180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  (void)HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;//RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;//RCC_HCLK_DIV2;

  (void)HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /**Configure the Systick interrupt time 
    */
  (void)HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/(uint32_t)1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#define USR_CAN_BTR_TS1_Pos        (16U)
#define USR_CAN_BTR_TS1_1          (uint32_t)((uint32_t)0x00000002U << USR_CAN_BTR_TS1_Pos) 
#define USR_CAN_BS1_3TQ            ((uint32_t)USR_CAN_BTR_TS1_1)
#define USR_CAN_BTR_TS2_Pos        (20U)
#define USR_CAN_BTR_TS2_0          (uint32_t)((uint32_t)0x00000001U << USR_CAN_BTR_TS2_Pos) 
#define USR_CAN_BS2_2TQ            ((uint32_t)USR_CAN_BTR_TS2_0)

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth  = CAN_SJW_1TQ;
  //hcan1.Init.BS1 = CAN_BS1_3TQ;//CAN_BS1_7TQ;
  hcan1.Init.TimeSeg1 = USR_CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = USR_CAN_BS2_2TQ;//CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  (void)HAL_CAN_Init(&hcan1);

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{	
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_13BIT;
  hspi1.Init.CLKPolarity = 0x00000002;//SPI_POLARITY_HIGH; //for qav
  hspi1.Init.CLKPhase = 0x00000001;//SPI_PHASE_2EDGE; //for qav
  hspi1.Init.NSS = 0x00000200;//SPI_NSS_SOFT; //for qav
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  (void)HAL_SPI_Init(&hspi1);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = 0x00000002;//SPI_POLARITY_HIGH; //for qav
  hspi2.Init.CLKPhase = 0x00000001;//SPI_PHASE_2EDGE; //for qav
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;//SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  (void)HAL_SPI_Init(&hspi2);
}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_13BIT;
  hspi3.Init.CLKPolarity = 0x00000002;//SPI_POLARITY_HIGH; //for qav
  hspi3.Init.CLKPhase = 0x00000001; //SPI_PHASE_2EDGE; //for qav
  hspi3.Init.NSS = 0x00000200; //SPI_NSS_SOFT; //for qav
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  (void)HAL_SPI_Init(&hspi3);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{	
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  static TIM_HandleTypeDef htim1;
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 360;//1800;//225;//1800;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;//100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  (void)HAL_TIM_Base_Init(&htim1);// != HAL_OK)

  sClockSourceConfig.ClockSource = 0x1000; //TIM_CLOCKSOURCE_INTERNAL; // for qav
  (void)HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 360;//1800;//225;//1800;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;//50;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  (void)HAL_TIM_Base_Init(&htim2);// != HAL_OK)

  sClockSourceConfig.ClockSource = 0x1000; //TIM_CLOCKSOURCE_INTERNAL; //for qav
  (void)HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  (void)HAL_UART_Init(&huart1);
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
	
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = 0x00000000U;//UART_PARITY_NONE; //for qav
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  (void)HAL_UART_Init(&huart3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_GRN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GRN_Pin */
  GPIO_InitStruct.Pin = LED_GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int_t line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	(void)HAL_UART_Transmit(&huart3, (uint8_t *)file, (uint16_t)line, (uint32_t)line);

  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
