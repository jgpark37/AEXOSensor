/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems��s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_TorqueLoad.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2018/01/02
* Description		: Torque sensor/Load cell
*******************************************************************************
*
*/

#ifndef TORQUE_LOAD_H_
#define TORQUE_LOAD_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/

#define ADC_INPUT_VOLTAGE_LOAD			5000U
#define ADC_INPUT_VOLTAGE_TORQ			5000U
#define ADC_RESOLUTION						65536U
#define ADC_RESOLUTION_BIT					16
#define LOADCELL_ADC_PER_1mV				(ADC_RESOLUTION/ADC_INPUT_VOLTAGE_LOAD) //13.1
#define TORQUE_ADC_PER_1mV				(ADC_RESOLUTION/ADC_INPUT_VOLTAGE_TORQ) //13.1

/* Private typedef -----------------------------------------------------------*/
enum {//ADCIC_BUF_LIST{
	ADCIC_BUF_TORQ1,
	ADCIC_BUF_TORQ2,
	ADCIC_BUF_LOAD,
	ADCIC_BUF_MAX,
};
	
enum { //torq cell drive line
	TCDL_LINE1,
	TCDL_LINE2,
	TCDL_MAX
};

typedef struct tagTORQUE_DRV {
	uint16_t data;
	float_t2 a;
	float_t2 b; //y = ax + b
	uint32_t oldValue;
	uint32_t sameCnt;
	struct {
		uint8_t all;
		struct {
			unsigned int devOK					:1;
			unsigned int reserved				:31;
		}b;
	}f;
}TORQUE_DRV_INFO;

typedef struct tagLOAD_CELL_DRV {
	uint16_t data;
	float_t2 a;
	float_t2 b; //y = ax + b (y:voltage, x:weight)
	float_t2 weight; //kgf
	uint32_t oldValue;
	uint32_t sameCnt;
	struct {
		uint8_t all;
		struct {
			unsigned int devOK					:1;
			unsigned int reserved				:31;
		}b;
	}f;
}LOAD_CELL_DRV_INFO;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern TORQUE_DRV_INFO TorqDrvInfo[TCDL_MAX];
extern LOAD_CELL_DRV_INFO LoadDrvInfo;
extern uint16_t ADCICBuf[ADCIC_BUF_MAX];

/* Private function prototypes -----------------------------------------------*/
static inline void TorqDrv_Init(void)
{
	uint8_t i, j;
	uint8_t *ptr;

	for (i = 0; i < (uint8_t)TCDL_MAX; i++) {
		ptr = (uint8_t *)&TorqDrvInfo[i];
		for (j = 0; j < sizeof(TORQUE_DRV_INFO); j++) {
			ptr[j] = 0;
		}
		TorqDrvInfo[i].f.b.devOK = 1;
	}
}

static inline void LoadDrv_Init(void)
{
	uint8_t i;
	uint8_t *ptr;
	GPIO_InitTypeDef GPIO_InitStruct;

	ptr = (uint8_t *)&LoadDrvInfo;
	for (i = 0; i < sizeof(LOAD_CELL_DRV_INFO); i++) {
		ptr[i] = 0;
	}
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;	//ADC_CONVERT
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_6;	//ADC_CS
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //ADC_CONVERT
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //ADC_CS

	LoadDrvInfo.a = 0.0045f;
	LoadDrvInfo.b = 2.3916f;
	LoadDrvInfo.f.b.devOK = 1;
}

static inline void ADCICDrv_Process(void)
{
	uint8_t dummy_tx;//[2] = {0xff, 0xff};

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //ADC_CS
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //ADC_CONVERT
	(void)HAL_SPI_TransmitReceive_DMA(&hspi2, &dummy_tx, 
					(uint8_t *)ADCICBuf, (uint16_t)ADCIC_BUF_MAX);
}

#endif //TORQUE_LOAD_H_

