/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems¡¯s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_AbsEncoder.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Abs Encoder
*******************************************************************************
*
*/

#ifndef ABS_ENCODER_H_PJG_
#define ABS_ENCODER_H_PJG_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"

/* Private define ------------------------------------------------------------*/
#define ABS_ENC_RES						(4096-1)

/* Private typedef -----------------------------------------------------------*/
typedef struct tagABS_ENCODER_DRV {
	uint16_t read;
	uint16_t data;
	uint32_t oldValue;
	uint32_t sameCnt;
	struct {
		uint8_t all;
		struct {
			unsigned int devOK					:1;
			unsigned int reserved				:31;
		}b;
	}f;
}ABS_ENC_DRV_INFO;

enum {//AED_CHANNEL{
	AEDCH_HIP,
	AEDCH_KNEE,
	AEDCH_MAX
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ABS_ENC_DRV_INFO AEncDrvInfo[AEDCH_MAX];
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

/* Private function prototypes -----------------------------------------------*/
//extern void AbsEncDrv_Process(uint8_t node);
static inline void AbsEncDrv_Process(uint8_t node)
{
	uint8_t dummy_tx;//[2] = {0xff, 0xff};
	
	if (node == (uint8_t)AEDCH_HIP) {
		(void)HAL_SPI_TransmitReceive_DMA(&hspi1, &dummy_tx, 
			(uint8_t *)&AEncDrvInfo[node].read, 1);
	}
	else {
		(void)HAL_SPI_TransmitReceive_DMA(&hspi3, &dummy_tx, 
			(uint8_t *)&AEncDrvInfo[node].read, 1);
	}
}

static inline void AbsEncDrv_Init(uint8_t node)
{
	uint8_t i;
	uint8_t *ptr;

	ptr = (uint8_t *)&AEncDrvInfo[node];
	for (i = 0; i < sizeof(ABS_ENC_DRV_INFO); i++) {
		ptr[i] = 0;
	}

	AEncDrvInfo[node].f.b.devOK = 1;
}

#endif //ABS_ENCODER_H_PJG_

