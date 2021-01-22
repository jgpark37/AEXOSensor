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
* in deciphering, decoding, reverse engineering or in any way altering the source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_ICD.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: ICD
*******************************************************************************
*
*/

#ifndef ICD_H_PJG_
#define ICD_H_PJG_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private define ------------------------------------------------------------*/
//#define USE_NOT_CHECK_SENSOR				
#define LEFT_SIDE_BOARD
#ifdef LEFT_SIDE_BOARD
#define CAN_RX_STD_ID				0x601
#define CAN_TX_STD_ID				0x581
#else
#define CAN_RX_STD_ID				0x602
#define CAN_TX_STD_ID				0x591
#endif
#define ICD_ERR_CHK_CNT			10000

/* Private typedef -----------------------------------------------------------*/
typedef struct tagICD {
	uint32_t errcode;
	uint8_t imuSendcnt;
	struct {
		uint8_t all;
		struct {
			unsigned int autoNotics				:1;		
			unsigned int initEnd				:1;		
			unsigned int reserved				:30;
		}b;
	}f;
}ICD;

//enum eICDERROR_CODE {
	//ICD_EC_NONE,
#define	ICD_EC_IMU			0x00000001
#define	ICD_EC_ENC1			0x00000100
#define	ICD_EC_ENC2			0x00001000
#define	ICD_EC_TORQ			0x00010000
#define	ICD_EC_TORQ2		0x00100000
#define	ICD_EC_LOAD			0x01000000
	//ICD_EC_MAX
//};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ICD icd;
//extern uint32_t LED_tick;	//1ms
//extern uint16_t LED_BlinkCnt;
//
extern void (*pfnSendData)(void);

/* Private function prototypes -----------------------------------------------*/
extern void ICD_SendData(void);
//extern void ICD_Init(void);
static inline void ICD_NullProcess(void)
{
}

static inline void ICD_Init(void)
{	
	pfnSendData = ICD_NullProcess;//ICD_SendData;
	icd.f.b.initEnd = 0;
}

static inline void ICD_Start(void)
{
	icd.f.b.initEnd = 1U;//value;
	pfnSendData = ICD_SendData;
}

#endif //_ICD_H_PJG_

