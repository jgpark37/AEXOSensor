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

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_ICD.h"
#include "hx_CANDrv.h"
#include "hx_IMU_MTix.h"
#include "hx_AbsEncoder.h"
#include "hx_TorqueLoad.h"

/* Private define ------------------------------------------------------------*/
#define ICD_DATA_FLOAT_TO_INT100				100

//
//=========================================================

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ICD icd;
void (*pfnSendData)(void);
//uint16_t LED_BlinkCnt; //pjg++190628

/* Private function prototypes -----------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////////////
static uint8_t ICD_Make1stData(void)
{
	uint16_t temp;
	uint32_t temp32;
	uint8_t ret;

	ret = 1U;
	CanInfo.tx.buf[0][0] = (uint8_t)AEncDrvInfo[AEDCH_HIP].data;
	CanInfo.tx.buf[0][1] = (uint8_t)(AEncDrvInfo[AEDCH_HIP].data>>8);
	
	//torque 1ch
	temp32 = (((uint32_t)TorqDrvInfo[TCDL_LINE1].data*10U)*(uint32_t)ADC_INPUT_VOLTAGE_TORQ);	//torque
	temp32 = (temp32>>(uint8_t)ADC_RESOLUTION_BIT);
	temp = (uint16_t)temp32;
	CanInfo.tx.buf[0][2] = (uint8_t)temp;
	CanInfo.tx.buf[0][3] = (uint8_t)(temp>>8);

	//torque 2ch
	temp32 = ((uint32_t)TorqDrvInfo[TCDL_LINE2].data*10U)*(uint32_t)ADC_INPUT_VOLTAGE_TORQ;	//torque2
	temp32 = (temp32>>(uint8_t)ADC_RESOLUTION_BIT);
	temp = (uint16_t)temp32;
	CanInfo.tx.buf[0][4] = (uint8_t)temp;
	CanInfo.tx.buf[0][5] = (uint8_t)(temp>>8);
	
	CanInfo.tx.buf[0][6] = (uint8_t)AEncDrvInfo[AEDCH_KNEE].data;
	CanInfo.tx.buf[0][7] = (uint8_t)(AEncDrvInfo[AEDCH_KNEE].data>>8);
	
	return ret;
}

static uint8_t ICD_Make2ndData(void)
{
	uint16_t temp;
	uint32_t temp32;
	float_t2 ftemp;
	uint8_t ret;

	ret = 1U;
	if ((MTixDrvInfo.f.b.devOK != 0U)) {
		icd.errcode &= (uint32_t)(~((uint32_t)ICD_EC_IMU)); //normal
	}
	else {
		icd.errcode |= (uint32_t)ICD_EC_IMU;
		ret = 0U;
	}

	ftemp = MTixDrvInfo.ori.roll*(float_t2)ICD_DATA_FLOAT_TO_INT100;
	temp = (uint16_t)ftemp;//(MTixDrvInfo.ori.roll*ICD_DATA_FLOAT_TO_INT100);
	CanInfo.tx.buf[0][0] = (uint8_t)temp;
	CanInfo.tx.buf[0][1] = (uint8_t)(temp>>8);
	ftemp = MTixDrvInfo.ori.pitch*(float_t2)ICD_DATA_FLOAT_TO_INT100;
	temp = (uint16_t)ftemp;
	CanInfo.tx.buf[0][2] = (uint8_t)temp;
	CanInfo.tx.buf[0][3] = (uint8_t)(temp>>8);
	ftemp = MTixDrvInfo.ori.yaw*(float_t2)ICD_DATA_FLOAT_TO_INT100;
	temp = (uint16_t)ftemp;
	CanInfo.tx.buf[0][4] = (uint8_t)temp;
	CanInfo.tx.buf[0][5] = (uint8_t)(temp>>8);

	temp32 = (uint32_t)LoadDrvInfo.data*(uint32_t)ADC_INPUT_VOLTAGE_LOAD;	//load
	temp32 = (temp32>>(uint8_t)ADC_RESOLUTION_BIT);
	temp = (uint16_t)temp32;
	CanInfo.tx.buf[0][6] = (uint8_t)temp;
	CanInfo.tx.buf[0][7] = (uint8_t)(temp>>8);

	return ret;
}

#ifndef USE_ERROR_IGNORE
static void ICD_MakeErrCodeData(uint32_t code)
{
	CanInfo.tx.buf[0][0] = (uint8_t)code;
	CanInfo.tx.buf[0][1] = (uint8_t)(code>>8);
	CanInfo.tx.buf[0][2] = (uint8_t)(code>>16);
	CanInfo.tx.buf[0][3] = (uint8_t)(code>>24);

	CanInfo.tx.buf[0][4] = 0;
	CanInfo.tx.buf[0][5] = 0;
	CanInfo.tx.buf[0][6] = 0;
	CanInfo.tx.buf[0][7] = 0;
}
#endif

//send 1ms freq
void ICD_SendData(void)
{
	(void)ICD_Make1stData();
	CANDrv_SetStdID(SID_T_BOARD);
	(void)CANDrv_WriteFile(&hcan1, CanInfo.tx.buf[0], 8);

	if (ICD_Make2ndData() != 0U) {
		CANDrv_SetStdID(SID_T_BOARD+1);
	}
	else {
		ICD_MakeErrCodeData(icd.errcode);
		CANDrv_SetStdID(SID_T_BOARD+2);
	}
	(void)CANDrv_WriteFile(&hcan1, CanInfo.tx.buf[0], 8);
}

