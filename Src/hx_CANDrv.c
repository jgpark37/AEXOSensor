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
* Filename		: hx_CAN.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f7xx_hal.h"
#include "hx_CANDrv.h"
//#include "Utilities.h"
//#include "cmsis_os.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//static CAN_RxHeaderTypeDef        RxMessage; 
CAN_TxHeaderTypeDef        TxMessage; 

CAN_INFO CanInfo;


/* Private function prototypes -----------------------------------------------*/


void CANDrv_SetStdID(uint32_t id)
{
	TxMessage.StdId = id;
}

