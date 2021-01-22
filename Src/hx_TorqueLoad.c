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
* Filename		: hx_TorqueLoad.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2018/01/02
* Description		: Torque sensor/Load cell
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_TorqueLoad.h"
//#include "main.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TORQUE_DRV_INFO TorqDrvInfo[TCDL_MAX];
LOAD_CELL_DRV_INFO LoadDrvInfo;
uint16_t ADCICBuf[ADCIC_BUF_MAX];

/* Private function prototypes -----------------------------------------------*/

