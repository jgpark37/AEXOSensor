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
* Filename		: hx_AbsEncoder.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Abs Encoder(RMB20SC13B:SSI, RS422, resolution:8192)
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_AbsEncoder.h"
//#include "main.h"

/* Private define ------------------------------------------------------------*/
//#define TEST_SW_METHOD

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ABS_ENC_DRV_INFO AEncDrvInfo[AEDCH_MAX];

/* Private function prototypes -----------------------------------------------*/

