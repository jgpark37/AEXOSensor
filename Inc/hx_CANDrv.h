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
* Filename		: hx_CANDrv.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

#ifndef CAN_H__
#define CAN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private define ------------------------------------------------------------*/
//#define USE_CAN_SET_ID
#define CAN_PORT_CNT							1
#define CAN_RX_BUF_NUM						(5)
#define CAN_TX_BUF_NUM							(8)

#define SUPPORT_LEFT_BD
#define SID_R_BASE								0x600
#define SID_T_BASE								0x310
#ifdef SUPPORT_LEFT_BD
#define SID_T_BOARD								( SID_T_BASE + 0 )
#define SID_R_BOARD								( SID_R_BASE + 1 )
#elif
#define SID_T_BOARD								( SID_T_BASE + 10 )
#define SID_R_BOARD								( SID_R_BASE + 2 )
#endif

/* Private typedef -----------------------------------------------------------*/
typedef struct tagCAN_INFO{
	struct {
		uint8_t head;
		uint8_t tail;
		uint8_t buf[CAN_RX_BUF_NUM][8];
		uint32_t id;
		uint8_t status;
	}rx;
	struct {
		uint8_t buf[CAN_TX_BUF_NUM][8]; 
		uint8_t cnt;
		uint32_t id;
		uint8_t status;
	}tx;
	uint32_t run_time;
	uint8_t commErr;
}CAN_INFO;

typedef struct tagCAN_PORT{
	uint32_t runTime;
}CAN_PORT;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

extern CAN_INFO CanInfo;
//extern CAN_RxHeaderTypeDef        RxMessage; 
extern CAN_TxHeaderTypeDef        TxMessage; 

/* Private function prototypes -----------------------------------------------*/
extern void CANDrv_SetStdID(uint32_t id);

static inline void CANDrv_WriteFile(CAN_HandleTypeDef *phcan, uint8_t *buf, uint8_t cnt)
{
	//uint32_t TxMailbox;

	TxMessage.DLC = cnt;
	uint32_t txMailBox = HAL_CAN_GetTxMailboxesFreeLevel(phcan);
	(void)HAL_CAN_AddTxMessage(phcan, &TxMessage, (uint8_t *)buf, &txMailBox);//CanBuf[0].txbuf);
}

static inline void CANDrv_Init(void)
{
	static CAN_FilterTypeDef myFilter;
	
	myFilter.FilterBank = 0; 
	myFilter.FilterMode = CAN_FILTERMODE_IDMASK; 
	myFilter.FilterScale = CAN_FILTERSCALE_16BIT; 
	myFilter.FilterIdHigh = 0;//CAN_STD_ID<<5;//0x0000; 
	myFilter.FilterIdLow = 0x0000; 
	myFilter.FilterMaskIdHigh = 0;//(CAN_STD_ID<<3)>>16;//0x0000; 
	myFilter.FilterMaskIdLow = 0;//((CAN_STD_ID<<3)&0xffff)|(0x1<<2);//0x0000; 
	myFilter.FilterFIFOAssignment = CAN_RX_FIFO0; 
	myFilter.FilterActivation = (uint32_t)DISABLE;//ENABLE; 
	myFilter.SlaveStartFilterBank = 0;//14;//0;
	(void)HAL_CAN_ConfigFilter(&hcan1, &myFilter);
		
	TxMessage.StdId = 0;//CAN_MOTOR_ID1&0x7FF; 
	TxMessage.ExtId = 0;//CAN_STD_ID&0x1FFFFFFF; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD; 
	TxMessage.DLC = 8; 

	//HAL_CAN_ConfigFifoWatermark(&hcan1, CAN_CFG_RX_FIFO0, fifo0RxNum);
	(void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	CanInfo.rx.head = 0;
	CanInfo.rx.tail = 0;
	CanInfo.rx.status = 0;
	CanInfo.tx.id = SID_T_BOARD;
	CanInfo.rx.id = SID_R_BOARD;
	CANDrv_SetStdID(SID_T_BOARD);
	(void)HAL_CAN_Start(&hcan1);
}

#endif //_CAN_H__

