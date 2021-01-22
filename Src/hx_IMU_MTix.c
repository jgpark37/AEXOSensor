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
* Filename		: hx_IMU_MTix.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F722
* Compiler		: IAR
* Created      	: 2017/11/21
* Description		: Xsens MTi-series
*******************************************************************************
*
*/
/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_IMU_MTix.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
//Data identifier
#define DI_GROUP_ORIENTATION					0x20
#define IMU_CMD_REQ_RETRY_NUM					5U

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IMU_MTIX_INFO MTixDrvInfo;
void (*pfnMTixDrv_Process)(void);
void (*pfnMTixDrv_Start)(void);

/* Private function prototypes -----------------------------------------------*/


////////////////////////////Function////////////////////////////////////////
static void MTixDrv_WaitRequestMsg(uint32_t timeout)
{
	IMU_tick = timeout;
	while (IMU_tick > 0U) {
		if (MTixDrvInfo.f.b.acked == 1U) {
			break;
		}
	}
}

static void MTixDrv_MakeMessage(uint8_t *buf, uint16_t data_len)
{
	uint16_t i;
	uint8_t chksum;
	uint32_t temp;

	buf[3] = (uint8_t)(data_len%(uint16_t)MTI_TX_BUF_SIZE);
	chksum = 0;
	for (i = 1U; i < (data_len+4U); i++) {
		chksum += buf[i];
	}
	buf[i] = (uint8_t)((255U-chksum)+1U);
	temp = (uint32_t)i+1U;
	(void)HAL_UART_Transmit(&huart3, buf, i+1U, temp);
	MTixDrvInfo.f.b.acked = 0;
}

// memcpy to reverse
static void MTixDrv_MemCpyR(uint8_t *des, const uint8_t *src, uint8_t cnt)
{
	uint8_t i;

	for (i = 0; i < cnt; i++) {
		des[i] = src[cnt-i-1U]; 
	}
}

static void MTixDrv_MemCpy(uint8_t *des, const uint8_t *src, uint8_t cnt)
{
	uint8_t i;

	for (i = 0; i < cnt; i++) {
		des[i] = src[i]; 
	}
}

static void MTixDrv_ParsingVarInit(void)
{
	MTixDrvInfo.pack.f.b.preamble = 0;
	MTixDrvInfo.pack.f.b.busID = 0;
	MTixDrvInfo.pack.dataNum = 0;
	MTixDrvInfo.pack.msgBuf[2] = 0;
}

static void MTixDrv_GoToConfig(void)
{
	MTixDrvInfo.pack.txBuf[2] = CMT_MID_GOTOCONFIG;
	MTixDrvInfo.f.b.goToConfigAck = 0;
	MTixDrv_MakeMessage(MTixDrvInfo.pack.txBuf, 0);
}

static void MTixDrv_GoToMeasurement(void)
{
	MTixDrvInfo.pack.txBuf[2] = CMT_MID_GOTOMEASUREMENT;
	MTixDrvInfo.f.b.goToMeasureAck = 0;
	MTixDrv_MakeMessage(MTixDrvInfo.pack.txBuf, 0);
}

static void MTixDrv_Reset(void)
{
	MTixDrvInfo.pack.txBuf[2] = CMT_MID_RESET;
	MTixDrvInfo.f.b.reset = 0;
	MTixDrv_MakeMessage(MTixDrvInfo.pack.txBuf, 0);
}

static void MTixDrv_ReqDID(void)
{
	MTixDrvInfo.pack.txBuf[2] = CMT_MID_REQDID;
	MTixDrvInfo.deviceId = 0;
	MTixDrvInfo.f.b.deviceIDAck = 0;
	MTixDrv_MakeMessage(MTixDrvInfo.pack.txBuf, 0);
}

static void MTixDrv_DeviceID(const uint8_t *buf)
{
	MTixDrv_MemCpy((uint8_t *)&MTixDrvInfo.deviceId, buf, 4);
}

static void MTixDrv_SetOutputConfiguration(struct OutputConfiguration *pconf, uint8_t n)
{
	uint8_t i, j;
	uint8_t *ptr;
	uint16_t temp;
	
	j = 0;
	MTixDrvInfo.pack.txBuf[2] = CMT_MID_SETOUTPUTCONFIGURATION;
	ptr = (uint8_t *)pconf;
	for (i = 0; i < (n*4U); i += 4U) {
		MTixDrvInfo.pack.txBuf[4U+i] = ptr[j+1U];	//value H
		MTixDrvInfo.pack.txBuf[5U+i] = ptr[j];				//value L
		MTixDrvInfo.pack.txBuf[6U+i] = ptr[j+3U];	//dtype H
		MTixDrvInfo.pack.txBuf[7U+i] = ptr[j+2U];	//dtype L
		j += 4U;
	}
	temp = ((uint16_t)n*4U);
	MTixDrvInfo.f.b.setOutConfigAck = 0;
	MTixDrv_MakeMessage(MTixDrvInfo.pack.txBuf, temp);
}

static void MTixDrv_OrientationData(const uint8_t *buf)
{
	MTixDrv_MemCpyR((uint8_t *)&MTixDrvInfo.ori.roll, &buf[0], 4);
	MTixDrv_MemCpyR((uint8_t *)&MTixDrvInfo.ori.pitch, &buf[4], 4);
	MTixDrv_MemCpyR((uint8_t *)&MTixDrvInfo.ori.yaw, &buf[8], 4);
}

static void MTixDrv_Parsing_MTData2(const uint8_t *buf, uint16_t len)
{
	uint16_t i;
	uint8_t group;
	uint8_t size;

	MTixDrvInfo.items = 0;
	i = 0;
	while (i < len) {
		group = buf[i];
		size = buf[i+2U];
		i += 3U;
		MTixDrvInfo.items++;
		if (group == (uint8_t)DI_GROUP_ORIENTATION) {
			MTixDrv_OrientationData(&buf[i]);
		}
		i += size;
	}
}

static void MTixDrv_Protocol(void)
{
	uint8_t *ptr;

	ptr = &MTixDrvInfo.pack.msgBuf[4];

	switch (MTixDrvInfo.pack.mid) {
		case CMT_MID_RESETACK:
			MTixDrvInfo.f.b.acked = 1;
			MTixDrvInfo.f.b.reset = 1;
			break;
		case CMT_MID_DEVICEID:
			MTixDrvInfo.f.b.acked = 1;
			MTixDrvInfo.f.b.deviceIDAck = 1;
			MTixDrv_DeviceID(ptr);
			break;
		case CMT_MID_MTDATA2:
			MTixDrvInfo.f.b.acked = 1;
			MTixDrvInfo.f.b.goToMeasureAck = 1;
			MTixDrv_Parsing_MTData2(ptr, MTixDrvInfo.pack.len);
			break;
		case CMT_MID_GOTOCONFIGACK:
			MTixDrvInfo.f.b.acked = 1;
			MTixDrvInfo.f.b.goToConfigAck = 1;
			break;
		default :
			if (MTixDrvInfo.pack.mid == (uint8_t)CMT_MID_SETOUTPUTCONFIGURATIONACK) {
				MTixDrvInfo.f.b.acked = 1;
				MTixDrvInfo.f.b.setOutConfigAck = 1;
			}
			break;
	}
}

static uint8_t MTixDrv_ReceivedMsg(void)
{
	uint8_t ret;

	ret = 0;
	switch (MTixDrvInfo.pack.dataNum) {
		case 2: //message id position
			MTixDrvInfo.pack.mid = MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
			MTixDrvInfo.pack.f.b.extLen = 0;
			MTixDrvInfo.pack.headerSize = 4;
			break;
		case 3: //length position
			MTixDrvInfo.pack.len = MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
			break;
		case 4: //start of extended length position
			if (MTixDrvInfo.pack.dataNum == 
				(MTixDrvInfo.pack.len+MTixDrvInfo.pack.headerSize)) {
				if (MTixDrvInfo.pack.checksum == 0U) {
					ret = 1;
				}
				MTixDrvInfo.pack.f.b.preamble = 0;
			}
			break;
		default:
			if (MTixDrvInfo.pack.dataNum >= 
				(MTixDrvInfo.pack.len+MTixDrvInfo.pack.headerSize)) {
				if (MTixDrvInfo.pack.checksum == 0U) {
					ret = 1;
				}
				MTixDrvInfo.pack.f.b.preamble = 0;
			}
			break;
	}

	return ret;
}

static void MTixDrv_ReceivingMsg(void)
{
	switch (MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail]) {
		case CMT_PREAMBLE:				//preamble
			MTixDrvInfo.pack.f.b.preamble = 1;
			MTixDrvInfo.pack.f.b.busID = 0;
			MTixDrvInfo.pack.dataNum = 0;
			MTixDrvInfo.pack.msgBuf[MTixDrvInfo.pack.dataNum] = 
								MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
			break;
		case CMT_BID_MASTER:				//bus id or address
			if ((MTixDrvInfo.pack.f.b.preamble == 1U) && (MTixDrvInfo.pack.dataNum == 0U)) {
				MTixDrvInfo.pack.f.b.busID = 1;
				MTixDrvInfo.pack.dataNum = 1;
				MTixDrvInfo.pack.checksum = MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
				MTixDrvInfo.pack.msgBuf[MTixDrvInfo.pack.dataNum] = 
									MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
			}
			break;
		default:
			MTixDrvInfo.pack.f.b.preamble = 0;
			break;
	}
}

static void MTixDrv_configureMotionTracker(void)
{
	uint8_t i;//, j;

	MTixDrv_GoToConfig();
	MTixDrvInfo.f.b.goToConfigAck = 0;
	i = 0;
	while (i < IMU_CMD_REQ_RETRY_NUM) {
		MTixDrv_WaitRequestMsg(600);
		if (MTixDrvInfo.f.b.goToConfigAck == 1U) {
			i = IMU_CMD_REQ_RETRY_NUM+10U;
		}
		else {
			MTixDrv_GoToConfig();
		}
		i++;
	}
	MTixDrv_ParsingVarInit();
	
	MTixDrv_ReqDID();
	MTixDrvInfo.f.b.deviceIDAck = 0;
	i = 0;
	while (i < IMU_CMD_REQ_RETRY_NUM) {
		MTixDrv_WaitRequestMsg(600);
		if (MTixDrvInfo.f.b.deviceIDAck == 1U) {
			i = IMU_CMD_REQ_RETRY_NUM+10U;
		}
		else {
			MTixDrv_ReqDID();
		}
		i++;
	}
	MTixDrv_ParsingVarInit();

	if (MTixDrvInfo.f.b.deviceIDAck == 1U) {
		MTixDrv_SetOutputConfiguration(MTixDrvInfo.conf.buf, MTixDrvInfo.conf.cnt);
		MTixDrvInfo.f.b.setOutConfigAck = 0;
		i = 0;
		while (i < IMU_CMD_REQ_RETRY_NUM) {
			MTixDrv_WaitRequestMsg(300U*(uint32_t)MTixDrvInfo.conf.cnt);
			if (MTixDrvInfo.f.b.setOutConfigAck == 1U) {
				MTixDrvInfo.f.b.devOK = 1;
				i = IMU_CMD_REQ_RETRY_NUM+10U;
			}
			MTixDrv_SetOutputConfiguration(MTixDrvInfo.conf.buf, MTixDrvInfo.conf.cnt);
			i++;
		}
		MTixDrv_ParsingVarInit();
	}
}

static void MTixDrv_measureMotionTracker(void)
{
	uint8_t i;
	
	MTixDrv_GoToMeasurement();
	MTixDrvInfo.f.b.goToMeasureAck = 0;
	i = 0;
	while (i < IMU_CMD_REQ_RETRY_NUM) {
		MTixDrv_WaitRequestMsg(600);
		if (MTixDrvInfo.f.b.goToMeasureAck == 1U) {
			MTixDrvInfo.f.b.devOK = 1;
			i = IMU_CMD_REQ_RETRY_NUM+10U;
		}
		else {
			MTixDrv_GoToMeasurement();
		}
		i++;
	}
}

void MTixDrv_Process(void)
{
	if (MTixDrvInfo.pack.head != MTixDrvInfo.pack.tail) {
		if ((MTixDrvInfo.pack.f.b.preamble == 1U) && (MTixDrvInfo.pack.f.b.busID == 1U)) {
			MTixDrvInfo.pack.dataNum++;
			if (MTixDrvInfo.pack.dataNum < (uint16_t)MTI_MSG_BUF_SIZE) {
				MTixDrvInfo.pack.msgBuf[MTixDrvInfo.pack.dataNum] = 
										MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
				MTixDrvInfo.pack.checksum += MTixDrvInfo.pack.buf[MTixDrvInfo.pack.tail];
				if (MTixDrv_ReceivedMsg() == 1U) {
					MTixDrv_Protocol();
				}
			}
		}
		else {
			MTixDrv_ReceivingMsg();
		}		
		MTixDrvInfo.pack.tail++;
		if (MTixDrvInfo.pack.tail >= (uint16_t)MTI_RX_BUF_SIZE) {
			MTixDrvInfo.pack.tail = 0;
		}
	}
}

static void MTixDrv_resetMotionTracker(void)
{
	uint8_t i;

	MTixDrv_Reset();
	MTixDrvInfo.f.b.reset = 0;
	i = 0;
	while (i < IMU_CMD_REQ_RETRY_NUM) {
		MTixDrv_WaitRequestMsg(800);
		if (MTixDrvInfo.f.b.reset == 1U) {
			i = IMU_CMD_REQ_RETRY_NUM+10U;
		}
		else {
			MTixDrv_Reset();
		}
		i++;
	}
}

static void MTixDrv_InitConfiguration(void)
{
	uint8_t i;

	i = 0;
	MTixDrvInfo.conf.buf[i].dtype = XDI_EulerAngles;
	MTixDrvInfo.conf.buf[i].freq = 400;
	i++;
	MTixDrvInfo.conf.cnt = i;
}

void MTixDrv_Start(void)
{
	MTixDrv_InitConfiguration();

	MTixDrv_resetMotionTracker();
	MTixDrv_configureMotionTracker();
	MTixDrv_measureMotionTracker();
}


