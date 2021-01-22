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
* Filename		: hx_IMU_MTix.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F722
* Compiler		: IAR
* Created      	: 2017/11/21
* Description		: Xsens MTi-series
*******************************************************************************
*
*/

#ifndef IMU_MTIX_H_PJG_
#define IMU_MTIX_H_PJG_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
//#include "Xbusmessage.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define MTI_RX_BUF_SIZE									257	//max 6536
#define MTI_TX_BUF_SIZE									136//(7+32*4+1)	//max 255
#define MTI_MSG_BUF_SIZE								257	//max 255
#define MTI_CONF_BUF_SIZE								5

//////////////////////////////////////////////////////////////////////////////////////////
// Field message indices
#define CMT_PREAMBLE							0xFA
#define CMT_BID_MASTER							0xFF
// All Message identifiers
#define CMT_MID_GOTOMEASUREMENTACK			0x11
// WakeUp state messages
#define CMT_MID_WAKEUP						0x3E
// Valid in all states
#define CMT_MID_RESET							0x40
#define CMT_MID_RESETACK						0x41
#define CMT_MID_CONFIGURATION					0x0D
// Config state messages
#define CMT_MID_REQDID							0x00
#define CMT_MID_DEVICEID						0x01
#define CMT_MID_INITBUS							0x02
#define CMT_MID_GOTOMEASUREMENT				0x10
#define CMT_MID_SETOUTPUTCONFIGURATION		0xC0
#define CMT_MID_SETOUTPUTCONFIGURATIONACK	0xC1
// Measurement state
#define CMT_MID_GOTOCONFIG					0x30
#define CMT_MID_GOTOCONFIGACK				0x31
#define CMT_MID_MTDATA2						0x36	//pjg++171201

/* Private typedef -----------------------------------------------------------*/
typedef struct tagXBUS_HEADER {
	uint8_t preamble;				// 0xfa
	uint8_t bid;					// bus ID address : 0x01-first device, 0xff : master device
	uint8_t mid;					// message id
	uint8_t len;					// 0xff : extended message mark
	uint8_t *data;
	uint8_t checksum;
}XBUS_HEADER;

typedef struct tagXBUS_HEADER_EX {
	uint8_t preamble;
	uint8_t bid;
	uint8_t mid;
	uint8_t len;
	uint16_t lenEx;
	uint8_t *data;
	uint8_t checksum;
}XBUS_HEADER_EX;

enum XsDataIdentifier
{
	//Timestamp
	XDI_PacketCounter  		= 0x1020,
	XDI_SampleTimeFine 	= 0x1060,
	//Orientation Data
	XDI_Quaternion     		= 0x2010,
	XDI_RotationMatrix		= 0x2020,	//pjg++180306	
	XDI_EulerAngles			= 0x2030,	//pjg++180306	
	//Acceleration
	XDI_DeltaV         		= 0x4010,
	XDI_Acceleration   		= 0x4020,
	XDI_FreeAcceleration   	= 0x4030,	//pjg++180306
	//Angular Velocity(Gyro)
	XDI_RateOfTurn     		= 0x8020,
	XDI_DeltaQ         		= 0x8030,
	//Magnetic
	XDI_MagneticField  		= 0xC020,
	//Status
	XDI_StatusWord     		= 0xE020,
};

struct OutputConfiguration
{
	/*! \brief Data type of the output. */
	enum XsDataIdentifier dtype;
	/*!
	 * \brief The output frequency in Hz, or 65535 if the value should be
	 * included in every data message.
	 */
	uint16_t freq;
};

typedef struct tagIMU_MTIX_INFO {
	//protocol
	struct {
		uint8_t buf[MTI_RX_BUF_SIZE];
		uint16_t head;
		uint16_t tail;
		uint8_t msgBuf[MTI_MSG_BUF_SIZE];
		uint8_t txBuf[MTI_TX_BUF_SIZE];
		uint16_t dataNum;
		uint8_t headerSize;
		uint16_t len;
		uint8_t mid;
		uint8_t checksum;
		struct {
			uint8_t all;
			struct {
				unsigned int preamble				:1;
				unsigned int busID					:1;
				unsigned int extLen					:1;
				unsigned int reserved				:29;
			}b;
		}f;
	}pack;
	uint32_t deviceId;
	//
	// sensor data
	uint8_t items;	//Number of items in MTData2
	//Temperature
	struct {
		float_t2 temp;
	}temp;	//Temperature, 1 Hz
	struct {
		uint16_t PacketCounter; //PacketCounter
	}time;	//Timestamp, 2000 Hz
	struct {
		float_t2 roll, pitch, yaw; //Euler Angles
	}ori; //Orientation, 400 Hz

	struct {
		uint8_t byte; //Status
		uint32_t word; //Status
		uint32_t Rssi;
	}status; //Status, 2000 Hz
	struct {
		struct OutputConfiguration buf[MTI_CONF_BUF_SIZE];
		uint8_t cnt;
	}conf;
	struct {
		uint8_t all;
		struct {
			unsigned int acked					:1;
			unsigned int goToConfigAck			:1;
			unsigned int devOK					:1;
			unsigned int reset					:1;
			unsigned int setOutConfigAck		:1;
			unsigned int goToMeasureAck			:1;
			unsigned int deviceIDAck			:1;
			unsigned int reserved				:25;
		}b;
	}f;
	uint32_t oldValue;
	uint32_t sameCnt;
}IMU_MTIX_INFO;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern IMU_MTIX_INFO MTixDrvInfo;
extern uint32_t IMU_tick;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern void (*pfnMTixDrv_Process)(void);
extern void (*pfnMTixDrv_Start)(void);


/* Private function prototypes -----------------------------------------------*/
extern void MTixDrv_Process(void);
extern void MTixDrv_Start(void);
//extern void MTixDrv_Init(void);
static inline void MTixDrv_Init(void)
{
	uint16_t i;
	uint8_t *ptr;

	ptr = (uint8_t *)&MTixDrvInfo;
	for (i = 0; i < sizeof(IMU_MTIX_INFO); i++) {
		ptr[i] = 0;
	}
	MTixDrvInfo.pack.txBuf[0] = CMT_PREAMBLE;	//Preamble
	MTixDrvInfo.pack.txBuf[1] = CMT_BID_MASTER;	//BID

	pfnMTixDrv_Start = MTixDrv_Start;
	pfnMTixDrv_Process = MTixDrv_Process;
}

#endif //IMU_MTIX_H_PJG_

