/**********************************************************************************************************************
 * @file  CanBusTask.h
 *
*** @版权归属:IMCA战队
*                        ___          ___          ___
*             ___       /\__\        /\  \        /\  \
*            /\  \     /::|  |      /::\  \      /::\  \
*            \:\  \   /:|:|  |     /:/\:\  \    /:/\:\  \
*            /::\__\ /:/|:|__|__  /:/  \:\  \  /::\~\:\  \
*         __/:/\/__//:/ |::::\__\/:/__/ \:\__\/:/\:\ \:\__\
*        /\/:/  /   \/__/~~/:/  /\:\  \  \/__/\/__\:\/:/  /
*        \::/__/          /:/  /  \:\  \           \::/  /
*         \:\__\         /:/  /    \:\  \          /:/  /
*          \/__/        /:/  /      \:\__\        /:/  /
*                       \/__/        \/__/        \/__/
**********************************************************************************************************************/
#ifndef __CANBUSTASK
#define __CANBUSTASK
#include "stm32g4xx_hal.h"
#include "math.h"
#include "fdcan.h"
#include "Control_buck_boost.h"
// #include "GimbalControl.h"
// #include "UserLib.h"

#define FILTER_BUF_LEN		5

typedef uint16_t 	u16;
typedef int16_t 	s16;
typedef uint8_t 	u8;
typedef uint32_t 	u32;

/*定义CAN发送或是接收的ID*/
typedef enum
{
	CAN1_CAP_RECEIVE_ID = 0x107, //超级电容接受主控帧
  	CAN1_CAP_SEND_ID = 0x108, //超级电容发送帧
  	
	CAN_3508Motor1_ID = 0x201,
	CAN_3508Motor2_ID = 0x202,
	CAN_3508Motor3_ID = 0x203,
	CAN_3508Motor4_ID = 0x204,
	CAN_2006Motor_ID  = 0x205,
	CAN_YAW_Motor3_ID = 0x207, //YAW电机ID3
//	CAN2_IMU_ID = 0x401,  
//  CAN2_IMU_ID2 = 0x402, 这里是C板板间通信的id，用于传输陀螺仪的数据
//  CAN2_IMU_ID3 = 0x403,
}CAN_Message_ID;



/*CAN1过滤器的配置和CAN的开启*/
void CANFilterInit(void);


/*发送Byte信息*/
void SendByteData(FDCAN_HandleTypeDef *hcan,uint32_t id,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8);
/*发送HalfWord信息*/
void SendHalfWordData(FDCAN_HandleTypeDef *hcan,uint32_t id,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void Cap_Ctrler_SendCanData(FDCAN_HandleTypeDef *hcan,uint32_t id,sample_FloatData *p_Sample_Data);   /*发送超电信息*/

#endif

