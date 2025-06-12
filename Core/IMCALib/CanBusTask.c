/**********************************************************************************************************************
 * @file  CanBusTask.c
 * @brief CAN1�˲��������ú�CAN�Ŀ�������CAN�����Ͻ��ձ��ĺͷ��ͱ��ĵ�CAN������
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

#include "CanBusTask.h"
#include "Control_buck_boost.h"
#include "Ramp.h"
#include "scheduler.h"
#include <string.h>
#include "Vofa_send.h"
// #include "IMU_Task.h"
// #include "CapInteract.h"
// #include "ShootControl.h"
// #include "RcTask.h"
// #include "GimbalControl.h"
uint16_t can_cnt;
// uint8_t can_mode=2;

/**********************************************************************************************************************
  * @Func	 CANFilterInit
  * @Brief   CAN1�˲�������
  * @Param	 FDCAN_HandleTypeDef* _hcan
  * @Retval	 None
 *********************************************************************************************************************/
void CANFilterInit(void)
{

	FDCAN_FilterTypeDef	CAN_FilterConfigStructure;
	
	CAN_FilterConfigStructure.IdType = FDCAN_STANDARD_ID;            
	CAN_FilterConfigStructure.FilterType = FDCAN_FILTER_MASK;
	CAN_FilterConfigStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  
	CAN_FilterConfigStructure.FilterID1 = CAN1_CAP_RECEIVE_ID; 
	CAN_FilterConfigStructure.FilterID2 = 0xffff; 
	CAN_FilterConfigStructure.FilterIndex = 0; 	
	

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN_FilterConfigStructure)!=HAL_OK)
	{
			Error_Handler();
	}	
	if(HAL_FDCAN_Start(&hfdcan1)!=HAL_OK)
	{
			Error_Handler();
	}
	if(HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,FDCAN_TX_BUFFER0)!=HAL_OK)
	{
			Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
	{
		Error_Handler();
	}

	
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) 
{
	__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
	if(hfdcan->Instance == FDCAN1) 
	{
	MX_FDCAN1_Init();
	CANFilterInit();
	}
}


	
/**********************************************************************************************************************
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None 
  * @arg SUPERCAP_cmd_code
 *********************************************************************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    
	FDCAN_RxHeaderTypeDef   rx_header={0};
	uint8_t                 rx_data[8]={0};

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data); /*recive can data*/
	can_cnt++;
	// led_state[1]=1;
	if(hfdcan->Instance == FDCAN1)
	{
		if (rx_header.Identifier==CAN1_CAP_RECEIVE_ID)
		{
			{	
//超级电容
/**	
[0]		是否用缓冲，0x00：启用百分比缓冲,降低功率,以恢复缓冲值 
			0x01: 不缓冲，按照内部采样来闭环 
			0x02: 按缓冲值来闭环，自动调整功率来提高功率利用
[1]:	(uint8_t)	等级下的功率限制
[2]:	(uint8_t)	当前剩余的的缓冲值
[3]:	控制命令 @arg SUPERCAP_cmd_code
[4]:	命令参数，若无，应为0x00
[5:6]:	保留，应为0x00
[7]:	和校验 ；若和校验不通过，则不执行控制命令*/

			Cap_Data.buffer_mode= rx_data[0];   //缓冲模式
			if(rx_data[1]>130)
			{
				Cap_Data.can_powerlimit_value=45;
			}
			else if(rx_data[1]<15)
			{
				Cap_Data.can_powerlimit_value=15;
			}
			else
			{
			Cap_Data.can_powerlimit_value=rx_data[1];
			}
			Cap_Data.buffer_val=rx_data[2];
			uint8_t cmd=rx_data[3];
			uint8_t cmd_val=rx_data[4];
			// Cap_Data.Power_Limit =Cap_Data.can_powerlimit_value;
			// Cap_Data.can_powerlimit_value =Cap_Data.can_powerlimit_value;



			uint8_t sum=0;
			for (size_t i = 0; i < 7; i++)
			{
				sum+=rx_data[i];
			}
			if (sum==rx_data[7])
			{
				if (cmd==0x20)
				{
					Cap_Data.Can_cmd=cmd;
				}
				if (cmd==0xB1)
				{
					if(Cap_Data.state_core==0xE2||Cap_Data.state_core==0x00||Cap_Data.state_core==0xE3)
					{
						Cap_Data.state_core=0x00;
						Scheduler_continue(Start_task);
						Cap_Data.Can_cmd=0xAA;
					}
				}
				if (cmd==0xB2)
				{
					Scheduler_stop(Start_task);
					// if (Cap_Data.state_core==0xA1)
					// {
					// 	Cap_Data.state_core=0x00;
					// }
					Cap_Data.state_core=0xE3;
					DC_DC_stop();
				}
				if (cmd==0xB3)
				{
					if(cmd_val==0xB3)
					Scheduler_stop(wdg_flash);
				}
				if (cmd==0xC1)
				{
					if(cmd_val!=0x00)
					{
						Scheduler_set_RateTime(can_sent,cmd_val);
						Cap_Data.Can_sand=0;
						Cap_Data.Can_cmd=0xAA;
					}
					else if (cmd_val==0x00)
					{
						Scheduler_set_RateTime(can_sent,1);
						Cap_Data.Can_sand=2;
						Cap_Data.Can_cmd=0xAA;
					}
					
				}
				if (cmd==0xD1)
				{
					clamp_ui32_limit(0,100,(uint32_t*)&cmd_val);
					Cap_Data.Power_buffer_percent=cmd_val;
					Cap_Data.Can_cmd=0xAA;
				}
				// if (cmd==0xD3)
				// {
				// 	float temp=(float)cmd_val/10;
				// 	clamp_f_limit(1.3f,24.3f,&temp);
				// 	Cap_Data.Cap_full_voltage_value=temp;
				// 	if (temp>21.5f)
				// 	{
				// 		Scheduler_continue(Cap_V_Limit);
				// 	}
				// 	else
				// 		Scheduler_stop(Cap_V_Limit);
				// 	Cap_Data.Can_cmd=0xAA;
				// }				
				if (cmd==0xD4)
				{
					if(cmd_val==0x01)
					{
						Cap_Data.Cap_hight_V_mod=0;
						// pid_cap_current.MaxOutput=24.3;
					}
					if(cmd_val==0x02)
					{
						Cap_Data.Cap_hight_V_mod=2.1f;
						// pid_cap_current.MaxOutput=25.8;
					}
					Cap_Data.Can_cmd=0xAA;
				}
				

				Cap_Data.message_time_to_live=20;
				led_state[1]=1;
				
			}
			else
			{
				Cap_Data.Can_cmd=0xA9;
			}
			}
		} 
	}

}

/// @brief 
/// @param hcan 
/// @param id 
/// @param p_Sample_Data 
/// @arg SUPERCAP_state_code
void Cap_Ctrler_SendCanData(FDCAN_HandleTypeDef *hcan,uint32_t id,sample_FloatData *p_Sample_Data)   /*发送超电信息*/
{
	uint8_t tx_data[8]={0};
/**
[0]:	电容电压 
[1:2]:	电容功率		int16_t[高8位:低8位]		//瞬时
[3;4]:	底盘电机功率	int16_t[高8位:低8位]		//1ms均值
[5]:	电池裁判功率	uint8_t					//100ms均值
[6]:	状态码/响应码 			@arg SUPERCAP_state_code
[7]:	和校验 : 前7位数据的加和
*/
	int16_t cap_Volt_temp=(p_Sample_Data->sample_Cap_V * 10);
	clamp_i32_limit (0,255,(int32_t*)&cap_Volt_temp);
	tx_data[0] = (uint8_t)cap_Volt_temp; // 电容电压24.3->243

	int16_t cap_power_int16_temp = p_Sample_Data->sample_Cap_Power;
	// int16_t cap_chassis_int16_temp = Cap_Data.CHAS_Power_per0S001;
	int16_t cap_chassis_int16_temp = Sample_Data.sample_Chassis_Power;
	// uint8_t bat_int8_temp = (uint8_t)Cap_Data.BAT_Power_per0S1;
	uint8_t bat_int8_temp = (uint8_t)Sample_Data.sample_Battery_Power;
	tx_data[1] = cap_power_int16_temp >> 8;
	tx_data[2] = cap_power_int16_temp;
	tx_data[3] =cap_chassis_int16_temp>>8;
	tx_data[4] =cap_chassis_int16_temp;
	tx_data[5] =bat_int8_temp;
	if (Cap_Data.Can_cmd==0)
	{
		if(Cap_Data.state_core==0xA1)
		{
			if (Cap_Data.Cap_hight_V_mod>2)
			{
				tx_data[6] =0xA3;
			}
			else
			tx_data[6] =0xA1;
		}
		else
		tx_data[6] =Cap_Data.state_core;
	}
	else if(Cap_Data.Can_cmd==0x20)//返回版本号
	{
		tx_data[6] = Cap_Data.Soft_Version;
		Cap_Data.Can_cmd=0;
	}
	else if(Cap_Data.Can_cmd==0xAA)//返回Ok
	{
		tx_data[6] = 0xAA;
		Cap_Data.Can_cmd=0;
	}
	else if (Cap_Data.Can_cmd==0xA9)
	{
		tx_data[6] = 0xA9;
		Cap_Data.Can_cmd=0;
	}
	
	uint8_t sum=0;
	for (size_t i = 0; i < 7; i++)
	{
		sum+=tx_data[i];
	}
	tx_data[7] =sum;


	FDCAN_TxHeaderTypeDef   tx_header={0};
		tx_header.Identifier = id;
		tx_header.IdType = FDCAN_STANDARD_ID;
		tx_header.TxFrameType = FDCAN_DATA_FRAME;
		tx_header.DataLength = 0x08;
    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);

}	


void SendByteData(FDCAN_HandleTypeDef *hcan,uint32_t id,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8)
{
    
    FDCAN_TxHeaderTypeDef   tx_header={0};
    uint8_t               tx_data[8];

		tx_header.Identifier = id;
		tx_header.IdType = FDCAN_STANDARD_ID;
		tx_header.TxFrameType = FDCAN_DATA_FRAME;
		tx_header.DataLength = 0x08;
		tx_data[0] = iq1;
		tx_data[1] = iq2;
		tx_data[2] = iq3;
		tx_data[3] = iq4;
		tx_data[4] = iq5;
		tx_data[5] = iq6;
		tx_data[6] = iq7;
		tx_data[7] = iq8;
	
   HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
}	


void SendHalfWordData(FDCAN_HandleTypeDef *hcan,uint32_t id,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    
    FDCAN_TxHeaderTypeDef   tx_header={0};
    uint8_t               tx_data[8];
	
		tx_header.Identifier = id;
		tx_header.IdType = FDCAN_STANDARD_ID;
		tx_header.TxFrameType = FDCAN_DATA_FRAME;
		tx_header.DataLength = 0x08;
		tx_data[0] = (iq1 >> 8);
		tx_data[1] = iq1;
		tx_data[2] = (iq2 >> 8);
		tx_data[3] = iq2;
		tx_data[4] = (iq3 >> 8);
		tx_data[5] = iq3;
		tx_data[6] = (iq4 >> 8);
		tx_data[7] = iq4;
	
	    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header,tx_data);
}
