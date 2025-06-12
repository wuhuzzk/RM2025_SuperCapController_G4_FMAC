/******************************************************************************
*** @File           : Control_buck_boost.h
*** @Description    : DCDC控制程序
*** @Attention      : None
*** @Author         : TJL
*** @Date           : 2025/5/5
*** @版权归属:
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
******************************************************************************/
#ifndef _Control_Buck_Boost
#define _Control_Buck_Boost

#include "stm32g4xx_hal.h"
#include "hrtim.h"
#include "usart.h"
#include "Pid.h"



typedef struct
{

	uint16_t adc_Battery_I;           // 电池电流
	uint16_t adc_Cap_I_OPAmp;               // 电容电流
	uint16_t adc_Cap_I;               // 电容电流
	uint16_t adc_Pin24V_Battery_V; // 24V电压// 电池电压
	uint16_t adc_Chassis_I;           // 底盘电流,电容
	uint16_t adc_Cap_V;       // 电感温度, // 电容电压
} adc_RawData;

extern adc_RawData ADC_RawData;

typedef struct
{
								 // 电池电压
	float sample_Battery_I;       // 电池电流
	float sample_Cap_V;           // 电容电压
	float sample_Cap_I;           // 电容电流
	float sample_Pin24V_V;        // 24V母线电压
	float sample_Chassis_I;       // 底盘电流

	float sample_Battery_Power;
	float sample_Cap_Power;
	float sample_Chassis_Power;
} sample_FloatData;

/// @brief 超电控制相关数据包
typedef struct
{
	float cap_I_arm;	  // 电容目标电流
	uint32_t cap_I_arm_DAC;	  // 电容目标电流
	float cap_V_Fmac_delta;	  
	float Ratio;		  // 占空比
	uint16_t Power_Limit; // 实际功率限制
	float Power_PID_delta_arm;
	float BAT_Power_integration;
	float BAT_Power_per0S1; 	// 100ms均值
	float CHAS_Power_integration;
	float CHAS_Power_per0S001; 	// 1ms均值
	float Cap_I_integration;
	float Cap_I_per0S001; 	// 1ms均值
	float Cap_I_per1S0; 	// 1s均值

	uint8_t volatile PID_State;	 // PID运行状态   // 0:stop // 1:work // 2:I_PID //3: Power_PID //7：只计算pid，不输出pwm // 9:等待启动
	uint8_t DCDC_start_step;	//0xC1 启动完成
	uint16_t Stop_cnt;
	
	// uint8_t buffer_state;		 // 是否启用缓冲
	uint8_t buffer_val;			 // 剩余缓冲
	uint16_t Power_buffer_value; // 缓冲功率
	float Power_buffer_percent;	 // 百分比缓冲
	uint8_t buffer_mode;		 // 缓冲的方式// 0x01:保守（-2w） 0x02：均衡（缓存<60,降功率） 0x03 :激进（缓存>50，逐步提高功率）

	float Cap_farad_value;		  // 测量的电容值
	float Cap_full_voltage_measured; // 测量的最大电压
	float Cap_full_voltage_value; // 设定的最大电压
	float Cap_full_voltage_Dset;	//默认设定值
	float Cap_V_C_fb;			 // 电容电压补偿
	float Cap_hight_V_mod;
	uint32_t debug_cnt; // 测试计数

	uint16_t can_powerlimit_value; // can接受到的功率限制
	uint8_t Can_sand; // 是否发送can
	uint8_t Can_cmd;
	uint8_t state_core; 
	/// @arg SUPERCAP_state_code
	uint8_t Soft_Version;

	uint16_t message_time_to_live; // CAN消息的TTL，超时则使用内置的缓冲逻辑
} cap_data;

extern cap_data Cap_Data;

extern sample_FloatData  Sample_Data;


extern Pid_t pid_battery_power;
extern Pid_t pid_power_buffer;

extern Pid_t* Pid_arrry[2];

void Calculate_AdcToFloat(void);
void I_Loop_AdcToFloat(void);

void Power_calcuate_v2_0();

void Power_Loop_Pid();
void I_Loop_Fmac();

void HrtimerAB_UpdataForDuty(float a2_duty, float b2_duty);

void Duty_calcuate_V2_0(float ratio); //ratio=Vcap/V24


void DC_DC_start(void);
void DC_DC_stop(void);

void PowerlimitUpdata(void);


void Danger_tag(void);

uint8_t CAP_V_LIMIT(void);

void I_loop_test(void);

uint8_t I_loop_test_run(void);

/** @defgroup SUPERCAP_state_code
  * 
  * [ 0x01 : 0x1f ] : 软件版本号
  * [ 0x20 : 0x9f ] : 保留区
  * 
  * 0x00:未启动(启动电压>20V)
  * 
  * 0x91:UVLO：V<10V
  * 0x92:UVLO：V<15V,500ms
  * 0x93:过压
  * 0x94:电容过流
  * 0x95:battery过流
  * 0xA1:正常工作()
  * 0xA2:环路自检中
  * 0xA3:高压容组模式
  * 0xA9:超电接受到的数据包，和校验不通过
  * 0xAA:命令执行确认
  * 0xE3:主动关闭
  * 0xEE:串口debug中，停止状态
*/

/** @defgroup SUPERCAP_cmd_code
 * 0x20 : 查询软件版本号
 * 0xB1 ：确认线路正常，恢复自动启动MOS管(仅在0xE2停机状态和手动停止状态下有效)
 * 0xB2 : 主动停止MOS管，暂停自动重启，需手动启动（0XB1）
 * 0xB3 : val: 0xB3重启
 * 0xC1 : 设置Can发送间隔，默认10ms，参数范围(uint8_t) 0-255,0为响应式，单位为毫秒
 * 0xD1: 设置百分比缓冲的值 默认90%；参数范围(uint8_t) 0-100 ，不保存到flash
 * 0xD3(已移除)：设置电容充电电压，值：(uint8_t)(电压值 * 10),内部限制为1.3V-24.3V，若大于21.5V，当满足一定条件时，会限制到21.5V
 * 0xD4:设置电容组充电电压,0x01:旧版24.3V电容组，实际限压23.5V；0x02：新版25.8V电容组，实际限压25.6V
 */

#endif
