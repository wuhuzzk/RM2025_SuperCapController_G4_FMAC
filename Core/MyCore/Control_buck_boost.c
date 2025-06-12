/******************************************************************************
*** @File           : Control_buck_boost.c
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
#include "Control_buck_boost.h"
#include "Vofa_send.h"
#include "Ramp.h"
#include "scheduler.h"
#include "Pid.h"
#include "math.h"
#include "fdcan.h"
#include "CanBusTask.h"
#include <stdlib.h>
adc_RawData ADC_RawData;
sample_FloatData Sample_Data;

Pid_t pid_battery_power;
Pid_t pid_power_buffer;

Pid_t *Pid_arrry[2] = {&pid_battery_power, &pid_power_buffer};

cap_data Cap_Data = {.buffer_mode = 2, .Power_Limit = 45, .PID_State = 0, .Power_buffer_value = 2, .Power_buffer_percent = 0.966f, .can_powerlimit_value = 82, .Cap_full_voltage_value = 23.5, .Cap_full_voltage_measured = 0, .Cap_full_voltage_Dset = 23.5, .Cap_hight_V_mod = 0, .DCDC_start_step = 0, .Soft_Version = 0x03, .Can_sand = 0};
extern uint8_t led_state[4];

#define MiN(x, y) (x < y) ? x : y
#define MAX(x, y) (x > y) ? x : y
/*
电流采样：
IN240A1的放大倍数为20，电池采样10mR=0.010R，其余采样5mR=0.005R
参考电压为AGND和VERF3.3的中值：1.65V ，超采样到16bit，adc共65536，1.65V=32768
	I=(Rawdata*3.3f /4096-1.65f)/20/0.01;
	I=(Rawdata*3.3f /4096-1.65f)/20/0.005;
电压采样：
30K和3K分压，比例为1/11，运算放大的比例为1 + 1/11 = 12/11
	V=Rawdata*3.3f/4096*11/12*11;
 */
void Calculate_AdcToFloat(void)
{

	float battery_I_temp = (ADC_RawData.adc_Battery_I * 3.3f / 32767 - 1.65f) / 20 / 0.01f;
	Sample_Data.sample_Battery_I = battery_I_temp * 0.9807f - 0.0567f;
	float chassis_I_temp = (ADC_RawData.adc_Chassis_I * 3.3f / 32767 - 1.65f) / 20 / 0.005f;
	if (chassis_I_temp > -0.147f)
	{
		Sample_Data.sample_Chassis_I = chassis_I_temp * 0.9676f - 0.1885f;
	}
	else
	{
		Sample_Data.sample_Chassis_I = chassis_I_temp * 0.9820f - 0.2169f;
	}
	Sample_Data.sample_Cap_V = ADC_RawData.adc_Cap_V * 3.3f / 32767 * 11 / 12 * 11;
	Sample_Data.sample_Pin24V_V = ADC_RawData.adc_Pin24V_Battery_V * 3.3f / 32767 * 11 / 12 * 11;

	Sample_Data.sample_Battery_Power = Sample_Data.sample_Pin24V_V * Sample_Data.sample_Battery_I;
	Sample_Data.sample_Cap_Power = Sample_Data.sample_Cap_V * Sample_Data.sample_Cap_I;
	Sample_Data.sample_Chassis_Power = Sample_Data.sample_Pin24V_V * Sample_Data.sample_Chassis_I;

	Cap_Data.BAT_Power_integration += Sample_Data.sample_Battery_Power;
}
// uint16_t temp_last;
void I_Loop_AdcToFloat(void)
{
	// 分段线性校准，但性能开销大
	/* 	float temp = (ADC_RawData.adc_Cap_I * 3.3f / 16383 - 1.65f) / 20 / 0.002f;
		if (temp>0)
		{
			Sample_Data.sample_Cap_I = temp * 1.0286f -0.1080f;
		}
		else if(temp<-0.35f)
			Sample_Data.sample_Cap_I = temp * 1.0156f -0.2701f;
		else
			Sample_Data.sample_Cap_I = temp *1.0152f -0.2025f;
	 */
	Sample_Data.sample_Cap_I = (ADC_RawData.adc_Cap_I * 3.3f / 16383 - 1.65f) / 20 / 0.002f;
}

#define Period_M HRTIM1->sMasterRegs.MPER
#define DCDC_MIN_DUTY 0.01f
#define DCDC_MAX_DUTY (1 - DCDC_MIN_DUTY)
/*
总重装值为54400
占空比最小1%最大99%(在总周期上的时长)
 */
void HrtimerAB_UpdataForDuty(float a2_duty, float b2_duty)
{
	uint16_t hight_area = Period_M * (1 - a2_duty - b2_duty);

	hight_area = (hight_area < 0) ? 0 : (hight_area > Period_M) ? Period_M
																: hight_area;
	uint16_t cmp1temp = a2_duty * Period_M;
	uint16_t cmp2temp = cmp1temp + hight_area / 2;
	uint16_t cmp3temp = cmp2temp + b2_duty * Period_M;

	HRTIM1->sMasterRegs.MCMP1R = cmp1temp;
	HRTIM1->sMasterRegs.MCMP2R = cmp2temp;
	HRTIM1->sMasterRegs.MCMP3R = cmp3temp;
}

#define Dead_multiple (1 - 2 * DCDC_MIN_DUTY)
void Duty_calcuate_V2_0(float ratio) // ratio=Vcap/V24
{
	float a2 = DCDC_MIN_DUTY, b2 = DCDC_MIN_DUTY;

	if (ratio > 1) // boosts 升压充电到电容 或者buck放电
	{
		float ratio_temp = (ratio - 1) * Dead_multiple + 1 + DCDC_MIN_DUTY;
		HrtimerAB_UpdataForDuty(a2, 1 - (1 / ratio_temp));
	}
	else // buck 降压充电到电容 或者boosts放电
	{
		float ratio_temp = Dead_multiple * ratio + DCDC_MIN_DUTY;
		HrtimerAB_UpdataForDuty(1 - ratio_temp, b2);
	}
}

void Power_Loop_Pid()
{

	float power = Cap_Data.Power_Limit - Sample_Data.sample_Chassis_Power;

	pid_calc(&pid_battery_power, Sample_Data.sample_Battery_Power, Cap_Data.Power_Limit);

	Cap_Data.cap_I_arm = (power + pid_battery_power.last_delta_out) / Sample_Data.sample_Cap_V;
	if (Sample_Data.sample_Cap_V < 10)
	{
		Cap_Data.cap_I_arm = KalmanFilter(&kalman_Cap_I_arm, Cap_Data.cap_I_arm, 1, 30);
	}
	// 电流限幅
	clamp_f_limit(-24, 16, &Cap_Data.cap_I_arm);
}
void I_Loop_Fmac()
{
	float error = (Cap_Data.cap_I_arm - Sample_Data.sample_Cap_I) / 32;
	clamp_f_limit(-1, 1, &error);
	Fmac_write_data(error);
}

void DC_DC_start(void)
{
	led_state[3] = 0;
	Cap_Data.cap_I_arm = 0;

	Cap_Data.Ratio = Sample_Data.sample_Cap_V / Sample_Data.sample_Pin24V_V;
	Duty_calcuate_V2_0(Cap_Data.Ratio);

	Pid_clear_i(&pid_battery_power); // 清空功率环积分

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);

	Cap_Data.PID_State = 2; // pid start
}

void DC_DC_stop(void)
{
	if (Cap_Data.PID_State != 0)
	{
		Cap_Data.Stop_cnt++;
	}

	Cap_Data.PID_State = 0; // pid stop
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	led_state[3] = 0;

	if (Cap_Data.state_core == 0xA1 || Cap_Data.state_core == 0xA3)
	{
		Cap_Data.state_core = 0x00;
	}
	Cap_Data.cap_I_arm = 0;
	can_sent();
}

// 设置涓流充电阈值
#define Trickle_charging_V 1.5f
void PowerlimitUpdata(void)
{
	if (Cap_Data.PID_State != 1)
	{
		return;
	}
	// 涓流充电，电流补偿
	float temp_V_limit = Sample_Data.sample_Cap_V + Trickle_charging_V - Cap_Data.Cap_full_voltage_value - Sample_Data.sample_Cap_I * 0.115f;

	float battery_power_limit_temp = Cap_Data.can_powerlimit_value;
	if (Cap_Data.message_time_to_live == 20)
	{
		led_state[1] = 1;
		//缓存能量逻辑
		if (Cap_Data.buffer_mode == 2)
		{
			if (Cap_Data.buffer_val < 60)
			{
				battery_power_limit_temp = Cap_Data.can_powerlimit_value * Cap_Data.Power_buffer_percent - (60 - Cap_Data.buffer_val) * 0.2f;
			}
			else if (Cap_Data.buffer_val > 61)
			{
				battery_power_limit_temp = Cap_Data.can_powerlimit_value + (Cap_Data.buffer_val - 60) * 0.1f;
			}
		}
		else if (Cap_Data.buffer_mode == 3)
		{
			static uint32_t cnt_buf_60;
			if (Cap_Data.buffer_val > 50) // 10Hz
			{
				if (cnt_buf_60 == 0)
				{
					pid_power_buffer.last_delta_out = 0;
				}
				else if (temp_V_limit < 0)
				{
					pid_power_buffer.last_delta_out -= 0.02f;
				} // 0.2W/s

				cnt_buf_60++;
			}
			else
			{
				cnt_buf_60 = 0;
				battery_power_limit_temp = battery_power_limit_temp - pid_calc(&pid_power_buffer, Cap_Data.buffer_val, 60);
			}
		}

		else if (Cap_Data.buffer_mode == 1)
		{
			battery_power_limit_temp = Cap_Data.can_powerlimit_value - Cap_Data.Power_buffer_value;
			if (Cap_Data.buffer_val < 60)
			{
				battery_power_limit_temp = Cap_Data.can_powerlimit_value - Cap_Data.Power_buffer_value - 2 - (60 - Cap_Data.buffer_val) * 0.2f;
			}
		}
	}
	if (Cap_Data.message_time_to_live == 0 || Cap_Data.message_time_to_live == 1) // Can失效后
	{
		int16_t hot_t = Sample_Data.sample_Battery_Power - Cap_Data.can_powerlimit_value;
		if (hot_t < 0)
		{
			hot_t = 0;
		}
		clamp_ui32_limit(0, 20, (uint32_t *)&hot_t);
		battery_power_limit_temp = battery_power_limit_temp - hot_t - Cap_Data.Power_buffer_value;
	}

	clamp_f_limit(15, Cap_Data.can_powerlimit_value + 20, &battery_power_limit_temp);

	// 在阈值前开始涓流充电，不包括动能回收
	if (temp_V_limit > 0)
	{
		float temp_P_limit = battery_power_limit_temp * (1 - temp_V_limit / Trickle_charging_V);
		if (temp_P_limit < 2)
			Cap_Data.Power_Limit = 2;
		else
			Cap_Data.Power_Limit = (uint16_t)temp_P_limit;
	}
	else
	{
		Cap_Data.Power_Limit = battery_power_limit_temp;
	}

	if (Cap_Data.message_time_to_live > 1)
	{
		Cap_Data.message_time_to_live--;
	}
}
#undef Trickle_charging_V

/* 用于检测电容容值，并进行限压 */

// 过流过压检测
void Danger_tag(void)
{
	if ((int16_t)Sample_Data.sample_Pin24V_V < 10)
	{
		Cap_Data.state_core = 0x91;
		DC_DC_stop();
		led_state[3] = 1;
	}
	if ((int16_t)Sample_Data.sample_Pin24V_V > 36)
	{
		Cap_Data.state_core = 0x93;
		DC_DC_stop();
		led_state[3] = 1;
	}
	if (abs((int)Sample_Data.sample_Cap_I) > 32)
	{
		Cap_Data.state_core = 0x94;
		DC_DC_stop();
		led_state[3] = 1;
	}
	if ((int16_t)Sample_Data.sample_Battery_I > 16)
	{
		Cap_Data.state_core = 0x95;
		DC_DC_stop();
		led_state[3] = 1;
	}
}

/// @brief 检测运动，来限制充电电压
/// @param
/// @return 1 限制 0 不限制
uint8_t CAP_V_LIMIT(void)
{
	static uint16_t chassis_time;
	if (Sample_Data.sample_Chassis_Power < -15)
	{
		return 1;
	}
	if (Sample_Data.sample_Chassis_Power < 0)
	{
		chassis_time++;
		return 0;
	}
	else if (Sample_Data.sample_Chassis_Power > 30)
	{
		if (chassis_time < 3000)
		{
			return 1;
		}
		else if (Sample_Data.sample_Chassis_Power > 45)
		{
			return 1;
		}
	}
	return 0;
}

// 用于环路检测
uint16_t i_loop_cnt;
void I_loop_test(void)
{
	// if (Cap_Data.DCDC_start_step>=5&&Cap_Data.DCDC_start_step<10)
	{
		if (fabs(Cap_Data.cap_I_arm - Sample_Data.sample_Cap_I) < 0.15f)
		{
			i_loop_cnt++;
		}
	}
}

uint8_t I_loop_test_run(void)
{
	if (Cap_Data.state_core != 0xE1)
	{
		if (Cap_Data.PID_State == 9)
		{
			Cap_Data.cap_I_arm = 0.10f;
			return 0;
		}
		if (Cap_Data.PID_State == 0)
		{
			DC_DC_start();
			Cap_Data.cap_I_arm = 0.10f;
			return 0;
		}
		if (Cap_Data.DCDC_start_step >= 6 || Cap_Data.DCDC_start_step < 10)
		{
			Cap_Data.cap_I_arm = 0.10f;

			return 1;
		}
		if (Cap_Data.DCDC_start_step == 10)
		{
			if (i_loop_cnt > 100)
			{
				Cap_Data.cap_I_arm = 0;

				i_loop_cnt = 0;
				return 1;
			}
			else
				return 2; // stop
		}
	}

	return 0;
}
