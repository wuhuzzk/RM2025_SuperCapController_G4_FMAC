/******************************************************************************
*** @File           : scheduler.c
*** @Description    : 时间轮询任务
*** @Attention      : None
*** @Author         : TJL
******************************************************************************/
#include "main.h"
#include "scheduler.h"
#include <fdcan.h>
#include "CanBusTask.h"
#include "Control_buck_boost.h"
#include "Vofa_send.h"
#include "Anotc.h"
#include "FIFO.h"
#include "uart_data_handle.h"
#include "status_tip.h"
#include "iwdg.h"
uint8_t task_num;
uint8_t uart1_test_data[] = {0xFE, 0x01, 0x02, 0x03};
u32 cnt_on_P;
u32 cnt_on_I;

extern scheduler_task_t scheduler_task[];
extern fifo_s_t uart_tx_fifo;

//串口调试器发送数据，由于if分支会产生性能浪费，故用宏定义
#define Uart_Enfor 1

#if Uart_Enfor == 1
#define uasrt_power power_Uart_sent();
#define uasrt_current  
#define uasrt_sent  {free_Uart_sent, 1, 0},
#define uasrt_it   //__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXFE);

#elif Uart_Enfor == 2
// #define uasrt_power &&0
#define uasrt_power 
#define uasrt_current  current_Uart_sent();
#define uasrt_sent  {free_Uart_sent, 1, 0},
#define uasrt_it  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXFE);
#elif Uart_Enfor == 0
#define uasrt_power 
#define uasrt_current 
#define uasrt_it   
#define uasrt_sent  
#endif  
static void led_flash(void)
{ // 10ms
	status_flash();
	
}
void wdg_flash(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}
static void Power_limit_Updata(void)
{ // 100ms
	PowerlimitUpdata();

	Cap_Data.BAT_Power_per0S1 = (Cap_Data.BAT_Power_integration / 5000);
	float data[2] = {Cap_Data.Power_Limit, Cap_Data.BAT_Power_per0S1};
	Cap_Data.BAT_Power_integration = 0;
}

static void Power_Loop(void)
{
	Calculate_AdcToFloat();

	uasrt_power
	
	cnt_on_P++;

	Danger_tag();
	
	if (Cap_Data.DCDC_start_step >= 5 && Cap_Data.DCDC_start_step <= 10)
	I_loop_test();

	if (Cap_Data.PID_State == 1 || Cap_Data.PID_State == 3)
	{
		Power_Loop_Pid();
	}
}
void average1S(void)
{  //异常停止，自动重启
	static uint8_t cnt_reset;
	if(Cap_Data.state_core<0xE1)
	{
		if (Cap_Data.state_core!=0xA1&&Cap_Data.state_core!=0xA3)
		{
			cnt_reset++;
		}
		else
		{
			cnt_reset = 0;
		}
	}
	if (cnt_reset > 100)
	{
		Scheduler_stop(wdg_flash);
	}
}

static void I_Loop()
{
	I_Loop_AdcToFloat();

	uasrt_current
	
	I_Loop_Fmac();
}

void can_sent(void)
{
	if (Cap_Data.Can_sand == 1)
	{
		return;//响应式Can
	}
	else if (Cap_Data.Can_sand == 2)
	{
		Cap_Data.Can_sand = 1;
	}
	Cap_Ctrler_SendCanData(&hfdcan1, CAN1_CAP_SEND_ID, &Sample_Data);
}
void UVLO(void)
{	//欠压检测
	static uint16_t UVLO_cnt;
	if (Sample_Data.sample_Pin24V_V<15)
	{
		UVLO_cnt++;
	}
	else if (UVLO_cnt>5)
	{
		UVLO_cnt=0;
	}
	if (UVLO_cnt>500)
	{
		Cap_Data.state_core = 0x92;
		Scheduler_stop(wdg_flash);
	}
	
}

void free_Uart_sent(void)
{
	// if(__HAL_DMA_GET_FLAG(&huart1,DMA_FLAG_TC1)==0)
	// return;
	static uint8_t buf[4096];					  // 发送缓冲区
	uint32_t len = fifo_s_used(&uart_tx_fifo);	  // 待发送数据长度
	fifo_s_gets(&uart_tx_fifo, (char *)buf, len); // 从 FIFO 取数据
	HAL_UART_Transmit_DMA(&huart1, buf, len);	  // 发送
	// Vofa_Send_Data2_U3(len,0);
	// fifo_s_flush(&uart_tx_fifo);
	
}

void Cap_V_Limit(void)
{ // 1ms=1000hz，检测运动状态，用于预留动能回收所需的空间
	uint8_t C_state = CAP_V_LIMIT();
	static u16 C_state_cnt;
	if (C_state == 0)
	{
		C_state_cnt++;
	}
	if (C_state == 1)
	{
		Cap_Data.Cap_full_voltage_value = Cap_Data.Cap_full_voltage_Dset-2.3f+Cap_Data.Cap_hight_V_mod;
		C_state_cnt = 0;
	}
	else if (C_state_cnt > 500)
	{
		if(C_state_cnt==551)
		{
			Cap_Data.Cap_full_voltage_value+=0.1f;
			C_state_cnt = 501;
		}
		if(Cap_Data.Cap_full_voltage_value >= Cap_Data.Cap_full_voltage_Dset+Cap_Data.Cap_hight_V_mod)
		{
			Cap_Data.Cap_full_voltage_value = Cap_Data.Cap_full_voltage_Dset+Cap_Data.Cap_hight_V_mod;
		}
	}
}
static void sys_data_sent(void)
{
	Vofa_Send_Data2_U3(cnt_on_P, cnt_on_I);
	cnt_on_P = 0;
	cnt_on_I = 0;
}
void Start_task(void)
{//启动流程任务，包括电压检测，电流闭环检测
	static uint8_t error_Iloop_cnt;
	if (CoreDebug->DHCSR & 0x00000001) // 如果处于sw_debug,则不执行，需要重上电
	{
		DC_DC_stop();
		Scheduler_stop(Start_task);
		return;
	}
	if (Cap_Data.state_core >= 0xE1)
	{
		Cap_Data.DCDC_start_step = 0;
		error_Iloop_cnt = 0;
		DC_DC_stop();
		Scheduler_stop(Start_task);
		led_state[3] = 1;
		return;
	}

	if (Sample_Data.sample_Pin24V_V < 15)
	{
		if (Cap_Data.DCDC_start_step != 0)
		{
			Cap_Data.DCDC_start_step = 0;
			DC_DC_stop();
			// return;
		}
	}
	// 自动重启
	if (Cap_Data.DCDC_start_step == 0xC1)
	{
		if (Cap_Data.PID_State == 0|| Cap_Data.state_core==0x00)
			Cap_Data.DCDC_start_step = 0;
	}
	if (Cap_Data.state_core == 0xA1)
	{
		return;
	}
	if (Cap_Data.DCDC_start_step >= 0 && Cap_Data.DCDC_start_step < 5)
	{
		if (Sample_Data.sample_Pin24V_V > 20 && Sample_Data.sample_Pin24V_V < 28)
			Cap_Data.DCDC_start_step++;
		else
		{
			Cap_Data.DCDC_start_step = 0;
			Cap_Data.Stop_cnt=0;
		}
	}
	if (Cap_Data.DCDC_start_step >= 5 && Cap_Data.DCDC_start_step <= 10)
	{
		if (error_Iloop_cnt == 5)
		{
			// Cap_Data.state_core=0xE1;
			// return;
			Scheduler_stop(wdg_flash);
		}
		uint8_t temp_state = I_loop_test_run();
		Cap_Data.state_core = 0xA2;
		if (temp_state == 1)
		{
			Cap_Data.DCDC_start_step++;
		}
		else if (temp_state == 2)
		{
			Cap_Data.DCDC_start_step = 5;
			error_Iloop_cnt++;
		}
	}
	if (Cap_Data.DCDC_start_step == 11&&Cap_Data.PID_State==2)
	{
		Cap_Data.state_core = 0xA1;
		Cap_Data.DCDC_start_step = 0xC1;
		error_Iloop_cnt = 0;
		Cap_Data.PID_State = 1;
		
		Scheduler_continue(UVLO);
	}
}

static void Stop_cnt_task(void)
{

	if (Cap_Data.Stop_cnt > 2)
	{
		Scheduler_stop(wdg_flash);
		Scheduler_stop(Start_task);
		DC_DC_stop();

	}
	Cap_Data.Stop_cnt = 0;
}

//************************************//
scheduler_task_t scheduler_task[] =
	{
		//@毫秒

		{can_sent, 1, 0},
		// {average, 1, 0},
		{average1S, 100, 0},
		{sys_data_sent, 1000, 0},
		{Power_limit_Updata, 100, 0},
		// {I_Loop, 0, 0},
		// {Power_Loop, 0, 0},
		{led_flash, 10, 0},
		{wdg_flash, 5, 0},
		{Cap_V_Limit, 1, 0},
		{Start_task, 1, 0},
		{UVLO,1,0},
		{Stop_cnt_task, 500, 0},

		// {free_Uart_sent, 1, 0},
		uasrt_sent

};

//************************************//

void Scheduler_init(void)
{
	task_num = sizeof(scheduler_task) / sizeof(scheduler_task_t);
	uasrt_it
}

void Scheduler_run(void)
{
	for (int i = 0; i < task_num; i++)
	{
		int32_t now_time = HAL_GetTick();
		if (now_time - scheduler_task[i].last_run >= scheduler_task[i].rate_ms)
		{
			scheduler_task[i].last_run = now_time;
			scheduler_task[i].task_func();
		}
	}
}
//停止任务，没有防溢出
void Scheduler_stop(void (*task_func_P)(void))
{
	for (size_t i = 0; i < task_num; i++)
	{
		if (scheduler_task[i].task_func == task_func_P)
			scheduler_task[i].last_run = 0x7fffffff;
	}
}
//继续任务
void Scheduler_continue(void (*task_func_P)(void))
{
	for (size_t i = 0; i < task_num; i++)
	{
		if (scheduler_task[i].task_func == task_func_P)
			scheduler_task[i].last_run = 0;
	}
}
//设置任务执行间隔
void Scheduler_set_RateTime(void (*task_func_P)(void), uint16_t time)
{
	for (size_t i = 0; i < task_num; i++)
	{
		if (scheduler_task[i].task_func == task_func_P)
			scheduler_task[i].rate_ms = time;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//定时器回调
	// if (htim->Instance == TIM1) // 100khz 
	{
		{
			Power_Loop();
		}
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// ADC转换完成回调
	if (hadc->Instance == ADC5)
	{
		I_Loop();
	}

	
}