/******************************************************************************
*** @File           : scheduler.c
*** @Description    : 时间轮询任务系统
*** @Attention      : None
*** @Author         : TJL
******************************************************************************/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "main.h"

typedef struct
{
	void(*task_func)(void);
	uint16_t rate_ms;
	int32_t volatile last_run;
}scheduler_task_t;

void average1S(void);

void can_sent(void);

void UVLO(void);

void free_Uart_sent(void);

void Cap_V_Limit(void);

void Start_task(void);

void Scheduler_init(void);
void Scheduler_run(void);
void Scheduler_stop(void (*task_func_P)(void));
void Scheduler_continue(void (*task_func_P)(void));
void Scheduler_set_RateTime(void (*task_func_P)(void), uint16_t time);
extern uint8_t led_state[4];
extern uint16_t led_cnt[4];

void wdg_flash(void);

static void Power_limit_Updata(void);
#endif


