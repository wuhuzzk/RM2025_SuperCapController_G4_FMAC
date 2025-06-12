/******************************************************************************
*** @File           : uart_data_handle.h
*** @Description    : 串口数据处理及命令识别
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
#ifndef _uart_data_handle
#define _uart_data_handle

#include "stm32g4xx_hal.h"

#define RXBUFFERSIZE  256     //最大接收字节数
extern char RxBuffer[RXBUFFERSIZE];   //接收数据
extern uint8_t aRxBuffer;			//接收中断缓冲
extern uint8_t Uart2_Rx_Cnt;		//接收缓冲计数
#undef RXBUFFERSIZE

void Uart_data_handle(void);

void usart_free_IRQHandler(void);

void power_Uart_sent(void);

void current_Uart_sent(void);

#endif