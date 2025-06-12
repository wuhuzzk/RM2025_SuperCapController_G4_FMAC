//*****************************************************************************
//* @File           : Anotc.h
//* @Description    : 发送数据给匿名助手/匿名上位机 波特率：500000
//* @Attention      : None
//* @Author         : Lmy
//* @Date           : 2024/10/16
//* @版权归属:
//*                        ___          ___          ___
//*             ___       /\__\        /\  \        /\  \
//*            /\  \     /::|  |      /::\  \      /::\  \
//*            \:\  \   /:|:|  |     /:/\:\  \    /:/\:\  \
//*            /::\__\ /:/|:|__|__  /:/  \:\  \  /::\~\:\  \
//*         __/:/\/__//:/ |::::\__\/:/__/ \:\__\/:/\:\ \:\__\
//*        /\/:/  /   \/__/~~/:/  /\:\  \  \/__/\/__\:\/:/  /
//*        \::/__/          /:/  /  \:\  \           \::/  /
//*         \:\__\         /:/  /    \:\  \          /:/  /
//*          \/__/        /:/  /      \:\__\        /:/  /
//*                       \/__/        \/__/        \/__/
//****************************************************************************/
#ifndef ANOTC_H
#define ANOTC_H
/*---------------------------------------------------------------------------*/
/* 头文件 includes BEGIN */
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "stm32G4xx_it.h"
#include "usart.h"
/* 头文件 includes E N D */
/*---------------------------------------------------------------------------*/
/* 宏定义 define BEGIN */
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp)+3))

#define ANOTC_UART     huart1                   //通过重新定义IMU_UART和IMU_DMA_RX
#define ANOTC_DMA_TX   (hdma_uart1_tx)            //可选择接受数据的串口和DMA

#define ANOTC_HEAD      0xAB
#define ANOTC_S_ADDR    0xFF
#define ANOTC_D_ADDR    0xFF
#define ANOTC_ID        0xF1
#define ANOTC_LEN_L     11
#define ANOTC_LEN_H     0
/* 宏定义 define E N D */
/*---------------------------------------------------------------------------*/
/* 结构体类型定义 typedef struct BEGIN */

/* 结构体类型定义 typedef struct E N D */
/*---------------------------------------------------------------------------*/
/* 全局变量声明 variables BEGIN */

/* 全局变量声明 variables E N D */
/*---------------------------------------------------------------------------*/
/* 函数原型声明 function prototypes BEGIN */
void Auotc_Init();
void Anotc_SendData(uint8_t u8,uint16_t u16,float f,uint32_t u32);
void Anotc_printf(const char *format, ...);
void Anotc_Send_float_Data(float *f_p, uint8_t count, uint8_t id);
/* 函数原型声明 function prototypes E N D */
#endif //ANOTC_H
