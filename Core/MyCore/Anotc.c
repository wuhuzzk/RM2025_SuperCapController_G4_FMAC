/******************************************************************************
*** @File           : Anotc.c
*** @Description    : 发送数据给匿名助手/匿名上位机 波特率：500000
*** @Attention      : None
*** @Author         : Lmy
*** @Date           : 2024/10/16
*** @版权归属:
***                        ___         ___         ___
***              ___      /\__\       /\  \       /\  \
***             /\  \    /::|  |     /::\  \     /::\  \
***             \:\  \  /:|:|  |    /:/\:\  \   /:/\:\  \
***             /::\__\/:/|:|__|__ /:/  \:\  \ /::\~\:\  \
***          __/:/\/__/:/ |::::\__/:/__/ \:\__/:/\:\ \:\__\
***         /\/:/  /  \/__/~~/:/  \:\  \  \/__\/__\:\/:/  /
***         \::/__/         /:/  / \:\  \          \::/  /
***          \:\__\        /:/  /   \:\  \         /:/  /
***           \/__/       /:/  /     \:\__\       /:/  /
***                       \/__/       \/__/       \/__/
******************************************************************************/
/* 头文件 includes BEGIN */
#include "Anotc.h"
#include "FIFO.h"
/* 头文件 includes E N D */
/*---------------------------------------------------------------------------*/
/* 局部变量定义 BEGIN */
uint8_t tx_buffer[100];
uint16_t frame_len = 0;
uint8_t sum_check = 0;
uint8_t add_check = 0;

//FIXME: 测试用
uint8_t u8;
uint16_t u16;
float f;
uint32_t u32;

// const uint8_t TX_FIFO_SIZE = 100;
// static uint8_t buf[TX_FIFO_SIZE];				//发送缓冲区
fifo_s_t uart_tx_fifo;
uint8_t uart_tx_area[2048]={0};

/* 局部变量定义 E N D */
/*---------------------------------------------------------------------------*/
/* 外部引用全局变量 BEGIN */

/* 外部引用全局变量 E N D */
/*---------------------------------------------------------------------------*/

/*---- 函数区 ----*/
/**
 * @Function_name       Auotc_Init
 * @Brief               匿名协议初始化\n
            |__帧头__|_源地址__|_目标地址_|_功能码_|___数据内容____|_数据内容_|___和校验___|__附加校验___|\n
            |_HEAD__|_S_ADDR_|_D_ADDR_|__ID___|_____LEN_____|__DATA___|_SUM_CHECK_|_ADD_CHECK_|\n
            |_1Byte_|_1Byte__|_1Byte__|_1Byte_|_1Byte|1Byte_|__nByte__|___1Byte___|___1Byte___|\n
 * @Remarks             None
**/
void Auotc_Init()
{
    tx_buffer[0]=ANOTC_HEAD;
    tx_buffer[1]=ANOTC_S_ADDR; //源地址
    tx_buffer[2]=ANOTC_D_ADDR; //目标地址
    tx_buffer[3]=ANOTC_ID;     //功能码（ID）
    tx_buffer[4]=ANOTC_LEN_L;  //数据长度（2字节）
    tx_buffer[5]=ANOTC_LEN_H;  //数据长度

    frame_len =  tx_buffer[4] + tx_buffer[5] * 255;
    fifo_s_init(&uart_tx_fifo,uart_tx_area,2048);
}
/*---------------------------------------------------------------------------*/
/**
 * @Function_name       Anotc_SendData
 * @Brief               发送数据给匿名助手/匿名上位机
 * @Input_param         TODO : 需要在这里补充发送到匿名助手/匿名上位机的数据
 * @Remarks             None
**/
void Anotc_SendData(uint8_t _u8,uint16_t _u16,float _f,uint32_t _u32)
{
    uint8_t _cnt = 6;

    // TODO : 改成中断+DMA+FIFO缓存发送

    tx_buffer[_cnt++]=BYTE0(_u8);//1字节

    tx_buffer[_cnt++]=BYTE0(_u16);//2字节
    tx_buffer[_cnt++]=BYTE1(_u16);

    tx_buffer[_cnt++]=BYTE0(_f);//4字节
    tx_buffer[_cnt++]=BYTE1(_f);
    tx_buffer[_cnt++]=BYTE2(_f);
    tx_buffer[_cnt++]=BYTE3(_f);

    tx_buffer[_cnt++]=BYTE0(_u32);//4字节
    tx_buffer[_cnt++]=BYTE1(_u32);
    tx_buffer[_cnt++]=BYTE2(_u32);
    tx_buffer[_cnt++]=BYTE3(_u32);

    // tx_buffer[4]=_cnt-6;

    sum_check = 0;
    add_check = 0;

    /*校验计算*/
    for(uint16_t i=0; i < (frame_len+6); i++)
    {
       sum_check += tx_buffer[i];
       add_check += sum_check;
    }
    tx_buffer[_cnt++] = sum_check;//和校验
    tx_buffer[_cnt++] = add_check;//附加校验

    HAL_UART_Transmit_DMA(&ANOTC_UART,tx_buffer,_cnt);//串口发送
}
/*---------------------------------------------------------------------------*/
/**
 * @Function_name       Anotc_printf
 * @Brief               发送给匿名printf函数
 * @Input_param         要发送的变长参数
 * @Remarks             无匿名协议发送，仅验证使用
**/
void Anotc_printf(const char *format, ...)
{
    char buffer[1024];                  // 创建一个缓冲区来存储结果字符串
    va_list args;                       // 声明一个 va_list 类型的变量
    va_start(args, format);             // 初始化 va_list 变量
    vsprintf(buffer, format, args);     // 使用 vsprintf 将格式化的数据写入缓冲区
    va_end(args);                       // 清理 va_list 变量

    HAL_UART_Transmit_DMA(&ANOTC_UART, (uint8_t*)buffer,strlen(buffer));//超时串口发送
}

/// @brief Anotc_Send_float_Data
/// @param f_p float_data
/// @param count sizeof( f_p )
void Anotc_Send_float_Data(float* f_p,uint8_t count,uint8_t id)
{
    tx_buffer[3]=id;
    uint8_t _cnt = 6;

    for (uint8_t i = 0; i < count; i++)
    {
        tx_buffer[_cnt]=BYTE0(f_p[i]);//4字节
        tx_buffer[_cnt+1]=BYTE1(f_p[i]);
        tx_buffer[_cnt+2]=BYTE2(f_p[i]);
        tx_buffer[_cnt+3]=BYTE3(f_p[i]);
		_cnt=_cnt+4;
    }
        tx_buffer[4]=_cnt-6;
        // tx_buffer[4]=32;
    
   
    sum_check = 0;
    add_check = 0;

    /*校验计算*/
    for(uint16_t i=0; i < (_cnt); i++)
    {
       sum_check += tx_buffer[i];
       add_check += sum_check;
    }
    tx_buffer[_cnt++] = sum_check;//和校验
    tx_buffer[_cnt++] = add_check;//附加校验

    
    fifo_s_puts(&uart_tx_fifo, (char *)tx_buffer, _cnt);	

    // HAL_UART_Transmit_DMA(&ANOTC_UART,tx_buffer,_cnt);//串口发送
}