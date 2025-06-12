/**
  **********************************2022 CKYF***********************************
  * @file    Vofa_send.c
  * @brief   ��Vofa+��λ��ͨѶ����
	* @author  �������� - ����
  ******************************************************************************
  * @attention
  *	
  * ��һ������λ������2/4/8��float���ݡ�
  *
  **********************************2022 CKYF***********************************
  */
	
#include "Vofa_send.h"
#include "string.h"
#include "usart.h"
#include "FIFO.h"
extern fifo_s_t uart_tx_fifo;

Vofa_data_m_2 Vofa_data_2={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_4 Vofa_data_4={.tail={0x00,0x00,0x80,0x7f}};
Vofa_data_m_8 Vofa_data_8={.tail={0x00,0x00,0x80,0x7f}};

void Vofa_Send_Data2(float data1, float data2)
{
	Vofa_data_2.ch_data[0] = data1;
	Vofa_data_2.ch_data[1] = data2;
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&Vofa_data_2, sizeof(Vofa_data_2));   
}

void Vofa_Send_Data4(float data1, float data2,float data3, float data4)
{
	Vofa_data_4.ch_data[0] = data1;
	Vofa_data_4.ch_data[1] = data2;
	Vofa_data_4.ch_data[2] = data3;
	Vofa_data_4.ch_data[3] = data4;
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&Vofa_data_4, sizeof(Vofa_data_4));   
}

void Vofa_Send_Data8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8)
{
	Vofa_data_8.ch_data[0] = data1;
	Vofa_data_8.ch_data[1] = data2;
	Vofa_data_8.ch_data[2] = data3;
	Vofa_data_8.ch_data[3] = data4;
	Vofa_data_8.ch_data[4] = data5;
	Vofa_data_8.ch_data[5] = data6;
	Vofa_data_8.ch_data[6] = data7;
	Vofa_data_8.ch_data[7] = data8;
	// fifo_s_puts(&uart_tx_fifo,(char *)&Vofa_data_8, sizeof(Vofa_data_8));
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&Vofa_data_8, sizeof(Vofa_data_8));   
}

void Vofa_Send_Data2_U3(float data1, float data2)
{
	Vofa_data_2.ch_data[0] = data1;
	Vofa_data_2.ch_data[1] = data2;
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Vofa_data_2, sizeof(Vofa_data_2));   
}
