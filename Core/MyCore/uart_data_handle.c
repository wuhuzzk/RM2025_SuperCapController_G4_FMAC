/******************************************************************************
*** @File           : uart_data_handle.c
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
#include "main.h"
#include "usart.h"
#include "uart_data_handle.h"
#include "stdlib.h"
#include "string.h"
#include "hrtim.h"
#include "Control_buck_boost.h"
#include "scheduler.h"
#include "Anotc.h"
extern uint16_t ADC_comp[8];
extern uint8_t scan_A;
extern uint16_t scan_d[2][8];
extern DMA_HandleTypeDef hdma_usart1_rx;


char cmd[16];
uint8_t uart_data_temp[64];



#define RXBUFFERSIZE  256     //最大接收字节数
char RxBuffer[RXBUFFERSIZE] = { 0 };   //接收数据

uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart2_Rx_Cnt = 0;		//接收缓冲计数


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART3)
	{
		if (Uart2_Rx_Cnt >= 255)  //溢出判断
		{
			Uart2_Rx_Cnt = 0;
			memset(RxBuffer, 0x00, sizeof(RxBuffer));
		}
		else
		{
			RxBuffer[Uart2_Rx_Cnt++] = aRxBuffer;   //接收数据转存

			if ((RxBuffer[Uart2_Rx_Cnt - 1] == '*'))//判断结束
			{
				Uart2_Rx_Cnt = 0;
				Uart_data_handle();
			}
		}
	}
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&aRxBuffer, 1);   //再开启接收中断
}

void Uart_data_handle(void)
{
	memcpy(uart_data_temp,RxBuffer,64);
	memset(RxBuffer,0,64);
	for(int i=0;i<64;i++)
	{
		if(uart_data_temp[i]=='B'&&uart_data_temp[i+1]=='T'&&uart_data_temp[i+2]=='+')
		{
			int t;
			memset(cmd,0,16);
			for(t=0;t<15&&uart_data_temp[i+3+t]!='='&&(t+i)<60;t++)//BT+cmd=1+2*
			{
				cmd[t]=uart_data_temp[i+3+t];			
			}	
			cmd[t++]='=';
			
			if(strcmp(cmd,"RST=")==0)//cmd:BT+RST=*
			{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,1);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,0);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,1);
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
				// __set_FAULTMASK(1);//禁止所有的可屏蔽中断
				// NVIC_SystemReset();//软件复位
				Scheduler_stop(wdg_flash);
				
			}

			if(strcmp(cmd,"Start=")==0)//cmd:BT+Start=0*
			{
				char a_t[16] = { 0 };
				int a=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtol(a_t, &q, 10);
				if (a==1)
				{
					Scheduler_continue(Start_task);
				}
				else
				// DC_DC_start();
				Cap_Data.PID_State=a;
				
				// HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); 
				// HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0);
			}
			if(strcmp(cmd,"Stop=")==0)//cmd:BT+Stop=*
			{
				DC_DC_stop();
				Cap_Data.state_core=0xEE;
				
				// HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); 
				// HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
			}
			if(strcmp(cmd,"period=")==0)//cmd:BT+period=54400*
			{
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); 
				char a_t[16] = { 0 };
				int a=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtol(a_t, &q, 10);
				clamp_i32_limit(5000,65502,&a);
				HRTIM1->sMasterRegs.MPER = a;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			
			if(strcmp(cmd,"ratio=")==0)//cmd:BT+ratio=1*
			{
				char a_t[16] = { 0 };
				float a=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtod(a_t, &q);
				clamp_f_limit(0,1.2f,&a);
				Cap_Data.Ratio=a;
				Duty_calcuate_V2_0(Cap_Data.Ratio);
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			
			if(strcmp(cmd,"armi=")==0)//cmd:BT+armi=1*
			{
				char a_t[16] = { 0 };
				float a=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtod(a_t, &q);
				Cap_Data.cap_I_arm=a;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			if(strcmp(cmd,"CVfb=")==0)//cmd:BT+CVfb=1*
			{
				char a_t[16] = { 0 };
				float a=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtod(a_t, &q);
				Cap_Data.Cap_V_C_fb=a;
				
				
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			
			if (strcmp(cmd, "limit=") == 0)//cmd:BT+limit=111*
			{
				char a_t[16] = { 0 },b_t[16] = { 0 };
				int a,b=0;
				char* q;

				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				a=strtol(a_t, &q, 10);
				Cap_Data.can_powerlimit_value=a;
				Cap_Data.Power_Limit=a;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			
			if (strcmp(cmd, "pid=") == 0)//cmd:BT+pid=p+i+d+cc
			{
				char data_t[16] = { 0 }, a_t[16] = { 0 },b_t[16] = { 0 },c_t[16] = { 0 },f_t[16] = { 0 };
				float aa,bb,dd,ff;
				int cc=0;
				char* q;

				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						a_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				
				t++;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						b_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				
				t++;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						data_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						c_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				t++;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
					f_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				
				aa = strtod(a_t, &q);
				bb= strtod(b_t, &q);
				dd= strtod(data_t, &q);
				ff= strtod(f_t, &q);
				cc= strtol(c_t, &q,10);//1:bat_power 2:cap_I
				PID_struct_updata(Pid_arrry[cc-1],aa,bb,dd);

				Pid_arrry[cc-1]->f=ff;
				
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			if (strcmp(cmd, "buffermode=") == 0)//cmd:BT+buffermode=a*
			{
				char data_t[16] = { 0 };
				int cc=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						data_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				cc= strtol(data_t, &q,10);//1: 0.9_buff 2:old_cup 3:val_mod
				Cap_Data.buffer_mode=cc;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			if (strcmp(cmd, "bufferval=") == 0)//cmd:BT+bufferval=a*
			{
				char data_t[16] = { 0 };
				int cc=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						data_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				cc= strtod(data_t, &q);
				Cap_Data.Power_buffer_value=cc;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			if (strcmp(cmd, "bufferper=") == 0)//cmd:BT+bufferper=a*
			{
				char data_t[16] = { 0 };
				int cc=0;
				char* q;
				for (int k = 0;!(uart_data_temp[i + 3 + t] == '+' || uart_data_temp[i + 3 + t] == '*') && (t + i) < 60;t++)
				{
					if (uart_data_temp[i + 3 + t] <= '9' && uart_data_temp[i + 3 + t] >= '0' || uart_data_temp[i + 3 + t] == '-'||uart_data_temp[i+3+t]=='.')
					{
						data_t[k] = uart_data_temp[i + 3 + t];k++;
					}
				}
				cc= strtod(data_t, &q);
				Cap_Data.Power_buffer_percent=cc;
				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			}
			return;
		}
		
	}
	return;
}


void usart_free_IRQHandler(void)
{  
	// if(USART1 == huart1.Instance)                                   //判断是否是串口1（！此处应写(huart->Instance == USART1)
	{
		// __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清楚空闲中断标志（否则会一直不断进入中断）
		
		// if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXFE))   //判断是否是空闲中断
		{
			__HAL_UART_CLEAR_TXFECF(&huart1);
			free_Uart_sent();
		}
	}
}
extern uint32_t cnt_on_P;
void power_Uart_sent(void)
{
		// 定义一个浮点型数组，用于存储要发送的数据
		float data[6] = {
			// Sample_Data.sample_Battery_I,
			// ADC_RawData.adc_Temp_Cap_V[0],
			// Cap_Data.Power_Limit,
			Sample_Data.sample_Cap_I,
			// Cap_Data.Cap_I_per0S001,
			Cap_Data.cap_I_arm,

			// cnt_on_P,
			// Cap_Data.Cap_farad_value,
			// Cap_Data.Ratio,

			// Cap_Data.state_core,
			// Fmac_Read_data(),
			

			Sample_Data.sample_Pin24V_V,
			
			Sample_Data.sample_Cap_V,
			
			// (Fmac_Read_data()),
			// Cap_Data.cap_I_arm,
			// ADC_RawData.adc_Cap_I_OPAmp,
			// hfmac.Instance->WDATA,
			
			
			Sample_Data.sample_Chassis_Power,
			// Sample_Data.sample_Battery_I,
			// Sample_Data.sample_Chassis_I,
			// cnt_on_P
			Sample_Data.sample_Battery_Power
			// Sample_Data.sample_Cap_V*Sample_Data.sample_Cap_I
		};

		// if ((uint16_t)cnt_on_P%2==0)
		{
			Anotc_Send_float_Data(data, 6, 0xF1);
		}

}

void current_Uart_sent(void)
{
	
		float data[2] = {
			
			Sample_Data.sample_Cap_I,
			Cap_Data.cap_I_arm};
		// if ((uint16_t)cnt_on_I%2==0)
		Anotc_Send_float_Data(data, 2, 0xF2);

	
}