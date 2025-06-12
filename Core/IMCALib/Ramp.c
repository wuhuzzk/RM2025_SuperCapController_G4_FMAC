/******************************************************************************
*** @File           : Ramp.c
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
******************************************************************************/
#include "Ramp.h"

/*一阶低通滤波结构体*/
one_kalman_filter_init_t kalman_Cap_I_arm;

void FloatToChar(float fNum, unsigned char *strBuf, int nLen)
{
  if (nLen < 4)
  return;
  int i = 0;
//  unsigned char nTmp;
  char *p = (char *)&fNum;
  for (i = 0; i < 4; i++) {
  strBuf[i] = *p;
  p++;
 }
}

void CharToFloat(float *fNum, unsigned char *strBuf, int nLen) 
{
 if (nLen < 4)
 return;
 int i = 0;
// unsigned char nTmp;
 char *p = (char *)fNum;
 for (i = 0; i < 4; i++) {
 *p = strBuf[i];
 p++;
 }
}

void Uint16ToChar(uint16_t fNum,unsigned char *strBuf,int nLen)
{
	if (nLen < 2)
  return;
  int i = 0;
//  unsigned char nTmp;
  char *p = (char *)&fNum;
  for (i = 0; i < 2; i++) {
  strBuf[i] = *p;
  p++;
 }
}

void CharToUint16(uint16_t *fNum,unsigned char *strBuf,int nLen)
{
	if (nLen < 2)
  return;
  int i = 0;
//  unsigned char nTmp;
  char *p = (char *)fNum;
  for (i = 0; i < 2; i++) {
  *p = strBuf[i];
  p++;
 }
}

void int16ToChar(int16_t fNum,unsigned char *strBuf,int nLen)
{
	if (nLen < 2)
  return;
  int i = 0;
//  unsigned char nTmp;
  char *p = (char *)&fNum;
  for (i = 0; i < 2; i++) {
  strBuf[i] = *p;
  p++;
 }
}

void CharToint16(int16_t *fNum,unsigned char *strBuf,int nLen)
{
	if (nLen < 2)
  return;
  int i = 0;
//  unsigned char nTmp;
  char *p = (char *)fNum;
  for (i = 0; i < 2; i++) {
  *p = strBuf[i];
  p++;
 }
}





void GimbalSlow(float *rec , float target , float slow_Inc)
{
  if(fabs(*rec - target) < slow_Inc) *rec = target;
  else {
    if((*rec) > target) (*rec) -= slow_Inc;
    if((*rec) < target) (*rec) += slow_Inc;
  }
}

/*************************************************************************************************************
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/*************************************************************************************************************
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
*************************************************************************************************************/
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}




/****************************************************************************
* 函数名 :
* 功 能 :斜坡函数
* 输 入 :每1ms 增加slow_Inc一次
* 全局变量 :
* 输 出 :
*****************************************************************************/
void SlopeSlow(float *rec , float target , float slow_Inc)
{
	if(*rec < target)
	{
		*rec += slow_Inc;
		if(*rec > target)
		{
			*rec = target;
		}
	}
	else if(*rec > target)
	{
		*rec -= slow_Inc;
		if(*rec < target)
		{
			*rec = target;
		}
	}
}




/**
 * @brief  一维卡尔曼滤波
  * @param  none
  * @retval none
  * @other  Q：过程噪声，Q增大，动态响应变快，收敛稳定性变坏
      R：测量噪声，R增大，动态响应变慢，收敛稳定性变好
  */
float  KalmanFilter(one_kalman_filter_init_t * data,const float ResrcData,float ProcessNoise_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNoise_Q;
   float x_mid;
   float x_now;
   float p_mid ;
   float p_now;
   float kg;
   x_mid=data->x_last;
   p_mid=data->p_last+Q;
   kg=p_mid/(p_mid+R);
   x_now=x_mid+kg*(ResrcData-x_mid);

   p_now=(1-kg)*p_mid;
   data->p_last = p_now;
   data->x_last = x_now;
   return x_now;
}
/**************************************END*******************************************/


