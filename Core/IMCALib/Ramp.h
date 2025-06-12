/******************************************************************************
*** @File           : Ramp.h
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
#ifndef __RAMP_H
#define __RAMP_H
#include "stm32g4xx_hal.h"
#include "math.h"
// #include "mytype.h"
// #include "ChassisControl.h"


//����������Ƽ�� 0.001s
#define CHASSIS_CONTROL_TIME 0.001f
/*************************************************************************************************/
//��ֵԽ���˲�ϵ����Խ����ֵ�仯Խƽ��
#define CHASSIS_ACCEL_X_NUM 0.6666666667f  //0.0633333333f  //����ƽ��
#define CHASSIS_ACCEL_Y_NUM 0.5333333333f  //0.0799999999f  //ǰ���˶�
#define CHASSIS_ACCEL_W_NUM 0.0833333333f  //0.1333333333f 

//#define CHASSIS_ACCEL_X_NUM 0.0733333333f  //����ƽ��
//#define CHASSIS_ACCEL_Y_NUM 0.4899999999f  //ǰ���˶�

/************************************************************************************************/


//���ڵ���һ���˲�����
typedef struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;


//typedef struct{
//    
//    int32_t compare_value_up;
//    int32_t compare_value_dowm;
//    int sin_ramp_switch;
//    
//}SinRampState;
typedef struct
{
 float x_last;
 float p_last;
 float out;
} one_kalman_filter_init_t;

extern one_kalman_filter_init_t kalman_Cap_I_arm;



//һ���˲���ʼ��
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//һ���˲�����
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);



void ChassisRampInit(void);

void SlopeSlow(float *rec, float target, float slow_Inc);

//int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);

//void SinRampInit(SinRampState *sin_state);

//void SinTest(void);

float  KalmanFilter(one_kalman_filter_init_t * data,float ResrcData,float ProcessNoise_Q,float MeasureNoise_R);

void FloatToChar(float fNum, unsigned char *strBuf, int nLen);
void CharToFloat(float *fNum, unsigned char *strBuf, int nLen) ;
void Uint16ToChar(uint16_t fNum,unsigned char *strBuf,int nLen);
void CharToUint16(uint16_t *fNum,unsigned char *strBuf,int nLen);
void int16ToChar(int16_t fNum,unsigned char *strBuf,int nLen);
void CharToint16(int16_t *fNum,unsigned char *strBuf,int nLen);



#endif
