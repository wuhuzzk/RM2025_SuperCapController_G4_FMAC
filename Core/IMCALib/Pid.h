#ifndef __pid_H
#define __pid_H


#include "stm32G4xx_hal.h"


enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};


/*PID结构体*/
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    float f;//前馈

    float set[3];	//目标值，NOW, LAST, LLAST
    float get[3];	//测量值
    float err[3];	//误差
	
    
    float pout;		//P输出				
    float iout;		//I输出					
    float dout;		//D输出

    float pos_out;        //本次位置式输出，即 pos_out = pout + iout + dout
    float last_pos_out;   //上次位置式输出
    float delta_u;        //本次增量值
    float delta_out;      //本次增量式输出 = last_delta_out + delta_u
    float last_delta_out; //上次增量式输出

    float max_err;              //最大偏差
    float deadband;				//err < deadband return
    uint32_t pid_mode;          //PID模式，分位置式和增量式
    float MaxOutput;			//输出限幅
    float MinOutput;			//输出限幅
    float IntegralLimit;		//积分限幅
    float IntegralLimitmin;		//积分限幅

	uint8_t feedforward_cap;	//是否使用电容前馈控制

    
    void (*f_param_init)(struct __pid_t *pid,  /*定义PID参数初始化的函数指针*/
                    uint32_t pid_mode,
                    float maxOutput,
                    float minOutput,
                    float integralLimit,
                    float IntegralLimitmin,
                    float p,
                    float i,
                    float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);   //pid参数修改

}Pid_t;

typedef struct	__FFC_t
{
	
	float rin;
	float lastRin;
	float perrRin;
	float lastOut;
	float perrOut;
	float	FFC_pos_out;
	float	set_point;

}FFC_t;

/*定义PID结构体初始化函数*/
void PID_struct_init(Pid_t* pid, uint32_t mode, float maxout, float minout,float intergral_limit,float intergral_limit_min,float kp, float ki, float kd);
void PID_struct_updata(Pid_t* pid,float kp, float ki, float kd);
/*PID计算函数*/
float pid_calc(Pid_t* pid, float fdb, float ref);
float FeedForwardController(FFC_t *FFC,int32_t target,float num1,float num2);
float FeedForward_Cap(FFC_t *FFC, float I_set);
void clamp_f_limit(float MIN, float MAX, float *a);
void clamp_i32_limit( int32_t MIN,int32_t MAX,int32_t *a);
void clamp_ui32_limit(uint32_t MIN, uint32_t MAX, uint32_t *a);
void Pid_clear_i(Pid_t *pid);

extern float angle_err;

//float KfPidCalc(Pid_t* pid, extKalman_t *Kf, float get, float set);

		
#endif

