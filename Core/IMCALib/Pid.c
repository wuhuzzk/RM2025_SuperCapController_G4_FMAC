/*************************************************************************************************************
 * @file			pid.c
 * @brief   		对于PID, 反馈/测量习惯性叫get/measure/real/fdb,
 *					期望值输入一般叫set/target/ref
 * @details 		此pid文件经过修改，分离上限和下限，精简到只剩增量式PID
************************************************************************************************************/
  
  
#include "Pid.h"
#include <math.h>

#define ABS(x)		((x>0)? (x): (-x))  /*取绝对值*/

extern Pid_t pid_cap_current;
FFC_t ffc_c;

/**********************************************************************************************************************
 * @brief   数值限幅
 * @param   float *a        输入数据的指针变量
 *          float ABS_MAX   最值
 * @retval  None
**********************************************************************************************************************/
void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void clamp_f_limit( float MIN,float MAX,float *a)
{
    if(*a > MAX)
        *a = MAX;
    if(*a < MIN)
        *a = MIN;
}
void clamp_i32_limit( int32_t MIN,int32_t MAX,int32_t *a)
{
    if(*a > MAX)
        *a = MAX;
    if(*a < MIN)
        *a = MIN;
}
void clamp_ui32_limit( uint32_t MIN,uint32_t MAX,uint32_t *a)
{
    if(*a > MAX)
        *a = MAX;
    if(*a < MIN)
        *a = MIN;
}

/**********************************************************************************************************************
 * @brief   PID参数初始化
 * @param   pid_t* pid                  PID结构体
 *          uint32_t mode               PID模式选择，分位置式PID和增量式PID
 *          uint32_t maxout             输出限幅
 *          uint32_t intergral_limit    积分限幅
 *          float kp, ki, kd            PID的三个参数
 * @retval  None
**********************************************************************************************************************/
static void pid_param_init(
        Pid_t *pid,
        uint32_t mode,
        float maxout,
        float minout,
        float intergral_limit,
        float intergral_limit_min,
        float 	kp,
        float 	ki,
        float 	kd )
{
    
    pid->IntegralLimit = intergral_limit;
    pid->IntegralLimitmin = intergral_limit_min;
    pid->MaxOutput = maxout;
    pid->MinOutput = minout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}


///*************************调试参数赋值******************************/
//static void pid_reset(Pid_t	*pid, float kp, float ki, float kd)
//{
//    pid->p = kp;
//    pid->i = ki;
//    pid->d = kd;
//}


/**********************************************************************************************************************
 * @brief   calculate delta PID and position PID
 * @param   pid_t* pid：PID结构体
 *          set: target
 *          get：measure
 * @retval  None
 * @others  PID相关资料：https://blog.csdn.net/as480133937/article/details/89508034
**********************************************************************************************************************/
float pid_calc(Pid_t* pid, float get, float set)
{
    
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	/*set - measure，得到偏差*/
    // if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
	// 	return 0;   /*如果偏差大于最大偏差就跳出PID计算*/
	// if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	// 	return 0;  /*偏差在死区范围内不作处理，跳出PID计算*/
    
    // if(pid->pid_mode == POSITION_PID) /*位置式PID计算*/
    // {
    //     pid->pout = pid->p * pid->err[NOW];
    //     pid->iout += pid->i * pid->err[NOW];
    //     pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    //     clamp_f_limit(pid->IntegralLimitmin, pid->IntegralLimit,&(pid->iout)); /*积分限幅*/
    //     pid->pos_out = pid->pout + pid->iout + pid->dout;

    //     clamp_f_limit(pid->MinOutput, pid->MaxOutput,&(pid->pos_out));  /*限定输出值的大小*/

    //     pid->last_pos_out = pid->pos_out;	/*update last time*/ 
    // }
    // else if(pid->pid_mode == DELTA_PID) /*增量式PID计算*/
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        clamp_f_limit(pid->IntegralLimitmin, pid->IntegralLimit,&(pid->iout)); /*积分限幅*/
        pid->delta_u = pid->pout + pid->iout + pid->dout; /*本次增量值*/
        pid->delta_out = pid->last_delta_out + pid->delta_u; /*本次增量式输出*/
        clamp_f_limit(pid->MinOutput, pid->MaxOutput,&(pid->delta_out));  /*输出限幅*/
        pid->last_delta_out = pid->delta_out;	/*update last time*/
    }

	

	/*更新数据*/
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    // pid->get[LLAST] = pid->get[LAST];
    // pid->get[LAST] = pid->get[NOW];
    // pid->set[LLAST] = pid->set[LAST];
    // pid->set[LAST] = pid->set[NOW];
    // return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out; /*PID输出*/
    return  pid->delta_out; /*PID输出*/
	
}

    //电容前馈 y(n)=0.08 *x(n) - 0.0799985 x(n-1) + y(n-1)

/**********************************************************************************************************************
 * @brief   PID总体的初始化
 * @param   pid_t* pid                  PID结构体
 *          uint32_t mode               PID模式选择，分位置式PID和增量式PID
 *          uint32_t maxout             输出限幅
 *          uint32_t intergral_limit    积分限幅
 *          float kp, ki, kd            PID的三个参数
 * @retval  None
**********************************************************************************************************************/
void PID_struct_init(
        Pid_t* pid,
        uint32_t mode,
        float maxout,
        float minout,
        float intergral_limit,
        float intergral_limit_min,
        float 	kp,
        float 	ki,
        float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init; /*将pid_param_init函数的首地址赋值给指针变量f_param_init*/
//    pid->f_pid_reset = pid_reset;
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout,minout, intergral_limit, intergral_limit_min,kp, ki, kd);
	
}
void PID_struct_updata(Pid_t* pid,float 	kp,float 	ki,float 	kd)
{

        uint32_t mode=pid->pid_mode;
        float maxout=pid->MaxOutput;
        float minout=pid->MinOutput;
        float intergral_limit=pid->IntegralLimit;
        float intergral_limit_min=pid->IntegralLimitmin;
    pid->f_param_init = pid_param_init; 

		

    pid->f_param_init(pid, mode, maxout,minout, intergral_limit, intergral_limit_min,kp, ki, kd);
	
}
/*********************************************************************************************************************
*@brief 		前馈控制器处理
*@param			FFC_t	 FFC	前馈结构体
*						int32_t	target 	系统输入目标值
*						num1						由系统动力学建模得来的常数a
*						num2						由系统动力学建模得来的常数a
*						rin							当前系统反馈值
*						lastrin					上一时刻系统反馈值
*						perrrin					上上一时刻系统反馈值
*@retval		具体了解前馈控制器的定义请前往以下网址：https://blog.csdn.net/foxclever/article/details/81048086?spm=1001.2014.3001.5506
**********************************************************************************************************************/

float FeedForwardController(FFC_t *FFC,int32_t target,float num1,float num2)
{
	
	FFC->rin=target;									//将目标值赋到rin上
	
  float result;
 
  result=num1*(FFC->rin-FFC->lastRin)+num2*(FFC->rin-2*FFC->lastRin+FFC->perrRin);
 
	FFC->lastRin= FFC->rin;						//转移上次输出与上上次输出
  FFC->perrRin= FFC->lastRin;
  FFC->FFC_pos_out=result;					//将输出值赋到结构体输出值中

	return result;
}




void Pid_clear_i(Pid_t* pid)
{
    if(pid->pid_mode == POSITION_PID) /*位置式PID计算*/
    {
        pid->iout=0;
    }
    else if(pid->pid_mode == DELTA_PID) /*增量式PID计算*/
    {
        pid->last_delta_out =0;
    }
    pid->err[NOW]=0;
    pid->err[LAST]=0;
    pid->err[LLAST]=0;
}



