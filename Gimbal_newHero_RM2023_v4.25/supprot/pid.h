#ifndef _PID_H
#define _PID_H

#include "struct_typedef.h"

/**
  * @brief          枚举两种PID模式：位置PID和增量式PID
  * @param[in]      设置PID_POSTION=0,PID_DELTA=1,相当于宏定义“#define PID_POSTION 0””#define PID_DELTA 1”
  */
enum PID_MODE
{
	PID_POSITION=0,
	PID_DELTA
};



/**
  * @brief          自定义PID结构体，包含PID要用的所有参数，有些是中间变量，定义进去可能为了调试的时候看的更明白
  * @param[in]      PID本身属性变量：mode,Kp,Ki,Kd;限制变量：max_out,max_iout;外界输入变量：set,fdb;中间用于计算的变量：out,Pout,Iout,Dout,Dbuf[3],error[3];
  */
typedef struct
{
	uint8_t mode;//PID模式：增量式和位置式
	
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;
	
	fp32 max_out;//最大输出
	fp32 max_iout;//最大积分输出
	
	fp32 set;//set，目标值
	fp32 fdb;//feedback,反馈值

	fp32 out;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	
	fp32 Dbuf[3];//微分项，Dbuf[0]是现在的，Dbuf[1]是上一次的，Dbuf[2]是上上次的
	fp32 error[3];//误差项，error[0]是现在的，error[1]是上一次的，error[2]是上上次的
	
}pid_type_def;



void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

fp32 PID_calc(pid_type_def *pid,	fp32 ref,	fp32 set);

void pid_clear(pid_type_def *pid);






#endif
