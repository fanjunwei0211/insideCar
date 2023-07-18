#ifndef _PID_H
#define _PID_H

#include "struct_typedef.h"

/**
  * @brief          ö������PIDģʽ��λ��PID������ʽPID
  * @param[in]      ����PID_POSTION=0,PID_DELTA=1,�൱�ں궨�塰#define PID_POSTION 0����#define PID_DELTA 1��
  */
enum PID_MODE
{
	PID_POSITION=0,
	PID_DELTA
};



/**
  * @brief          �Զ���PID�ṹ�壬����PIDҪ�õ����в�������Щ���м�����������ȥ����Ϊ�˵��Ե�ʱ�򿴵ĸ�����
  * @param[in]      PID�������Ա�����mode,Kp,Ki,Kd;���Ʊ�����max_out,max_iout;������������set,fdb;�м����ڼ���ı�����out,Pout,Iout,Dout,Dbuf[3],error[3];
  */
typedef struct
{
	uint8_t mode;//PIDģʽ������ʽ��λ��ʽ
	
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;
	
	fp32 max_out;//������
	fp32 max_iout;//���������
	
	fp32 set;//set��Ŀ��ֵ
	fp32 fdb;//feedback,����ֵ

	fp32 out;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	
	fp32 Dbuf[3];//΢���Dbuf[0]�����ڵģ�Dbuf[1]����һ�εģ�Dbuf[2]�����ϴε�
	fp32 error[3];//����error[0]�����ڵģ�error[1]����һ�εģ�error[2]�����ϴε�
	
}pid_type_def;



void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

fp32 PID_calc(pid_type_def *pid,	fp32 ref,	fp32 set);

void pid_clear(pid_type_def *pid);






#endif
