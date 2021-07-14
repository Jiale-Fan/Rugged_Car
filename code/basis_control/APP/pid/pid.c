#include "pid.h"
//#include "usart.h"
#include <stdio.h>

struct pid_uint pid_Task;

/****************************************************************************
*�������ƣ�PID_Init(void)
*�������ܣ���ʼ��PID�ṹ�����
****************************************************************************/

void PID_Init(void)
{
//����1024ԭ�������ָ��������㣬ȫ�����������㣬����PID�����������ٶȻ����
/***********************�����ٶ�pid****************************/
	pid_Task.Kp = 1024 * 0.08;//0.4
 	pid_Task.Ki = 1024 * 0;	
	pid_Task.Kd = 1024 * 0.01; 
	pid_Task.Ur = 1024 * 359;
	pid_Task.Adjust   = 0;
	pid_Task.En       = 1;
	pid_Task.speedSet = 0;
	pid_Task.speedNow = 0;
	reset_Uk(&pid_Task);		
}

/***********************************************************************************************
 �� �� ����void reset_Uk(PID_Uint *p)
 ��    �ܣ���ʼ��U_kk,ekk,ekkk
 ˵    �����ڳ�ʼ��ʱ���ã��ı�PID����ʱ�п�����Ҫ����
 ��ڲ�����PID��Ԫ�Ĳ����ṹ�� ��ַ
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}

/***********************************************************************************************
 �� �� ����s32 PID_commen(int set,int jiance,PID_Uint *p)
 ��    �ܣ�����PID
 ˵    ���������ⵥ��PID�Ŀ�����
 ��ڲ���������ֵ��ʵ��ֵ��PID��Ԫ�ṹ��
 �� �� ֵ��PID������
************************************************************************************************/

s32 PID_common(int set,int detect,struct pid_uint *p)
{
	int ek=0,U_k=0;

	ek=detect - set;                                                               
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
  p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	//printf("set=%d#now=%d#ek=%d#uk=%d#",set,detect,ek,U_k>>10);
	return U_k>>10; 
}

/***********************************************************************************
** �������� ��void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** �������� ��pidѡ����	      
***********************************************************************************/

void Pid_Which(struct pid_uint *p)
{
	/**********************�����ٶ�pid*************************/
	if(p->En == 1)
	{									
		p->Adjust = -PID_common(p->speedSet, p->speedNow, p);		
	}	
	else
	{
		p->Adjust = 0;
		reset_Uk(p);
		p->En = 2; 
	}
}

/*******************************************************************************
 * ��������Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * ����  ��Pid����
 *******************************************************************************/

void Pid_Ctrl(int *Motor)
{
	Pid_Which(&pid_Task); 
	*Motor += pid_Task.Adjust;
	if (*Motor < 0)
	{
		*Motor = 0;
	}
	if (*Motor > 3600)
	{
		*Motor = 3599;
	}
}

