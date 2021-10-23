/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"

//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ��
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/

float kp, kd, ki, measuredValue, target, timeElapsed, previousError, integral, output, error, increment, thresholdValue;

double doubleAbs(double number){
    return number>0?number:-number;
}

int beta(){
    double error = target - measuredValue;
    return thresholdValue>doubleAbs(error)?0:1;
}


void PID_Controller()
{
	error = target - measuredValue;
	integral += error;
	increment = kp * error + beta() * ki * integral + kd * (error - previousError) / timeElapsed;
	output = measuredValue + increment;
	previousError = error;
}

void PID_Init()
{
	//Only for Test use
	timeElapsed = 1.0;
	measuredValue = 50.0;
	target = 100.0;
	kp=0.6;
	ki=0.02;
	kd=0.05;
	thresholdValue=70.0;
	//Test End

	previousError = 0.0;
	integral = 0.0;
	output = 0.0;
}

void main()
{
	DisableGlobalIRQ(); //�ر����ж�
	board_init();		//��ʼ���ڲ��Ĵ�������ɾ���˾���롣

	//�˴���д�û�����(���磺�����ʼ�������)

	EnableGlobalIRQ(); //�������ж�
	PID_Init(); 
	while(1){
		P54=0;
        measuredValue = output;
       	PID_Controller();
		delay_ms(100);
		printf("%lf\n",output/1000);
	}

}
