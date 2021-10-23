/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"

//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位，
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/

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
	DisableGlobalIRQ(); //关闭总中断
	board_init();		//初始化内部寄存器，勿删除此句代码。

	//此处编写用户代码(例如：外设初始化代码等)

	EnableGlobalIRQ(); //开启总中断
	PID_Init(); 
	while(1){
		P54=0;
        measuredValue = output;
       	PID_Controller();
		delay_ms(100);
		printf("%lf\n",output/1000);
	}

}
