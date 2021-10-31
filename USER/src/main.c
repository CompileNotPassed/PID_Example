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
#include "pid.c"
#include "sendware.c"
#define DIR P35
#define PWM_FREQ_MAX 2500
//board.h文件中FOSC的值设置为0,则程序自动识别系统频率

/*board.h文件中FOSC的值设置不为0，则系统频率为FOSC的值，
在使用stc-isp工具下载程序的时候需要将IRC频率设置为FOSC的值*/

/*在board_init中,已经将P54引脚设置为复位，
如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可*/

int16 duty,dataFrame[1];


void main()
{
	DisableGlobalIRQ(); //关闭总中断
	board_init();		//初始化内部寄存器，勿删除此句代码。

	//此处编写用户代码(例如：外设初始化代码等)
	pwm_init(PWM4P_P66, 10000, 0);
	ctimer_count_init(CTIM0_P34);

	EnableGlobalIRQ(); //开启总中断
	PID_Init();
	while (1)
	{
		P45 = 0;
		if(DIR == 1)
		{
			measuredValue = -ctimer_count_read(CTIM0_P34);
		}
		else
		{
			measuredValue = ctimer_count_read(CTIM0_P34);
		}

		ctimer_count_clean(CTIM0_P34);
		PID_Controller();

		duty+=(int)(2.29*increment);
		if(duty>=PWM_FREQ_MAX){
			duty=PWM_FREQ_MAX;
		}
		if(duty<=0){
			duty=0;
		}
		
		pwm_duty(PWM4P_P66,duty);
		dataFrame[0]=measuredValue;
		printf("duty=%d,output=%d",duty,output);
		vcan_sendware(dataFrame,sizeof(dataFrame));
		delay_ms(100);
	}
}