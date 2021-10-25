#include "headfile.h"

float kp, kd, ki;
int16 timeElapsed, previousError, integral, error, increment,target, measuredValue, output,thresholdValue;

int abs(int number){
    return number>0?number:-number;
}

int beta(){
    double error = target - measuredValue;
    return thresholdValue>abs(error)?0:1;
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
	timeElapsed = 1;
	measuredValue = 0;
	target = 500;
	kp = 0.05;
	ki = 0.0;
	kd = 0.0;
	thresholdValue = 300;
	//Test End

	previousError = 0;
	integral = 0;
	output = 0;
	error=0;
	increment=0;
}