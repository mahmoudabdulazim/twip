#include "ros/ros.h"
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <twip/ThreeDOFState.h>
#include <twip/ControlSignal.h>

using namespace std;
using namespace ros;

int pwmPin_A1= 0;
int pwmPin_A2= 2;
int dirPin_B1= 1;
int dirPin_B2= 3;

double error,derror,kp=5,kd=3,oldPitch = 45,control;

void flipDir(int pinDir)
{
	int flag = digitalRead(pinDir);
	switch (flag)
	{
		case 1:
			digitalWrite(pinDir,LOW);
		case 0:
			digitalWrite(pinDir,HIGH);
	}
}
void SetUPMotorPins(void)
{
	pinMode(pwmPin_A1,PWM_OUTPUT);
	pinMode(pwmPin_A2,PWM_OUTPUT);
	pinMode(dirPin_B1,OUTPUT);
	pinMode(dirPin_B2,OUTPUT);
	digitalWrite(dirPin_B1,LOW);
	digitalWrite(dirPin_B2,HIGH);
	softPwmCreate(pwmPin_A1,0,100);
	softPwmCreate(pwmPin_A2,0,100);
}

void ControlSignal2PWM(int controlsignal,int PWMPin,int DirPin)
{
	if (controlsignal > 0)
	{
		digitalWrite(DirPin,HIGH);
	}
	else
	{
		digitalWrite(DirPin,LOW);
	}
	softPwmWrite(PWMPin,abs((int)controlsignal/30));
}
void driveMotors(const twip::ControlSignal::ConstPtr& Control)
{
	ControlSignal2PWM((int)Control->Motor1,pwmPin_A1,dirPin_B1);
	ControlSignal2PWM((int)Control->Motor2,pwmPin_A2,dirPin_B2);
	ROS_INFO("Control Signals are :   Motor 1 = %.3f,         || Motor 2 = %.3f",Control->Motor1,Control->Motor2);
}
int main(int argc, char  **argv)
{
	wiringPiSetup();
	SetUPMotorPins();
	init(argc,argv,"MotorController");
	NodeHandle MD;
	ros::Subscriber sub = MD.subscribe("/ControlAction",1,driveMotors);
	ros::spin();
	return 0;
}
