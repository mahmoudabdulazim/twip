#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stropts.h>
#include <poll.h>
#include "ros/ros.h"
#include "twip/ThreeDOFState.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Eigen/Dense>
#include <pigpio.h>

#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#define MPU6050_AXGAIN 1638.4
#define MPU6050_GYOFFSET 0
#define MPU6050_GZOFFSET 0
#define MPU6050_AXOFFSET 0


#define MOTOR_1_PWM 17
#define MOTOR_2_PWM 27
#define MOTOR_1_DIR 18
#define MOTOR_2_DIR 22
#define PWM_FREQ    10000
#define PWM_RANGE   3000

MPU6050 mpu;

using namespace std;

bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize=42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion qs;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

int16_t rpq[3];
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double y = 0;

void InitializeMPU()
{
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	printf("Testing device connections...\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0)
	{
	        printf("Enabling DMP...\n");
	        mpu.setDMPEnabled(true);

	        mpuIntStatus = mpu.getIntStatus();
	        printf("DMP ready!\n");
	        dmpReady = true;

	        packetSize = mpu.dmpGetFIFOPacketSize();
		printf("Setup Done");
	}
	else
	{
        	printf("DMP Initialization failed (code %d)\n", devStatus);
	}
}

void SetUPMotorPins(void)
{
//	Specifying GPIO pins as OUTPUT
	gpioSetMode(MOTOR_1_PWM,PI_OUTPUT);
        gpioSetMode(MOTOR_2_PWM,PI_OUTPUT);
        gpioSetMode(MOTOR_1_DIR,PI_OUTPUT);
        gpioSetMode(MOTOR_2_DIR,PI_OUTPUT);

//	Initial Direction to be positive
        gpioWrite(MOTOR_1_DIR,0);
        gpioWrite(MOTOR_2_DIR,0);

// 	for Debugging purposes
        printf("Initializing PWM on pins: %d and %d \n",MOTOR_1_PWM,MOTOR_2_PWM);

//	Setting PWM Frequency for Motors
        gpioSetPWMfrequency(MOTOR_1_PWM, PWM_FREQ);
        gpioSetPWMfrequency(MOTOR_2_PWM, PWM_FREQ);

//	Setting PWM Duty Cycle Range for Motors
        gpioSetPWMrange(MOTOR_1_PWM,3000);
        gpioSetPWMrange(MOTOR_2_PWM,3000);
}

void ControlSignal2PWM(double PWMR,double PWML)
{
//	Writing Motors PWM Value
	gpioPWM(MOTOR_1_PWM,(int)abs(PWMR));
	gpioPWM(MOTOR_2_PWM,(int)abs(PWML));

//	Specifying Motors Direction
        if (PWMR < 0)
        {
		gpioWrite(MOTOR_1_DIR,0);
        }
        else
        {
		gpioWrite(MOTOR_1_DIR,1);
        }
        if (PWML < 0)
        {
		gpioWrite(MOTOR_2_DIR,0);
        }
        else
        {
		gpioWrite(MOTOR_2_DIR,1);
        }
}

void readMPUData()
{
    if (!dmpReady) return;
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024)
    {
        mpu.resetFIFO();
    }
    else if (fifoCount >= 42)
    {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&qs, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &qs);
        mpu.dmpGetYawPitchRoll(ypr, &qs, &gravity);
	mpu.dmpGetGyro(rpq,fifoBuffer);
    }
}

void InputSaturation(double &PWMR,double &PWML)
{
	if (PWMR >  3000)	PWMR =  3000;
	if (PWMR < -3000)	PWMR = -3000;

	if (PWML >  3000)	PWML =  3000;
	if (PWML < -3000)	PWML = -3000;
}

int main(int argc, char **argv)
{
//	Declaration of Matrices containers
	Eigen::MatrixXd K(2,7);
	Eigen::MatrixXd States(7,1);
	Eigen::MatrixXd Input(2,1);

//	Full State-Feedback Controller gain
	K << -1900, -11530,0,-3540,2110,0,82,-1900, -11530,0,-3240,2110,0,82;

//	Necessary Declarations
	int timeout = 1,ret;
	int counter = 0;
	signed char x;
	unsigned char data[3];
	float prevX = 0,prevV = 0,prevTime;
	float X=0;
	double PWMR,PWML;

//	Opticl Flow Sensor RAW Data handling
	struct pollfd filedesc[1];
	filedesc[0].fd     = open("/dev/input/mice",O_RDWR);
	filedesc[0].events = POLLIN;

//	ROS Necessary Initialization
	ros::init(argc, argv, "StatePublisher");
	ros::NodeHandle IMU;
	ros::Publisher states_pub = IMU.advertise<twip::ThreeDOFState>("TWIPStates", 1);
	ros::Rate loop_rate(80);

//	Initializing time for use in subsequent steps
	prevTime = ros::Time::now().toSec();

//	Initialize PIGPIO Library

	gpioInitialise();

//	Setting UP Motors and MPU
        SetUPMotorPins();
	InitializeMPU();
	usleep(100000);

	while (ros::ok())
	{
//		Reading MPU DMP Data into correspnding containers
		readMPUData();

//		Building the States Vector
                States(1,0)     = ypr[1]+0.195+0.035;
                States(2,0)     = ypr[2];
                States(4,0)     = (float)(M_PI*rpq[1]/180-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
                States(5,0)     = (float)(M_PI*rpq[2]/180-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
		y	 	= (ypr[1]+0.23)*(ros::Time::now().toSec() - prevTime) + y;
		if (y >  1) y =  5;
		if (y < -1) y = -5;

		States(6,0)	= y;
//		Timeout handling for Optical Flow sensor 
		ret		= poll(filedesc,1,timeout);

//		If data are available to be read:
		if (ret > 0)
		{
//			Read Optical Flow Sensor data
			read(filedesc[0].fd,data,sizeof(data));

			x 		= (signed char)data[2];
                        X       	+= (float)0.0032*(x)/127.0;
                        prevX    	= (float)X;

                        States(0,0)	= prevX;
                        States(3,0)	= (float)(0.0032*x/(127*(ros::Time::now().toSec() - prevTime)));
			prevV 		= States(3,0);
		}
//		If no data is available, use data from previous measurement
		else
		{
			States(0,0)	= prevX;
			States(3,0)	= prevV;
		}

//		Calculating Controller Gains
		Input = -K * States;

		PWMR  = Input(0,0);
		PWML  = Input(1,0);

		InputSaturation(PWMR,PWML);
//		Writing Control Signal to Motors
		ControlSignal2PWM(PWMR,PWML);

//		Debugging Info
		if (counter % 20 == 0)
		{
			ROS_INFO("Pitch is: %.4f || Yaw is: %.4f || X: %.4f || V: %.4f || Pitch Rate: %.4f || Yaw Rate: %.4f",States(1,0),States(2,0),States(0,0),States(3,0),States(4,0),States(5,0));
			ROS_INFO("Right Motor PWM is:  %.2f	|| Left Motor PWM is: %.2f",PWMR,PWML);
		}
//		ROS Necessary Calls for callback and rate handling
		ros::spinOnce();
		loop_rate.sleep();
		counter++;

		if (counter > 1000) counter =0;
//		Updating time
		prevTime	 = ros::Time::now().toSec();
	}

// 	Necessary clean up of gpios
        gpioTerminate();

	return 0;
}
