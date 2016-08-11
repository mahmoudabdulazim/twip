#!/usr/bin/env python

import rospy
from twip.msg import ControlSignal
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

PWMPin_A1 = 11
PWMPin_A2 = 13
DIRPin_B1 = 12
DIRPin_B2 = 15

GPIO.setup(PWMPin_A1, GPIO.OUT)
GPIO.setup(DIRPin_B1, GPIO.OUT)
GPIO.setup(PWMPin_A2, GPIO.OUT)
GPIO.setup(DIRPin_B2, GPIO.OUT)

def driveMotors(data):

	global Motor1,Motor2

        Motor1.ChangeDutyCycle(abs(100*data.Motor1/3000))
        Motor2.ChangeDutyCycle(abs(100*data.Motor2/3000))
        if data.Motor1 > 0:
                GPIO.output(DIRPin_B1, GPIO.HIGH)
        elif data.Motor1 < 0:
                GPIO.output(DIRPin_B1, GPIO.LOW)
	
	if data.Motor2 > 0:  
                GPIO.output(DIRPin_B2, GPIO.HIGH)
        elif data.Motor2 < 0:
                GPIO.output(DIRPin_B2, GPIO.LOW)

	rospy.loginfo("Motor 1: %.3f || Motor 2: %.3f" % (int(round(abs(data.Motor1))),int(round(abs(data.Motor2)))))

def listener():
	global Motor1,Motor2
	Motor1 = GPIO.PWM(PWMPin_A1,1000)
	Motor2 = GPIO.PWM(PWMPin_A2,1000)

	rospy.init_node('custom_listener', anonymous=True)
	rospy.Subscriber("ControlAction", ControlSignal,driveMotors,queue_size = 1)
		
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	
	Motor1.stop()
	Motor2.stop()
	GPIO.cleanup()

if __name__ == '__main__':
    listener()
