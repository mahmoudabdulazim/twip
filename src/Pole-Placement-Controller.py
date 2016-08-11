#!/usr/bin/env python

import numpy as np
import rospy
from twip.msg import ThreeDOFState
from twip.msg import ControlSignal

K = np.matrix([[-1865,-8969,0,-1203,64,-21],[-1865,-8969,0,-1031.8,64,21]])

Pub = rospy.Publisher('ControlAction',ControlSignal,queue_size=1)

def sendControl(data):

	msg        = ControlSignal()
	states     = np.matrix([[data.X],[data.Pitch],[data.Yaw],[data.V],[data.PitchRate],[data.YawRate]])
	u          = K*states
	msg.Motor1 = - u[np.array([0])]
	msg.Motor2 = - u[np.array([1])]

	if (msg.Motor1 > 2999):
		msg.Motor1 = 2999

	if (msg.Motor1 < -2999):
		msg.Motor1 = -2999

	if (msg.Motor2 > 2999):
		msg.Motor2 = 2999

        if (msg.Motor2 < -2999):
                msg.Motor2 = -2999


	Pub.publish(msg)
	rospy.loginfo(msg)

def listener():
	rospy.init_node('Pole_Placement_Controller',anonymous=True)
	rospy.Subscriber("TWIPStates",ThreeDOFState,sendControl)
	rospy.spin()

if __name__ == '__main__':
	listener()
