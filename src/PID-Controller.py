import rospy
from twip.msg import ThreeDOFState
from twip.msg import ControlSignal

Pub = rospy.Publisher('ControlAction',ControlSignal,queue_size=1)

Kp = 
Kd = 
Ki = 

def sendControl(data):
	msg = ControlSignal()
	u = Kp*data.Pitch + Kd*PitchRate

	msg.Motor1 = u
	msg.Motor2 = u

	if (abs(msg.Motor1)>2999)
		msg.Motor1 = 2999

	if (abs(msg.Motor2)>2999)
		msg.Motor2 = 2999


	Pub.publish(msg)

def listener():
        rospy.init_node('Pole_Placement_Controller',anonymous=True)
        rospy.Subscriber("TWIPStates",ThreeDOFState,sendControl)
        rospy.spin()

if __name__ == '__main__':
        listener()

