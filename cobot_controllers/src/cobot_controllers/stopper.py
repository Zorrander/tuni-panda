import rospy 
from std_msgs.msg import Int16







def callback(data,stop_srv):

	req = StopActionRequest()
	if data == "stop":
		srv_res + stop_srv(req)


def stopper():

	rospy.init_node("stopper_action", anonymous=True)
	request_publisher = rospy.Publisher('/request_detection', Int16 , queue_size=10)
	stop_srv = rospy.ServiceProxy("/stop_actions", StopAction)

	data = input("give command: ") 


	return data




if __name__ == '__main__':
	stopper()


