import rospy 
from sensor_msgs.msg import CameraInfo


class CameraParameters(object):
	"""docstring for CameraParameters"""
	def __init__(self):
		self.camera_focal = 0
		self.ctrX = 0
		self.ctrY = 0

	def camera_info_callback(self, msg):
		self.camera_focal = msg.K[0]
		self.ctrX = msg.K[2]
		self.ctrY = msg.K[5]

