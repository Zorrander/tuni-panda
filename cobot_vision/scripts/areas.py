#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class AreasServer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.areas = ["Shared", "Delivery", "Robot"]
        self.image_pub = rospy.Publisher("/workspace", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, camera_msg):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        except CvBridgeError as e:
          print(e)

        edges = cv2.Canny(cv_image,100,200)

        cv2.imshow("Image window", edges)
        cv2.waitKey(3)
        '''
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)
        '''


if __name__ == "__main__":
    rospy.init_node('workspace_detector')
    AreasServer()
    print "Workspace detection running."
    rospy.spin()
