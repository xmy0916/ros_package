#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    cv2.imshow("ori", image )
    # END BRIDGE
    # BEGIN HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv )
    # END HSV
    # BEGIN FILTER
    lower_yellow = numpy.array([ 26,  43, 46])
    upper_yellow = numpy.array([34, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # END FILTER
    masked = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow("window2", mask ) 
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
