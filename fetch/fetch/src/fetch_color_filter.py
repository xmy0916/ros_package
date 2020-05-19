#!/usr/bin/env python
# BEGIN ALL
# coding=utf-8
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import sensor


class Catch:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('head_camera/rgb/image_raw', 
                                      Image, self.image_callback)
  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    #cv2.imshow("ori", image )
    # END BRIDGE
    # BEGIN HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv )
    # END HSV
    # BEGIN FILTER
    lower_green = numpy.array([ 35,  43, 46])
    upper_green = numpy.array([77, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # END FILTER
    masked = cv2.bitwise_and(image, image, mask=mask)
    #cv2.imshow("res", mask ) 
    
    f_x = (2.8 / 3.984) * 160
    f_y = (2.8 / 2.952) * 120
    c_x = 160 * 0.5
    c_y = 120 * 0.5
    img = sensor()
    img = image
    for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
            degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
        print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)

    cv2.imshow("window", img)
    cv2.waitKey(3)

rospy.init_node('Catch')
catchbin = Catch()
rospy.spin()
# END ALL
