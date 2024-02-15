#!/usr/bin/env python

import time
from math import sin, cos

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimpleMover():

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)

        if rospy.has_param('/profi2022_bachelor_solution/altitude_desired'):
            self.altitude_desired = rospy.get_param('/profi2022_bachelor_solution/altitude_desired')
        else:
            rospy.logerr("Failed to get param '/profi2022_bachelor_solution/altitude_desired'")

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("cam_1/camera/image", Image, self.camera_cb)
        rospy.Subscriber("cam_2/camera/image", Image, self.camera_cb2)
        self.rate = rospy.Rate(30)

        #
        # cv2.namedWindow("uraaa")
        # cv2.namedWindow("uraaa 2")
        # rospy.logerr("named window created")



        rospy.on_shutdown(self.shutdown)


    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image1 = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_cb2(self, msg):

        try:
            cv_image2 = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image2 = cv_image2
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def enable_motors(self):

        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ")
            print(e)


    def take_off(self):

        self.enable_motors()

        start_time = time.time()
        end_time = start_time + 3
        twist_msg = Twist()
        twist_msg.linear.z = 1.0

        while (time.time() < end_time) and (not rospy.is_shutdown()):
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()



    def spin(self):

        self.take_off()

        start_time = time.time()
        while not rospy.is_shutdown():
            twist_msg = Twist()
            t = time.time() - start_time
            twist_msg.linear.z = 0.8 * cos(1.2 * t)
            twist_msg.linear.y = 0.8 * sin(0.6 * t)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

            if self.Image1 is not None and self.Image2 is not None:
                cv2.imshow("Down view camera from Robot", self.Image1)
                cv2.imshow("Front view camera from Robot", self.Image2)
                cv2.waitKey(3)



    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


simple_mover = SimpleMover()

simple_mover.spin()
