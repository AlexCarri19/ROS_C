#!/usr/bin/env python
import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class filter() :
    def _init_(self) :
        self.image_pub = rospy.Publisher("segmented_image", Image)
        self.camera_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_cb)
        self.bridge_object = CvBridge()
        self.image_received = 0
        rate = rospy.Rate(50)

        segmented_image = Image()

        self.H_min = 0
        self.H_max = 0
        self.S_min = 0
        self.S_max = 0
        self.V_min = 0
        self.V_max = 0

        # Creating a control TrackBar for the HSV values
        cv2.namedWindow('HSV_TrackBars')
        cv2.createTrackbar('H_min', 'HSV_TrackBars', 0, 180, self.H_min_cb)
        cv2.setTrackbarPos('H_min', 'HSV_TrackBars', 0)
        cv2.createTrackbar('H_max', 'HSV_TrackBars', 0, 180, self.H_max_cb)
        cv2.setTrackbarPos('H_max', 'HSV_TrackBars', 180)
        cv2.createTrackbar('S_min', 'HSV_TrackBars', 0, 255, self.S_min_cb)
        cv2.setTrackbarPos('S_min', 'HSV_TrackBars', 0)
        cv2.createTrackbar('S_max', 'HSV_TrackBars', 0, 255, self.S_max_cb)
        cv2.setTrackbarPos('S_max', 'HSV_TrackBars', 255)
        cv2.createTrackbar('V_min', 'HSV_TrackBars', 0, 255, self.V_min_cb)
        cv2.setTrackbarPos('V_min', 'HSV_TrackBars', 0)
        cv2.createTrackbar('V_max', 'HSV_TrackBars', 0, 255, self.V_max_cb)
        cv2.setTrackbarPos('V_max', 'HSV_TrackBars', 255)

        '''
        For yellow filter : 
            - H_min = 30
            - H_max = 59
            - S_min = 255
            - S_max = 255
            - V_min = 0
            - V_max = 255
        '''

        while not rospy.is_shutdown() :
            if self.image_received :
                image = cv2.resize(self.cv_image, (300, 300))
                cv2.imshow("Camera", image)

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                hsv_max = np.array([self.H_max, self.S_max, self.V_max])

                hsv_mask = cv2.inRange(hsv, hsv_min, hsv_max)
                hsv_filter = cv2.bitwise_and(image, image, mask = hsv_mask)

                cv2.imshow("Filter", hsv_filter)

                segmented_image = self.bridge_object.cv2_to_imgmsg(hsv_filter, encoding = "passthrough")
                self.image_pub.publish(segmented_image)

            cv2.waitKey(1)
            rate.sleep()
        cv2.destroyAllWindows()

    def camera_cb(self, image_data) :
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(image_data, desired_encoding = "bgr8")
        except CvBridgeError as e :
            print(e)
        self.image_received = 1

    def H_min_cb(self, H_min) :
        self.H_min = cv2.getTrackbarPos('H_min', 'HSV_TrackBars')

    def H_max_cb(self, H_max) :
        self.H_max = cv2.getTrackbarPos('H_max', 'HSV_TrackBars')
    
    def S_min_cb(self, S_min) :
        self.S_min = cv2.getTrackbarPos('S_min', 'HSV_TrackBars')

    def S_max_cb(self, S_max) :
        self.S_max = cv2.getTrackbarPos('S_max', 'HSV_TrackBars')

    def V_min_cb(self, V_min) :
        self.V_min = cv2.getTrackbarPos('V_min', 'HSV_TrackBars')

    def V_max_cb(self, V_max) :
        self.V_max = cv2.getTrackbarPos('V_max', 'HSV_TrackBars')

if __name__ == "__main__" :
    rospy.init_node("color_filter", anonymous = True)
    filter()