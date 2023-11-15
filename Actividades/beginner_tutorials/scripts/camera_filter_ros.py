#!/usr/bin/env python
import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class filter() :
    def __init__(self) :
        rospy.on_shutdown(self.cleanup)

        ############    PUBLISHER   ####################### 
        self.image_pub = rospy.Publisher("segmented_image", Image)

        ############    SUBSCRIBERS   ####################### 
        self.camera_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_cb)
        
        self.bridge = CvBridge()
        self.image_received = 0
        r = rospy.Rate(50)

        segmented_image = Image()

        self.H_min = 0
        self.H_max = 26
        self.S_min = 151
        self.S_max = 255
        self.V_min = 0
        self.V_max = 225

        self.min_area = 4000

        # Creating a control TrackBar for the HSV values
        
        cv2.namedWindow('TrackBars')
        cv2.resizeWindow('TrackBars' , 500 , 500)

        cv2.createTrackbar('H_min' , 'TrackBars' , 0 , 180 , self.H_min_cb)
        cv2.createTrackbar('H_max' , 'TrackBars' , 0 , 180 , self.H_max_cb)
        cv2.createTrackbar('S_min' , 'TrackBars' , 0 , 255 , self.S_min_cb)
        cv2.createTrackbar('S_max' , 'TrackBars' , 0 , 255 , self.S_max_cb)
        cv2.createTrackbar('V_min' , 'TrackBars' , 0 , 255 , self.V_min_cb)
        cv2.createTrackbar('V_max' , 'TrackBars' , 0 , 255 , self.V_max_cb)

        cv2.setTrackbarPos('H_min' , 'TrackBars' , 0)
        cv2.setTrackbarPos('H_max' , 'TrackBars' , 180)
        cv2.setTrackbarPos('S_min' , 'TrackBars' , 0)
        cv2.setTrackbarPos('S_max' , 'TrackBars' , 255)
        cv2.setTrackbarPos('V_min' , 'TrackBars' , 0)
        cv2.setTrackbarPos('V_max' , 'TrackBars' , 255)

        #### Azul: #### Rojo: ###
        #H_min = 108 #H_min = 0
        #H_max = 155 #H_max = 0
        #S_min = 78  #S_min = 90
        #S_max = 228 #S_max = 255
        #V_min = 59  #V_min = 0
        #V_max = 255 #V_max = 255

        while not rospy.is_shutdown() :
            if self.image_received:
                image = cv2.resize(self.cv_image, (500, 500))
                cv2.imshow("Camera", image)

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                self.H_min = cv2.getTrackbarPos('H_min' , 'TrackBars')
                self.H_max = cv2.getTrackbarPos('H_max' , 'TrackBars')
                self.S_min = cv2.getTrackbarPos('S_min' , 'TrackBars')
                self.S_max = cv2.getTrackbarPos('S_max' , 'TrackBars')
                self.V_min = cv2.getTrackbarPos('V_min' , 'TrackBars')
                self.V_max = cv2.getTrackbarPos('V_max' , 'TrackBars')
                
                hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                hsv_max = np.array([self.H_max, self.S_max, self.V_max])

                hsv_mask = cv2.inRange(hsv, hsv_min, hsv_max)
                hsv_mask = cv2.erode(hsv_mask , None , iterations = 3)
                hsv_mask = cv2.dilate(hsv_mask , None , iterations = 3)
                res = cv2.bitwise_and(image, image, mask = hsv_mask)

                imgBlur = cv2.GaussianBlur(res , (7 , 7) , 1)
                imgGray = cv2.cvtColor(imgBlur , cv2.COLOR_BGR2GRAY)
                imCanny = cv2.Canny(imgGray , 166 , 171)

                cv2.imshow("Filter", imCanny)

                #self.getContours(imgGray , image)

                segmented_image = self.bridge.cv2_to_imgmsg(res, encoding = "passthrough")
                self.image_pub.publish(segmented_image)

            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()

    def camera_cb(self, image_data) :
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding = "bgr8")
        except CvBridgeError as e :
            print(e)
        self.image_received = 1

    def getContours(self , img ,img_tracking):
        contours , hierarchy , _= cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)[0]
            print(area)
            if area > self.min_area:
                per = cv2.arcLength(cnt , True)
                aprox = cv2.approxPolyDP(cnt , 0.02* per , True)
                x , y , w ,h = cv2.boundingRect(aprox)
                cx = int(x + w/2)
                cy = int(y + h/2)

                #mostar informacion 
                cv2.drawContours(img_tracking , cnt , -1 , (255 , 0 , 255))
                cv2.putText(img_tracking, 'cx', (20, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, str(cx), (80, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, 'cy', (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                cv2.putText(img_tracking, str(cy), (80, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
                
                #Control de movimiento
            else: 
                pass
                #Trazar una linea media
        cv2.line(img_tracking,(250,0),(250,500),(255,255,0),3)
        cv2.line(img_tracking,(0,250),(500,250),(255,255,0),3)

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

    def cleanup(self):     
        zero_0 = Twist()
        print("I'm dying, bye bye!!!")
        #self.cmd_vel_pub.publish(zero_0)

if __name__ == "__main__" :
    rospy.init_node("color_filter", anonymous = True)
    filter()