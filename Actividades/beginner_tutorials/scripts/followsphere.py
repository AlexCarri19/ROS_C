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
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 

        ############    SUBSCRIBERS   ####################### 
        self.camera_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_cb)
        
        self.bridge = CvBridge()
        self.image_received = 0
        r = rospy.Rate(50)

        segmented_image = Image()
        self.area_min = 4000

        self.H_min = 0
        self.H_max = 26
        self.S_min = 151
        self.S_max = 255
        self.V_min = 0
        self.V_max = 225
        

        while not rospy.is_shutdown() :
            if self.image_received:
                img = cv2.resize(self.cv_image, (500, 500))
                cv2.imshow("Camera", img)

                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                hsv_min = np.array([self.H_min, self.S_min, self.V_min])
                hsv_max = np.array([self.H_max, self.S_max, self.V_max])

                mask = cv2.inRange(hsv , hsv_min , hsv_max)
                mask = cv2.erode(mask , None , iterations = 3)
                mask = cv2.dilate(mask , None , iterations = 3)
                res = cv2.bitwise_and(img,img,mask = mask)

                #Contornos y la posicion del objeto 
                imgBlur = cv2.GaussianBlur(res , (7 , 7) , 1)
                imgGray = cv2.cvtColor(imgBlur , cv2.COLOR_BGR2GRAY)
                imCanny = cv2.Canny(imgGray , 166 , 171)

                cv2.imshow("Image" , imCanny)
                self.getContours(imgGray , img)

                segmented_image = self.bridge.cv2_to_imgmsg(res, encoding = "passthrough")
                self.image_pub.publish(segmented_image)

            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()

    def getContours(img ,img_tracking):
        area_min = 4000
        contours , hierarchy = cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            print(area)
            if area > area_min:
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
                print("Pos X: %i Pos Y: %i" % (cx ,cy))
            else: 
                direccion = 0

                #Trazar una linea media
            cv2.line(img_tracking,(250,0),(250,500),(255,255,0),3)
            cv2.line(img_tracking,(0,250),(500,250),(255,255,0),3)
            return cx

    def camera_cb(self, image_data) :
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding = "bgr8")
        except CvBridgeError as e :
            print(e)
        self.image_received = 1

    def cleanup(self):     
        zero_0 = Twist()
        print("I'm dying, bye bye!!!")
        self.cmd_vel_pub.publish(zero_0)

if __name__ == "__main__" :
    rospy.init_node("color_filter", anonymous = True)
    filter()