#!/usr/bin/env python
import cv2 
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

class filter() :
    def __init__(self) :
        rospy.on_shutdown(self.cleanup)

        print ("Uno")

        ############    PUBLISHER   ####################### 
        self.semaforo_pub = rospy.Publisher("semafaoro_flag", String , queue_size=1)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=1)

        ############    SUBSCRIBERS   ####################### 
        self.image_sub = rospy.Subscriber("/video_source/raw", Image, self.camera_cb)
        
        self.bridge = CvBridge()
        self.bridge_object1 = CvBridge()
        self.image_received = 0
        r = rospy.Rate(50)

        segmented_image = Image()

        print ("Dos")

        cv2.namedWindow('TrackBars')
        cv2.resizeWindow('TrackBars' , 500 , 500)

        def trackbar_cb(): pass

        H_min , H_max = 0 , 255
        S_min , S_max = 0 , 255
        V_min , V_max = 0 , 255

        init_Hmin , init_Hmax = 68 , 90
        init_Smin , init_Smax = 57 , 255
        init_Vmin , init_Vmax = 0 , 255
        area_min = 50

        
        cv2.createTrackbar('Speed' , 'TrackBars' , -100 , 100 , trackbar_cb) #Crear trackbar
        cv2.createTrackbar('H_min' , 'TrackBars' , 0 , 180 , trackbar_cb)
        cv2.createTrackbar('H_max' , 'TrackBars' , 0 , 180 , trackbar_cb)
        cv2.createTrackbar('S_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
        cv2.createTrackbar('S_max' , 'TrackBars' , 0 , 255 , trackbar_cb)
        cv2.createTrackbar('V_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
        cv2.createTrackbar('V_max' , 'TrackBars' , 0 , 255 , trackbar_cb)

        cv2.setTrackbarPos('H_min' , 'TrackBars' , init_Hmin)
        cv2.setTrackbarPos('H_max' , 'TrackBars' , init_Hmax)
        cv2.setTrackbarPos('S_min' , 'TrackBars' , init_Smin)
        cv2.setTrackbarPos('S_max' , 'TrackBars' , init_Smax)
        cv2.setTrackbarPos('V_min' , 'TrackBars' , init_Vmin)
        cv2.setTrackbarPos('V_max' , 'TrackBars' , init_Vmax)
        self.semaforo_flag = "Verde"

        self.min_area = 4000

        print ("Tres")

        #capture = cv2.VideoCapture(0)

        print ("Cuatro")

        self.image_received = 0

        area_A , area_R , area_V = 0 , 0 ,0 

        while not rospy.is_shutdown():
            if self.image_received == 1:
                print("Imagen recibida")
                image = cv2.resize(self.cv_image, (500, 500))
                #ret , image = capture.read()
                image = cv2.resize(image, (500, 500))

                #cv2.imshow("Camera", image)

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                H_min = cv2.getTrackbarPos('H_min' , 'TrackBars')
                H_max = cv2.getTrackbarPos('H_max' , 'TrackBars')
                S_min = cv2.getTrackbarPos('S_min' , 'TrackBars')
                S_max = cv2.getTrackbarPos('S_max' , 'TrackBars')
                V_min = cv2.getTrackbarPos('V_min' , 'TrackBars')
                V_max = cv2.getTrackbarPos('V_max' , 'TrackBars')
                
                hsv_min = np.array([H_min , S_min , V_min])
                hsv_max = np.array([H_max , S_max , V_max])

                mask = cv2.inRange(hsv , hsv_min , hsv_max)
                contours , _ = cv2.findContours (mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(image , contours , -1 , (0 , 255 , 0) , 2)

                imgGray = cv2.cvtColor(mask , cv2.COLOR_BGR2GRAY)
                area , xp , yp =self.getContours (imgGray , image)

                #cv2.imshow("Frame" , image)
                #cv2.imshow("Mask" , mask)

                #cv2.imshow("Original" , image)
                #cv2.imshow("Verde" , res_verde)
                #cv2.imshow("Amarilla" , res_amarillo)
                #cv2.imshow("Roja" , res_rojo)

                if area > 86:
                    #if posXV > 20 
                    self.semaforo_flag = "ON"
                    print ("Color")

                image = self.bridge_object1.cv2_to_imgmsg(image, encoding="bgr8") 

                self.semaforo_pub.publish(self.semaforo_flag)
                self.image_pub.publish(image)

            cv2.waitKey(1)
            r.sleep()
        cv2.destroyAllWindows()

    def camera_cb(self, image_data) :
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding = "bgr8")
            
        except CvBridgeError as e :
            print(e)
        self.image_received=1
        

    def getContours(self , img ,img_tracking):
        cx = 0
        cy = 0
        area = 0
        contours , hierarchy = cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
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
        return area , cx , cy

    def cleanup(self):     
        print("I'm dying, bye bye!!!")
        #self.cmd_vel_pub.publish(zero_0)

if __name__ == "__main__" :
    rospy.init_node("color_filter", anonymous = True)
    filter()