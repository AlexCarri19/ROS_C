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

        ############    PUBLISHER   ####################### 
        self.semaforo_pub = rospy.Publisher("semaforo_flag", String , queue_size=1)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=1)

        ############    SUBSCRIBERS   ####################### 
        self.image_sub = rospy.Subscriber("/video_source/raw", Image, self.camera_cb)
        
        self.bridge = CvBridge()
        self.bridge_object1 = CvBridge()
        self.image_received = 0
        r = rospy.Rate(30)

        segmented_image = Image()

        #Verde 1
        self.V_H_min , self.V_H_max = 68 , 90
        self.V_S_min , self.V_S_max = 57 , 255
        self.V_V_min , self.V_V_max = 0 , 255
        V_hsv_min = np.array([self.V_H_min, self.V_S_min, self.V_V_min])
        V_hsv_max = np.array([self.V_H_max, self.V_S_max, self.V_V_max])

        #Verde 2
        self.V_H_min_0 , self.V_H_max_0 = 0 , 180
        self.V_S_min_0 , self.V_S_max_0 = 0 , 255
        self.V_V_min_0 , self.V_V_max_0 = 204 , 225
        V_hsv_min_0 = np.array([self.V_H_min_0, self.V_S_min_0, self.V_V_min_0])
        V_hsv_max_0 = np.array([self.V_H_max_0, self.V_S_max_0, self.V_V_max_0])

        #Amarillo 
        self.A_H_min , self.A_H_max = 0 , 180
        self.A_S_min , self.A_S_max = 0 , 255
        self.A_V_min , self.A_V_max = 204 , 255
        A_hsv_min = np.array([self.A_H_min, self.A_S_min, self.A_V_min])
        A_hsv_max = np.array([self.A_H_max, self.A_S_max, self.A_V_max])

        #Amarillo 2
        self.A_H_min_0 , self.A_H_max_0 = 0 , 180
        self.A_S_min_0 , self.A_S_max_0 = 0 , 255
        self.A_V_min_0 , self.A_V_max_0 = 192 , 255
        A_hsv_min_0 = np.array([self.A_H_min_0, self.A_S_min_0, self.A_V_min_0])
        A_hsv_max_0 = np.array([self.A_H_max_0, self.A_S_max_0, self.A_V_max_0])

        #Rojo
        self.R_H_min , self.R_H_max = 171 , 180
        self.R_S_min , self.R_S_max = 34 , 255
        self.R_V_min , self.R_V_max = 149 , 255
        R_hsv_min = np.array([self.R_H_min, self.R_S_min, self.R_V_min])
        R_hsv_max = np.array([self.R_H_max, self.R_S_max, self.R_V_max])

        #Rojo 2
        self.R_H_min_0 , self.R_H_max_0 = 0 , 180
        self.R_S_min_0 , self.R_S_max_0 = 0 , 255
        self.R_V_min_0 , self.R_V_max_0 = 213 , 155
        R_hsv_min_0 = np.array([self.R_H_min_0, self.R_S_min_0, self.R_V_min_0])
        R_hsv_max_0 = np.array([self.R_H_max_0, self.R_S_max_0, self.R_V_max_0])

        self.semaforo_flag = "Verde"

        self.min_area = 65

        #capture = cv2.VideoCapture(0)


        self.image_received = 0

        area_A , area_R , area_V = 0 , 0 ,0 

        while not rospy.is_shutdown():
            if self.image_received == 1:
                #print("Imagen recibida")
                image = cv2.resize(self.cv_image, (500, 500))
                #ret , image = capture.read()
                image = cv2.resize(image, (500, 500))

                #cv2.imshow("Camera", image)

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                #Verde
                V_hsv_mask = cv2.inRange(hsv, V_hsv_min, V_hsv_max)
                V_hsv_mask_0 = cv2.inRange(hsv, V_hsv_min_0, V_hsv_max_0)
                res_Vmask = cv2.add(V_hsv_mask , V_hsv_mask_0)
                res_verde = cv2.bitwise_and(image ,image , mask = V_hsv_mask)

                contoursV1_0 , _ = cv2.findContours (V_hsv_mask_0 , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
                contoursV1 , _ = cv2.findContours (V_hsv_mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)

                imgGrayV = cv2.cvtColor(res_verde , cv2.COLOR_BGR2GRAY)
                area_V , posXV , posYV = self.getContours(imgGrayV , image)

                #Amarillo 
                A_hsv_mask = cv2.inRange(hsv, A_hsv_min, A_hsv_max)
                A_hsv_mask_0 = cv2.inRange(hsv, A_hsv_min_0, A_hsv_max_0)
                #A_hsv_mask = cv2.add(A_hsv_mask , A_hsv_mask_0)
                res_amarillo = cv2.bitwise_and(image ,image , mask = A_hsv_mask)

                contoursA1 , _ = cv2.findContours (A_hsv_mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
                #contoursA1_0 , _ = cv2.findContours (A_hsv_mask_0 , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)

                imgGrayA = cv2.cvtColor(res_amarillo , cv2.COLOR_BGR2GRAY)
                area_A , posXA , posYA = self.getContours(imgGrayA , image)

                #Rojo
                R_hsv_mask = cv2.inRange(hsv, R_hsv_min, R_hsv_max)
                R_hsv_mask_0 = cv2.inRange(hsv, R_hsv_min_0, R_hsv_max_0)
                #R_hsv_mask = cv2.add(R_hsv_mask , R_hsv_mask_0)
                res_rojo = cv2.bitwise_and(image ,image , mask = R_hsv_mask)

                contoursR1 , _ = cv2.findContours (R_hsv_mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
                contoursR1_0 , _ = cv2.findContours (R_hsv_mask_0 , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)

                imgGrayR = cv2.cvtColor(res_rojo , cv2.COLOR_BGR2GRAY)
                area_R , posXR , posYR = self.getContours(imgGrayR , image)

                #Contornos Verde
                cv2.drawContours(image , contoursV1 , -1 , (0 , 255 , 0) , 2)
                cv2.drawContours(image , contoursV1_0 , -1 , (0 , 255 , 0) , 2)

                #Contornos Amarillo 
                cv2.drawContours(image , contoursA1 , -1 , (0 , 255 , 0) , 2)
                #cv2.drawContours(image , contoursA1_0 , -1 , (0 , 255 , 0) , 2)

                #Contornos Rojo
                cv2.drawContours(image , contoursR1 , -1 , (0 , 255 , 0) , 2)
                cv2.drawContours(image , contoursR1_0 , -1 , (0 , 255 , 0) , 2)

                #cv2.imshow("Original" , image)
                #cv2.imshow("Verde" , res_verde)
                #cv2.imshow("Amarilla" , res_amarillo)
                #cv2.imshow("Roja" , res_rojo)


                semaforo = [0 , 0 , 0]
                semaforoVal = [area_R , area_A , area_V]

                if area_V > 55: #Un bloque de distancia 
                    semaforo[0] = 1

                if area_A > 20: #Un bloque de distancia 
                    semaforo[1] = 1

                if area_R > 60: #Un bloque de distancia 
                    semaforo[2] = 1

                if semaforo[2] == 1: #Un bloque de distancia 
                    #semaforoVal[0] > semaforoVal[1]
                    semaforo[2] = 1
                    self.semaforo_flag = "Rojo"
                    print ("Rojo x: %i y: %i a: %i" % (posXR , posYR , area_R))
                    print(semaforo)

                elif semaforo[0] == 1:
                    self.semaforo_flag = "Verde"
                    print ("Verde x: %i y: %i a: %i" % (posXV , posYV , area_V))
                    print(semaforo)

                elif semaforo[1] == 1 and semaforo[2] == 0:
                    if not area_R:
                        semaforo[1] = 1
                        self.semaforo_flag = "Amarillo"
                        print ("Amarillo x: %i y: %i a: %i" % (posXA , posYA , area_A))
                        print(semaforo)
                
                else:
                    self.semaforo_flag = "Nada"
                
                semaforo = [0 , 0 , 0]
                semaforoVal = [0 , 0 , 0]

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
            #print(area)
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