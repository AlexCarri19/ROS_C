#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        #self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber("/video_source/raw", Image, self.camera_callback)
        self.image_pub = rospy.Publisher('Semaforo', Image, queue_size=1)
        self.bridge_object1 = CvBridge()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.flag_pub = rospy.Publisher('flag', Int32, queue_size=1)
        rospy.Subscriber('wl', Float32, self.wl_cb)
        rospy.Subscriber('wr', Float32, self.wr_cb)
        self.rad_sub = rospy.Subscriber("radius", Int32, self.radius_cb)
        self.center_sub = rospy.Subscriber("center", Point, self.center_cb)


        # ROBOT CONSTANTS
        r = 0.05  # wheel radius [m]
        L = 0.19  # wheel separation [m]

        # Inicializo variables
        self.v= 0.0  # velocidad lineal del robot [m/s]
        w = 0.0  # velocidad angular del robot [rad/s]
        self.wl = 0.0  # vel angular rueda izquierda [rad/s]
        self.wr = 0.0  # vel angular rueda derecha [rad/s]
        e_theta_min = 20
        w_max = 0.5
        w_min = 0.05
        v_max = 0.2
        v_min = 0.05
        self.flag = 1
        self.image_received = 0
        self.xG = 0.0
        self.yG = 0.0
        e0 = 0.0
        e1 = 0.0
        e2 = 0.0
        e3 = 0.0
        e4 = 0.0
        e5 = 0.0
        u0 = 0.0
        u1 = 0.0
        u2 = 0.0
        u3 = 0.0
        
        #FUNCIONAN 
        #kp = 0.004
        #ki = 0.00005
        #kd = 0.0009    
        kp = 0.0045
        ki = 0.00006
        kd = 0.00012


        kp1 = 0.0
        ki1 = 0.0
        kd1 = 0.0
        Ts = rospy.get_param("/time", 0.1)
        Ts1 = rospy.get_param("/time", 0.1)
        K1 = kp + Ts * ki + kd / Ts
        K2 = -kp - 2.0 * kd / Ts
        K3 = kd / Ts
        K4 = kp1 + Ts1 * ki1 + kd1 / Ts1
        K5 = -kp1 - 2.0 * kd1 / Ts1
        K6 = kd1 / Ts1
        f_amarillo = 0
        f_verde = 0
        f_rojo = 0
        f_nada = 0
        self.radius = 0
        self.xc = 0.0

        v_msg = Twist()

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

        previous_time = rospy.get_time() #obtener dt

        rate = rospy.Rate(60) #20Hz  
        print("Node initialized") 

        while not rospy.is_shutdown():

            if self.radius:
                #print("Recibi el radio")
                delta_t = rospy.get_time()-previous_time
                previous_time = rospy.get_time() #lo actualizo luego de usarlo
                self.v= r*(self.wl+self.wr)/2.0
                w = r*(self.wr-self.wl)/L
                #distancia a la meta
                #try:
                #d = (142 - self.yc)
                #except:
                #    d = 0
                #if d != 0:
                #e3 = d
                #else:
                #    e3 = 0.001

                #error en el angulo
                e_theta = 200 - self.xc

                #e_theta = np.arctan2(np.sin(e_theta),np.cos(e_theta))

                e0 = e_theta
                
                u0 = K1*e0 + K2*e1 + K3*e2 + u1

                u2 = K4*e3 + K5*e4 + K6*e5 + u3
                
                if (u0 > 1):
                    u0 = 1
                elif (u0 < -1):
                    u0 = -1

                if (u2 > 1):
                    u2 = 1
                elif (u2 < -1):
                    u2 = -1

                #error para el controlador de velocidad angular (omega)  
                e2 = e1
                e1 = e0
                #accion de control para velocidad angular
                u1 = u0
                #error para el controlador de velocidad lineal  
                e5 = e4
                e4 = e3
                #accion de control para velocidad lineal
                u3 = u2
                
                m_input = u0 
                m_input1 = u2

                #print("m_input : " + str(m_input))
                #print("m_input1 : " + str(m_input1))

                if e_theta > e_theta_min:
                    #print("SE salio a la derecha")
                    w = m_input * e_theta * 0.01
                    self.v= 0.075
                elif e_theta > - e_theta_min and e_theta < e_theta_min:
                    #print("DEntro de los limites")
                    self.v= 0.15
                    w = 0.0
                elif e_theta < -e_theta_min:
                    #print("Se salio a la izquierda")
                    w = -m_input * e_theta *0.01
                    self.v= 0.075

                if w > w_max:
                    w = w_max
                elif w < -w_max:
                    w = -w_max
                elif w < w_min and w > 0:
                    w = w_min
                elif w > -w_min and w < 0:
                    w = -w_min

                if self.v> v_max:
                   self.v= v_max
                elif self.v< -v_max:
                   self.v= -v_max
                elif self.v< v_min and self.v> 0:
                   self.v= v_min
                elif self.v> -v_min and self.v< 0:
                   self.v= -v_min
                
                #v_msg.linear.x = v
                #v_msg.angular.z = w

                #print("Vel lineal: " + str(v_msg.linear.x))
                #print("Vel angular: " + str(v_msg.angular.z))

                if self.image_received == 1:
                    self.cv_image = self.cv_image_received.copy()
                    #offset = 150
                    #Se seleccionan las columnas del centro del eje horizontal
                    # Se resta offset al valor del centro para obtener el indice de inicio del rango
                    # Se suma offset al valor del centro para obtener el indice de fin del rango
                    #self.cv_image = self.cv_image[700:800, 200:300]
                    self.cv_image = cv2.resize(self.cv_image,(300,300), interpolation=cv2.INTER_AREA)                  
                    hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
                    

                    #print("Recibi la imagen")

                    #Valores para puzzlebot
                    min_green = np.array([30,120,0])
                    max_green = np.array([103,255,255])
                    min_red = np.array([148,79,86]) 
                    max_red = np.array([180,163,255]) 
                    min_yellow = np.array([0,51,100]) 
                    max_yellow = np.array([49,152,160])
                    #This mask has only one dimension, so its black and white
                    # Verde 
                    mask_green = cv2.inRange(hsv, min_green, max_green)
                    mask_green = cv2.erode(mask_green,None,iterations=3)
                    mask_green = cv2.dilate(mask_green,None,iterations=3)
                    #Rojo
                    mask_red = cv2.inRange(hsv, min_red, max_red)
                    mask_red = cv2.erode(mask_red,None,iterations=3)
                    mask_red = cv2.dilate(mask_red,None,iterations=3)
                    #Amarillo
                    mask_yellow = cv2.inRange(hsv, min_yellow, max_yellow)
                    mask_yellow = cv2.erode(mask_yellow,None,iterations=3)
                    mask_yellow = cv2.dilate(mask_yellow,None,iterations=3)

                    #We use the mask with the original image to get the colored post-processed image
                    res_green = cv2.bitwise_and(self.cv_image,self.cv_image, mask = mask_green)
                    res_red = cv2.bitwise_and(self.cv_image,self.cv_image, mask = mask_red)
                    res_yellow = cv2.bitwise_and(self.cv_image,self.cv_image, mask = mask_yellow)

                    #cv2.imshow('Imagen filtrada: Verde', res_green)
                    #cv2.imshow('Imagen filtrada: Rojo', res_red)
                    #cv2.imshow('Imagen filtrada: Amarillo', res_yellow)
                    seg_img = self.bridge_object1.cv2_to_imgmsg(res_red, encoding = "bgr8")
                    self.image_pub.publish(seg_img)
                    seg_img = self.bridge_object1.cv2_to_imgmsg(res_green, encoding = "bgr8")
                    self.image_pub.publish(seg_img)
                    seg_img = self.bridge_object1.cv2_to_imgmsg(res_yellow, encoding = "bgr8")
                    self.image_pub.publish(seg_img)

                    if (cv2.countNonZero(cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)))>1000:
                        f_amarillo = 0
                        f_rojo = 0
                        f_verde = 1

                        
                    elif cv2.countNonZero(cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY))>10000:
                        f_amarillo = 1
                        f_rojo = 0
                        f_verde = 0
                        
                         
                    elif cv2.countNonZero(cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY))>2000:
                        f_amarillo = 0
                        f_rojo = 1
                        f_verde = 0
                        
                    else:
                        f_nada = 1
                        
                    if f_verde == 1:
                        print("Estoy en verde")
                        v_msg.linear.x = self.v
                        v_msg.angular.z = w
                        #print("V = " + str(v))
                        #print("w = " + str(w))
                        #print(" ")
                        
                    elif f_amarillo == 1:
                        print("Estoy en amarillo")
                        v_msg.linear.x = self.v/2.0
                        v_msg.angular.z = w/2.0
                        #print("V = " + str(v_msg.linear.x))
                        #print("w = " + str(v_msg.angular.z))
                        #print(" ")
                        
                    elif f_rojo == 1:
                        print("Estoy en rojo")
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.0
                        #print("V = " + str(v_msg.linear.x))
                        #print("w = " + str(v_msg.angular.z))
                    
                    elif f_nada == 1 and f_rojo == 1:
                        print("No debo avanzar ya que me detuve con rojo")
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.0
                        #print("V = " + str(v_msg.linear.x))
                        #print("w = " + str(v_msg.angular.z))
                
                cv2.waitKey(1)

            else:
                print("stop")
                self.flag = 1
                self.flag_pub.publish(self.flag)
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                d = 1000000.0
                
            self.pub_cmd_vel.publish(v_msg)
            rate.sleep()

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data
      
    def camera_callback(self,data): 
        try: 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image_received = self.bridge_object1.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        except CvBridgeError as e: 
            print(e) 

        self.image_received=1
        #print("Entre al callback de la imagen")
    
    def radius_cb(self, msg):  
        #Callback del radio 
        self.radius =  msg.data  
        #print("I received this message in the callback: " + str(self.radius))
    
    def center_cb(self, msg):  
        #Callback de la distancia
        self.xc = msg.x
        self.yc = msg.y
        #print("I received this distance in the callback: " + str(self.xc))

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        v_msg = Twist() 
        self.pub_cmd_vel.publish(v_msg)
        f_amarillo = 0
        f_verde = 0
        f_rojo = 0
        

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("controller", anonymous=True)  
    Controller()
