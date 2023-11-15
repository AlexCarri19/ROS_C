#!/usr/bin/env python  

import rospy
import numpy as np
import cv2 
import time
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float32 
from std_msgs.msg import Int32 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 

class Controller():  

    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node.
        self.image_sub = rospy.Subscriber('/video_source/raw',Image,self.camera_callback)

        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=1)

        ############ ROBOT CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        ### Inicializo variables ###
        thetar = 0.0 #[rad]
        xr = 0.0 # [m] posicion del robot a lo largo del eje x
        yr = 0.0 # [m] posicion del robot a lo largo del eje y
        v= 0.0 #velocidad lineal del robot [m/s]
        w = 0.0 #velocidad angular del robot [rad/s]
        
	    #Wheels speed	
        self.wl = 0.0 #vel angular rueda izquierda [rad/s]
        self.wr = 0.0 #vel angular rueda derecha [rad/s]
        
	    #Distances	
        d = 100000.0 
        d_min = 0.05 #[m] d_min to the goal	
        e_theta_min = np.pi/90

        #V max y min

        w_max = 0.5
        w_min = 0.05
        v_max = 0.4
        v_min = 0.05
        
	    #Flag to request data	
        self.flag = 1

        #Flag to indicate that we have already received an image
        self.image_received = 0 
        
        #Objective points
        self.xG = 0.0
        self.yG = 0.0
        #error de la velocidad lineal
        e0 = 0.0
        e1 = 0.0
        e2 = 0.0

        #error de la velocidad angular
        e3 = 0.0
        e4 = 0.0
        e5 = 0.0

        #accion de control para velocidad angular
        u0 = 0.0
        u1 = 0.0

        #accion de control para velocidad lineal
        u2 = 0.0
        u3 = 0.0

        #ganancias para control de velocidad angular
        kp = 5.6
        ki = 0.004
        kd = 0.0

        #ganancias para control de velocidad lineal
        kp1 = 4.2
        ki1 = 0.0005
        kd1 = 0.0

        #tiempo para control de velocidad angular
        Ts = rospy.get_param("/time",0.1)

        #tiempo para control de velocidad lineal
        Ts1 = rospy.get_param("/time",0.1)

        #Controlador de velocidad angular
        K1 = kp + Ts*ki + kd/Ts
        K2 = -kp - 2.0*kd/Ts
        K3 = kd/Ts

        #Controlador de velocidad lineal
        K4 = kp1 + Ts1*ki1 + kd1/Ts1
        K5 = -kp1 - 2.0*kd1/Ts1
        K6 = kd1/Ts1

        
        #Se crean las banderas
        f_amarillo = 0
        f_verde = 0
        f_rojo = 0
        f_nada = 0
        


        ######### PUBLISHERS AND SUBSCRIBERS ################# 
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.flag_pub = rospy.Publisher('flag', Int32, queue_size=1) 
        rospy.Subscriber('wl', Float32, self.wl_cb)
        rospy.Subscriber('wr', Float32, self.wr_cb)
        rospy.Subscriber('pose', Pose, self.pose_cb)
        

        #Se crean los objetos
        v_msg=Twist()
        self.bridge_object = CvBridge()
        

        
        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

        previous_time = rospy.get_time() #obtener dt
        rate = rospy.Rate(40) #20Hz  
        print("Node initialized") 

        while not rospy.is_shutdown():

            if d >= d_min and self.flag == 0:
                delta_t = rospy.get_time()-previous_time
                previous_time = rospy.get_time() #lo actualizo luego de usarlo
                v= r*(self.wl+self.wr)/2.0
                w = r*(self.wr-self.wl)/L
                thetar = (thetar + (w * delta_t))
                thetar = np.arctan2(np.sin(thetar), np.cos(thetar)) #establecer los valores de teta entre -pi y pi
                xr = xr + v*np.cos(thetar)*delta_t
                yr = yr + v*np.sin(thetar)*delta_t

                #distancia a la meta
                d = np.sqrt((self.xG-xr)**2+(self.yG-yr)**2)
                #print("xG: " + str(self.xG))
                #print("yG: " + str(self.yG))
                e3 = d

                #Angulo para alcanzar el objetivo
                theta_g = np.arctan2(self.yG-yr,self.xG-xr)

                #crop e_theta from -pi to pi
                theta_g = np.arctan2(np.sin(theta_g),np.cos(theta_g))

                #error en el angulo
                e_theta = theta_g - thetar

                e_theta = np.arctan2(np.sin(e_theta),np.cos(e_theta))

                #error en teta y error en distancia eso es lo que va a controlar
                #publicaself.vy w
                #mi referencia es teta goal
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

                if abs(e_theta) > e_theta_min:
                    w = abs(m_input) * e_theta 
                    v= 0
                else:
                    v= abs(m_input1) * d
                    w = 0          
                
                if w > w_max:
                    w = w_max
                elif w < -w_max:
                    w = -w_max
                elif w < w_min and w > 0:
                    w = w_min
                elif w > -w_min and w < 0:
                    w = -w_min

                if v> v_max:
                   self.v= v_max
                elif v< -v_max:
                   self.v= -v_max
                elif v< v_min and v> 0:
                   self.v= v_min
                elif v> -v_min and v< 0:
                   self.v= -v_min
                
                
                
                #v_msg.linear.x = v
                #v_msg.angular.z = w

                

                if self.image_received == 1:
                    self.cv_image = cv2.resize(self.cv_image_received,(300,300))
                    #cv2.imshow('imagen sin filtro',self.cv_image)
                    hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
                    print("Recibi la imagen")
                    # each value of the vector corresponds to the H,S &self.vvalues respectively
                    #se crean los limites HSV
                    #Valores para la simulacion
                    #min_green = np.array([45,220,30])
                    #max_green = np.array([65,255,255])
                    #min_red = np.array([0,220,30]) 
                    #max_red = np.array([15,255,255]) 
                    #min_yellow = np.array([20,220,30]) 
                    #max_yellow = np.array([30,255,255])
                    #Valores para puzzlebot
                    min_green = np.array([45,220,0])
                    max_green = np.array([65,255,255])
                    min_red = np.array([0,100,0]) 
                    max_red = np.array([15,255,255]) 
                    min_yellow = np.array([20,200,0]) 
                    max_yellow = np.array([45,255,255])
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
                    seg_img = self.bridge_object.cv2_to_imgmsg(res_red, encoding = "passthrough")
                    self.image_pub.publish(seg_img)
                    seg_img = self.bridge_object.cv2_to_imgmsg(res_green, encoding = "passthrough")
                    self.image_pub.publish(seg_img)
                    seg_img = self.bridge_object.cv2_to_imgmsg(res_yellow, encoding = "passthrough")
                    self.image_pub.publish(seg_img)

                    if (cv2.countNonZero(cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)))>500:
                        f_amarillo = 0
                        f_rojo = 0
                        f_verde = 1

                        

                    elif cv2.countNonZero(cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY))>500:
                        f_amarillo = 1
                        f_rojo = 0
                        f_verde = 0
                        
                         

                    elif cv2.countNonZero(cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY))>500:
                        f_amarillo = 0
                        f_rojo = 1
                        f_verde = 0
                        
                    
                    else:
                        f_nada = 1
                        

                    if f_verde == 1:
                        print("Estoy en verde")
                        v_msg.linear.x = v
                        v_msg.angular.z = w
                        print("V = " + str(v))
                        print("w = " + str(w))
                        print(" ")
                        
                    elif f_amarillo == 1:
                        print("Estoy en amarillo")
                        v_msg.linear.x = v/2.0
                        v_msg.angular.z = w/2.0
                        #print("V = " + str(v_msg.linear.x))
                        #print("w = " + str(v_msg.angular.z))
                        #print(" ")
                        
                    elif f_rojo == 1:
                        print("Estoy en rojo")
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.0
                        print("V = " + str(v_msg.linear.x))
                        print("w = " + str(v_msg.angular.z))
                    
                    elif f_nada == 1 and f_rojo == 1:
                        print("No debo avanzar ya que me detuve con rojo")
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.0
                        print("V = " + str(v_msg.linear.x))
                        print("w = " + str(v_msg.angular.z))
                    
                        

                    
                    self.pub_cmd_vel.publish(v_msg)
                    

                    
                
                cv2.waitKey(1)


            else:
                print("stop")
                self.flag = 1
                self.flag_pub.publish(self.flag)
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                d = 1000000.0
                
            
            self.pub_cmd_vel.publish(v_msg)
            print(v_msg) 
            rate.sleep()

    def pose_cb(self, msg): 

        self.xG = msg.position.x 
        self.yG = msg.position.y  
        self.flag = 0
        self.flag_pub.publish(self.flag)
        print("callback pose") 

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data
      
    def camera_callback(self,data): 
        try: 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image_received = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        except CvBridgeError as e: 
            print(e) 

        self.image_received=1
        print("Entre al callback de la imagen")
 

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
