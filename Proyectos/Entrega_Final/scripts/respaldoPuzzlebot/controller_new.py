#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        #self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber("/video_source/raw", Image, self.camera_callback)
        rospy.Subscriber("semaforo_flag" , String , self.semaforo_cb)
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
        self.v = 0.0  # velocidad lineal del robot [m/s]
        self.w = 0.0  # velocidad angular del robot [rad/s]
        self.wl = 0.0  # vel angular rueda izquierda [rad/s]
        self.wr = 0.0  # vel angular rueda derecha [rad/s]
        e_pixel_min = 20
        w_max = 0.5 #0.4
        w_min = 0.05 #0.05
        v_max = 0.2
        v_min = 0.05
        self.flag = 1
        #self.image_received = 0 -->lo usamos? creo que no
        self.xG = 0.0
        self.yG = 0.0
        e0 = 0.0
        e1 = 0.0
        e2 = 0.0
        u0 = 0.0
        u1 = 0.0
        
        #FUNCIONAN 
        #kp = 0.000045
        #ki = 0.0000007
        #kd = 0.000035

        kp = 0.000045
        ki = 0.0000009
        kd = 0.000035
        

        Ts = 0.2
        K1 = kp + Ts * ki + kd / Ts
        K2 = -kp - 2.0 * kd / Ts
        K3 = kd / Ts
        self.radius = 0
        self.xc = 0.0

        self.v_msg = Twist()

        self.semaforo = "Verde"
        self.estado = "Move" 

        #Estado move permanece en seguidor de linea y algunas senales
        #Estado cruce tiene las instrucciones turn y forward con relacion
        #al semaforo 

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 
        previous_time = rospy.get_time() #obtener dt

        rate = rospy.Rate(60) #20Hz  
        print("Node initialized") 

        while not rospy.is_shutdown():
            
            test = rospy.get_time() - previous_time
            test = not self.radius and test > 0.2

            if self.radius:
                #print("Recibi el radio")
                #delta_t = rospy.get_time()-previous_time
                previous_time = rospy.get_time() #lo actualizo luego de usarlo
                self.v = r*(self.wl+self.wr)/2.0
                self.w = r*(self.wr-self.wl)/L
                self.estado = "Move"

                #error en el angulo
                e_pixel = 200 - self.xc

                #e_pixel = np.arctan2(np.sin(e_pixel),np.cos(e_pixel))

                e0 = e_pixel
                
                u0 = K1*e0 + K2*e1 + K3*e2 + u1
                
                if (u0 > 1):
                    u0 = 1
                elif (u0 < -1):
                    u0 = -1

                #error para el controlador de velocidad angular (omega)  
                e2 = e1
                e1 = e0
                #accion de control para velocidad angular
                u1 = u0
                m_input = u0

                if e_pixel > e_pixel_min:
                    #se sale a la derecha
                    self.w = m_input * e_pixel
                    self.v = 0.10
                elif e_pixel > - e_pixel_min and e_pixel < e_pixel_min:
                    #esta en los limites
                    self.v = 0.15
                    self.w = 0.0
                elif e_pixel < -e_pixel_min:
                    #se sale a la izquierda
                    self.w = -m_input * e_pixel
                    self.v = 0.10

                if self.w > w_max:
                    self.w = w_max
                elif self.w < -w_max:
                    self.w = -w_max
                elif self.w < w_min and self.w > 0:
                    self.w = w_min
                elif self.w > -w_min and self.w < 0:
                    self.w = -w_min

                if self.v > v_max:
                    self.v = v_max
                elif self.v < -v_max:
                    self.v = -v_max
                elif self.v < v_min and self.v > 0:
                    self.v = v_min
                elif self.v > -v_min and self.v < 0:
                    self.v = -v_min
                
                if self.semaforo and self.estado == "Move":
                    self.semaforoAct()
                    
                        
                else:
                    print("Fuera de scope 1")

                print("V = " + str(self.v_msg.linear.x))
                print("w = " + str(self.v_msg.angular.z))
                cv2.waitKey(1)

            else:
                self.estado = "Cruce"
                if test:
                    print("girando")
                    self.giros()
                
                
                
            self.pub_cmd_vel.publish(self.v_msg)
            rate.sleep()

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data
      
    #NECESITAMOS ESTE CALLBACK O YA NO? CREO QUE NO
    #def camera_callback(self,data): 
        #try: 
            # We select bgr8 because its the OpenCV encoding by default 
            #self.cv_image_received = self.bridge_object1.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        #except CvBridgeError as e: 
            #print(e) 

        #self.image_received=1
        #print("Entre al callback de la imagen")
    
    def giros(self):
        t = rospy.get_time()
        t = rospy.get_time() - t
        bandera_giro = 0
        if self.semaforo == "Verde" or self.semaforo == "Amarillo" or self.semaforo == "Nada" and bandera_giro == 0:
            self.center("l")
            bandera_giro = bandera_giro + 1
            self.estado = "Move"
        elif self.semaforo == "Verde" or self.semaforo == "Amarillo" or self.semaforo == "Nada" and bandera_giro == 1:
            self.center("r")
            bandera_giro = bandera_giro + 1
            self.estado = "Move"

        elif self.semaforo == "Verde" or self.semaforo == "Amarillo" or self.semaforo == "Nada" and bandera_giro == 2:     
            self.center("f")
            bandera_giro = 0
            self.estado = "Move"

        elif self.semaforo == "Rojo":
            print("Estoy en rojo")
            self.v_msg.linear.x = 0.0
            self.v_msg.angular.z = 0.0
            print ("Sin centro por %f" % t)
    
    def radius_cb(self, msg):  
        #Callback del radio 
        self.radius =  msg.data  
        #print("I received this message in the callback: " + str(self.radius))
    
    def center_cb(self, msg):  
        #Callback de la distancia
        self.xc = msg.x
        self.yc = msg.y
        #print("I received this distance in the callback: " + str(self.xc))
    
    def semaforo_cb(self, s):  
        self.semaforo =  s.data

    def semaforoAct(self):
        if self.semaforo == "Verde":
            print("Estoy en verde")
            self.v_msg.linear.x = self.v
            self.v_msg.angular.z = self.w
                        
        elif self.semaforo == "Amarillo":
            print("Estoy en amarillo")
            self.v_msg.linear.x = self.v/2.0
            self.v_msg.angular.z = self.w/2.0
                        
        elif self.semaforo == "Rojo":
            print("Estoy en rojo")
            self.v_msg.linear.x = 0.0
            self.v_msg.angular.z = 0.0

        elif self.semaforo == "Nada":
            print("No detecto ningun semaforo")
            self.v_msg.linear.x = self.v
            self.v_msg.angular.z = self.w

    def center(self , turn):
        self.v_msg.linear.x = 0.1
        self.v_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(2.5)
        self.v_msg.linear.x = 0.0
        #self.v_msg.angular.z = np.pi/2*4
        self.v_msg.angular.z = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(1)
        if turn == "r": self.turn_right()
        elif turn == "l": self.turn_left()
        elif turn == "f": self.forward()

    def turn_left(self):
        self.v_msg.linear.x = 0.0
        self.v_msg.angular.z = -np.pi/2
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(1)
        self.v_msg.linear.x = 0.1
        self.v_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(.5)

    def turn_right(self):
        self.v_msg.linear.x = 0.0
        self.v_msg.angular.z = np.pi/2
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(1)
        self.v_msg.linear.x = 0.1
        self.v_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(.5)

    def forward(self):
        self.v_msg.linear.x = 0.1
        self.v_msg.angular.z = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(1)

    def cleanup(self):  
        #Finish the node    
        self.v_msg = Twist() 
        self.pub_cmd_vel.publish(self.v_msg)
        

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("controller", anonymous=True)  
    Controller()
