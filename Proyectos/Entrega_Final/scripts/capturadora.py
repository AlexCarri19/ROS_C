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

        rospy.init_node("Capturadora")
        rate = rospy.Rate(60)

        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_callback)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.flag_pub = rospy.Publisher('flag', Int32, queue_size=1)
        
        self.radius = 0
        self.xc = 0.0
        bandera1 = False
        bandera2 = 0
        bandera_imagen1 = 0
        bandera_imagen = 0

        v_msg = Twist()

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

        previous_time = rospy.get_time() #obtener dt

        rate = rospy.Rate(60) #20Hz  
        print("Node initialized") 

        while not rospy.is_shutdown():
            if self.radius:
                if self.image_received == 1:
                    self.cv_image = self.cv_image_received.copy()
                    self.image_foto = self.cv_image_received.copy()
                    cv2.imshow('imagen para foto',self.image_foto) #Borrar esta linea si lo corres desde la jetson

                    if cv2.waitKey(1) == 108:#l 
                        print("Teclal")
                        bandera1 = True
                        bandera2 = 0

                    while bandera1 and bandera2 <= 10:
                        bandera_imagen = bandera_imagen1 + 1
                        nombre_imagen = 'robot_image' + str(bandera_imagen) + '.jpg'
                        cv2.imwrite(nombre_imagen, self.image_foto)
                        print("tome una foto")
                        bandera_imagen1 = bandera_imagen1 + 1
                        print(nombre_imagen)
                        print(bandera_imagen)
                        bandera2 = bandera2 + 1
                        time.sleep(0.5)

                    #offset = 150
                    #Se seleccionan las columnas del centro del eje horizontal
                    # Se resta offset al valor del centro para obtener el indice de inicio del rango
                    # Se suma offset al valor del centro para obtener el indice de fin del rango
                    #self.cv_image = self.cv_image[700:800, 200:300]
                    self.cv_image = cv2.resize(self.cv_image,(300,300), interpolation=cv2.INTER_AREA)                  
                    hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
                    


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