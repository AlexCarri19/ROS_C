#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float32 
from std_msgs.msg import Int32 
import numpy as np
import time

class Controller():  

    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 

        ############ ROBOT CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        ### Inicializo variables ###
        thetar = 0.0 #[rad]
        xr = 0.0 # [m] posicion del robot a lo largo del eje x
        yr = 0.0 # [m] posicion del robot a lo largo del eje y
        v = 0.0 #velocidad lineal del robot [m/s]
        w = 0.0 #velocidad angular del robot [rad/s]
        
	    #Wheels speed	
        self.wl = 0.0 #vel angular rueda izquierda [rad/s]
        self.wr = 0.0 #vel angular rueda derecha [rad/s]
        
	    #Distances	
        d = 100000.0 
        d_min = 0.05 #[m] d_min to the goal	
        e_theta_min = np.pi/90

        #V max y min

        w_max = 7.1
        w_min = 0.05
        v_max = 0.4
        v_min = 0.05
        
	    #Flag to request data	
        self.flag = 1
        
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

        ######### PUBLISHERS AND SUBSCRIBERS ################# 
        
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.flag_pub = rospy.Publisher('flag', Int32, queue_size=1) 
        rospy.Subscriber('wl', Float32, self.wl_cb)
        rospy.Subscriber('wr', Float32, self.wr_cb)
        rospy.Subscriber('pose', Pose, self.pose_cb)

        v_msg=Twist()
        
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
                v = r*(self.wl+self.wr)/2.0
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
                #publica v y w
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
                    v = 0
                else:
                    v = abs(m_input1) * d
                    w = 0          
                
                if w > w_max:
                    w = w_max
                elif w < -w_max:
                    w = -w_max
                elif w < w_min and w > 0:
                    w = w_min
                elif w > -w_min and w < 0:
                    w = -w_min

                if v > v_max:
                    v = v_max
                elif v < -v_max:
                    v = -v_max
                elif v < v_min and v > 0:
                    v = v_min
                elif v > -v_min and v < 0:
                    v = -v_min
                
                v_msg.linear.x = v
                v_msg.angular.z = w


                #print("x: ", str(xr))
                #print("y: ", str(yr))
                #print("theta: ", str(thetar))
                #print("Distancia: ", str(d))
                #print("Error theta: ", str(e_theta))
                #print("V = " + str(v))
                print("w = " + str(w))
                #print(" ")

            else:
                print("stop")
                self.flag = 1
                self.flag_pub.publish(self.flag)
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                d = 1000000.0
            
            self.pub_cmd_vel.publish(v_msg) 
            rate.sleep()  

    def pose_cb(self, msg): 

        self.xG = msg.position.x 
        self.yG = msg.position.y  
        self.flag = 0
        self.flag_pub.publish(self.flag)
        #print("callback pose") 

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data  

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        v_msg = Twist() 
        self.pub_cmd_vel.publish(v_msg) 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("controller", anonymous=True)  
    Controller()


# Puntos actividad 1
 #- [1.6, 0.0]
 #- [-0.8, 0.0]
 #- [0.0, 0.8]
 #- [0, -1.2]
 #- [0.8, 0.8]
 #- [-0.8, -0.8]
 #Puntos actividad 2
 #- [2.0, 0.0]
 #- [2.0, 2.0]
 #- [0.0, 2.0]
 #- [0.0, 0.0]
 #Puntos actividad 3
 #- [1.2, 1]
 #- [2, 0.5]
 #- [2.5, -1.0]
 #- [1.0, -2.0]
 #- [-2.0, 0.0]
