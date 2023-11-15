#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.init_node("PID")
        rate = rospy.Rate(60) #60Hz  

        rospy.Subscriber('wl', Float32, self.wl_cb)
        rospy.Subscriber('wr', Float32, self.wr_cb)
        rospy.Subscriber("radius", Int32, self.radius_cb)
        rospy.Subscriber("center", Point, self.center_cb)

        self.pub_v = rospy.Publisher('v', Float32, queue_size=1)
        self.pub_w = rospy.Publisher('w', Float32, queue_size=1)

        # ROBOT CONSTANTS
        r , L = 0.05 , 0.19   
        
        # Variables
        v , w = 0.0 , 0.0  # Linear [m/s] , Angular [rad/s]
        self.wl , self.wr = 0.0 , 0.0  # Angular izquierda [rad/s] Angular derecha [rad/s]
        e_theta_min = 20
        w_max , w_min = 0.15 , 0.05
        v_max , v_min = 0.2 , 0.05
        e0 , e1 , e2 = 0.0 , 0.0 , 0.0
        u0 , u1  = 0.0 , 0.0 
          
        kp = 0.000045
        ki = 0.0000007
        kd = 0.000035

        Ts = 0.2

        K1 = kp + Ts * ki + kd / Ts
        K2 = -kp - 2.0 * kd / Ts
        K3 = kd / Ts
        
        self.radius = 0
        self.xc = 0.0

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

        previous_time = rospy.get_time() #obtener dt

        print("Node initialized") 

        while not rospy.is_shutdown():
            if self.radius:
                #print("Recibi el radio")
                delta_t = rospy.get_time()-previous_time
                previous_time = rospy.get_time() #lo actualizo luego de usarlo
                v = r*(self.wl+self.wr)/2.0
                w = r*(self.wr-self.wl)/L
                e_theta = 200 - self.xc

                #e_theta = np.arctan2(np.sin(e_theta),np.cos(e_theta))

                e0 = e_theta
                
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
                #error para el controlador de velocidad lineal  
                
                m_input = u0 

                #print("m_input : " + str(m_input))
                #print("m_input1 : " + str(m_input1))

                if e_theta > e_theta_min:
                    #print("SE salio a la derecha")
                    w = m_input * e_theta 
                    v = 0.1
                elif e_theta > - e_theta_min and e_theta < e_theta_min:
                    #print("DEntro de los limites")
                    v = 0.2
                    w = 0.0
                elif e_theta < -e_theta_min:
                    #print("Se salio a la izquierda")
                    w = -m_input * e_theta
                    v = 0.1

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

                self.pub_v.publish(v)
                self.pub_w.publish(w)

                rate.sleep()

    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data

    def radius_cb(self, msg):  
        #Callback del radio 
        self.radius =  msg.data  
    
    def center_cb(self, msg):  
        #Callback de la distancia
        self.xc = msg.x
        self.yc = msg.y

    def cleanup(self):  
        #This function is called just before finishing the node  
        self.pub_v.publish(0)
        self.pub_w.publish(0)   
        
        

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("controller", anonymous=True)  
    Controller()