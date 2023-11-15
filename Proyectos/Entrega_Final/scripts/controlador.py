#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32

class Controller():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.init_node("Controler")
        rate = rospy.Rate(60)

        rospy.Subscriber("v" , Float32 , self.v_cb)
        rospy.Subscriber("w" , Float32 , self.w_cb)
        rospy.Subscriber("senal_flag" , String , self.senal_cb)
        rospy.Subscriber("semaforo_flag" , String , self.semaforo_cb)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Inicializo variables
        v = 0.0  # velocidad lineal del robot [m/s]
        w = 0.0  # velocidad angular del robot [rad/s]

        self.v = 0
        self.w = 0

        previous_time = rospy.get_time()

        self.semaforo = "Verde"
        self.sen = ""

        self.v_msg = Twist()

        while rospy.get_time() == 0:
            print("no simulated time has been received yet") 

        while not rospy.is_shutdown():

            test = rospy.get_time() - previous_time
            test = not self.radius and test > 0.2

            if self.v and self.w:
                if self.semaforo == "Verde":
                    t = 1
                    if self.sen == "Ahead": self.ahead_act(t)
                    elif self.sen == "Right": self.tRight_act(t)
                    elif self.sen == "Left": self.tLeft_act(t)
                    elif self.sen == "RoadWork": self.roadWork(t)
                    elif self.sen == "Stop": self.stop_act(t)
                    elif self.sen == "GiveWay": self.giveWay_act(t)
                    else:
                        self.v_msg.linear.x = self.v
                        self.v_msg.angular.z = self.w
                    print ("Verde")
                        
                elif self.semaforo == "Amarillo":
                    t = 2
                    if self.sen == "Ahead": self.ahead_act(t)
                    elif self.sen == "Right": self.tRight_act(t)
                    elif self.sen == "Left": self.tLeft_act(t)
                    elif self.sen == "RoadWork": self.roadWork(t)
                    elif self.sen == "Stop": self.stop_act(t)
                    elif self.sen == "GiveWay": self.giveWay_act(t)
                    else: 
                        self.v_msg.linear.x = self.v/2.0
                        self.v_msg.angular.z = self.w/2.0
                    print ("Amarillo")
                        
                elif self.semaforo == "Rojo":
                    print("Estoy en rojo")
                    self.v_msg.linear.x = 0.0
                    self.v_msg.angular.z = 0.0
                    
            else:
                self.v_msg.linear.x = 0.0
                self.v_msg.angular.z = 0.0
                
            self.pub_cmd_vel.publish(self.v_msg)
            rate.sleep()

    def v_cb(self, v):   
        self.v = v.data 

    def w_cb(self, w):  
        self.w = w.data
    
    def semaforo_cb(self, s):  
        self.semaforo =  s.data

    def senal_cb(self, sen):  
        self.sen =  sen.data

    def ahead_act(self , n):
        self.v_msg.linear = 0.40/n
        self.v_msg.angular = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)

    def tRight_act(self , n):
        self.v_msg.linear = 0.20/n
        self.v_msg.angular = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)
        self.v_msg.linear = 0
        self.v_msg.angular = np.pi/2/n
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)

    def tLeft_act(self , n):
        self.v_msg.linear = 0.20/n
        self.v_msg.angular = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)
        self.v_msg.linear = 0
        self.v_msg.angular = -np.pi/(2*n)
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)

    def roadWork(self):
        self.v_msg.linear = self.v*2/3
        self.v_msg.angular = self.w*2/3

    def stop_act(self , n):
        self.v_msg.linear = 0
        self.v_msg.angular = 0
        self.pub_cmd_vel.publish(self.v_msg)
        time.sleep(n)
    
    def giveWay_act(self , n):
        self.v_msg.linear = self.v/2
        self.v_msg.angular = self.w/2

    def round_act(self , n):
        pass

    def cleanup(self):     
        v_msg = Twist() 
        self.pub_cmd_vel.publish(v_msg)
        

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("controller", anonymous=True)  
    Controller()