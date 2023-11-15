#!/usr/bin/env python  

import rospy 
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import numpy as np  

class Path_Gen():  

    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 
        self.punto = rospy.get_param("/path")
        
        ######### PUBLISHERS AND SUBSCRIBERS ################# 

        self.pose_pub = rospy.Publisher('pose', Pose, queue_size=1) 
        rospy.Subscriber('flag', Int32, self.flag_cb)

        self.i = 0

        ############ CONSTANTS AND VARIABLES ################  
        self.pose = Pose() 
        self.x = 0.0 # x[m] Goal position  
        self.y = 0.0 # y[m] Goal position  
        self.position = 0 
        self.recorrido = 0
        self.ch_status = 0
     
        while rospy.get_time() == 0: 
            print("no simulated time has been received yet") 

        print("Got time") 

        start_time = rospy.get_time()  #Get the current time in float seconds 

        state = "Programa pose" 

        print(state) 

        r = rospy.Rate(40) #10 Hz 

        while not rospy.is_shutdown():
            #bandera de cuando enviar
            #for self.i in len(punto):
            if self.ch_status == 1 and self.recorrido<len(self.punto):
                self.pose.position.x = self.punto[self.i][0]
                self.pose.position.y = self.punto[self.i][1]
                self.i = self.i + 1
                self.recorrido=self.recorrido + 1
                self.ch_status = 0
                self.pose_pub.publish(self.pose)
            
    def flag_cb(self, msg): 
        self.ch_status = msg.data
        if(self.recorrido>len(self.punto)-1):
            print("Proceso terminado  ")
        else:
            print("Se recibio la bandera  " + str(self.ch_status))
        
    
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        self.i = 0
        self.recorrido = 0
        self.pose = Pose()
        self.pose_pub.publish(self.pose)

if __name__ == "__main__":  
    rospy.init_node("generador", anonymous=True)  
    Path_Gen()