#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 
import numpy as np 

#This class will make the puzzlebot move following a square 

class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ ROBOT CONSTANTS ################  
        r=0.05 #wheel radius [m] 
        L=0.19 #wheel separation [m] 

        ###########  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############## SUBSCRIBERS ##################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  

        v_msg=Twist() 

        x_pos = 0.0 #metros. Posicion del robot en eje x
        y_pos = 0.0 #metros. Posicion del robot en eje y 
        angle = 0.0 #rad oroentacion del robot  
        v = 0.0 #Velocidad linear del robot [m/s]
        w = 0.0 #Velocidad angular [rad/s]
        self.wl = 0.0 #Velocidad rueda left [rad/s]
        self.wr = 0.0 #Velocidad rueda right [rad/s]

        while rospy.get_time() == 0: 
            print("No simulated time has been received")
        
        rate = rospy.Rate(20) #20Hz
        print("Node initialized")
        t_ant = rospy.get_time() #Valor para obtener dt 

        while not rospy.is_shutdown(): 
            dt = rospy.get_time() - t_ant
            t_ant = rospy.get_time()

            v = r*(self.wr - self.wl)/2 #Velocidad linear
            w = r*(self.wr - self.wl)/L   #Velocidad angular 

            angle = angle + w * dt #Theta 
            angle = np.arctan2(np.sin(angle) , np.cos(angle))
            x_pos = x_pos + v *dt* np.cos(angle) #Posicion en X
            y_pos = y_pos + v *dt* np.sin(angle) #Posicion en Y

            print ("Angle: %10.3f X Pos: %10.3f Y Pos: %10.3f" % (angle , x_pos , y_pos))

            #self.pub_cmd_vel.publish(v_msg) #publish the robot's speed  
            rate.sleep()  
             

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
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("go_to_goal", anonymous=True)  
    GoToGoal()  