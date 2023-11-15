#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float32  

#This class will subscribe to the /message topic and publish to the /cmd_vel topic 
class SquareClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 

        #########PUBLISHERS AND SUBSCRIBERS ################# 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        rospy.Subscriber("total_time", Float32 , self.message_cb)  

        ############ CONSTANTS AND VARIABLES ################  
        vel = Twist() 
        state = "stop" 
        r = rospy.Rate(30) #20 Hz 

        counter = 0
        distance = 2.0
        angle = np.pi/2 #radianes angulo para giros (cuadrado = 90)
        self.total_time = 0.0
        t_linear = self.total_time/8.0 
        t_angular = self.total_time/8.0 
        self.time_received = 0
        start_time = 0 #Segundos, tiempo inicial de un movimiento (linear o angular)

        ############ Main Loop ################

        while rospy.get_time() == 0:
            print ("no simulated time has beeen received yet")
            print (state)

        while not rospy.is_shutdown():  
            if state == "stop": 
                vel.linear.x = 0.0  
                vel.angular.z = 0.0  

                if self.time_received==1: 
                    state = "move_linear" 
                    print("From Stop to Linear")
                    start_time = rospy.get_time()
                    t_linear = self.total_time/8.0
                    t_angular = self.total_time/8.0

            elif state == "move_linear": 
                vel.linear.x = distance/t_linear 
                vel.angular.z = 0.0 
                if rospy.get_time() - start_time >= t_linear:
                    print("change to move angular")
                    state = "move_angular"
                    counter = counter + 1
                    start_time = rospy.get_time()

            elif state == "move_angular":
                vel.linear.x = 0
                vel.angular.z = angle/t_angular
                if rospy.get_time() - start_time >= t_angular:
                    if counter < 4:
                        state = "move_linear"
                        counter = counter + 1
                        start_time = rospy.get_time()

                    else:
                        state = "stop"
                        print ("Change from turn to move linear")
                        start_time = rospy.get_time()

            elif state == "stop": #stop 
                print("stopped") 
                vel.linear.x = 0.0 
                vel.angular.z = 0.0 

            self.cmd_vel_pub.publish(vel) #publish the message 
            r.sleep()  #It is very important that the r.sleep function is called at least onece every cycle.  

     

    def message_cb(self, msg):  
        ## This function receives the ROS message as the msg variable.  
        self.total_time =  msg.data #msg.data is the string contained inside the ROS message 
        self.time_received = 1 
        print("I received this message in the callback ")      

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        zero_0 = Twist()
        print("I'm dying, bye bye!!!")
        self.cmd_vel_pub.publish(zero_0)
 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("puzzlebot_square", anonymous=True)  
    SquareClass()  