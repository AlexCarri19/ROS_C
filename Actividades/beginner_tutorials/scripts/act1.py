#!/usr/bin/env python  

import rospy  

from geometry_msgs.msg import Twist  

from geometry_msgs.msg import Pose 

from std_msgs.msg import Float32 

import numpy as np  

 

class MoveClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 

        ######### PUBLISHERS AND SUBSCRIBERS ################# 

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        rospy.Subscriber('pose', Pose, self.pose_cb) 

        rospy.Subscriber('time', Float32, self.time_cb) 

         

         

        ############ CONSTANTS AND VARIABLES ################  

        vel = Twist() 

        self.x = 0.0 # x[m] Goal position  

        self.y = 0.0 # y[m] Goal position  

        self.pose_received = 0 # A flag to check if the pose has been received 

        self.time_received = 0 # A flag to check if the time has been received 

        self.desired_time = 10.0 

         

         

        while rospy.get_time() == 0: 

            print("no simulated time has been received yet") 

        print("Got time") 

        start_time = rospy.get_time()  #Get the current time in float seconds 

        state = "stop" 

        print(state) 

        r = rospy.Rate(20) #20 Hz 

 

        while not rospy.is_shutdown():  

             

            if state == "stop": 

                vel.linear.x = 0.0 

                vel.angular.z = 0.0 

                if (self.pose_received == 1 and self.time_received==1): 

                    state = "turn" 

                    print("change from stop to turn") 

                    t_linear = self.desired_time/2.0 

                    t_angular = self.desired_time/2.0 

                    print("t_linear "+str(t_linear)) 

                    print("t_angular "+str(t_angular)) 

                    theta=np.arctan2(self.y,self.x) 

                    start_time = rospy.get_time() 

            elif state == "turn": 

                vel.linear.x = 0.0 

                vel.angular.z = theta/t_angular 

                if (rospy.get_time()-start_time>= t_angular): 

                    print("change from turn to move_linear") 

                    state="move_linear" 

                    print(state) 

                    start_time = rospy.get_time() 

                    distance = np.sqrt(self.x**2+self.y**2) 

            elif state =="move_linear": 

                vel.linear.x = distance/t_linear 

                vel.angular.z = 0.0 

                if (rospy.get_time()-start_time>= t_linear): 

                    print("change from move_linear to stop") 

                    state="stop" 

                    start_time = rospy.get_time() 

                    self.pose_received=0 

                    self.time_received=0 

            else: 

                state="stop" 

                vel.linear.x = 0.0 

                vel.angular.z = 0.0 

                self.pose_received=0 

                self.time_received=0 

 

 

             

            self.cmd_vel_pub.publish(vel) #publish the message 

            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     

    def pose_cb(self, msg): 

        self.x = msg.position.x 

        self.y = msg.position.y  

        self.pose_received = 1 

        print("callback pose") 

 

    def time_cb(self, msg): 

        self.desired_time = msg.data 

        self.time_received = 1 

        print("callback time") 

 

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        print("I'm dying, bye bye!!!")  

        stop_twist = Twist() 

        self.cmd_vel_pub.publish(stop_twist) 

 

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("move_to_point_open_loop", anonymous=True)  

    MoveClass()  