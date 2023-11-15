#!/usr/bin/env python  
import rospy  
import numpy as np 
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist 

# This class implements a simple obstacle avoidance algorithm 

class AvoidObstacleClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
 
        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 
         
        ######################## CONSTANTS AND VARIABLES ##############################  
        self.closest_angle = 0.0 #Angle to the closest object 
        self.closest_range = np.inf #Distance to the closest object 
        robot_vel = Twist() #The robot's velocity 
        robot_vel.linear.x = 0.5 #[m/s] 
        self.dAO = 1 #[m] Distance to start the avoid obstacles behavior 
        self.limAngle = np.pi/2
        self.secDis = .3
        self.secAngle = np.pi/2


        r = rospy.Rate(50) #10Hz is the lidar's frequency  
        print("Node initialized 10hz") 
        ############################### MAIN LOOP ##################################### 
        while not rospy.is_shutdown(): 
            if self.closest_range <= self.secDis and -self.secAngle < self.closest_angle < self.secAngle:
                robot_vel.linear.x = 0.0
                robot_vel.angular.z = 0.0  
                print("Estoy muy cerca... Dis: %f Angle: %f" % (self.closest_range , self.closest_angle))

            elif self.closest_range <= self.dAO and -self.limAngle< self.closest_angle < self.limAngle:
                robot_vel.linear.x , robot_vel.angular.z = self.avoid_obstacles_controller()
                print("Evitando")
                

            else:
                robot_vel.linear.x = 0.5
                robot_vel.angular.z = 0.0 
                print("Avanzando")

            self.cmd_vel_pub.publish(robot_vel) 
            r.sleep()  

    def avoid_obstacles_controller(self):
        kAO = 2.0 #Proportional constant for the angular speed velocity
        theta_AO = self.closest_angle + np.pi
        theta_AO = np.arctan2(np.sin(theta_AO) , np.cos(theta_AO)) #Limit the angle to -pi to pi
        v = 0.2 #Linear velocity of Avoid Obstacles [m/s]
        w = kAO * theta_AO
        return v , w

    def laser_cb(self, msg):  
        ## This function receives a message of type LaserScan and computes the closest object direction and range 
        self.closest_range = min(msg.ranges) 
        idx = msg.ranges.index(self.closest_range) 
        self.closest_angle = msg.angle_min + idx * msg.angle_increment 
        # Limit the angle to [-pi,pi] 
        self.closest_angle = np.arctan2(np.sin(self.closest_angle),np.cos(self.closest_angle)) 
   

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("avoid_obstacle", anonymous=True)  
    AvoidObstacleClass() 