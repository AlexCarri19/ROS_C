#!/usr/bin/env python  
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

# This class will subscribe to the /base_scan topic and print some data 

class LaserClosestObject() :  
    def _init_(self) :  
        rospy.on_shutdown(self.cleanup)  
 

        ############    SUBSCRIBERS   #######################  
        rospy.Subscriber("base_scan", LaserScan, self.lidar_cb)

        ################  INIT PUBLISHERS ################ 
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)


        ############ CONSTANTS AND VARIABLES ################  
        r = rospy.Rate(50)
        self.lidar = LaserScan()
        self.vel = Twist()  

        range_closest = 0.0
        index = 0
        safety_d = 0.3
        k_v = 0.5
        k_w = 1.7
        e_theta = 0.0
        theta_min = (np.pi / 180) * 5.0 

        while not rospy.is_shutdown() : 
            if self.lidar.ranges :
                range_closest = min(self.lidar.ranges)
                index = self.lidar.ranges.index(range_closest)
                angle_closest = self.lidar.angle_min + (index * self.lidar.angle_increment)

                e_theta = np.arctan2(np.sin(angle_closest), np.cos(angle_closest))

                if np.isinf(range_closest) :
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                else :
                    if range_closest > safety_d :
                        self.vel.linear.x = k_v * range_closest
                        self.vel.angular.z = k_w * e_theta
                    else :
                        if abs(e_theta) > theta_min :
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = k_v * e_theta
                        else :
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.0

                self.pub_cmd_vel.publish(self.vel)
            else :
                print("WAITING Lidar data")

            r.sleep()  

    def lidar_cb(self, lidar_msg) :  
        self.lidar = lidar_msg               

    def cleanup(self) :  
        print("NODE KILLED")
        zero = Twist()
        self.pub_cmd_vel.publish(zero)  
 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__" :  
    rospy.init_node("laser_closest_object", anonymous=True)  
    LaserClosestObject()