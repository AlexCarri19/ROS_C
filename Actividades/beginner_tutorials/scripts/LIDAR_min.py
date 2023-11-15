#!/usr/bin/env python  

import rospy  
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

# This class will subscribe to the /base_scan topic and print some data 

class LaserSubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############    SUBSCRIBERS AND PUBLISHERS   #######################  
        rospy.Subscriber("base_scan", LaserScan, self.lidar_cb)  

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        ############ CONSTANTS AND VARIABLES ################  
        self.lidar = LaserScan() #The data from the lidar will be kept here. 
        self.vel = Twist()
        r = rospy.Rate(50) #1Hz  
        print("Node initialized 50hz") 

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

                eth = np.arctan2(np.sin(angle_closest), np.cos(angle_closest))

                if np.isinf(range_closest) :
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                else :
                    if range_closest > safety_d :
                        self.vel.linear.x = k_v * range_closest
                        self.vel.angular.z = k_w * eth
                    else :
                        if abs(eth) > theta_min :
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = k_v * eth
                        else :
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.0

                self.pub_cmd_vel.publish(self.vel)
            else :
                print("WAITING Lidar data")

            r.sleep()  
            
            '''
            if (self.lidar.ranges):
                self.initialValues()

                closest = (min(self.lidar.ranges))   
                index = self.lidar.ranges.index(closest) 
                angleclosest = self.lidar.angle_min + index*self.lidar.angle_increment           
                print("Angle closest: %f" % (angleclosest))

            r.sleep() ''' #It is very important that the r.sleep function is called at least once every cycle.  

     

    def lidar_cb(self, lidar_msg):  
        ## This function receives the lidar message and copies this message to a member of the class  
        self.lidar = lidar_msg 
    

    def cleanup(self) :  
        print("NODE KILLED")
        zero = Twist()
        self.pub_cmd_vel.publish(zero) 

    def initialValues(self):
        print("Distancia maxima: %f" % (max(self.lidar.ranges)))
        print("Distancia minima: %f" % (min(self.lidar.ranges)))
        print("Angulo maxima: %f" % (self.lidar.angle_max))
        print("Angulo minimo: %f" % (self.lidar.angle_min))
        print("Header: %s" % (self.lidar.header.frame_id))
        print("First comp: %f" % (self.lidar.ranges[0]))
        print("First comp: %f" % (self.lidar.ranges[len(self.lidar.ranges)- 1]))

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("laser_scan_subscriber", anonymous=True)  
    LaserSubClass() 