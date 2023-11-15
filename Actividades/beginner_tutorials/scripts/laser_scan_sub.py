#!/usr/bin/env python  

import rospy  
import numpy as np
from sensor_msgs.msg import LaserScan 

# This class will subscribe to the /base_scan topic and print some data 

class LaserSubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############    SUBSCRIBERS   #######################  
        rospy.Subscriber("base_scan", LaserScan, self.lidar_cb)  

        ############ CONSTANTS AND VARIABLES ################  
        self.lidar = LaserScan() #The data from the lidar will be kept here. 
        r = rospy.Rate(1) #1Hz  
        print("Node initialized 1hz") 

         

        while not rospy.is_shutdown():  
            #print("ranges:") 
            #print(self.lidar.ranges) 
            if (self.lidar.ranges):
                self.initialValues()

                closest = (min(self.lidar.ranges))   
                index = self.lidar.ranges.index(closest) 
                angleclosest = self.lidar.angle_min + index*self.lidar.angle_increment           
                print("Angle closest: %f" % (angleclosest))

            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     

    def lidar_cb(self, lidar_msg):  
        ## This function receives the lidar message and copies this message to a member of the class  
        self.lidar = lidar_msg 

    

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        print("I'm dying, bye bye!!!")  

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