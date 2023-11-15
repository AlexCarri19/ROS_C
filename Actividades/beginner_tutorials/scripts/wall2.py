#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# This class implements a simple obstacle avoidance algorithm
class AvoidObstacleClass():
   def __init__(self):
       rospy.on_shutdown(self.cleanup)
       ####################### PUBLISEHRS AND SUBSCRIBERS
############################
       rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
       self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist,
queue_size=1)
       ######################## CONSTANTS AND VARIABLES
##############################
       self.closest_angle = 0.0 #Angle to the closest object
       self.closest_range = np.inf #Distance to the closest object
       robot_vel = Twist() #The robot's velocity
       self.dAO = 0.4 #[m] distance to start the avoid obstaclesbehavior
       self.dAOB = 0.3 #[m] distance to stop

       r = rospy.Rate(10) #10Hz is the lidar's frequency
       print("Node initialized 1hz")
       ############################### MAIN LOOP
#####################################
       while not rospy.is_shutdown():
           print("closest object distance: " + str(self.closest_range))
           print("theta_closest: " + str(self.closest_angle))
           if self.closest_range <= self.dAOB :
               robot_vel.linear.x = 0.0
               robot_vel.angular.z = 0.0
               print("stopped")
           elif self.closest_range <= self.dAO:
                robot_vel.linear.x,robot_vel.angular.z = self.wall_follower_controller()
                print("esta madre no sirve y se califico en clase nmms,pd q joto ")
           else:
               robot_vel.linear.x = 0.2
               robot_vel.angular.z = 0.0
               print("moving fordward")
           self.cmd_vel_pub.publish(robot_vel)
           r.sleep()

   def wall_follower_controller(self):
       theta_AO = self.closest_angle - np.pi
       #print("Angulo en theta: ", theta_AO)
       theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) #Limit the angle from -pi to pi
       thetafw = np.pi/2.0 + theta_AO
       thetafw = np.arctan2(np.sin(thetafw), np.cos(thetafw))
       kv = 5.0 #[m/s] Robot's linear vel while avoiding obstacles
       kw = 3.0 #. Proportional constant for the angular speedcontroller (we consider the maximum values)

       v_wf = kv
       w_wf = kw * thetafw
       return v_wf, w_wf
   
   def laser_cb(self, msg):
       ## This function receives a message of type LaserScan andcomputes the closest object direction and range
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
############################### MAIN PROGRAM
####################################
if __name__ == "__main__":
   rospy.init_node("avoid_obstacle", anonymous=True)
   AvoidObstacleClass()