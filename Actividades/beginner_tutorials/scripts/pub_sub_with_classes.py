#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String  

#This class will subscribe to the /message topic and publish to the /cmd_vel topic 
class PubSubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node. 

        #########PUBLISHERS AND SUBSCRIBERS ################# 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        rospy.Subscriber("string_topic", String, self.message_cb)  

        ############ CONSTANTS AND VARIABLES ################  
        vel = Twist() 
        self.my_string = "stop" 
        r = rospy.Rate(20) #2 Hz 
        zero_0 = Twist()

        while not rospy.is_shutdown():  
            if self.my_string == "move forward": 
                print("moving forward") 
                vel.linear.x = 0.1 
                vel.angular.z = 0.0

            elif self.my_string == "move 1m": 
                print("moving forward") 
                vel.linear.x = 0.1 
                vel.angular.z = 0.0
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(10)
                self.cmd_vel_pub.publish(zero_0)
                self.my_string = "STOP"

            elif self.my_string == "move 2m": 
                print("moving forward") 
                vel.linear.x = 0.2 
                vel.angular.z = 0.0
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(10)
                self.cmd_vel_pub.publish(zero_0)
                self.my_string = "STOP"

            elif self.my_string == "move 3m": 
                print("moving forward") 
                vel.linear.x = 0.3
                vel.angular.z = 0.0
                self.cmd_vel_pub.publish(vel)
                rospy.sleep(10)
                self.cmd_vel_pub.publish(zero_0)
                self.my_string = "STOP"

	    elif self.my_string == "l":
		print("funciona")
		vel.linear.x = 0.5
		vel.angular.z = 0.0

		rospy.sleep(10)

            else: #stop 
                print("stopped") 
                vel.linear.x = 0.0 
                vel.angular.z = 0.0 

            self.cmd_vel_pub.publish(vel) #publish the message 
            r.sleep()  #It is very important that the r.sleep function is called at least onece every cycle.  

     

    def message_cb(self, msg):  
        ## This function receives the ROS message as the msg variable.  
        self.my_string =  msg.data #msg.data is the string contained inside the ROS message  
        print("I received this message in the callback: " + self.my_string) 

         

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        zero_0 = Twist()
        print("I'm dying, bye bye!!!")
        self.cmd_vel_pub.publish(zero_0)


 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("pub_sub_with_classes", anonymous=True)  
    PubSubClass()  
