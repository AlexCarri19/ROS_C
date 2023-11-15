#!/usr/bin/env python

"""Este programa publica el radio y el centro de la pista
   Si no detecta la pista, entonces los valores seran cero
   Topicos publicados:
       /center  [Point]
       /radius  [Int32]
   Topicos suscritos:
       /camera/image_raw    [Image]
       /video_source/raw
"""

# import ROS stuff
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

# construct the argument parse and parse the arguments
class LineTracker():
    def __init__(self):
        print("ENTRE AL tracker_from_topic")
        rospy.on_shutdown(self.cleanup)
        # Init the ROS node
        self.pub_center = rospy.Publisher('center', Point, queue_size=10)
        self.pub_radius = rospy.Publisher('radius', Int32, queue_size=10)
        rospy.init_node('tracker')
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.camera_callback)
        self.image_pub = rospy.Publisher('short_image', Image, queue_size=1)
        self.bridge_object = CvBridge()  # Creates the bridge object between ROS and OpenCV images
        self.center_ros = Point()
        self.radius_ros = 0
        self.image_received_flag = 0  # This flag ensures that we received at least one image

        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video", help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
        self.args = vars(ap.parse_args())

        # Se definen los parametros maximos y minimos para el color "negro" de la pista utilizando HSV
        self.colorLower = (0, 0, 0)
        self.colorUpper = (50, 50, 50)
        self.pts = deque(maxlen=self.args["buffer"])

        # To adjust the execution rate of the while Loop
        ros_rate = rospy.Rate(10)  # 10Hz

        # keep looping
        while not rospy.is_shutdown():
            if self.image_received_flag == 1:

                # self.frame = self.frame1[150:800, 200:600]
                self.find_ball()

                # show the frame on the screen
                self.image_pub.publish(self.frame3)
                cv2.imshow("Frame Acortado", self.frame)
                #cv2.imshow("Frame Original", self.frame1)

                self.image_received_flag = 0

            # Publish the radius and the center of the ball
            self.pub_center.publish(self.center_ros)
            self.pub_radius.publish(self.radius_ros)
            print("Enviar radio y centro")
            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()

    def camera_callback(self, data):
        print("callback")
        try:
            # We select bgr8 because it's the OpenCV encoding by default
            self.frame1 = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def find_ball(self):
        # Resize the frame, blur it, and convert it to the HSV color space
        self.frame = self.frame1.copy()
        width = self.frame.shape[1]
        #Se calcula la mitad del ancho de la imagen
        center_x = width // 2
        print(str(center_x))
        print("Y mid =" + str(self.frame.shape[0]))
        #Se asigna un desplazamiento
        #Cantidad de pixeles a la izq y a la der que se mostraran
        offset = 70
        #Se seleccionan las columnas del centro del eje horizontal
        # Se resta offset al valor del centro para obtener el indice de inicio del rango
        # Se suma offset al valor del centro para obtener el indice de fin del rango
        self.frame = self.frame[700:800, center_x - offset:center_x + offset]
        self.frame = imutils.resize(self.frame, width=400)
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_black = cv2.inRange(hsv, self.colorLower, self.colorUpper)
        mask_black = cv2.erode(mask_black, None, iterations=2)
        mask_black = cv2.dilate(mask_black, None, iterations=2)
    
        res = cv2.bitwise_and(self.frame, self.frame, mask=mask_black)
        self.frame3 = self.bridge_object.cv2_to_imgmsg(res, encoding = "passthrough")

        # Find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask_black.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # Only proceed if at least one contour was found
        if len(cnts) > 0:
            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Only proceed if the radius meets a minimum size
            if radius > 10:
                # Draw the circle and centroid on the frame, then update the list of tracked points
                self.center_ros.x = float(x)
                self.center_ros.y = float(y)
                self.center_ros.z = 0  # As it is an image, z is not used.
                self.radius_ros = int(radius)
                print(self.radius_ros)
                print(self.center_ros)

                cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
            else:
                self.center_ros.x = 0
                self.center_ros.y = 0
                self.center_ros.z = 0
                self.radius_ros = 0
                cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
        else:
            # Publish a radius of zero if there is no detected object
            self.center_ros.x = 0
            self.center_ros.y = 0
            self.center_ros.z = 0
            self.radius_ros = 0
            cv2.circle(self.frame, (0, 0), 1, (0, 0, 0), 2)
        print(str(self.center_ros.x))
        print(str(self.center_ros.y))
        # Update the points queue
        self.pts.appendleft(center)

        # Loop over the set of tracked points
        for i in range(1, len(self.pts)):
            # If either of the tracked points is None, ignore them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            # Otherwise, compute the thickness of the line and draw the connecting lines
            thickness = int(np.sqrt(self.args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(self.frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

    def cleanup(self):
        print("Shutting down vision node")
        # Close all windows
        cv2.destroyAllWindows()

if __name__ == '__main__':  
    LineTracker()
