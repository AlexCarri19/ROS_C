#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

#Import the numpy library which will help with some matrix operations
import numpy as np

class ShowingImage(object):

    def __init__(self):
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node.
        
        # Suscribers
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.camera_callback)
        #Publisher
        self.image_pub = rospy.Publisher("segmented_image", Image, queue_size=1)
        self.bridge_object = CvBridge()
        self.image_received = 0 #Flag to indicate that we have already received an image
        r = rospy.Rate(10)

        # Valores para el HSV AZUL
        min_blue = np.array([100,100,20])
        max_blue = np.array([125,255,255])

        while not rospy.is_shutdown():
            #Si es que hay imagen entra a la condicion:
            if self.image_received:
                image = cv2.resize(self.cv_image,(500,500)) #Cambiar el tamano de la imagen
                cv2.imshow('Original',image) #Mostrar la imagen original

                #Se transforma la imagen de BGR a HSV
                img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                #Se aplica la mascara a la imagen original
                mask_b = cv2.inRange(img_hsv, min_blue, max_blue)
                #Se reestablece el color de la imagen original, pero considerando solo lo que ve la mascara
                res_b = cv2.bitwise_and(image, image, mask= mask_b)
                #Se imprime la imagen resultante
                cv2.imshow('Blue',res_b)
                #Se transforma de cv2 a imgmsg para hacerlo compatible con ROS 
                image_message = self.bridge_object.cv2_to_imgmsg(res_b, encoding="passthrough")
                #Se publica el imgmsg en el topico creado al comienzo
                self.image_pub.publish(image_message)

                cv2.waitKey(1)
                r.sleep()
        cv2.destroyAllWindows()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            print(np.shape(self.cv_image))
        except CvBridgeError as e:
            print(e)
        self.image_received=1

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('Filter_video', anonymous=True)
    showing_image_object = ShowingImage()