from djitellopy import Tello 
import cv2
import numpy as numpy

#Conectar al dron: 'TELLO-F100BB'
drone = Tello()
drone.connect()

#Iniciar camara 
drone.streamoff()
drone.streamon()
capture = cv2.VideoCapture(0)

def main ():
    print ("Main program inicialized")
    #drone.takeoff()
    while True:
        #obraining a new frame --- return,ImageName = capture.read() --- return = 0 no frame recieved
        #Webcam
        ret,img = capture.read()
        #Drone
        frame_read = drone.get_frame_read()
        img = frame_read.frame
        #Resizing the image --cv2.resize('ImageName' , (x_dimension , y_dimension))
        img = cv2.resize(img, (500 , 500))
        #Hsiwing the image in a window
        cv2.imshow("Image" , img)

        print(drone.get_battery())

        #Delay
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break

try:
    main()

except KeyboardInterrupt: 
    print('KeyboardInterrupt exception is caught')
    print ('LANDING')

    cv2.destroyAllWindows()

else:
    print('No exeptions are caught')