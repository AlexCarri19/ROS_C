from djitellopy import Tello 
import cv2
import numpy as numpy
import mediapipe as mp

#drone = Tello()
#drone.connect()
#drone.stream_on()

#Tamaño de la imagen
w , h = 360 , 240

capture = cv2.VideoCapture(0)
mp_face_detect = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

def main ():
    with mp_face_detect.FaceDetection(min_detection_confidence=0.75) as face_detection:
        
        while True:
            ret , img = capture.read()
            if ret == 1:
                img = cv2.resize(img , (w , h))

                cv2.imshow("Tracking" , img)

                key = cv2.waitKey(10) & 0xFF #Valores para usar las teclas asignado a una variable 

                #Despegar y dar valor a la bandera fly 
                if key == ord('l'):
                    #drone.land()
                    print("Finish")
                    break
try:
    main()

except KeyboardInterrupt:
    #Terminar con el programa si se termina desde la terminal o se interrumpe 
    #Apagado de emergencia 
    print('KeyboardInterrupt exception is caught')
    #drone.land()
    print ('ABOART')
    #drone.streamoff()
    cv2.destroyAllWindows()

else:
    print('No exeptions are caught')