import cv2
import mediapipe as mp
import numpy as np
from djitellopy import tello

#Conexion a la camara del Drone 
drone = tello.Tello()
drone.connect()
drone.streamon()

#capture = cv2.VideoCapture(0)

#Agregamos parametros a todo el codigo 
w, h = 360, 240 #ancho y alto 
fbRange = [6200, 6800] #Rango
pid = [0.4, 0.005, 0.005] #Controlador PI

w2 , h2  , r = int(w/2) , int(h/2) , 50 #Valores de referencia basados en alto y ancho

mp_face_detection = mp.solutions.face_detection #objeto face detection
mp_drawing = mp.solutions.drawing_utils #Objeto para dibujar los puntos 
def main():
    pError = 0
    status = 0
    cx = 0
    cy = 0
    area = 0 
    ud_speed = 30

    with mp_face_detection.FaceDetection(min_detection_confidence=0.85) as face_detection: #Agregamos el objeto para las imagenes del drone 
        while True:
            #ret, img = capture.read()

            img = drone.get_frame_read().frame
            img = cv2.resize(img, (w, h))
            img = cv2.flip(img , 1)#flip horizontal
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = face_detection.process(img) #Con el cambio de color le agregamos el face detection 
    
            if results.detections: #Si hay valores 
                for face_no, detection in enumerate(results.detections): 
                    mp_drawing.draw_detection(img, detection) #Dibujamos la caja 
                    if face_no == 0:
                        xmin = int(detection.location_data.relative_bounding_box.xmin * w)
                        ymin = int(detection.location_data.relative_bounding_box.ymin * h)
                        xd = int(detection.location_data.relative_bounding_box.width * w)
                        yd = int(detection.location_data.relative_bounding_box.height * h)
                        cx, cy, area = (xmin + (xd / 2), ymin + (yd / 2), xd * yd) #obtenemos los valores para las condicionales 
                        cv2.putText(img, "Tracking", (xmin, ymin), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
                #cv2.line(img,(0,h2+r),(w,h2 + r),(w2 + 5,h2 + 5,0),3)
                #cv2.line(img,(0,h2-r),(w,h2 - r),(w2 + 5,h2 + 5,0),3)
            fb_speed = 0
            error = cx - w // 2
            yaw_speed = pid[0] * error + pid[1] * (error - pError) #Controlador PI
            yaw_speed = int(np.clip(yaw_speed, -100, 100))
            if area > fbRange[0] and area < fbRange[1]: #Si esta en el area se queda estatico 
                fb_speed = 0
            elif area > fbRange[1]: #Si se acerca mucho (calculando con el area) se aleja
                fb_speed = -20
            elif area < fbRange[0] and area != 0: #Si se aleja demasiado se acerca 
                fb_speed = 20
            if cx == 0: #Si esta centrado se queda quieto 
                yaw_speed = 0
                error = 0
            if cy <= (h2 - r): #Si en las coordenadas verticales sube de mas el dron sube 
                ud_speed = 40
                print("up")
            elif cy >= (h2 + r): #Si el drone esta más abajo el dron baja 
                ud_speed = -40
                print("down")
            elif cy >= (h2 - r) and cy <= (h2 + r): #Si se mantiene en medio solo imprime su bandera y se queda en posicion 
                ud_speed = 0
                print("mid")

            cv2.imshow("Output", img)
            if (cv2.waitKey(10) == ord('t')): #Orden de despeque 
                if status == 0:
                    status = 1
                    drone.takeoff()
            if (cv2.waitKey(10) == ord('l')): #Orden de aterrizaje
                if status == 1:
                    status = 0
                    drone.land()

            if (cv2.waitKey(20) == ord('r')): #Subida manual
                ud_speed = 40
            elif (cv2.waitKey(20) == ord('f')): #Bajada manual
                ud_speed = -40
            else:
                ud_speed = 0

            #print("u/d speed: " + str(ud_speed), "yaw_speed: " + str(yaw_speed), "f/b speed: " + str(fb_speed),
                  #"Status: " + str(status))
            #print("u/d speed: " + str(ud_speed), "yaw_speed: " + str(yaw_speed), "f/b speed: " + str(fb_speed), "Status: " + str(status), "Battery: " + str(drone.get_battery()))

            if status == 1:
                drone.send_rc_control(0, fb_speed, ud_speed, yaw_speed) #Envio de las velocidades 

            if (cv2.waitKey(10) == ord('q')): #Terminar el programa
                cv2.destroyAllWindows()
                drone.streamoff()
                drone.land()
                break
try:
    main()
except KeyboardInterrupt: #Terminar el programa en emergencias 
    print('KeyboardInterrupt exception is caught')
    cv2.destroyAllWindows()
    drone.streamoff()
    drone.land()
else:
    print('No exceptions are caught')