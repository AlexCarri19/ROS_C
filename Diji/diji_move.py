'''Para usar el dron necesitamos la libreria djitellopy
   la cual nos sirve para generarl el objeto Tello que lo 
   identifica como el dron 
   
   Despues tenemos la libreria cv2 la cual es para trabajar
   con ventanas en la computadora y tiene algunos paquetes de 
   colores'''

from djitellopy import Tello 
import cv2
import numpy as numpy

#Conectar al dron: 'TELLO-F100BB'
drone = Tello() #General el objeto 
drone.connect()

#Ventana de la trackbar 
cv2.namedWindow('TrackBar') #Generar una ventana 
cv2.resizeWindow('TrackBar' , 500 , 500) #Dar dimensiones a la ventana 

#Trackbar initial speed y callback
def trackbar_cb(): pass #Callback
initial_speed = 50 
speed = 0
cv2.createTrackbar('Speed' , 'TrackBar' , 0 , 100 , trackbar_cb) #Crear trackbar

#Iniciar Valores de la velocidad 
cv2.setTrackbarPos('Speed' , 'TrackBar' , initial_speed) #Le damos el valor inicial de la velocidad 

#Iniciar camara 
capture = cv2.VideoCapture(0)

#Iniciar camara dron
drone.streamoff()
drone.streamon()

#Inicial velocidades del dron en los atributos del objeto
drone.left_right_velocity = 0
drone.for_back_velocity = 0
drone.up_down_velocity = 0
drone.yaw_velocity = 0



def main ():
    fly = False
    print ("Main program inicialized")
    while True:
        #obraining a new frame --- return,ImageName = capture.read() --- return = 0 no frame recieved
        #Webcam
        ret,img = capture.read()
        #Drone Image 
        frame_read = drone.get_frame_read()
        img = frame_read.frame

        #Obetenemos la posicion del trackbar e incertamos texto en la imagen del dron
        speed = cv2.getTrackbarPos('Speed' , 'TrackBar')#Asignar una variable a nuestra trackbar
        cv2.putText(img , 'Speed' , (0 , 50) , cv2.FONT_HERSHEY_TRIPLEX , 1 , (0 , 255 , 0) , 3) #Texto
        cv2.putText(img , str(speed) , (120 , 50) , cv2.FONT_HERSHEY_TRIPLEX , 1 , (0 , 255 , 0) , 3)

        #Resizing the image --cv2.resize('ImageName' , (x_dimension , y_dimension))
        #Le damos la misma forma que la ventana anterior para mantener un formato 
        img = cv2.resize(img, (500 , 500))

        #Showing the image in a window
        cv2.imshow("Image" , img)

        #Funcion que detecta que el dron este volando mediante "fly"
        #Y hace que el dron use la informacion de sus componentes para dirigirse 
        if fly == True:
            drone.send_rc_control(drone.left_right_velocity, drone.for_back_velocity, drone.up_down_velocity, drone.yaw_velocity)
			

        key = cv2.waitKey(10) & 0xFF #Valores para usar las teclas asignado a una variable 

        #Despegar y dar valor a la bandera fly 
        if key == ord('q'):
            fly = True
            print ("Despegue")
            drone.takeoff()

        #Aterrizar con tecla e
        elif key == ord('e'):
            print("Aterrisaje")
            drone.land()

        #Moverse adelante con tecla w
        elif key == ord('w'):
            drone.for_back_velocity = speed
            print ("Adelante")

        #Moverse atras con tecla d
        elif key == ord('s'):
            drone.for_back_velocity = -speed
            print ("Atras")

        #Moverse deracha con tecla d
        elif key == ord('d'):
            drone.left_right_velocity = speed
            print ("Derecha")

        #Mov izquierda con tecla a
        elif key == ord('a'):
            drone.left_right_velocity = -speed
            print ("Izquierda")

        #Mov arriba con tecla i
        elif key == ord('i'):
            drone.up_down_velocity = speed
            print ("Arriba")

        #Mov abajo con tecla k
        elif key == ord('k'):
            drone.up_down_velocity = -speed
            print ("Abajo")
        
        #Mov rot derecha con tecla l
        elif key == ord('l'):
            drone.yaw_velocity = speed
        
        #Mov rot izquierda con tecla j
        elif key == ord('j'):
            drone.yaw_velocity = -speed

        #Terminar el programa 
        elif key == ord('g'):
            drone.land()
            print ('LANDING')
            drone.streamoff() #Apagar camara del dron
            cv2.destroyAllWindows() #Cerrar ventanas 
            break
        
        #Mantener las velocidades en 0 para que se quede quiero si no recive ordenes 
        else:
            drone.left_right_velocity = 0
            drone.for_back_velocity = 0
            drone.up_down_velocity = 0
            drone.yaw_velocity = 0

        print ("Adelante/Atras: %f Derecha/Izquieda: %f Arriba/Abajo: %f Yaw: %f" % (drone.left_right_velocity , drone.for_back_velocity , drone.up_down_velocity , drone.yaw_velocity))


try:
    main()

except KeyboardInterrupt:
    #Terminar con el programa si se termina desde la terminal o se interrumpe 
    #Apagado de emergencia 
    print('KeyboardInterrupt exception is caught')
    drone.land()
    print ('LANDING')
    drone.streamoff()
    cv2.destroyAllWindows()

else:
    print('No exeptions are caught')