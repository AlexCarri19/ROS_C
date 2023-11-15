from djitellopy import Tello
import cv2
import numpy as np

#HSV initial values, remember its a cilinder scale
#HSV es util para el machine vision por la escala y comportamiento 
#H -> 180 = 360°
#S -> 255 = max
#V -> 255 = max

#Para la busqueda de colores lo mejor para usar son los rangos ya que varian mucho 
#en los colores 

#H_min , H_max = 0 , 180
#S_min , S_max = 0 , 255
#V_min , V_max = 0 , 255

#Conectar al dron: 'TELLO-F100BB'
#drone = Tello() #General el objeto 
#drone.connect()

H_min , H_max = 0 , 255
S_min , S_max = 0 , 255
V_min , V_max = 0 , 255

init_Hmin , init_Hmax = 68 , 90
init_Smin , init_Smax = 57 , 255
init_Vmin , init_Vmax = 0 , 255
area_min = 50

#TrackBars Window
cv2.namedWindow('TrackBars')
cv2.resizeWindow('TrackBars' , 500 , 500)

def trackbar_cb(): pass

initial_speed = -1
speed = 0
cv2.createTrackbar('Speed' , 'TrackBars' , -100 , 100 , trackbar_cb) #Crear trackbar
cv2.createTrackbar('H_min' , 'TrackBars' , 0 , 180 , trackbar_cb)
cv2.createTrackbar('H_max' , 'TrackBars' , 0 , 180 , trackbar_cb)
cv2.createTrackbar('S_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('S_max' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('V_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('V_max' , 'TrackBars' , 0 , 255 , trackbar_cb)

cv2.setTrackbarPos('H_min' , 'TrackBars' , init_Hmin)
cv2.setTrackbarPos('H_max' , 'TrackBars' , init_Hmax)
cv2.setTrackbarPos('S_min' , 'TrackBars' , init_Smin)
cv2.setTrackbarPos('S_max' , 'TrackBars' , init_Smax)
cv2.setTrackbarPos('V_min' , 'TrackBars' , init_Vmin)
cv2.setTrackbarPos('V_max' , 'TrackBars' , init_Vmax)
cv2.setTrackbarPos('Speed' , 'TrackBars' , initial_speed)

def getContours(img ,img_tracking):
    #Con esta funcion obtenemos el contorno del objeto, la posicion del centro y el area
    global cx , cy , area
    contours , hierarchy = cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt) #Encontramos el area 
        print(area)
        if area > area_min:
            #En base a la imagen aproximamos la posicion del centro 
            per = cv2.arcLength(cnt , True)
            aprox = cv2.approxPolyDP(cnt , 0.02* per , True)
            x , y , w ,h = cv2.boundingRect(aprox)
            cx = int(x + w/2) #Vertical del objeto 
            cy = int(y + h/2) #Horizontal del centro 

            #mostar informacion 
            cv2.drawContours(img_tracking , cnt , -1 , (255 , 0 , 255) , 2)
            cv2.putText(img_tracking, 'cx', (20, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, str(cx), (80, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, 'cy', (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, str(cy), (80, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, 'Area', (20, 150), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, str(area), (80, 150), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			
			#Control de movimiento
            print("Pos X: %i Pos Y: %i" % (cx ,cy))
        else: 
            print("Objeto pequeño o parametros erroneos")

			#Trazar una linea media


def main():
    #Inicia su recorrido con la bandera y un despegue 
    print("Main Program")
    #drone.takeoff()
    flag = False
    while True:
        #Dentro del ciclo enviamos informacion al dron respecto a sus movimientos y obtenemos los frames 
        #drone.send_rc_control(drone.left_right_velocity, drone.for_back_velocity, drone.up_down_velocity, drone.yaw_velocity)
        #ret, img = capture.read()
        img = cv2.imread('/home/alexcarri/catkin_ws/src/actividades/Mod4/Verde/Semaforo_102.jpg')
        #frame_read = drone.get_frame_read()
        #img = frame_read.frame
        img = cv2.resize(img ,(500 , 500))
        #Girar la imagen
        img = cv2.flip(img , 1)#flip horizontal
        img_tracking = img.copy()

        #Cambiar escala de colores 
        hsv_img = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)

        #Trackbars del filtro HSV
        color = cv2.getTrackbarPos('Speed' , 'TrackBars')
        H_min = cv2.getTrackbarPos('H_min' , 'TrackBars')
        H_max = cv2.getTrackbarPos('H_max' , 'TrackBars')
        S_min = cv2.getTrackbarPos('S_min' , 'TrackBars')
        S_max = cv2.getTrackbarPos('S_max' , 'TrackBars')
        V_min = cv2.getTrackbarPos('V_min' , 'TrackBars')
        V_max = cv2.getTrackbarPos('V_max' , 'TrackBars')

        hsv_min = np.array([H_min , S_min , V_min])
        hsv_max = np.array([H_max , S_max , V_max])

        hsv_min1 = np.array([68 , 57 , 0])
        hsv_max1 = np.array([90 , 255 , 255])

        mask = cv2.inRange(hsv_img , hsv_min , hsv_max)
        contours , _ = cv2.findContours (mask , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img , contours , -1 , (0 , 255 , 0) , 2)

        mask1 = cv2.inRange(hsv_img , hsv_min1 , hsv_max1)
        contours1 , _ = cv2.findContours (mask1 , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img , contours1 , 10 , (0 , 255 , 0) , 2)

        #resultMask = cv2.add(mask , mask1)
        inverse = cv2.bitwise_not(mask)
        resultMask = cv2.add(mask , mask1)

        mask1 = cv2.bitwise_and(img , img , mask = mask1)

        

        imgGray = cv2.cvtColor(mask1 , cv2.COLOR_BGR2GRAY)
        #getContours (imgGray , img)

        cv2.imshow("Frame" , img)
        cv2.imshow("Mask" , resultMask)
        
        
        #Filtro de colores
        #mask = cv2.inRange(hsv_img , hsv_min , hsv_max)
        #mask1 = cv2.erode(mask , None , iterations = 3)
        #mask2 = cv2.dilate(mask1 , None , iterations = 3)
        #res = cv2.bitwise_and(img,img,mask = mask2)

        #Contornos y la posicion del objeto 
        #imgBlur = cv2.GaussianBlur(res , (7 , 7) , 1)
        #imgGray = cv2.cvtColor(imgBlur , cv2.COLOR_BGR2GRAY)
        #imCanny = cv2.Canny(imgGray , 166 , 171)

        #cv2.imshow("Image" , mask)
        #getContours(imgGray , img) 

        if cv2.waitKey(1) & 0xFF == ord('t'): #Bandera para iniciar el tracking 
            flag = True

        if flag:
            if cx < 250:#Movimiento en sentido del reloj en base a la ubicacion horizontal 
                #drone.yaw_velocity = speed 
                cv2.putText(img_tracking, 'Rotating CW', (20, 400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)

            elif cx > 250: #Movimiento contra reloj en base a la ubicacion horizontal 
                #drone.yaw_velocity = -speed 
                cv2.putText(img_tracking, 'Rotating CCW', (20, 400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)

        #Terminar el programa
        if cv2.waitKey(1) & 0xFF == ord('l'):
            flag = False
            #drone.land()
            print('Landing')
            cv2.destroyAllWindows()
            break

try:
    main()

except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    #drone.land()
    print ('LANDING')
    #drone.streamoff()
    cv2.destroyAllWindows()

else:
    print('No exeptions are caught')