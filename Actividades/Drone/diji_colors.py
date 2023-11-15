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

H_min , H_max = 0 , 21
S_min , S_max = 173 , 255
V_min , V_max = 123 , 255

initial_HSV = 0
area_min = 4000

init_Hmin , init_Hmax = 0 , 26
init_Smin , init_Smax = 58 , 159
init_Vmin , init_Vmax = 56 , 255

#TrackBars Window
cv2.namedWindow('TrackBars')
cv2.resizeWindow('TrackBars' , 500 , 500)

def trackbar_cb(): pass

cv2.createTrackbar('H_min' , 'TrackBars' , 0 , 180 , trackbar_cb)
cv2.createTrackbar('H_max' , 'TrackBars' , 0 , 180 , trackbar_cb)
cv2.createTrackbar('S_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('S_max' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('V_min' , 'TrackBars' , 0 , 255 , trackbar_cb)
cv2.createTrackbar('V_max' , 'TrackBars' , 0 , 255 , trackbar_cb)

cv2.setTrackbarPos('H_min' , 'TrackBars' , initial_HSV)
cv2.setTrackbarPos('H_max' , 'TrackBars' , 180)
cv2.setTrackbarPos('S_min' , 'TrackBars' , initial_HSV)
cv2.setTrackbarPos('S_max' , 'TrackBars' , 255)
cv2.setTrackbarPos('V_min' , 'TrackBars' , initial_HSV)
cv2.setTrackbarPos('V_max' , 'TrackBars' , 255)

cv2.setTrackbarPos('H_min' , 'TrackBars' , init_Hmin)
cv2.setTrackbarPos('H_max' , 'TrackBars' , init_Hmax)
cv2.setTrackbarPos('S_min' , 'TrackBars' , init_Smin)
cv2.setTrackbarPos('S_max' , 'TrackBars' , init_Smax)
cv2.setTrackbarPos('V_min' , 'TrackBars' , init_Vmin)
cv2.setTrackbarPos('V_max' , 'TrackBars' , init_Vmax)

#hsv_min = np.array([H_min , S_min , V_min])
#hsv_max = np.array([H_max , S_max , V_max])

capture = cv2.VideoCapture(0)

def getContours(img ,img_tracking):
    contours , hierarchy = cv2.findContours (img , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
        if area > area_min:
            per = cv2.arcLength(cnt , True)
            aprox = cv2.approxPolyDP(cnt , 0.02* per , True)
            x , y , w ,h = cv2.boundingRect(aprox)
            cx = int(x + w/2)
            cy = int(y + h/2)

            #mostar informacion 
            cv2.drawContours(img_tracking , cnt , -1 , (255 , 0 , 255))
            cv2.putText(img_tracking, 'cx', (20, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, str(cx), (80, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, 'cy', (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
            cv2.putText(img_tracking, str(cy), (80, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			
			#Control de movimiento
            print("Pos X: %i Pos Y: %i" % (cx ,cy))
        else: 
            direccion = 0
            print("Objeto pequeño")

			#Trazar una linea media
    cv2.line(img_tracking,(250,0),(250,500),(255,255,0),3)
    cv2.line(img_tracking,(0,250),(500,250),(255,255,0),3)


def main():
    print("Main Program")
    while True:
        ret, img = capture.read()
        img = cv2.resize(img ,(500 , 500))
        #Girar la imagen
        img = cv2.flip(img , 1)#flip horizontal

        #Ccambiar escala de colores 
        hsv_img = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)

        #Trackbars

        H_min = cv2.getTrackbarPos('H_min' , 'TrackBars')
        H_max = cv2.getTrackbarPos('H_max' , 'TrackBars')
        S_min = cv2.getTrackbarPos('S_min' , 'TrackBars')
        S_max = cv2.getTrackbarPos('S_max' , 'TrackBars')
        V_min = cv2.getTrackbarPos('V_min' , 'TrackBars')
        V_max = cv2.getTrackbarPos('V_max' , 'TrackBars')

        hsv_min = np.array([H_min , S_min , V_min])
        hsv_max = np.array([H_max , S_max , V_max])

        #Filtro de colores
        mask = cv2.inRange(hsv_img , hsv_min , hsv_max)
        mask = cv2.erode(mask , None , iterations = 3)
        mask = cv2.dilate(mask , None , iterations = 3)
        res = cv2.bitwise_and(img,img,mask = mask)

        #Contornos y la posicion del objeto 
        imgBlur = cv2.GaussianBlur(res , (7 , 7) , 1)
        imgGray = cv2.cvtColor(imgBlur , cv2.COLOR_BGR2GRAY)
        imCanny = cv2.Canny(imgGray , 166 , 171)

        cv2.imshow("Image" , imCanny)
        getContours(imgGray , img)

        

        if cv2.waitKey(1) & 0xFF == ord('l'):
            print('Landing')
            cv2.destroyAllWindows()
            break

try:
    main()

except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    #drone.land()
    print ('LANDING')

    cv2.destroyAllWindows()

else:
    print('No exeptions are caught')