from djitellopy import Tello
import cv2
import numpy as np

#Connecting to Tello drone
drone = Tello()
#drone.connect()

#HSV initial values
H_min = 0
H_max = 21
S_min = 173
S_max = 255
V_min = 123
V_max = 255

#Initializing Tello velocities
drone.left_right_velocity = 0 
drone.for_back_velocity = 0 
drone.up_down_velocity = 0
drone.yaw_velocity = 0

area_min = 4000
direccion = 0

hsv_min = np.array([H_min, S_min, V_min])
hsv_max = np.array([H_max, S_max, V_max])

#Camera init
capture = cv2.VideoCapture(0)

def getContours(img,img_tracking):
	contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	global direccion
	for cnt in contours:
		area = cv2.contourArea(cnt)
		print(area)
		if area > area_min:
			per = cv2.arcLength(cnt, True)
			approx = cv2.approxPolyDP(cnt, 0.02 * per, True)
			x , y , w, h = cv2.boundingRect(approx)
			cx = int (x + (w/2))
			cy = int (y + (h/2))

			#Mostrar informacion en imagen
			cv2.drawContours(img_tracking, cnt, -1, (255, 0, 255), 7)
			cv2.putText(img_tracking, 'cx', (20, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			cv2.putText(img_tracking, str(cx), (80, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			cv2.putText(img_tracking, 'cy', (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			cv2.putText(img_tracking, str(cy), (80, 100), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
			
			#Control de movimiento
			if cx < 250:
				direccion = 1
			elif cx > 250:
				direccion = 2
		else: 
			direccion = 0

			#Trazar una linea media
	cv2.line(img_tracking,(250,0),(250,500),(255,255,0),3)
	cv2.line(img_tracking,(0,250),(500,250),(255,255,0),3)

def main():
	print ("Main program")
	while True:
		
		#Obtaining a new frame from webcam --- return,ImageName = capture.read() --- return = 0 no frame recieved
		ret,img = capture.read()

		#Img Format
		#Resizing the image --- cv2.resize('ImageName',(x_dimension,y_dimension))
		img = cv2.resize(img,(500,500))
		
		#Hor flip
		img = cv2.flip(img,1)
		img_tracking = img.copy()

		#Convert to HSV Color Space
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		#Mask color 
		mask = cv2.inRange(hsv_img, hsv_min, hsv_max)
		mask = cv2.erode(mask, None, iterations=3)
		mask = cv2.dilate(mask, None, iterations=3)
		result = cv2.bitwise_and(img,img, mask = mask)

		#Getting contours and object position
		imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
		imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
		imgCanny = cv2.Canny(imgGray, 166, 171)

		getContours(imgGray, img_tracking)

		

		#Moviendo el drone
		if direccion == 1:
			drone.yaw_velocity = 100
			cv2.putText(img_tracking, 'Rotating CW', (20, 400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
		elif direccion == 2:
			drone.yaw_velocity = -100
			cv2.putText(img_tracking, 'Rotating CCW', (20, 400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)
		elif direccion == 0:
			drone.yaw_velocity = 0
			cv2.putText(img_tracking, 'Stop', (20, 400), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),3)

		#Showing the image in a window
		cv2.imshow("Image",img_tracking)

		#Creating a delay and breaking the program
		if cv2.waitKey(1) & 0xFF == ord('l'):
			#Stopping the drone before break
			print("landing")
			#drone.land()
			#drone.streamoff()
			#Clossing the cv2 windows
			cv2.destroyAllWindows()
			break

try:
	main()

except KeyboardInterrupt:
	print('KeyboardInterrupt exception is caught')
	#Stopping the drone before break
	print("landing")
	#drone.land()
	#drone.streamoff()
	#Clossing the cv2 windows
	cv2.destroyAllWindows()

else:
	print('No exceptions are caught')