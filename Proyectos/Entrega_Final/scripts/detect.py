# Importamos librerias
import torch
import cv2
import numpy as np
import pandas 

# Leemos el modelo
model = torch.hub.load('ultralytics/yolov5', 'custom',
                       path = '/home/alexcarri/catkin_ws/src/final_challenge2/scripts/last.pt')

# Realizo Videocaptura
cap = cv2.VideoCapture(0)

# Empezamos
while True:
    # Realizamos lectura de frames
    ret, frame = cap.read()

    # Correccion de color
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Realizamos las detecciones
    detect = model(frame)

    info = detect.pandas().xyxy[0]  # im1 predictions
    print(info)

    # Mostramos FPS
    cv2.imshow('Detector de señales', np.squeeze(detect.render()))

    # Leemos el teclado
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()