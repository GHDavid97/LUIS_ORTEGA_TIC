from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import yaml
import numpy as np

CAMERA_PARAMETERS_INPUT_FILE = "cam1.yaml"
Kernel= np.ones((5,5), np.uint8)

with open(CAMERA_PARAMETERS_INPUT_FILE) as f:
    loadeddict = yaml.safe_load(f)
    mtx = loadeddict.get('camera_matrix')
    dist = loadeddict.get('dist_coeff')
    mtx = np.array(mtx)
    mtx_inv = np.linalg.inv(mtx)
    dist = np.array(dist)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    
    dst=cv2.undistort(img,mtx,dist)
    # show the frame
    # Convertimos en escala de grises
    gris = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
    # Aplicar suavizado Gaussiano
    gris = cv2.GaussianBlur(gris, (9,9),0)
    
    # Detectamos los bordes con Canny
    canny = cv2.Canny(gris, 20, 70)
    
    #dilaracion
    #dilatacion = cv2.dilate(canny, None, iterations=2)
    dilatacion= cv2.dilate(canny, Kernel, iterations=2 )
    cv2.imshow('webCam', dilatacion)
    # Buscamos los contornos
    (contornos,jerarquia) = cv2.findContours(dilatacion.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #dibujamos rectangulo en los contornos
    for c in range(len(contornos)):
        cnt= contornos[c]
        
        if cv2.contourArea(contornos[c]) < 1500: # Eliminamos los contornos más pequeños
            continue
        #Calculamos el centroide del contorno
        M = cv2.moments(cnt)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        cv2.circle(dst,(cx,cy),5,(0,0,255),-1)
        # Obtenemos el bounds del contorno, el rectángulo mayor que engloba al contorn
        (x, y, w, h) = cv2.boundingRect(cnt)
        # Dibujamos el rectángulo del bounds
        cv2.rectangle(dst, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    
    cv2.imshow("frame",dst)
    
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
cv2.destroyAllWindows()