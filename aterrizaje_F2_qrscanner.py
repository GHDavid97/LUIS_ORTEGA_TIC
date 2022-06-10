from picamera.array import PiRGBArray
from picamera import PiCamera
from pymavlink import mavutil
from datetime import datetime
from csv import writer
import cv2
import yaml
import numpy as np
import pandas as pd
import pyzbar.pyzbar as pyzbar
import math
import time

CAMERA_PARAMETERS_INPUT_FILE = "cam1.yaml"

with open(CAMERA_PARAMETERS_INPUT_FILE) as f:
    loadeddict = yaml.safe_load(f)
    mtx = loadeddict.get('camera_matrix')
    dist = loadeddict.get('dist_coeff')
    mtx = np.array(mtx)
    mtx_inv = np.linalg.inv(mtx)
    dist = np.array(dist)

def requerir_mensaje(mensaje,intervalo):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)     #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
    print()
    print(msg)
    return

def registrar(namefile,estado): #Registramos nuevas filas al archivo csv del data frame
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    msg=the_connection.messages['LOCAL_POSITION_NED']
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    yaw=msg0.yaw
    i+=1
    DATOS=[i,msg.z,msg.x,msg.y,msg.vx,msg.vy,yaw,estado]
    with open(namefile,"a",newline="") as f:
        wo=writer(f)
        wo.writerow(DATOS)
        f.close()
        return

# the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762') # STIL LOCAL 
# the_connection = mavutil.mavlink_connection('tcp:172.31.69.215:5762') # SITL REMOTO 
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600) # PROTOTIPO

the_connection.wait_heartbeat()

mode_id=the_connection.mode_mapping()['GUIDED']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)


#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_ORTEGA_LUIS/Datos/DATOS_F2/F2_"+day+".csv"

lista=["tiempo","altitud_imu","x","y","vx","vy","yaw","estado"]
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 
i=0

requerir_mensaje(245,1000000)
requerir_mensaje(32,1000000)
requerir_mensaje(30,1000000)

# SCANNEAR QR

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
font = cv2.FONT_HERSHEY_PLAIN
# allow the camera to warmup
time.sleep(0.1)

center=False
tracker=cv2.TrackerCSRT_create()
tracker_init=False
for frame0 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img=frame0.array
    frame=cv2.undistort(img,mtx,dist)
    #frame=cv2.resize(frame, (640,480))
    wimg=int(frame.shape[1]*0.5)
    himg=int(frame.shape[0]*0.5)
    decodedObjects = pyzbar.decode(frame)
    for obj in decodedObjects:
        # print("Data", obj.data)
        cv2.putText(frame, str(obj.data), (50, 50), font, 3,
                    (255, 0, 0), 3)    
    try:
        # print(decodedObjects[0].rect.left)
        if str(obj.data)=="b'HELIPAD'" and tracker_init==False:
            x,y,w,h=decodedObjects[0].rect.left,decodedObjects[0].rect.top,decodedObjects[0].rect.width,decodedObjects[0].rect.height
            cx=int(x+w/2)
            cy=int(y+h/2)
            cv2.putText(frame,"x:"+str(cx-wimg)+",y:"+str(himg-cy),(cx,cy),font,2,(255,0,0),2)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)
            BB=(x,y,w,h)
            tracker.init(frame,BB)
            tracker_init=True
            registrar(namefile,"HELIPAD")
        else:
            print("QR desconocido")
            registrar(namefile,"desconocido")
    except:
        print("SIN HELIPUERTO")
        registrar(namefile,"no existe")
    
    if tracker_init==True:
        track_success,BB=tracker.update(frame)
        if track_success:
            x,y,w,h=BB[0],BB[1],BB[2],BB[3]
            cx=int(x+w/2)
            cy=int(y+h/2)
            cv2.putText(frame, "Trackerx:"+str(cx-wimg)+",Trackery:"+str(himg-cy),(cx,cy),font,3,(255,0,0),3)
                       
            rx=cx-wimg
            ry=himg-cy
            Cimg=(wimg,himg)
            Cqr=(cx,cy)
            # print(math.dist(Cimg,Cqr))
            #CENTRAMOS
            if math.dist(Cimg,Cqr)<20:
                the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                center=True
                tracker_init=False
                # cap.release()
                cv2.destroyAllWindows()
                registrar(namefile,"complete!")
                break
            the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                    the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), ry/200, rx/200, 0, 0, 0, 0, 0, 0, 0, 0, 0)) 
            print("rx: ",rx,"ry: ",ry)
            registrar(namefile,"tracker")

    cv2.imshow("image ",frame)
    rawCapture.truncate(0)
    # if (cv2.waitKey(1) == ord('s')):
    if center == True or (cv2.waitKey(1) == ord('s')):
        break
cv2.destroyAllWindows()

#SET MODE

mode_id=the_connection.mode_mapping()['LOITER']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)