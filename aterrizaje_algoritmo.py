from picamera.array import PiRGBArray
from picamera import PiCamera
from pymavlink import mavutil
from datetime import datetime
from csv import writer
import pyzbar.pyzbar as pyzbar
import pandas as pd
import numpy as np
import adafruit_lidarlite
import time
import board
import busio
import math
import cv2
import yaml

CAMERA_PARAMETERS_INPUT_FILE = "cam1.yaml"

with open(CAMERA_PARAMETERS_INPUT_FILE) as f:
    loadeddict=yaml.safe_load(f)
    mtx=loadeddict.get("camera_matrix")
    dist=loadeddict.get("dist_coeff")
    mtx=np.array(mtx)
    mtx_inv=np.linalg.inv(mtx)
    dist=np.array(dist)
    
def aterrizar(velocidad_inicial):
    the_connection.mav.param_set_send(the_connection.target_system,the_connection.target_component,b'LAND_SPEED',velocidad_inicial,mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
    msg = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
    print()
    print(msg)

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                         mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print()
    print(msg)

    try:
        ho=sensor.distance*1.0 # ALTURA REAL EN CM AL MOMENTO DE DESCENDER 
    except:
        ho=200 #ALTURA INICIAL DE PRUEBA EN CM
    hf=25 #MEDICION PREVIA DE ALTURA EN CM ENTRE EL SENSOR DE DISTANCIA Y EL SUELO 
    while 1:
        try:
            velocidad=(velocidad_inicial/(ho-hf))*(sensor.distance-hf) #CURVA LINEAL DE VELOCIDAD DE DESCENSO
            h=sensor.distance
            if velocidad <=20:
                velocidad=20
            if velocidad >50:
                velocidad=50
        except:
            velocidad=20
        the_connection.mav.param_set_send(the_connection.target_system,the_connection.target_component,b'LAND_SPEED',velocidad,mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
        registrar(namefile,sensor.distance)
        print("velocidad :", velocidad, " cm/s")
        if velocidad == 20 or h<=hf*1.1:
            break
    return

def mover_verificar(letra,d):
    global puntos
    if letra=="x":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), d, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.x
        if a+d>=0:
            high=1.1
            low=0.9
        else:
            high=0.9
            low=1.1
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            print(d," d ", a," a ",msg.x," msg.x")
            if msg.x<(a+d)*high and msg.x>(a+d)*low:
                print("mov x complete")
                return
            registrar(namefile,sensor.distance)
            try:
                dis=sensor.distance*0.01
            except:
                dis=np.mean(puntos)
            puntos.append(dis)
    if letra=="y":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, d, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.y
        if a+d>=0:
            high=1.15
            low=0.85
        else:
            high=0.85
            low=1.15
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            print(d," d ", a," a ",msg.x," msg.y")
            if msg.y<a+d+0.1 and msg.y>a+d-0.1:
                print("mov y complete")
                return
            registrar(namefile,sensor.distance)
            try:
                dis=sensor.distance*0.01
            except:
                dis=np.mean(puntos)
            puntos.append(dis)

def test_zona(lado_cuadrado):
    global puntos
    print("test zona")
    puntos.clear()
    mover_verificar("x",lado_cuadrado*0.5) 
    mover_verificar("y",lado_cuadrado*0.5)
    mover_verificar("x",-lado_cuadrado)
    mover_verificar("y",-lado_cuadrado)
    mover_verificar("x",lado_cuadrado)
    mover_verificar("y",lado_cuadrado*0.5)
    mover_verificar("x",-lado_cuadrado*0.5)
    desviacion=np.std(puntos)
    print(puntos)
    print(len(puntos)," datos")
    print(desviacion, " std")
    puntos.clear()
    if desviacion<0.16:
    	return True
    else:
        return False 

def requerir_mensaje(mensaje,intervalo):
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,	mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)		#CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
	msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
	print()
	print(msg)
	return

def safe_altitud(alt): 
    contador_error=0
    while 1:
        try:
            altitud=sensor.distance*0.01
            print(altitud, " m")
        except: #error en la lectura del sensor
            contador_error+=1
            if contador_error==10: # al decimo error en la lectura se mueve al frente
                mover_verificar("x",1) #mover 2m al frente
                contador_error=0

        if altitud<=alt+0.1:
            print("ALTITUD SEGURA")
            registrar(namefile,altitud*100) # SE MULTIPLICA POR 100 PARA CONSERVAR EL REGISTRO EN CM DEL SENSOR EN OTROS PROCESOS
            break
        elif altitud>alt+0.1:
            if altitud>alt+1:
                the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component,
	                                                            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111000111),0,0,0,0,0,1,0,0,0,0,0)) #USANDO VELOCIDAD
                registrar(namefile,altitud*100)
            else:
                the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component,
                                                               mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111000000),0,0,0.1,0,0,0.1,0,0,0,0,0)) #USANDO POSICION + VELOCIDAD
                registrar(namefile,altitud*100)
        else:
            break
    return 	

def registrar(namefile,altitud): #Registramos nuevas filas al archivo csv del data frame
    global i
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    msg=the_connection.messages['LOCAL_POSITION_NED']
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    yaw=msg0.yaw
    i+=1
    DATOS=[i,msg.z,altitud,msg.x,msg.y,msg.vx,msg.vy,msg.vz,yaw]
    with open(namefile,"a",newline="") as f:
        wo=writer(f)
        wo.writerow(DATOS)
        f.close()
        return

# the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762') # STIL LOCAL 
# the_connection = mavutil.mavlink_connection('tcp:172.31.69.213:5762') # SITL REMOTO 
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600) # PROTOTIPO

the_connection.wait_heartbeat()

i2c=busio.I2C(board.SCL,board.SDA)
sensor=adafruit_lidarlite.LIDARLite(i2c)
contador_error=0

#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_ORTEGA_LUIS/Datos/DATOS_ALGORITMO/ALGORITMO_"+day+".csv"

lista=["tiempo","altitud_imu","altitud_lidar","x","y","vx","vy","vz","yaw"]
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 
i=0
puntos=[]
requerir_mensaje(245,100000) # EXTENDED_SYS_STATE cada 100ms
requerir_mensaje(32,100000) # LOCAL_POSITION_NED cada 100ms
requerir_mensaje(30,1000000) # ATTITUDE cada 100ms

while 1:
    msg = the_connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True) 
    print(msg.landed_state) #landed_state:(0: undefined)(1:landed on ground)(2:MAV is in air)(3:MAV currently taking off)(4:MAV currently landing)
    if(msg.landed_state==4):
    	break

#SET MODE

# mode_id=the_connection.mode_mapping()['GUIDED']
mode_id=4 # GUIDED
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

#DETENER LANDING

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

safe_altitud(4.0) # argumento: altitud segura en metros

# SCANNEAR QR
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
font = cv2.FONT_HERSHEY_PLAIN

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
            registrar(namefile,sensor.distance)
        else:
            print("QR desconocido")
    except:
        print("SIN HELIPUERTO")
        test=test_zona(2.0)
        if test==True:
            break
        else:
            mover_verificar("x",2)
            tracker_init=False
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
            print(math.dist(Cimg,Cqr))
            #CENTRAMOS
            if math.dist(Cimg,Cqr)<20:
                the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                center=True
                tracker_init=False
                cap.release()
                break
            the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                                    the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), ry/400, rx/400, 0, 0, 0, 0, 0, 0, 0, 0, 0)) 
            print("rx: ",rx,"ry: ",ry)
            try:
                registrar(namefile,sensor.distance)
            except:
                registrar(namefile,0)
        else:
            print("SE PERDIO EL QR")
            tracker_init=False
            
    cv2.imshow("image ",frame)
    rawCapture.truncate(0)
    # if (cv2.waitKey(1) == ord('s')):
    if center == True or (cv2.waitKey(1) == ord('s')):
        break

cv2.destroyAllWindows()
aterrizar(50)

#DESARMAR
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)