from pymavlink import mavutil
from datetime import datetime
from csv import writer
import time
import numpy as np
import pandas as pd
import adafruit_lidarlite
import time
import board
import busio
import math

def mover_verificar(letra,d):
    if letra=="x":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), d, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.x 
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            if msg.x<a+d+0.1 and msg.x>a+d-0.1:
                return
    if letra=="y":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, d, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.y
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            if msg.y<a+d+0.1 and msg.y>a+d-0.1:
                return 
def requerir_mensaje(mensaje,intervalo):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)     #CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
    msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
    print()
    print(msg)
    return

def registrar(altitud): #Registramos nuevas filas al archivo csv del data frame
    global i
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    msg=the_connection.messages['LOCAL_POSITION_NED']
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    yaw=msg0.yaw
    i+=1
    DATOS=[i,msg.z,altitud,msg.x,msg.y,msg.vz,yaw]
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
namefile="/home/pi/TIC_ORTEGA_LUIS/Datos/DATOS_F1/F1_"+day+".csv"

lista=["tiempo","altitud_imu","altitud_lidar","x","y","velocidad_z","yaw"]
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 
i=0

requerir_mensaje(245,1000000)
requerir_mensaje(32,1000000)
requerir_mensaje(30,1000000)
# 
# while 1:
#     msg = the_connection.recv_match(type='EXTENDED_SYS_STATE', blocking=True) 
#     print(msg.landed_state) #landed_state:(0: undefined)(1:landed on ground)(2:MAV is in air)(3:MAV currently taking off)(4:MAV currently landing)
#     if(msg.landed_state==4):
#     	break

# mode_id=the_connection.mode_mapping()['GUIDED']
mode_id=4 # GUIDED
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))


# TO SAFE ALTITUD CON SENSOR
while 1:
	try:
		altitud=sensor.distance*0.01
		print(altitud, " m")
	except:
		contador_error+=1
		if contador_error==10: # AL DECIMO ERROR DE LECTURA SE MUEVE AL FRENTE
			mover_verificar("x",1) #mover 2 m al frente
			contador_error=0
			break
	if altitud<=3.1:
		print("ALTITUD SEGURA")
		break
	elif altitud>3.1:
		if altitud>4:
			the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component,
                                                                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111000111),0,0,0,0,0,1,0,0,0,0,0)) #USANDO VELOCIDAD
			registrar(altitud)
		else:
			the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component,
	                                                              mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,int(0b110111000000),0,0,0.1,0,0,0.1,0,0,0,0,0)) #USANDO POSICION + VELOCIDAD
			registrar(altitud)
	else:
		break	

#SET MODE

mode_id=the_connection.mode_mapping()['RTL']
mode_id=6 # RTL

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)
registrar(altitud)