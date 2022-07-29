from pymavlink import mavutil
import pandas as pd
from datetime import datetime
from csv import writer
import time
import math
import numpy as np
import adafruit_lidarlite
import board
import busio

## ETAPA DEL ATERRIZAJE GUIADO: TEST_ZONA
def mover_verificar(letra,d):
    global puntos
    if letra=="x":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), d, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.x 
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            registrar()
            try:
                d=msg.z*1
            except:
                d=np.mean(puntos)
            puntos.append(d)
            if msg.x<a+d+0.1 and msg.x>a+d-0.1:
                return
    if letra=="y":
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, d, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
        a=msg.y
        while 1:
            msg=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
            registrar()
            try:
                d=msg.z*1
            except:
                d=np.mean(puntos)
            puntos.append(d)
            if msg.y<a+d+0.1 and msg.y>a+d-0.1:
                return

def distancia(anterior):
    try:
        distancia=sensor.distance*0.01
        return distancia
    except:
        #ERROR EN LA MEDIDA
        return anterior 

def test_zona(lado_cuadrado):
    global puntos
    puntos.clear()
    mover_verificar("x",lado_cuadrado*0.5)
    puntos.append(distancia(0))  
    mover_verificar("y",lado_cuadrado*0.5)
    puntos.append(distancia(puntos[0]))
    mover_verificar("x",-lado_cuadrado)
    puntos.append(distancia(puntos[1]))
    mover_verificar("y",-lado_cuadrado)
    puntos.append(distancia(puntos[2]))
    mover_verificar("x",lado_cuadrado)
    puntos.append(distancia(puntos[3]))
    mover_verificar("y",lado_cuadrado*0.5)
    puntos.append(distancia(puntos[4]))
    mover_verificar("x",-lado_cuadrado*0.5)
    puntos.append(distancia(puntos[5]))
    desviacion=np.std(puntos)
    print(puntos)
    print(desviacion," std")
    puntos.clear()
    if desviacion<0.12:
        return True
    else:
        return False

def requerir_mensaje(mensaje,intervalo):
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,	mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)		#CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
	msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
	print()
	print(msg)
	return

def registrar(): #Registramos nuevas filas al archivo csv del data frame
    global i
    msg0=the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True)
    msg=the_connection.messages['LOCAL_POSITION_NED']
    msg0=the_connection.recv_match(type="ATTITUDE",blocking=True)
    yaw=msg0.yaw
    i+=1
    DATOS=[i,msg.z,msg.x,msg.y,msg.vx,msg.vy,msg.vz,yaw]
    with open(namefile,"a",newline="") as f:
        wo=writer(f)
        wo.writerow(DATOS)
        f.close()
        return
    
# the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762') # STIL LOCAL 
# the_connection = mavutil.mavlink_connection('tcp:172.31.69.213:5762') # SITL REMOTO 
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600) # PROTOTIPO

the_connection.wait_heartbeat()

#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_ORTEGA_LUIS/Datos/DATOS_F3/F3_"+day+".csv"

lista=["tiempo","altitud_imu","x","y","vx","vy","vz","yaw"]
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 
i=0
puntos=[]
requerir_mensaje(245,100000) # EXTENDED_SYS_STATE cada 1ms
requerir_mensaje(32,100000) # LOCAL_POSITION_NED cada 1ms
requerir_mensaje(30,1000000)

i2c=busio.I2C(board.SCL,board.SDA)
sensor=adafruit_lidarlite.LIDARLite(i2c)

#SET MODE

# mode_id=the_connection.mode_mapping()['GUIDED']
mode_id=4 #GUIDED

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

#TEST

a=test_zona(1.5)
print(a)

#SET MODE

# mode_id=the_connection.mode_mapping()['RTL']
mode_id=6 # RTL
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)