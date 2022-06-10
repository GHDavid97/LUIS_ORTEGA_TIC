from pymavlink import mavutil
from datetime import datetime
from csv import writer
import pandas as pd
import time
import math
import adafruit_lidarlite
import board
import busio

## ETAPA FINAL DEL ATERRIZAJE GUIADO: DESCENSO

def requerir_mensaje(mensaje,intervalo):
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,	mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)		#CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
	msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
	print()
	print(msg)
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
# the_connection = mavutil.mavlink_connection('tcp:172.31.69.215:5762') # SITL REMOTO 
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600) # PROTOTIPO

the_connection.wait_heartbeat()

#CSV
now=datetime.now()
day=now.strftime("%d_%m_%Hh%Mm")
namefile="/home/pi/TIC_ORTEGA_LUIS/Datos/DATOS_F4/F4_"+day+".csv"

lista=["tiempo","altitud_imu","altitud_lidar","x","y","vx","vy","vz","yaw"]
df=pd.DataFrame(columns=lista)
df.to_csv(namefile, index=False) #index=False para eliminar la columna unnamed:0 que se crea 
i=0

requerir_mensaje(245,100000) # EXTENDED_SYS_STATE cada 1ms
requerir_mensaje(32,100000) # LOCAL_POSITION_NED cada 1ms
requerir_mensaje(30,1000000)

i2c=busio.I2C(board.SCL,board.SDA)
sensor=adafruit_lidarlite.LIDARLite(i2c)

#SET MODE

mode_id=the_connection.mode_mapping()['GUIDED']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

#DETENER LANDING

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

# DESCENSO

the_connection.mav.param_set_send(the_connection.target_system,the_connection.target_component,b'LAND_SPEED',50,mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
msg = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
print()
print(msg)


the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

#ho=200 # altura a la que empieza el descenso en cm
try:
	ho=sensor.distance*1.0
except:
	ho=200
hf=10 # altura inicial en cm

while 1:
    print(sensor.distance, " cm")
    try:
    	velocidad=(50/(ho-hf))*(sensor.distance-hf) #CURVA LINEAL DE VELOCIDAD
    	# velocidad=(50/(ho**2-hf**2))*(sensor_distance**2-hf**2) #CURVA CUADRATICA DE VELOCIDAD
    except:
    	velocidad=10
    the_connection.mav.param_set_send(the_connection.target_system,the_connection.target_component,b'LAND_SPEED',velocidad,mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
    msg = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
    print()
    print(msg)
    print("velocidad :",velocidad, " cm/s")
    registrar(namefile,sensor.distance)
    if velocidad<=5 and sensor.distance<=hf*1.1:
        break

# DESARMAR 
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)
