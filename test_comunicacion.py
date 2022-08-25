from pymavlink import mavutil
import time
import cv2

def requerir_mensaje(mensaje,intervalo):
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,	mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mensaje, intervalo, 0, 0, 0, 0, 0)		#CON ESTE COMANDO SE REQUIERE LA INFORMACION DE ALGUN MENSAJE CON FRECUENCIA EN US #245:EXTENDED_SYS_STATUS
	msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
	print()
	print(msg)
	return

# HEARTBEAT
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600) 

the_connection.wait_heartbeat()

print(the_connection)
print("Heartbeat from system (system %u component %u)"
	  % (the_connection.target_system, the_connection.target_component))

quit() # comentar si se desea pprobar el cambio de modo 

requerir_mensaje(245,100000) # EXTENDED_SYS_STATE cada 1ms

#SET MODE
"""
COMMAND_ACK {command : 511, result : 0}
{'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5,
'RTL': 6,'CIRCLE': 7, 'POSITION': 8, 'LAND': 9, 'OF_LOITER': 10, 'DRIFT': 11,
'SPORT': 13, 'FLIP': 14,'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17, 'THROW': 18,
'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21, 'FLOWHOLD': 22, 'FOLLOW': 23,
'ZIGZAG': 24, 'SYSTEMID': 25, 'AUTOROTATE': 26, AUTO_RTL': 27}
"""
# mode_id=the_connection.mode_mapping()['GUIDED']
mode_id=4 #GUIDED=4
print(the_connection.mode_mapping())

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

# DETENER EL UAV EN SU ULTIMA POSICION
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))


input("Press Enter to continue")

# SET MODE
mode_id=6 #RTL

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

print(the_connection.mode_mapping())
