from pymavlink import mavutil
import time
the_connection = mavutil.mavlink_connection('/dev/serial0',baud=57600)

the_connection.wait_heartbeat()

print(the_connection)
print("Heartbeat from system (system %u component %u)"
	  % (the_connection.target_system, the_connection.target_component))

#SET MODE

mode_id=the_connection.mode_mapping()['GUIDED']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

#ARM AND TAKEOFF

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 3)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

time.sleep(10)

#SET MODE

mode_id=the_connection.mode_mapping()['LOITER']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

time.sleep(10)

#SET MODE

mode_id=the_connection.mode_mapping()['GUIDED']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)
the_connection.set_mode(mode_id)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print()
print(msg)


