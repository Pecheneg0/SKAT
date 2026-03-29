from pymavlink import mavutil
import time


master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
master.wait_heartbeat()

def switch_to_vtol_mode():
    master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0, 3, 0, 0, 0, 0, 0, 0)
    ack = None
    ack = master.recv_match(type = 'MAV_CMD_DO_VTOL_TRANSITION', blocking= True)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Принято")
        else:
            print(f"Отклонено {ack.result}")
switch_to_vtol_mode()
