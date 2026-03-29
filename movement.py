
from pymavlink import mavutil
master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
master.wait_heartbeat()

master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b0000111111000), 80, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0))


