
from pymavlink import mavutil
master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
master.wait_heartbeat()
master.set_mode("QLAND")
#master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
