
from pymavlink import mavutil 
import time


print ("Попытка подключиться к SITL ")
master=mavutil.mavlink_connection('udpin:127.0.0.1:14551')
print ('waiting heartbeat')
master.wait_heartbeat()
print("{master.target_system}, {master.target_component}")
while 1:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(msg.lat, msg.lon)
    time.sleep(2)

