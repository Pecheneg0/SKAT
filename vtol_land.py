from pymavlink import mavutil
import time
master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
master.wait_heartbeat()
print("Подключн")
def send_land_command():
    msg = master.recv_match(type = 'GLOBAL_POSITION_INT', blocking = True)
    lat = msg.lat / 1e7
    lon = msg.lon /1e7
    alt = msg.alt / 1e7

    print (f"Местоположение : Широта: {lat} | Долгота: {lon} | Высота: {alt} ")

    master.mav.command_long_send(master.target_system, master.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
            0, 0, 0, 0, 0, 
            lat,
            lon, 
            0)

    print ("Команда посадки отправлена")
    ack = None
    ack = master.recv_match(type = 'COMMAND_ACK', blocking = True, timeout = 5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND:
        if ack.result ==mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Принято")
        else:
            print(f"Отлонено {ack.result}")


def get_yaw():
    try: 
        msg = master.recv_match(type = 'ATTITUDE', blocking = True)
        if msg:
            yaw = msg.yaw
            print(f"Курс: {yaw} | Остальное: {msg}")
    except Exception as e:
        print(f"Ошибка {e}")
        


def switch_to_qland_mode():
    master.set_mode("QACRO")
    print("QLAND")
    time.sleep(2)

get_yaw()
switch_to_qland_mode()
send_land_command()

