from pymavlink import mavutil
import time

            # Создаем соединение
master = mavutil.mavlink_connection('udpin:127.0.0.1:14551', baud=57600)

            # Ждем heartbeat
master.wait_heartbeat()
print("Подключение установлено")

            # Кастомный MAVLink message ID (должен совпадать в Lua)
CUSTOM_MSG_ID = 33

def send_custom_data(value1, value2, value3):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        31000,
        0,
        value1,
        value2,
        value3,
        0,0,0,0
    )
    print(f"Отправлено: {value1}, {value2}, {value3}")

                            # Пример отправк
while True:
    send_custom_data (1.23, 4.56, 0)
    time.sleep(3)

