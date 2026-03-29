from pymavlink import mavutil
import math
import logging
import time
class PrecisionLandingSystem:
    def __init__(self):
        self.master = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
        self.master.wait_heartbeat()
        print(f"Подключение установлено {self.master.target_system}")

        self.land_speed = 0.3
        self.min_altitude = 2
        self.pos_tolerance = 0.8
        self.yaw_tolerance = 5.0
        
        self.test_sequence = [
            {"n": 30, "e": 0.15},
            {"n": 0.02, "e": 12},
            {"n": 6, "e": 0.03},
            {"n": -0.21, "e": -0.3},
            {"n": -0.02, "e": -0.2},
            {"n": 0.03, "e": -0.09},
            {"n": 34, "e": 0.06}
        ]
        self.test_index = 0
        self.land_command_sent = False
        

    def set_yaw_north(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, 0, 30, 0, 0, 0, 0, 0)
        print("ориентация на север")

        ack = None
        start_time = time.time()
        while time.time() - start_time < 3:
            ack = self.master.recv_match(type="COMMAND_ACK", blocking = True)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_CONDITION_YAW:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Команда принята ")
                    break
                else :
                    print(f"команда отклонена {ack.result}")
                    return False
                time.sleep(0.1)

    
        start_time = time.time()
        while time.time() - start_time < 15:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking = True)
            heading = msg.hdg / 100.0
            if abs(heading) < self.yaw_tolerance or abs(heading - 360) < self.yaw_tolerance:
                print(f"Ориентация завершена. Курс {heading:.1f}*")
                return True
            time.sleep(0.1)
        print("Предупреждение. Ориентация на север не достигнута")
        return False

    def get_current_altitude(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking = True)
        return msg.relative_alt / 1000
    
    def move_to_offset(self, dx, dy, target_alt=None):
        current_alt = self.get_current_altitude()
    
        if target_alt is None:
            target_alt = current_alt

        self.master.mav.set_position_target_local_ned_send(
            10, 
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b100111111000),
            dx, dy, target_alt,
            0, 0, 0, 0, 0, 0, 0, 0 )

    def land (self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0 )

        print ("Команда посадки отправлена")

    def execute_landing(self): # переписать функцию.
#        if not self.set_yaw_north():
 #           print("Дрон не ориентирован на север")
  #          return 
   #     print ("\n=== Начало процедуры посадки ===")
        
        try:
             while not self.land_command_sent : 
                altitude = self.get_current_altitude()
                print(f"Текущая высота : {altitude:.2f} m")

                test_data = self.test_sequence[self.test_index]
                n_offset = test_data["n"]
                e_offset = test_data["e"]
                
                self.test_index = (self.test_index + 1) % len(self.test_sequence)

               # print(f"Тестовые смещения : Север {n_offset}, Восток : {e_offset}")
                if altitude > self.min_altitude :
                
                    if abs(n_offset) < self.pos_tolerance and abs(e_offset) < self.pos_tolerance:
                        print("Малые смещения, снижение")
                      # altitude = self.get_current_altitude()
                        if altitude > 10:
                            new_alt = 2
                            self.move_to_offset(n_offset, e_offset, new_alt)
                        elif 2 < altitude <= 10 :
                            new_alt = 1
                            self.move_to_offset(n_offset, e_offset, new_alt)

                    else:
                        print(f"Коррекция : Север={n_offset}, восток = {e_offset}")
                        self.move_to_offset(n_offset, e_offset, 0)
                    time.sleep(2)




                else :
                    if abs(n_offset) < self.pos_tolerance and abs(e_offset) < self.pos_tolerance:
                        self.land()
                        self.land_command_sent = True
                    else:
                        print(f"Коррекция : Север={n_offset}, восток = {e_offset}")
                        self.move_to_offset(n_offset, e_offset, 0)


        
        except Exception as e:
            print (f"Критическая ошибка : {str(e)}")
        finally:
            print("Посадка завершена")


if __name__ == "__main__":
    try:

        lander = PrecisionLandingSystem()
        lander.execute_landing()
    except KeyboardInterrupt:
        print("Программа завершене пользователем")
    except Exception as e : 
        print (f"Фатальная ошибка {str(e)}")




