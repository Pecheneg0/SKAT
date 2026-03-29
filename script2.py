
from pymavlink import mavutil
import time 
import logging 

MODE_OFF = 0
MODE_LETTERS = 1
MODE_ARUCO = 2
RC_CHANNEL = 6
LOOP_DELAY = 0.1

class DroneController:
    def __init__(self):
        self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
        self.current_mode = MODE_OFF
        self.last_rc_switch_time = 0
        self.last_wp_switch_time = 0
        self.wp_switch_flags = set()
        self.last_rc_value = 0
        self.rc_stable_count = 0

        self.wp_mode_mapping = {
            3: MODE_LETTERS,
            6: MODE_ARUCO,
            10: MODE_OFF
        }
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("modeswitcher")

    def wait_heartbeat(self):
        self.logger.info("Ожидание heartbeat")
        self.master.wait_heartbeat()
        self.logger.info(f"Gодключено к системе {self.master.target_system}")

    def waypoint_current(self):
        msg = self.master.recv_match(type='MISSION_CURRENT', blocking=True)
        return msg.seq if msg else -1
    def check_mission_mode(self):
        current_wp=self.waypoint_current()
        if current_wp == -1:
            return self.current_mode
       # if current_wp == 0:
        #    self.wp_switch_flags.clear()

        if current_wp in self.wp_mode_mapping: # and current_wp not in self.wp_switch_flags:
            new_mode = self.wp_mode_mapping[current_wp]
           # self.wp_switch_flags.add(current_wp)
           # self.last_wp_switch_time = time.time()* 1000
           #self.logger.info(f"Переключение по точке PW{current_wp} -> {new_mode}")
            if new_mode != self.current_mode:
                self.last_wp_switch_time = time.time() * 1000
                self.logger.info(f"Переключение по точке {current_wp} -> {new_mode}")
            return new_mode
        return self.current_mode

    def check_rc_mode(self):
        msg = self.master.recv_match(type='RC_CHANNELS', blocking=True)
        if not msg:
            return self.current_mode

        try:
            rc_value = getattr(msg, f'chan{RC_CHANNEL}_raw')
        except AttributeError:
            return self.current_mode

        if abs(rc_value - self.last_rc_value) < 50 :
            self.rc_stable_count += 1 
        else :
            self.rc_stable_count = 0
            self.last_rc_value = rc_value

        if self.rc_stable_count >= 3:
            if rc_value < 1200:
                new_mode = MODE_OFF
            elif 1200 <= rc_value < 1700:
                new_mode = MODE_LETTERS
            else:
                new_mode = MODE_ARUCO

            if new_mode != self.current_mode:
                self.last_rc_switch_time = time.time() * 1000
                self.logger.info(f"Переключение по RC {rc_value} -> {new_mode}")
                return new_mode
        return self.current_mode

    def determine_mode(self):
        current_time = time.time() * 1000
        rc_mode = self.check_rc_mode()
        wp_mode = self.check_mission_mode()

        rc_is_newer = (current_time - self.last_rc_switch_time) < (current_time - self.last_wp_switch_time)
        return rc_mode if rc_is_newer else wp_mode
        
       # if self.last_rc_switch_time > self.last_wp_switch_time:
        #    return rc_mode
       # else:
        #    return wp_mode

    def run(self):
        self.wait_heartbeat()
        self.last_rc_switch_time = time.time() * 1000
        self.last_wp_switch_time = time.time() * 1000

        try:
            while 1:
                new_mode = self.determine_mode()
                
                if new_mode != self.current_mode:
                    self.current_mode = new_mode 
                    self.logger.info(f"Активный режим : {self.get_mode_name()}")

                time.sleep(LOOP_DELAY)
        except KeyboardInterrupt:
            self.logger.info("Завершение работы")
            print("Завершение работы") 
    def get_mode_name(self):
        return {
            MODE_OFF: "OFF",
            MODE_LETTERS: "LETTERS",
            MODE_ARUCO: "ARUCO"
        }.get(self.current_mode, "UNKNOWN")


if __name__ == "__main__":
    controller = DroneController()
    controller.run()
             


















