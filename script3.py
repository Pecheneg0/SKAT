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
        self.last_rc_value = None
        self.last_wp_value = None
        self.process_LETTERS = None
        self.process_ARUCO = None
        self.no_process = None
        self.last_wp_time = 0
        self.last_rc_time = 0
        self.wp_interval = 3
        self.rc_interval = 0.3

        self.wp_modes = {12: MODE_LETTERS, 6: MODE_ARUCO, 0: MODE_OFF}

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("modeswitcher")
        self.logger.info("initisionalise")

    def wait_heartbeat(self):
        self.logger.info("waiting heartbeat")
        self.master.wait_heartbeat()
        self.logger.info("Подключено к системе")

    def get_current_waypoint(self):
        current_time = time.time()
        if current_time - self.last_wp_time < self.wp_interval:
            return None # Пропускаем запрос
        self.last_wp_time = current_time
        msg = self.master.recv_match(type='MISSION_CURRENT', blocking=True)
       # print("wp")
        return msg.seq if msg else None

    def get_rc_value(self):
        current_time = time.time()
        if current_time - self.last_rc_time < self.rc_interval:
            return None
        self.last_rc_time = time.time()
        msg = self.master.recv_match(type='RC_CHANNELS', blocking=True )
        if msg:
            try:
               # print("rc")
                return getattr(msg, f'chan{RC_CHANNEL}_raw')
            except AttributeError:
                pass
        return None

    def rc_to_mode (self, rc_value):
        if rc_value < 1200: return MODE_OFF
        if rc_value < 1700: return MODE_LETTERS
        return MODE_ARUCO




    def check_mode (self):
        wp = self.get_current_waypoint() 
        rc_val = self.get_rc_value()
        new_mode = None
        if rc_val is not None and rc_val != self.last_rc_value:
            new_mode = self.rc_to_mode(rc_val)
            self.last_rc_value = rc_val
            self.logger.info (f"RC: {rc_val} -> {self.mode_name(new_mode)}")
        elif wp is not None and wp != self.last_wp_value and wp in self.wp_modes:
            new_mode = self.wp_modes[wp]
            self.last_wp_value = wp
            self.logger.info(f"wp : {wp} ->  {self.mode_name(new_mode)}")                                                    
        if new_mode is not None and new_mode !=  self.current_mode :
                                                                        
            self.current_mode = new_mode
           # self.logger.info (f"Активный режим {self.mode_name(new_mode)}")
        return self.current_mode 

    
    def run(self):
        self.wait_heartbeat()
        try:
            while 1:
        
                self.current_mode = self.check_mode()
                if self.current_mode == MODE_OFF and not self.no_process:
                    print("Камеры освобождены, функции остановлены")
                    self.no_process = True
                    self.process_LETTERS = None
                    self.process_ARUCO = None
                elif self.current_mode == MODE_LETTERS and not self.process_LETTERS:
                    print("Камера инициализирована и запущен скрипт Letters")
                    self.no_process = None
                    self.process_LETTERS = True
                    self.process_ARUCO = None
                elif self.current_mode == MODE_ARUCO and not self.process_ARUCO:
                    
                    self.master.mav.command_long_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15, 0, 0, 0, 0, 0)
                    print("Камера инициализирована и запущен скрипт Aruco")
                    self.no_process = None
                    self.process_LETTERS = None
                    self.process_ARUCO = True
                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            print("Завершение работы ")
    def mode_name(self, mode):
        return ["OFF", "LETTERS", "ARUCO"][mode]

if __name__ == "__main__":
    controller = DroneController()
    controller.run()











