from pynput import keyboard

from drones import drone

state = 0


class KeyController():

    def __init__(self, drone_controller):
        print("starting key control...")
        self.drone_controller = drone_controller

        try:
            self.drone = self.drone_controller.drones["CrazyFlie"]
        except KeyError:
            print("No CrazyFlie")

        try:
            self.drone = self.drone_controller.drones["PD"]
        except KeyError:
            print("No PD")

        self.state = 3
        self.pid = None
        self.tuning_interval = 0.001

    def on_press(self, key):
        if key == keyboard.Key.esc:
            return False  # stop listener
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
            # self.keys.append(k)  # store it in global-like variable
        print('Key pressed: ' + k)

        if k == 'up':
            throttle = self.drone.throttle + 10
            if throttle > 1900:
                throttle = 1900
            self.drone.throttle = throttle

        if k == 'down':
            throttle = self.drone.throttle - 10
            if throttle < 900:
                throttle = 900
            self.drone.throttle = throttle

        if k == 'left':
            # flieg links
            throttle = self.drone.ppm[1]
            pitch = self.drone.ppm[3]
            roll = self.drone.ppm[0]
            self.drone.ppm[2] -= 10
            yaw = self.drone.ppm[2]
            self.drone.command(throttle, pitch, yaw, roll)
            pass
        if k == 'right':
            # flieg rechts
            throttle = self.drone.ppm[1]
            pitch = self.drone.ppm[3]
            roll = self.drone.ppm[0]
            self.drone.ppm[2] += 10
            yaw = self.drone.ppm[2]
            #self.drone.command(throttle, pitch, yaw, roll)
            pass
        if k == 'a':
            # increase p
            term = self.pid.Kp + self.tuning_interval
            self.pid.setKp(term)
            drone.save_cfg(self.drone)
            print("P:", term)
        if k == 'y':
            # decrease p
            p = self.pid.Kp - self.tuning_interval
            self.pid.setKp(p)
            drone.save_cfg(self.drone)
            print("P:", p)
        if k == 's':
            # increase i
            term = self.pid.Ki + self.tuning_interval
            self.pid.setKi(term)
            drone.save_cfg(self.drone)
            print("I:", term)
        if k == 'x':
            # decrease i
            term = self.pid.Ki - self.tuning_interval
            self.pid.setKi(term)
            drone.save_cfg(self.drone)
            print("I:", term)
        if k == 'd':
            # increase d
            term = self.pid.Kd + self.tuning_interval
            self.pid.setKd(term)
            drone.save_cfg(self.drone)
            print("I:", term)
        if k == 'c':
            # decrease d
            term = self.pid.Kd - self.tuning_interval
            self.pid.setKd(term)
            drone.save_cfg(self.drone)
            print("I:", term)
        if k == 'q':
            print("Pressing target reached")
            self.drone.target_reached = True

        if k == 'k':
            self.drone.set_armed(not self.drone.armed)

    def change_mode(self):
        # 0 = pitch
        # 1 = yaw
        # 2 = roll
        # 3 = throttle

        self.state = (self.state + 1) % 4

        if self.state == 0:
            self.pid = self.drone.pid_pitch
            print("PID: pitch")
        if self.state == 1:
            self.pid = self.drone.pid_yaw
            print("PID: yaw")
        if self.state == 2:
            self.pid = self.drone.pid_roll
            print("PID: roll")
        if self.state == 3:
            self.pid = self.drone.pid_throttle
            print("PID: throttle")

    def on_release(self, key):
        pass

    def start_key_control(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()  # start to listen on a separate thread
