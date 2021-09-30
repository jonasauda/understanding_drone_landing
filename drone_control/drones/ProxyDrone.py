from drones.drone import Drone
import time
import numpy as np
import serial
from datetime import datetime

try:

    serial_connection = serial.Serial()
    serial_connection.baudrate = 115200
    serial_connection.timeout = 1
    serial_connection.port = "COM5"
    serial_connection.open()
except serial.serialutil.SerialException:
    print("please plug in COM5 antenna for Proxydrone")

last_cmd_t = 0


def command_serial(drone_name, throttle, yaw, pitch, roll, armed, sleep_time=0.0):

    global last_cmd_t

    ppm_0 = int(roll)
    ppm_1 = int(throttle)
    ppm_2 = int(yaw)
    ppm_3 = int(pitch)

    ppm_4 = 1100

    if armed:
        ppm_4 = 1800  # ARM

    ppm_5 = 1100
    ppm_6 = 1100
    ppm_7 = 1100

    serial_msg = ""
    serial_msg += drone_name + "#"
    serial_msg += str(ppm_0) + "#"
    serial_msg += str(ppm_1) + "#"
    serial_msg += str(ppm_2) + "#"
    serial_msg += str(ppm_3) + "#"
    serial_msg += str(ppm_4) + "#"
    serial_msg += str(ppm_5) + "#"
    serial_msg += str(ppm_6) + "#"
    serial_msg += str(ppm_7) + "\r"
    # print(serial_msg)
    byte_msg = serial_msg.encode()

    # byte_msg = b"HALLO\r"
    # print(byte_msg)
    # print(len(byte_msg))
    t_diff_cmd = datetime.timestamp(datetime.now()) - last_cmd_t
    if t_diff_cmd > 0.05:
        serial_connection.write(byte_msg)
        last_cmd_t = datetime.timestamp(datetime.now())
    # print("SENT!")
    buffer_size = 3
    # ans = serial_connection.read(buffer_size)
    # print("Transceiver response:", ans)
    # time.sleep(sleep_time)


class ProxyDrone(Drone):

    def __init__(self, name, type):
        super().__init__(name, type)
        self.ppm = [1500, 900, 1500, 1500, 1100, 1100, 1100, 1100]

        # Thread this somehow

        self.safety_radius = 1000
        self.armed = False

        self.min_controlling_height = 200

        self.throttle = 900

        '''
        self.serial_connection = serial.Serial()
        self.serial_connection.baudrate = 115200
        self.serial_connection.port = "COM3"
        self.serial_connection.open()
        '''

    def set_armed(self, armed):
        if armed:
            print("Drone:", self.drone_name, "armed!")
            self.ppm[4] = 1800
            self.armed = True
            #self.app.armed.text = "ARMED"
        else:
            print("Drone:", self.drone_name, "disarmed!")
            self.ppm[4] = 1100
            self.armed = False
            #self.app.armed.text = "DISARMED"

    def map_yaw(self, yaw):
        return 400.0 * yaw + 1500.0

    def map_pitch(self, pitch):
        return 400.0 * pitch + 1500.0

    def map_roll(self, roll):
        return 400.0 * roll + 1500.0

    def map_throttle(self, throttle):
        # TODO: maybe if drone is above target the additional term needs to be smaller than hover throttle
        return 500.0 * throttle + 1550.0

    def land(self, drone_height):
        self.ppm[1] = self.ppm[1]-5
        if drone_height < self.min_controlling_height:
            self.armed = False


    def steer(self, yaw, pitch, roll, throttle, drone_pos):
        #print("PPM (un-clamped) yaw =", yaw, "pitch =", pitch, "roll =", roll, "throttle =", throttle)

        # TODO: check if this is bad
        yaw = self.map_yaw(yaw)
        pitch = self.map_pitch(pitch)
        roll = self.map_roll(roll)
        throttle = self.map_throttle(throttle)

        #print("PPM (un-clamped) yaw =", yaw, "pitch =", pitch, "roll =", roll, "throttle =", throttle)

        # TODO: find limit that fit drone

        base = 1500
        _range = 25
        _range_yaw = 180

        yaw_min = base - _range_yaw
        yaw_max = base + _range_yaw

        pitch_min = base - _range
        pitch_max = base + _range
        roll_min = base - _range
        roll_max = base + _range

        throttle_min = 900
        throttle_max = 1650

        yaw = int(np.round(max(yaw_min, min(yaw_max, yaw))))
        pitch = int(np.round(max(pitch_min, min(pitch_max, pitch))))
        roll = int(np.round(max(roll_min, min(roll_max, roll))))
        #throttle = int(np.round(max(throttle_min, min(throttle_max, throttle))))

        # TODO: remove this to fly with automatic throttle
        # throttle = self.throttle

        if drone_pos[1] < self.min_controlling_height:
            yaw = 1500
            pitch = 1500
            roll = 1500
            throttle = self.throttle
            # print("Below controlling height! Current height:", drone_pos[1])

        yaw = 1500
        pitch = 1500
        roll = 1500
        throttle = 1000
        print("PID Adjusted yaw =", yaw, "pitch =", pitch, "roll =", roll, "throttle =", throttle)

        '''
        self.app.pid_throttle.text = "pid_throttle: " + str(throttle)
        self.app.pid_yaw.text = "pid_yaw: " + str(yaw)
        self.app.pid_pitch.text = "pid_pitch: " + str(pitch)
        self.app.pid_roll.text = "pid_roll: " + str(roll)
        

        self.app.direction_forward_backward.text = "dir pitch: backwards" if pitch > 1500 else "dir pitch: forward"
        self.app.direction_side.text = "dir roll: left" if roll > 1500 else "dir roll: right"
        '''

        if self.target_reached:
            self.ppm[4] = 1100

        command_serial(self.drone_name, throttle, yaw, pitch, roll, self.armed)

        # throttle neutral @ 900
        # yaw neutral @ 1500
        # pitch neutral @ 1500
        # roll neutral @ 1500
        delay = 0.05
        delay = 0
        time.sleep(delay)
