from drones.drone import Drone, State
import json
import socket
import time

import numpy as np
import serial
from scipy.spatial.transform import Rotation as R

import cf_firmware.cflib
from cf_firmware.cflib.crazyflie import Crazyflie
from logger import Logger
from pid import PID


class CrazyFlie(Drone):

    def __init__(self, name, type):
        super().__init__(name, type)

        self.safety_radius = 1000

        print("Init CrazyFlie...")
        self.crazyflie = Crazyflie()
        cf_firmware.cflib.crtp.init_drivers()

        self.crazyflie.open_link("radio://0/80/2M")
        self.crazyflie.connected

        time.sleep(1)
        #self.crazyflie.commander.set_client_xmode(True)
        self.crazy_ready = True

        self.state = State.INITIALIZED

        self.cmd_count = 0

        '''
        self.serial_connection = serial.Serial()
        self.serial_connection.baudrate = 115200
        self.serial_connection.port = "COM3"
        self.serial_connection.open()
        '''

        print("CrazyFlie init!")

        self.start_with_zero()

    def steer(self, yaw, pitch, roll, throttle, drone_position):

        limit_roll_pitch = 20
        gain_roll_pitch = 20

        yaw_min = -30
        yaw_max = 30

        pitch_min = -limit_roll_pitch
        pitch_max = limit_roll_pitch

        roll_min = -limit_roll_pitch
        roll_max = limit_roll_pitch

        throttle_min = 0
        throttle_max = 65000

        # print("pitch:", pitch, "roll:", roll)

        yaw = yaw * 90.0
        pitch = pitch * gain_roll_pitch
        roll = roll * gain_roll_pitch

        hover_throttle = 42500.0  # + 900  # 42500 looks good for hovering
        throttle = (65000.0 - hover_throttle) * throttle + hover_throttle

        yaw = int(np.clip(yaw, a_min=yaw_min, a_max=yaw_max))
        pitch = int(np.clip(pitch, a_min=pitch_min, a_max=pitch_max))
        roll = int(np.clip(roll, a_min=roll_min, a_max=roll_max))
        throttle = int(np.clip(throttle, a_min=throttle_min, a_max=throttle_max))

        #

        # yaw = 0
        # pitch = 0
        # yaw = 0
        # roll = 0

        if drone_position[1] < 250:
            roll = 0  # positive: right
            pitch = 0  # positive: forward
        # throttle = 0

        if self.cmd_count % 10 == 0:

            if self.target_reached:
                # print("CrazyFlie: Disarmed")
                self.crazyflie.commander.send_stop_setpoint()
                pass
            else:
                self.crazyflie.commander.send_setpoint(
                    roll,
                    pitch,
                    yaw,
                    throttle
                )
                self.cmd_count = 0
                #print("CrazyFlie: PID trimmed yaw =", yaw, "pitch =", pitch, "roll =", roll, "throttle =", throttle)

        self.cmd_count += 1

        time.sleep(0.02)

    def start_with_zero(self):
        self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
        print("init with 0000!")
