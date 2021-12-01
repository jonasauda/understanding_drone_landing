import json
import socket
import time
from abc import abstractmethod
from datetime import datetime
import numpy as np
import serial
from scipy.spatial.transform import Rotation as R

import cf_firmware.cflib
from cf_firmware.cflib.crazyflie import Crazyflie
from logger import Logger
from pid import PID

serial_connection = serial.Serial()


class State:
    INIT = 0
    INITIALIZED = 1
    STARTING = 2
    FLOATING = 3
    LANDING = 4
    LANDING_COMPLETED = 5


class WayPoint:

    def __init__(self, label, type, x, y, z, checkpoint_radius):
        self.label = label
        self.type = type
        self.x = x
        self.y = y
        self.z = z
        self.checkpoint_radius = checkpoint_radius
        self.disarm_height = -1
        self.resting_time = 0.0


class Drone:

    def __init__(self, name, type):
        self.state = State.INIT
        self.id = 0
        self.drone_name = name
        self.type = type

        # Thread this somehow
        self.pid_yaw = PID()
        self.pid_pitch = PID()
        self.pid_roll = PID()
        self.pid_throttle = PID()

        self.safety_radius = 1000
        self.armed = False
        self.warm_up = 25

        self.min_controlling_height = 80 * 1

        self.state = State.INITIALIZED

        self.path = ""

        self.way_point_index = 0
        self.way_points = []

        self.target_reached = False
        self.resting_time = 0.0


    def configure_pid(self, pid_yaw_p, pid_yaw_i, pid_yaw_d, pid_pitch_p, pid_pitch_i, pid_pitch_d, pid_roll_p,
                      pid_roll_i, pid_roll_d, pid_throttle_p, pid_throttle_i, pid_throttle_d):
        self.pid_yaw.setKp(pid_yaw_p)
        self.pid_yaw.setKi(pid_yaw_i)
        self.pid_yaw.setKd(pid_yaw_d)

        print("yaw PID:", pid_yaw_p, pid_yaw_i, pid_yaw_d)

        self.pid_pitch.setKp(pid_pitch_p)
        self.pid_pitch.setKi(pid_pitch_i)
        self.pid_pitch.setKd(pid_pitch_d)

        self.pid_roll.setKp(pid_roll_p)
        self.pid_roll.setKi(pid_roll_i)
        self.pid_roll.setKd(pid_roll_d)

        self.pid_throttle.setKp(pid_throttle_p)
        self.pid_throttle.setKi(pid_throttle_i)
        self.pid_throttle.setKd(pid_throttle_d)

    def info(self):
        print("Drone:", self.drone_name)
        print("Type:", self.type)
        print("State:", self.state)
        print("Way points defined: ", len(self.way_points))


def calculate_yaw_pitch_roll(drone, target_reached=False):
    # print("Position:", drone.drone_position)
    # print("Rotation:", drone.drone_rotation)

    drone_position = drone.drone_position
    drone_rotation = drone.drone_rotation

    # drone.app.gui_drone_pos.text = "gui_drone_pos: " + str(drone_position)
    # drone.app.gui_drone_rot.text = "gui_drone_rot: " + str(drone_rotation)

    target_position = drone.target_position
    target_rotation = drone.target_rotation

    # drone.app.gui_target_pos.text = "gui_target_pos: " + str(target_position)
    # drone.app.gui_target_rot.text = "gui_target_rot: " + str(target_rotation)

    # print("Drone pos:", drone_position)
    # print("Target pos:", target_position)

    print("Drone rot:", drone_rotation)
    print("Target rot:", target_rotation)

    # print("Drone status:", drone.state)

    # Converting mm to m

    drone_x = drone_position[0] / 1000.0
    drone_y = drone_position[1] / 1000.0
    drone_z = drone_position[2] / 1000.0

    target_x = target_position[0] / 1000.0
    target_y = target_position[1] / 1000.0
    target_z = target_position[2] / 1000.0

    '''
    drone_x = np.clip(drone_x, a_min=-2, a_max=2)
    drone_y = np.clip(drone_y, a_min=0, a_max=4)
    drone_z = np.clip(drone_z, a_min=-2, a_max=2)

    target_x = np.clip(target_x, a_min=-2, a_max=2)
    target_y = np.clip(target_y, a_min=0, a_max=4)
    target_z = np.clip(target_z, a_min=-2, a_max=2)
    '''

    # Normalizing to -1, 1

    # drone_rot_x = drone_rotation[0] / 180
    drone_rot_y = drone_rotation[1] / 180.0
    # drone_rot_z = drone_rotation[2] / 180

    # target_rot_x = target_rotation[0] / 180
    target_rot_y = target_rotation[1] / 180.0
    # target_rot_z = target_rotation[2] / 180

    # Rotation target vector

    alpha = drone_rotation[1]
    # print("alpha =", alpha)

    alpha_rad = np.deg2rad(alpha)
    # print("alpha_rad =", alpha_rad)

    # Rotate everything back
    r = R.from_rotvec([0.0, -alpha_rad, 0.0])  # check if sign is correct

    target = [
        target_x,
        target_y,
        target_z
    ]

    drone_pos = [
        drone_x,
        drone_y,
        drone_z
    ]

    # print("target =", target)
    target_rotated = np.round(r.apply(target), decimals=7)
    # print("target_rotated (rot)=", target_rotated)

    # print("drone_pos =", drone_pos)
    drone_rotated = np.round(r.apply(drone_pos), decimals=7)
    # print("drone_rotated (rot)=", drone_rotated)

    # drone.app.drone_rotated.text = "drone_rotated: " + str(drone_rotated)

    target_rotated_x = target_rotated[0]
    target_rotated_z = target_rotated[2]

    drone_rotated_x = drone_rotated[0]
    drone_rotated_z = drone_rotated[2]

    drone.pid_yaw.SetPoint = target_rot_y
    drone.pid_yaw.update(feedback_value=drone_rot_y)

    drone.pid_pitch.SetPoint = target_rotated_x
    drone.pid_pitch.update(feedback_value=drone_rotated_x)

    drone.pid_roll.SetPoint = target_rotated_z
    drone.pid_roll.update(feedback_value=drone_rotated_z)

    drone.pid_throttle.SetPoint = target_y
    drone.pid_throttle.update(feedback_value=drone_y)

    yaw = drone.pid_yaw.output

    rot_diff = abs(target_rot_y - drone_rot_y)

    if rot_diff > 1.0:
        yaw *= -1.0

    pitch = drone.pid_pitch.output
    roll = drone.pid_roll.output
    throttle = drone.pid_throttle.output

    # TODO: check if this is bad
    yaw = np.clip(yaw, a_min=-1, a_max=1)
    pitch = np.clip(pitch, a_min=-1, a_max=1)
    roll = np.clip(roll, a_min=-1, a_max=1)
    throttle = np.clip(throttle, a_min=0, a_max=1)

    # print("PID [-1,0,1] yaw =", yaw, "pitch =", pitch, "roll =", roll, "throttle =", throttle)

    drone.steer(yaw, pitch, roll, throttle, drone_position)


def save_cfg(self):
    # print("saving Drone & PID values...")
    with open('./drones.json') as json_file:
        data = json.load(json_file)
        drones = data["drones"]
        d = dict()
        d["name"] = self.drone_name
        d["type"] = self.type
        d["pid_yaw_p"] = self.pid_yaw.getKp()
        d["pid_yaw_i"] = self.pid_yaw.getKi()
        d["pid_yaw_d"] = self.pid_yaw.getKd()
        d["pid_pitch_p"] = self.pid_pitch.getKp()
        d["pid_pitch_i"] = self.pid_pitch.getKi()
        d["pid_pitch_d"] = self.pid_pitch.getKd()
        d["pid_roll_p"] = self.pid_roll.getKp()
        d["pid_roll_i"] = self.pid_roll.getKi()
        d["pid_roll_d"] = self.pid_roll.getKd()
        d["pid_throttle_p"] = self.pid_throttle.getKp()
        d["pid_throttle_i"] = self.pid_throttle.getKi()
        d["pid_throttle_d"] = self.pid_throttle.getKd()
        d["safety_radius"] = self.safety_radius
        for i in range(len(drones)):
            if drones[i]["name"] == d["name"]:
                drones[i] = d

        new_data = {"drones": drones}
        new_json = json.dumps(new_data)
        with open('../drones.json', 'w') as f:
            f.write(new_json)
        # print("...saved!")


def control_drone(drone, vinter_receiver):
    drone.state = State.STARTING
    logger = Logger()
    drone.drone_position = None
    drone.drone_rotation = None

    drone.target_position = None
    drone.target_rotation = None

    print("Control Loop: drone:", drone.drone_name)

    loop_count = 0

    while True:

        mocap_frame = vinter_receiver.get_current_mocap_frame()

        if mocap_frame is not None:

            way_point = drone.way_points[drone.way_point_index]

            drone.target_rotation = [0, 0, 0]
            drone.target_position = [
                way_point.x,
                way_point.y,
                way_point.z
            ]

            for body in mocap_frame.bodies:

                centroid = [body.Centroid.X,
                            body.Centroid.Y,
                            body.Centroid.Z]

                rotation = [body.Rotation.X,
                            body.Rotation.Y,
                            body.Rotation.Z,
                            body.Rotation.W]

                if body.Name == drone.drone_name:
                    drone.drone_position = np.array(centroid)  # - [0, 200, 0]  # HACK
                    converted_rot = R.from_quat(rotation).as_euler('yxz', degrees=True)
                    drone.drone_rotation = [converted_rot[1], converted_rot[0], converted_rot[2]]
                '''
                if body.Name in ["PD_Hand", "PD_Backpack"] and way_point.label == "end":
                    drone.target_position = np.array(centroid) + [0, 50, 0]
                    rot = R.from_quat(rotation).as_euler('yxz', degrees=True)
                    drone.target_rotation = [rot[1], rot[0], rot[2]]

                if body.Name == "PD_Debug_Target":
                    drone.target_position = np.array(centroid) + [0, 1500, 0]
                    rot = R.from_quat(rotation).as_euler('yxz', degrees=True)
                    drone.target_rotation = [rot[1], rot[0], rot[2]]
                '''

            if drone.drone_position is not None and drone.target_position is not None and way_point.label != "end":
                dist = np.linalg.norm(np.array(drone.target_position) - np.array(drone.drone_position))
                # print("dist:", dist)
                if dist <= drone.way_points[drone.way_point_index].checkpoint_radius and drone.resting_time == 0.0:
                    drone.way_point_index += 1
                    if drone.way_point_index == len(drone.way_points):
                        drone.way_point_index = 0
                    print(drone.target_position)
                    print("Next waypoint:")
                    print(drone.way_points[drone.way_point_index].x,
                          drone.way_points[drone.way_point_index].y,
                          drone.way_points[drone.way_point_index].z)

                    drone.resting_time = drone.way_points[drone.way_point_index].resting_time

                drone.resting_time -= 1

                if drone.resting_time < 0.0:
                    drone.resting_time = 0.0

                #print("resting_time:", drone.resting_time)

            if drone.target_position is not None and drone.drone_position is not None:
                dir_target = drone.target_position - drone.drone_position
                # print("Dir target:", dir_target)
                dist = np.linalg.norm(dir_target)
                # print("dist:", dist)
                if dist <= way_point.checkpoint_radius and way_point.label == "end":

                    drone.way_points[drone.way_point_index].y -= 3
                    if drone.way_points[drone.way_point_index].y < way_point.disarm_height:
                        drone.target_reached = True
                        print("target reached!")

                if dist > drone.safety_radius:
                    # print("Flying towards target")
                    calculate_yaw_pitch_roll(drone, target_reached=False)
                else:
                    # print("Safety radius reached")
                    calculate_yaw_pitch_roll(drone, target_reached=True)
                '''
                logger.log_frame(drone.target_position[0], drone.target_position[1], drone.target_position[2],
                                 # target_pos
                                 drone.target_rotation[0], drone.target_rotation[1], drone.target_rotation[2],
                                 # target_rot
                                 drone.drone_name,  # drone name
                                 drone.drone_position[0], drone.drone_position[1], drone.drone_position[2],  # drone_pos
                                 drone.drone_rotation[0], drone.drone_rotation[1], drone.drone_rotation[2],  # drone_rot
                                 )
                '''
                if loop_count % 250 == 0:
                    print("way point label:", way_point.label)
                    print("way point (x,y,z):", way_point.x, way_point.y, way_point.z)
                    print("DIST:", dist)
                    print("drone pos:", drone.drone_position)
                    print("drone target:", drone.target_position)

                    loop_count = 0

                loop_count += 1
