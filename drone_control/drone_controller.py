import json

from drones.CrazyFlie import CrazyFlie
from drones.ProxyDrone import ProxyDrone
from drones.drone import serial_connection, Drone, control_drone, WayPoint

import threading

# TODO: this is just a temporary solution
import serial


class DroneController:

    def __init__(self, vinter_receiver):
        self.drones = dict()
        with open('drones.json') as json_file:
            data = json.load(json_file)
            drones = data["drones"]
            for d in drones:

                active = d["active"]

                if active:

                    drone_name = d["name"]
                    drone_type = d["type"]

                    pid_yaw_p = float(d["pid_yaw_p"])
                    pid_yaw_i = float(d["pid_yaw_i"])
                    pid_yaw_d = float(d["pid_yaw_d"])

                    pid_pitch_p = float(d["pid_pitch_p"])
                    pid_pitch_i = float(d["pid_pitch_i"])
                    pid_pitch_d = float(d["pid_pitch_d"])

                    pid_roll_p = float(d["pid_roll_p"])
                    pid_roll_i = float(d["pid_roll_i"])
                    pid_roll_d = float(d["pid_roll_d"])

                    pid_throttle_p = float(d["pid_throttle_p"])
                    pid_throttle_i = float(d["pid_throttle_i"])
                    pid_throttle_d = float(d["pid_throttle_d"])

                    if drone_type == "ProxyDrone":
                        drone = ProxyDrone(drone_name, drone_type)

                    if drone_type == "CrazyFlie":
                        drone = CrazyFlie(drone_name, drone_type)

                    drone.active = d["active"]

                    drone.configure_pid(
                        pid_yaw_p=pid_yaw_p,
                        pid_yaw_i=pid_yaw_i,
                        pid_yaw_d=pid_yaw_d,

                        pid_pitch_p=pid_pitch_p,
                        pid_pitch_i=pid_pitch_i,
                        pid_pitch_d=pid_pitch_d,

                        pid_roll_p=pid_roll_p,
                        pid_roll_i=pid_roll_i,
                        pid_roll_d=pid_roll_d,

                        pid_throttle_p=pid_throttle_p,
                        pid_throttle_i=pid_throttle_i,
                        pid_throttle_d=pid_throttle_d
                    )

                    drone.safety_radius = d["safety_radius"]

                    drone.path = d["path"]
                    with open("flight_paths/" + drone.path) as json_file:
                        path = json.load(json_file)
                        for wp in path:
                            label = wp["label"]
                            x = wp["x"]
                            y = wp["y"]
                            z = wp["z"]

                            checkpoint_radius = wp["checkpoint_radius"]

                            _wp = WayPoint(label, type, x, y, z, checkpoint_radius)

                            try:
                                resting_time = wp["resting_time"]
                                _wp.resting_time = resting_time
                            except KeyError:
                                pass

                            if label == "end":
                                _wp.disarm_height = wp["disarm_height"]

                            drone.way_points.append(_wp)

                    drone.info()
                    self.drones[drone.drone_name] = drone
                    print("Added drone:", drone.drone_name)

        self.vinter_receiver = vinter_receiver

    def start_control_loop(self):  # , app):
        for d in self.drones.values():
            if d.active:
                # d.app = app
                print("starting control loop for:", d.drone_name)
                thread = threading.Thread(target=control_drone, args=(d, self.vinter_receiver))
                thread.start()
