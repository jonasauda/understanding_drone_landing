from drone_controller import DroneController
from input_controller import KeyController
from vinter_receiver import VinterReceiver

vinter_receiver = VinterReceiver()
drone_controller = DroneController(vinter_receiver)
drone_controller.start_control_loop()
key_controller = KeyController(drone_controller)
key_controller.start_key_control()
