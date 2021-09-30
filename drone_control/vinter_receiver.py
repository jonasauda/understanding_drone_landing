import threading
import socket

import model_pb2


class VinterReceiver:

    def rec(self):
        print("Starting vinter receiving thread...")
        self.frame = model_pb2.MocapFrame()
        UDP_PORT = 2020
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('192.168.1.11', UDP_PORT))
        while True:
            data, addr = sock.recvfrom(2048)
            # print(data)
            self.frame.ParseFromString(data)

    def __init__(self):
        self.listen_UDP = threading.Thread(target=self.rec)
        self.listen_UDP.start()

    def get_current_mocap_frame(self):
        return self.frame
