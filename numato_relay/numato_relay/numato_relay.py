import serial
#import time
from threading import Lock

from numato_relay_interfaces.srv import SetRelay

import rclpy
from rclpy.node import Node

class NumatoRelay(Node):
    def __init__(self):
        super().__init__('NumatoRelay')
        self.service_set_relay = self.create_service(SetRelay, '/set_relay', self.set_relay)

        self.port = '/dev/ttyACM0'
        self.baud = 19200
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()
        
    def set_relay(self, request, response):
        if (request.relay_state):
            relay_state = 'on' # 'on' or 'off' of whether the relay should be Closed or Open ( on = True = Closed = relay powered )
            response.relay_response = True
        else:
            relay_state = 'off'
            response.relay_response = False
        relay_channel = request.relay_channel # Integer from 0 - 7, aligned with what physical relay to operate
        self.serial_lock.acquire()
        self.serial_port.write(f"relay {relay_state} {relay_channel}\n\r".encode("utf-8"))
        self.serial_port.flush()
        self.serial_lock.release()
        return response   
        
    def read_relay(self):
        for i in range(8):
            self.serial_lock.acquire()
            self.serial_port.write(f"relay read {i} \n\r".encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
            self.serial_lock.release()
            if 'on' in response:
                print('relay ' + str(i) + ': on')
            elif 'off' in response:
                print('relay ' + str(i) + ': off')
            else:
                print('relay ' + str(i) + ': does not exist')

def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    numato_relay.read_relay()
    rclpy.spin(numato_relay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
