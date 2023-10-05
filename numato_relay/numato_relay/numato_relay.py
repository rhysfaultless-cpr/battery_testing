import serial
#import time
from threading import Lock

from numato_relay_interfaces.srv import SetRelay

import rclpy
from rclpy.node import Node

class NumatoRelay(Node):
    def __init__(self):
        super().__init__('NumatoRelay')
        self.service_set_relay_0_on = self.create_service(SetRelay, '/set_relay_0_on', self.set_relay_0_on)
        self.service_set_relay_0_off = self.create_service(SetRelay, '/set_relay_0_off', self.set_relay_0_off)
        
        self.port = '/dev/ttyACM0'
        self.baud = 19200
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()
        

    def set_relay_0_on(self, request, response):
        if (request.relay_request):
            self.serial_lock.acquire()
            self.serial_port.write(f"relay on 0\n\r".encode("utf-8"))
            self.serial_port.flush()
            self.serial_lock.release()
            response.relay_response = True
        else:
            response.relay_response = False
        return response

    def set_relay_0_off(self, request, response):
        if (request.relay_request):
            self.serial_lock.acquire()
            self.serial_port.write(f"relay off 0\n\r".encode("utf-8"))
            self.serial_port.flush()
            self.serial_lock.release()
            response.relay_response = True
        else:
            response.relay_response = False
        return response

def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    rclpy.spin(numato_relay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
