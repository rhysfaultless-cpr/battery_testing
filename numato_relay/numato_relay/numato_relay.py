import serial
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
        self.relay_state_array = []
        
    def set_relay(self, request, response):
        if (request.relay_state):
            relay_state = 'on' # 'on' or 'off' of whether the relay should be Closed or Open ( on = True = Closed = relay powered )
            response.relay_response = True
        else:
            relay_state = 'off'
            response.relay_response = False

        relay_channel = request.relay_channel # Integer starting from 0, aligned with what physical relay to operate
    
        return response   

    def read_relay(self):
        index_count = 0
        while True:
            self.serial_lock.acquire()
            self.serial_port.write(f"relay read {index_count} \n\r".encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
            self.serial_lock.release()
            if 'on' in response:
                print('Adding relay ' + str(index_count) + ' to relay_states as ON')
                self.set_relay_state(index_count, True)
            elif 'off' in response:
                print('Adding relay ' + str(index_count) + ' to relay_states as OFF')
                self.set_relay_state(index_count, False)
            else:
                print('Relay ' + str(index_count) + ' does not exist. Exiting the read_relay loop.')
                break
            index_count += 1
        
    def set_relay_state(self, index, content):
        self.relay_state_array.insert(index, content)

def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    numato_relay.read_relay()
    rclpy.spin(numato_relay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
