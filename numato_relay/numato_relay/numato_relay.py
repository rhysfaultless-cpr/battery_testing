import serial
from threading import Lock
from numato_relay_interfaces.srv import SetRelay
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NumatoRelay(Node):
    def __init__(self):
        super().__init__('NumatoRelay')
        self.service_set_relay = self.create_service(SetRelay, '/set_relay', self.set_relay)
        self.publisher_ = self.create_publisher(String, '/numato_relay_state', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_index = 0

        self.port = '/dev/ttyACM0'
        self.baud = 19200
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()
        self.relay_state_array = []
        

    def set_relay(self, request, response):
        response.relay_response_string = ' '
        response.relay_response_bool = False
        if (type(request.relay_state) is bool):     
            if (type(request.relay_channel) is int):
                if (not (request.relay_channel < 0) ):
                    if ( not(request.relay_channel > (self.get_number_of_relays()-1) ) ):
                        if (not (self.get_relay_state(request.relay_channel) == request.relay_state) ): # Check if the requested relay_channel is already set to the requested relay_state
                            if (request.relay_state):
                                relay_state = 'on' # 'on' or 'off' of whether the relay should be Closed or Open ( on = True = Closed = relay powered )
                                response.relay_response_bool = True
                                response.relay_response_string = 'Relay has been turned on.'
                            else:
                                relay_state = 'off'
                                response.relay_response_bool = False
                                response.relay_response_string = 'Relay has been turned off.'
                            self.change_relay_state(request.relay_channel, request.relay_state)
                            self.serial_lock.acquire()
                            self.serial_port.write(f"relay {relay_state} {request.relay_channel}\n\r".encode("utf-8"))
                            self.serial_port.flush()
                            self.serial_lock.release()
                        else:
                            print('Relay is already set to ' + str(request.relay_state) )
                            response.relay_response_string = 'Relay is already set to ' + str(request.relay_state)
                    else:
                        print('The requested relay_channel is larger than what this Relay PCBA supports')
                        response.relay_response_string = 'The requested relay_channel is larger than what this Relay PCBA supports'
                else:
                    print('The requested relay_channel needs to be a positive Integer')
                    response.relay_response_string = 'The requested relay_channel needs to be a positive Integer'
            else:
                print('The requested relay_channel needs to be an Integer')
                response.relay_response_string = 'The requested relay_channel needs to be an Integer'
        else:
            print('The requested relay_state needs to be an Bool')
            response.relay_response_string = 'The requested relay_state needs to be an Bool'
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


    def change_relay_state(self, index, content):
        self.relay_state_array[index] = content


    def set_relay_state(self, index, content):
        self.relay_state_array.insert(index, content)


    def get_relay_state(self, index):
        return self.relay_state_array[index]


    def get_number_of_relays(self):
        return len(self.relay_state_array)
    
    def timer_callback(self):
        msg = String()
        data_for_message = 'Numato relay states: \n\r'
        for relay_index in range(self.get_number_of_relays()):
            data_for_message += ('  Relay ' + str(relay_index) + "'s state: " + str(self.get_relay_state(relay_index)) + ' \n\r')
        data_for_message += ' \n\r'
        msg.data = data_for_message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.timer_index += 1


def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    numato_relay.read_relay()
    rclpy.spin(numato_relay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
