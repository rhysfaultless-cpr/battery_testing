### Setup

1.  Create a workspace and src directory, `~/ros2_ws/src/`
2.  Copy the 2 folders and contents of this repository to the src directory:
    -   `~/ros2_ws/src/numato_relay/`
    -   `~/ros2_ws/src/numato_relay_interfaces/`
3.  Build the workspace:
    ```
    cd ~/ros2_ws
    colcon build
    ```

4.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```

5.  Run the package
    ```
    ros2 run numato_relay service
    ```

<br />

## Sending service on command line:

After starting the package per the steps in Setup:
1.  Open a new terminal.
2.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```
3.  ReService
    ```
    ros2 service call /set_relay numato_relay_interfaces/srv/SetRelay "{relay_channel: 3, relay_state: False}"
    ```
    
    -   Changing the integer beside `relay_channel:` to match which relay you want to control.
    -   Changing the boolean beside `relay_state:` to:
        -  True for closing the relay (_powered_)
        -  False for opening the relay (_unpowered_)

<br />

## Hardware details
-   /dev/ttyACM0
    -   `sudo chmod 777 /dev/ttyACM0`
-   19200 baud

<br />

## Details of the changes
I started from ROS 1 repository [https://github.com/clearpathrobotics/numato_relay_interface].

I needed to make changes related to:
-   New structure for Services in ROS 2
-   Changing from `rospy` to `rclpy`

I decided to continue using the library PySerial `import serial`.
I reviewed [micro-ros-agent](https://github.com/micro-ROS/micro-ROS-Agent) and [eProsima's Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) before deciding on PySerial.
The two other options require changes to the microcontroller's firmware, which is not an option for the Numato USB-relay PCBAs.

I also considered writing this in C++ to make the Service and serial connection faster.
I decided to use Python since I am more familiar with it, and wanted to focus on the ROS 2 implementation (_which is new to me as of October 2023_).

<br />

## Relay's serial hardware:
From running `dmesg`:
```
[ 6814.712823] usb 1-1.1: USB disconnect, device number 9
[ 6816.364636] usb 1-1.1: new full-speed USB device number 10 using ehci-pci
[ 6816.482097] usb 1-1.1: New USB device found, idVendor=2a19, idProduct=0c01, bcdDevice= 1.00
[ 6816.482110] usb 1-1.1: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[ 6816.482115] usb 1-1.1: Product: Numato Lab 4 Channel USB Relay Module
[ 6816.482119] usb 1-1.1: Manufacturer: Numato Systems Pvt. Ltd.
[ 6816.483717] cdc_acm 1-1.1:1.0: ttyACM0: USB ACM device
```

<br />

## Structure of serial messages
-   Using the lines:
    ```
    for i in range(8):
        self.serial_lock.acquire()
        self.serial_port.write(f"relay read {i} \n\r".encode("utf-8"))
        response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
        print(response)
        self.serial_port.flush()
        self.serial_lock.release()
    ```

-   The respoonse is:
    ```
    b'relay read 0 \n\n\roff\n\r>'
    b'relay read 1 \n\n\roff\n\r>'
    b'relay read 2 \n\n\roff\n\r>'
    b'relay read 3 \n\n\roff\n\r>'
    b'relay read 4 \n\n\r>'
    b'relay read 5 \n\n\r>'
    b'relay read 6 \n\n\r>'
    b'relay read 7 \n\n\r>'
    ```

<br />

## References
1.  Install pip for python3: `sudo apt install python3-pip`
2.  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
3.  https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html
4.  https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html 
5.  https://www.youtube.com/watch?v=E_xBPI8SQig
    -   https://roboticsbackend.com/ros2-create-custom-message/
6.  https://numato.com/docs/4-channel-usb-relay-module/#the-command-set-9
