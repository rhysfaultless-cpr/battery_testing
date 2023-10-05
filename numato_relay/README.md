- Install pip for python3: `sudo apt install python3-pip`
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html
- https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html 
- https://www.youtube.com/watch?v=E_xBPI8SQig
	- https://roboticsbackend.com/ros2-create-custom-message/



## For creating the numato package:

1.  `ros2 pkg create --build-type ament_python numato_relay`

## What I need to do to control the Numato hardware:
-   confirmed that serial messages control the Numato board
    -   /dev/ttyACM0
    -   19200 baud
    -   https://numato.com/docs/4-channel-usb-relay-module/#the-command-set-9
-   Send serial messages
    -   Set the port
    -   Set the baud rate
    -   Test that the port works
    -   Use PySerial as per usual?
        The old Numator driver used PySerial `import serial`.
        It also used `rospy`, which seems obsolete, where ROS 2 now uses `rclpy`.
        `rclpy` = [ROS Client Library for the Python language](https://github.com/ros2/rclpy).
    -   From looking into `rclpy`, it looks like it does not directly support Serial.
        Some people have used PySerial, and ROS 2 publishing in the same python file, but it doesn't seem like the intended implementation of ROS 2.
    -   [micro-ros-agent](https://github.com/micro-ROS/micro-ROS-Agent) is the computer side package for communicating with a micro-ROS microcontroller.
        - This package also mentions using [eProsima's Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent).
        - This uses C and C++.
    -   I am going to use PySerial since it seems simple, and an understandable choice.

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

## Sending service on command line:
Terminal A:
```
ros2 run numato_relay service
```

Terminal B:
```
ros2 service call /set_relay numato_relay_interfaces/srv/SetRelay "{relay_channel: 2, relay_state: True}"
```

- was getting an error, that was resolved by running 
```
sudo chmod 777 /dev/ttyACM0
```