# ROS2 Driver for CoDrone Edu

[Japanese Version README](https://github.com/jyamauchi780/CoDroneEdu-ROS2/blob/main/README_ja.md)

## Overview
- This is a package for using CoDrone Edu, developed by Robolink, with ROS2.
- It is developed for ROS2 Humble, and its functionality with other versions has not been tested.
- Please use it with due consideration for safety.
- Disclaimer: This project is not affiliated with, endorsed by, or supported by Robolink. Use at your own risk.

## System Requirements
- Robolink CoDrone Edu
- Ubuntu 22.04
- ROS2 Humble
- Motion capture system (when using a velocity observer)

## Installation
### Installing required Python packages
```sh
    $ sudo apt install ros-humble-tf-transformations
    $ pip install -r requirements.txt
```
### Build
```sh
    $ mkdir -p ~/ros2_ws/src/
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/jyamauchi780/CoDroneEdu-ROS2.git
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ . ~/ros2_ws/install/setup.bash
```

## Fixing the device name of the CoDrone Edu smart controller
Since this package is designed to control multiple CoDrone Edu drones connected to a single Ubuntu computer simultaneously, the device name of each smart controller must be fixed with a unique identifier.
If you are using only one CoDrone Edu and wish to avoid the following settings, modify the launch file as follows:
```python
    'robot_dev', default_value='CoDrone1',
```
Change it to:
```python
    'robot_dev', default_value='/dev/ttyACM0',
```

### How to fix the device name
1. Connect the smart controller to the Ubuntu computer and execute the following command in the terminal:
```sh
    $ sudo dmesg
```
Check the serial number displayed in the "SerialNumber" field.

2. Create a file named `99-seed-serial.rules` in `/etc/udev/rules.d/` and add the following line:
```sh 
    SUBSYSTEM=="tty", ATTRS{serial}=="***", SYMLINK+="CoDrone1", MODE="0666"
```
Replace `***` with the serial number found in step 1 and assign an appropriate name to `SYMLINK+`.
If you set `SYMLINK+="CoDrone1"`, the next step is unnecessary.

3. In the launch file you are using, set the default value of `robot_dev` as follows:
```python
    'robot_dev', default_value='CoDrone1',
```
Replace `CoDrone1` with the name assigned to `SYMLINK+`.

## Demonstration
1. Connect the smart controller to the Ubuntu computer via a Micro USB cable and insert the battery into CoDrone Edu.
2. Execute the following command to operate CoDrone Edu using the smart controller:
```sh
    $ ros2 launch codrone_ros2_driver joy_teleop.launch.py
```
`robot_dev` must be assigned a fixed device name.
If the device name is not fixed or set to `CoDrone1`, this can be omitted.
To use a control system designed to treat CoDrone Edu as an integrator, run the following command:
```sh
    $ ros2 launch codrone_ros2_driver joy_teleop_as_integrator.launch.py
```

## Smart controller key settings
### Joystick
#### Left joystick
- Up/Down: Forward/Backward translation (x-axis)
- Left/Right: Left/Right translation (y-axis)

#### Right joystick
- Up/Down: Up/Down translation (z-axis)
- Left/Right: Yaw rotation

### Directional buttons
- Up: Takeoff
- Down: Landing
- Right: Autonomous flight mode (flying according to `cmd_vel`)
- Left: Smart controller mode

#### Other buttons
- H: Emergency stop
- Power Button: Disconnect from ROS and control CoDrone Edu using only the smart controller


## License
This software is released under the MIT License, see [LICENSE](https://github.com/jyamauchi780/CoDroneEdu-ROS2/blob/main/LICENSE).

