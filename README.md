# ros2_metafly

***This project was first developed in Iron and then modified for being compatible with Humble. Therefore, the Humble version has not been as rigorously tested as the Iron version.***

Portfolio post that uses this repo: [https://adityanairs.website/BirdFlying/](https://adityanairs.website/BirdFlying/)

![](https://github.com/GogiPuttar/adityanairswebsite.github.io/blob/main/assets/images/birdsideview.gif?raw=true)

**REQUIREMENTS:** 
- ROS 2 Humble (for Iron, see the [`ros2/iron`](https://github.com/GogiPuttar/ros2_metafly/tree/ros2/iron) branch).
- ROS 2 Jazzy might also work but has not been rigorously tested. 

To cleanly swap your ROS 2 distribution:
Eg. changing to `humble`
```
unset CMAKE_PREFIX_PATH
unset AMENT_PREFIX_PATH
source /opt/ros/humble/setup.bash
echo $CMAKE_PREFIX_PATH   # should be empty
echo $AMENT_PREFIX_PATH   # should be /opt/ros/humble
echo $ROS_DISTRO          # should be humble
```

## `metafly_control`

Package that handles low level controls communication between the **transmitter module**, **ROS2 network**, and **PS3 Controller**. 

Command for using a PS3 controller to fly the MetaFly:

`ros2 launch metafly_control ps3.launch.xml`

**REQUIREMENTS:** 
- PS3 Controller
- LibSerial on Linux (Eg. `sudo apt install libserial-dev`)
- [`metafly_interfaces`](#metafly_interfaces)

## `metafly_high`

Package that handles high level controls and motion planning.

Command that launches the high level controller and triggers all the relevant launch files for it:

`ros2 launch metafly_high high_level.launch.py policy:=returning use_high_rviz:=true`

where `policy` can be:
- basic
- PID
- switching
- returning
- drift

**REQUIREMENTS:** 
- ROS 2 TF2 Transformations (Eg. `sudo apt install ros-iron-tf*` or `sudo apt install ros-humble-tf*`)
- [`metafly_interfaces`](#metafly_interfaces)

## `metafly_high_cpp` (in progress)
C++ version of the metafly_high package. Currently non-functional.

## `metafly_interfaces`
Houses custom message and service types:
- `metafly_interfaces/Controls` for speed and steering commands to MetaFly

## `metafly_listener`
Package that handles UDP communication between **OptiTrack system** and **ROS2 network**

Command for launching just the listener. Used to test communications:

`ros2 launch metafly_listener listener.launch.py`

**REQUIREMENTS:** 
- ROS 2 TF2 Transformations (Eg. `sudo apt install ros-iron-tf*` or `sudo apt install ros-humble-tf*`)
- [`metafly_interfaces`](#metafly_interfaces)

## `metaflylib` (in progress)
C++ library holding utility functions and classes for `metafly_high_cpp`. Compiles but is a work in progress.

## `arduino`
Arduino scripts that initializes the transmitter module.

**REQUIREMENTS:** 
- MetaFly transmitter module. [(More Info)](https://adityanairs.website/BirdFlying/#:~:text=Transmitter%20%2D%20Feedforward)
- Arduino IDE for uploading code

## `logging`
Useful logging scripts for analyzing data from the system.

Command to run the recording node that stores pose data and controls commands as `.pkl` files:

`ros2 run metafly_high logger`

**REQUIREMENTS:**
- PKL Library (Eg. `pip install python-pkl`)

**REQUIREMENTS:** 
- ROS 2 TF2 Transformations (Eg. `sudo apt install ros-iron-tf*` or `sudo apt install ros-humble-tf*`)
- [`metafly_interfaces`](#metafly_interfaces)

