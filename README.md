# ros2_metafly

Portfolio post that uses this repo: [https://adityanairs.website/BirdFlying/](https://adityanairs.website/BirdFlying/)

![](https://adityanairs.website/assets/images/birdsideview.gif)


## `metafly_control`

Package that handles low level controls communication between the **transmitter module**, **ROS2 network**, and **PS3 Controller**. 

Command for using a PS3 controller to fly the MetaFly
`ros2 launch metafly_control ps3.launch.xml`

## `metafly_high`

Package that handles high level controls and motion planning.

Command that launches the high level controller and trieggers all the relevant launch files for it.

`ros2 launch metafly_high high_level.launch.py policy:=returning use_high_rviz:=true`

where `policy` can be:
- basic
- PID
- switching
- returning
- drift

## `metafly_high_cpp` (in progress)
C++ version of the metafly_high package. Currently non-functional.

## `metafly_interfaces`
Houses custom message and service types:
- `metafly_interfaces/Controls` for speed and steering commands to MetaFly

## `metafly_listener`
Package that handles UDP communication between **OptiTrack system** and **ROS2 network**

Command for launching just the listener. Used to test communications.

`ros2 launch metafly_listener listener.launch.py`

## `metaflylib` (in progress)
C++ library holding utility functions and classes for `metafly_high_cpp`. Compiles but is a work in progress.

## `arduino`
Arduino scripts that initializes the transmitter module.

## `logging`
Useful logging scripts for analyzing data from the system.

