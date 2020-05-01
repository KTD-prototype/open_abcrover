<!-- # open_abcrover -->
<!-- &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/KTD-prototype/open_abcrover/blob/media/media/rover_overview.jpg" width="480">
<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/KTD-prototype/open_abcrover/blob/media/media/logo.jpg" width="360"> -->
<div align="center">
<img src="https://github.com/KTD-prototype/open_abcrover/blob/media/media/banner.jpg" width="720">
</div>

# Overview
This is an open source, affordable ground platform for remote-operated & autonomous driving experiments.
It is :  
* Simple and Expandable
    * Main chassis made of aluminum extruded beam
    * Cheap and easy-to-by electronics
    * 3D printed brackets and mounts easy to be customized
  
* All terrain driving
    * High power motor and big wheel
    * Moderate size for indoor use, same foot print as domestic robotic vacuums
    * Convertible brackets to boost up wheel base and tread width for stable outdoor locomotion
    
<br>
<br>

# Operability confirmed environments
The open ABC rover is developed and tested conditions below. Of course, you can make any changes and modifications along with your preferences.
## PC environments (both develop and operate)
* Ubuntu 18.04 LTS
  * python2.7.17
  * ROS melodic

## Peripherals
* Main on-board computer : [Jetson Nano Developer Kit(ver.B01)](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
* Sub on-board controller : [Arduino Mega 2560 REV3](https://store.arduino.cc/usa/mega-2560-r3)
* Joy stick : [Logitec F710 wireless gamepad](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.940-000117.html)

## Packages
* wheel odometry for differential-drive wheeled robot : [wheel_odometry](https://github.com/KTD-prototype/wheel_odometry)
* joy signal processing : [joy - ROS Wiki](http://wiki.ros.org/joy)
* velocity command generator from joy : [joy_teleop - ROS Wiki](http://wiki.ros.org/joy_teleop)

<br>
<br>

# Part list, assembly and wiring
Detailed documents are arriving soon!

<br>
<br>

# Install Software
## Install, download and build related packages
Assume your workspace are named as ~/catkin_ws
```
$ cd ~/catkin_ws/src
$ git clone git@github.com:KTD-prototype/open_abcrover.git #main package
$ git clone git@github.com:KTD-prototype/wheel_odometry.git #wheel odometry package
$ sudo apt install ros-melodic-joy ros-melodic-joystick-teleop #additional packages
$ cd ~/catkin_ws
$ catkin_make
```

## Prepare sub-onboard comtroller : arduino mega or equivalents
Pick up codes from the directroy you've downloaded ; .../open_abcrover/scripts/arduino/open_abcrover_control_v1.0  

Upload them to your own board. The main file is named : open_abcrover_control_v1.0.ino

<br>
<br>

# How to use
## Power source
Any types of batteies can be used. In testing, I used two 4-cell Li-po batteries(14.8V at nominal voltage) One for motors, another for on-board computer with DCDC convirsion into 5V via BEC (battery eliminator circuitry)

In my choice, maximum voltage acceptable to motors are 12V. In those case, you have to regulate motor command by your own script.

Be careful to use Li-po or Li-ion batteries, not to over-charge, over-discharge, damage batteries!

<br>

## Check before operation
* Power source connection?
* All peripherals connected? (e.g. USB wireless receiver for your joystick)
* No shorted circuit? Be super-carefule especially around power source wiring such as terminal blocks.
* Wheels and boards are firmly mounted?


<br>

## Launch!
Type as bellow at your terminal.
```
$ roslaunch open_abcrover teleop_sample.launch
```

This file launchs all relative nodes to test default settings for remote operation via your joystick.


<br>

## Discription
### Remote operation
Once you launched as discripted above, you can drive the rover by your joy stick.
By default, you can enable remote operation by button LB, and get the rover move by left joystick. The maximum speed for this mode is around **1 km/h**.

Furthermore, you can enable **TURBO** mode to raise the maximum speed up to **4 km/h** , by pressing A button.

Buttion assignment and labels are according to joystick I used : [Locitech F710](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.940-000117.html) with **X-Input** mode.


### Nodes
* **/joy_node** : Receive signals from joystick and send as commands from an operator
* **/teleop_twist_joy** : Receive commands from an operator and convert into velocity commands. Also regulates maximum gains of the velocity control by those paramters.
* **/velocity_controller** : Receive the velocity commands and convert into motor commands by PID control. Also receives joy command directly to check what operational mode the rover is.
* **/arduino_interface** : Receive the motor commands and send them to arduino. Receive IMU, encoder, battery voltage data and publish them.
* **/wheel_odometry** : Receive encoder data and calculate position, velocity and posture of the rover, and publish them.


### Messages
* **/joy** : Raw commands from an operator
* **/cmd_vel** : Velocity command from an operator, with Twist message.
* **/motor_commands** : Motor comand for each two motors, calculated by PID controller in the velocity_controller node. Regulated not to exceed the motor capacity.
* **/operation_mode** : Indicates whether remote operation is enabled or not, turbo drive is enabled or not, and check joystick connection or other exceptional states of control system.
* **/encoder_2wheel** : Incremented encoder data from two motors.
* **/imu** : Imu data includes posture angle at quaternions, gyros, accelerometers.
* **/wheel_odometry** : Position, velocity and posture calculated by motor encoders.

<br>

### Indicators
The code suppose to have two full-color LEDs on the rover's circuit, connnected to your arduino.
By default, each colors indicates as below:
* Blue : The rover is ready, but not engaged for remote operation.
* Green : Remote operation is engaged, maximum speed is 1 km/h approx.
* Yellow : Remote operation with turbo is engaged, maximum speed is 4 km/h approx.
* White : Autonomous driving is engaged (*planning to implement in future!*)
* Red : Communication link between on-board PC and joystick has lost. (*also for other exceptional states, in future!*)

The two LEDs will blinks when the battery voltages are getting low. By default, blinks every 0.75 seconds under 3.6 V/cell, every 0.25 seconds under 3.375 V/cell.