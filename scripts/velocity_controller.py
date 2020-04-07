# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import signal
import time
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# global paramters for velocity control
cont = True
# current velocity
g_linearvel = 0.0  # x
g_angularvel = 0.0  # z
# past velocity at last loop
g_past_linearvel = 0.0  # x
g_past_angularvel = 0.0  # z
# velocity command
g_cmd_linear = 0.0  # x
g_cmd_angular = 0.0  # z
# activation rate of this node
G_RATE = 20  # Hz


# velocity control
def velocity_control(linear_vel, past_linear_vel, angular_vel, past_angular_vel, linear_cmd, angular_cmd):
    global G_RATE

    # local parameters
    Pgain_LINEAR = 5.0
    Dgain_LINEAR = 0.05
    Pgain_ANGULAR = 2.0
    Dgain_ANGULAR = 0.02
    pwm_offset = 0.0  # command offset for robot rotation
    pwm_L = 0  # command for left motor
    pwm_R = 0  # command for right motor
    dt = 1.0 / G_RATE  # cycle length (seconds)

    # PD control for linear velocity (didn't introduce I control to keep the code simple)
    pwm_L = Pgain_LINEAR * (linear_cmd - linear_vel) - \
        Dgain_LINEAR * (linear_vel - past_linear_vel) / dt
    pwm_R = -1 * pwm_L

    # PD control for angular velocity
    pwm_offset = Pgain_ANGULAR * \
        (angular_cmd - angular_vel) - Dgain_ANGULAR * \
        (angular_vel - past_angular_vel)
    pwm_L = pwm_L - pwm_offset
    pwm_R = pwm_R + pwm_offset

    # regulate pwm command
    pwm_L = regulate_pwm(pwm_L)
    pwm_R = regulate_pwm(pwm_R)

    # publish command
    pwm_command.data[0] = pwm_L
    pwm_command.data[1] = pwm_R
    pub_pwm_commands.publish(pwm_command)


# pwm command regulator
def regulate_pwm(pwm):
    if pwm > 127:
        pwm = 127
    elif pwm < -127:
        pwm = -127
    return pwm


# callback : update velocity command
def callback_update_command(twist):
    global g_cmd_linear, g_cmd_angular
    g_cmd_linear = twist.linear.x
    g_cmd_angular = twist.angular.z


# callback : store and update robot's velocity
def callback_update_odometry(odometry):
    global g_linearvel, g_past_linearvel, g_angularvel, g_past_angularvel
    # store
    g_past_linearvel = g_linearvel
    g_past_angularvel = g_angularvel

    # update
    g_linearvel = odometry.twist.twist.linear.x
    g_angularvel = odometry.twist.twist.angular.z


def handler(signal, frame):
    global cont
    cont = False


def velocity_controller_main():
    global cont, G_RATE
    rate = rospy.Rate(G_RATE)

    while cont:
        try:
            velocity_control(g_linearvel, g_past_linearvel,
                             g_angularvel, g_past_angularvel,
                             g_cmd_linear, g_cmd_angular)

        except KeyboardInterrupt:
            cont = False

        rate.sleep()
    rospy.signal_shutdown('finished!')
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('velocity_controller')

    # publisher for pwm command
    pub_pwm_commands = rospy.Publisher(
        'pwm_commands', Int8MultiArray, queue_size=1)
    pwm_command = Int8MultiArray()
    pwm_command.data.append(0)
    pwm_command.data.append(0)

    # subscriber for wheel odometry
    rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                     callback_update_odometry, queue_size=1)

    # subscriber for velocity command
    rospy.Subscriber('cmd_vel', Twist, callback_update_command, queue_size=1)

    # signal handler for KeyboardInterrupt
    signal.signal(signal.SIGINT, handler)

    # main function
    velocity_controller_main()
