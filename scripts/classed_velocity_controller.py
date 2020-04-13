#!/usr/bin/env python
# -*- coding: utf-8 -*-


# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node
import rospy
import signal
import time
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# velocity control
class Velocity_controller():
    def __init__(self):
        # initialize node
        rospy.init_node('velocity_controller', disable_signals=True)

        # publisher for pwm command
        self.pub_motor_commands = rospy.Publisher(
            'motor_commands', Int16MultiArray, queue_size=1)
        self.motor_command = Int16MultiArray()
        self.motor_command.data.append(0, 0)  # for motor commnad left/right
        # self.motor_command.data.append(0)  # for right motor command

        # subscriber for wheel odometry
        self.sub_odom = rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                                         self.callback_update_odometry, queue_size=1)

        # subscriber for velocity command
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist,
                                            self.callback_update_command, queue_size=1)

        self.linear_vel = 0.0
        self.past_linear_vel = 0.0
        self.err_linear_vel = 0.0
        self.angular_vel = 0.0
        self.past_angular_vel = 0.0
        self.err_angular_vel = 0.0
        self.ODOM_RATE = 100.0

        rospy.spin()

    def callback_update_odometry(self, odometry):
        # store
        self.past_linear_vel = self.linear_vel
        self.past_angular_vel = self.angular_vel
        # update
        self.linear_vel = odometry.twist.twist.linear.x
        self.angular_vel = odometry.twist.twist.angular.z

    def callback_update_command(self, twist):
        cmd_linear = twist.linear.x
        cmd_angular = twist.angular.z
        self.velocity_control(cmd_linear, cmd_angular)

    def velocity_control(self, cmd_linear, cmd_angular):
        # local parameters
        Pgain_LINEAR = 40.0
        Dgain_LINEAR = 0.027
        Igain_LINEAR = 0.0
        Pgain_ANGULAR = 1.0
        Dgain_ANGULAR = 0.0
        Igain_ANGULAR = 0.0
        command_offset = 0.0  # command offset for robot rotation
        motor_command_L = 0  # command for left motor
        motor_command_R = 0  # command for right motor
        dt = 1.0 / self.ODOM_RATE  # cycle length (seconds)
        self.err_linear_vel = self.err_linear_vel + \
            (cmd_linear - self.linear_vel)
        self.err_angular_vel = self.err_angular_vel + \
            (cmd_angular - self.angular_vel)

        # PD control for linear velocity (didn't introduce I control to keep the code simple)
        motor_command_L = Pgain_LINEAR * (cmd_linear - self.linear_vel) - \
            Dgain_LINEAR * (self.linear_vel - self.past_linear_vel) / \
            dt + Igain_LINEAR * self.err_linear_vel
        motor_command_R = -1 * motor_command_L

        # PD control for angular velocity
        command_offset = Pgain_ANGULAR * \
            (cmd_angular - self.angular_vel) - Dgain_ANGULAR * \
            (self.angular_vel - self.past_angular_vel) + \
            Igain_ANGULAR * self.err_angular_vel

        motor_command_L = motor_command_L - command_offset
        motor_command_R = motor_command_R - command_offset

        # regulate pwm command
        motor_command_L = self.regulate_and_shift_command(motor_command_L)
        motor_command_R = self.regulate_and_shift_command(motor_command_R)
        self.publish_command(motor_command_L, motor_command_R)

        self.last_time = time.time()

    def publish_command(self, command_L, command_R):
        self.motor_command.data[0] = command_L
        self.motor_command.data[1] = command_R
        self.pub_motor_commands.publish(self.motor_command)
        pass

    # pwm command regulator
    def regulate_and_shift_command(self, command):
        # maximum output for motor driver : vnh5019. The maximum value is 400
        # according to the driver library, but regulated up to 300 for motor
        # protection drived at 14.8V, higher than nominal voltage : 12V
        MAXIMUM_OUTPUT = 300

        # regulate
        if command > MAXIMUM_OUTPUT:
            command = MAXIMUM_OUTPUT
        elif command < -1 * MAXIMUM_OUTPUT:
            command = -1 * MAXIMUM_OUTPUT

        # shift to ensure it is positive number
        command = command + 300

        return command


if __name__ == '__main__':
    # make class
    vel_ctrl = Velocity_controller()
