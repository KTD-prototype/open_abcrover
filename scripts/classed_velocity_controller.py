#!/usr/bin/env python
# -*- coding: utf-8 -*-


# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node
import rospy
import signal
import time
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# velocity control
class Velocity_controller():
    def __init__(self):
        # initialize node
        rospy.init_node('velocity_controller', disable_signals=True)

        # publisher for pwm command
        self.pub_pwm_commands = rospy.Publisher(
            'pwm_commands', Int8MultiArray, queue_size=1)
        self.pwm_command = Int8MultiArray()
        self.pwm_command.data.append(0)  # for left motor commnad
        self.pwm_command.data.append(0)  # for right motor command

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
        pwm_offset = 0.0  # command offset for robot rotation
        pwm_L = 0  # command for left motor
        pwm_R = 0  # command for right motor
        dt = 1.0 / self.ODOM_RATE  # cycle length (seconds)
        self.err_linear_vel = self.err_linear_vel + \
            (cmd_linear - self.linear_vel)
        self.err_angular_vel = self.err_angular_vel + \
            (cmd_angular - self.angular_vel)

        # PD control for linear velocity (didn't introduce I control to keep the code simple)
        pwm_L = Pgain_LINEAR * (cmd_linear - self.linear_vel) - \
            Dgain_LINEAR * (self.linear_vel - self.past_linear_vel) / \
            dt + Igain_LINEAR * self.err_linear_vel
        pwm_R = -1 * pwm_L

        # PD control for angular velocity
        pwm_offset = Pgain_ANGULAR * \
            (cmd_angular - self.angular_vel) - Dgain_ANGULAR * \
            (self.angular_vel - self.past_angular_vel) + \
            Igain_ANGULAR * self.err_angular_vel

        pwm_L = pwm_L - pwm_offset
        pwm_R = pwm_R - pwm_offset

        # regulate pwm command
        pwm_L = self.regulate_pwm(pwm_L)
        pwm_R = self.regulate_pwm(pwm_R)
        self.publish_command(pwm_L, pwm_R)

        self.last_time = time.time()

    def publish_command(self, pwm_L, pwm_R):
        self.pwm_command.data[0] = pwm_L
        self.pwm_command.data[1] = pwm_R
        self.pub_pwm_commands.publish(self.pwm_command)
        pass

    # pwm command regulator
    def regulate_pwm(self, pwm):
        if pwm > 127:
            pwm = 127
        elif pwm < -127:
            pwm = -127

        if abs(pwm) < 2:
            pwm = 0
        return pwm


if __name__ == '__main__':
    # make class
    vel_ctrl = Velocity_controller()
