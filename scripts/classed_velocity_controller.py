#!/usr/bin/env python
# -*- coding: utf-8 -*-


# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node
import rospy
import signal
import time
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

cont = True

# class for this node


class Velocity_Controller():
    def __init__(self):
        global cont
        # initialize node
        rospy.init_node('velocity_controller', disable_signals=True)

        # publisher for pwm command
        self.pub_motor_commands = rospy.Publisher(
            'motor_commands', Int16MultiArray, queue_size=1)
        self.motor_command = Int16MultiArray()
        self.motor_command.data.extend([0, 0])  # for motor commnad left/right
        # self.motor_command.data.append(0)  # for right motor command

        # publisher for operation mode
        self.pub_operation_mode = rospy.Publisher(
            'operation_mode', Int8, queue_size=1, latch=True)
        # subscriber for wheel odometry
        self.sub_odom = rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                                         self.callback_update_odometry, queue_size=1)
        # subscriber for velocity command
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist,
                                            self.callback_update_command, queue_size=1)
        # subscriber for velocity command
        self.sub_cmd_vel = rospy.Subscriber('joy', Joy,
                                            self.callback_update_operationmode, queue_size=1)

        # parameters on velocity command
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        # parameters on rover velocity
        self.linear_vel = 0.0
        self.past_linear_vel = 0.0
        self.err_linear_vel = 0.0
        self.angular_vel = 0.0
        self.past_angular_vel = 0.0
        self.err_angular_vel = 0.0
        # refresh rate of odometry
        self.ODOM_RATE = 100.0
        # parameters for operation mode
        self.operation_mode = 0  # 0:teleop, 1:teleop_turbo, 2:autonomous

        rate = rospy.Rate(100)
        while cont:
            try:
                self.velocity_control(self.cmd_linear, self.cmd_angular)
            except KeyboardInterrupt:
                cont = False
        rate.sleep()

    def callback_update_odometry(self, odometry):
        # store
        self.past_linear_vel = self.linear_vel
        self.past_angular_vel = self.angular_vel
        # update
        self.linear_vel = odometry.twist.twist.linear.x
        self.angular_vel = odometry.twist.twist.angular.z

    def callback_update_command(self, twist):
        self.cmd_linear = twist.linear.x
        self.cmd_angular = twist.angular.z
        # print(self.cmd_linear, self.cmd_angular)
        # self.velocity_control(cmd_linear, cmd_angular)

    def callback_update_operationmode(self, joy):
        # store current mode
        current_mode = self.operation_mode

        # chack joy command
        self.operation_mode = 0  # disabled

        if joy.buttons[3] == 1 and joy.buttons[5] == 1:
            self.operation_mode = 3  # autonomous
        if joy.buttons[4] == 1:
            self.operation_mode = 1  # teleop
        if joy.buttons[0] == 1:
            self.operation_mode = 2  # teleop with turbo

        # if mode has been changed
        if self.operation_mode != current_mode:
            self.pub_operation_mode.publish(self.operation_mode)

    def velocity_control(self, cmd_linear, cmd_angular):
        # parameters for gains
        Pgain_LINEAR = 100.0
        Igain_LINEAR = 0.007
        Dgain_LINEAR = 0.25
        Pgain_ANGULAR = 15.0
        Igain_ANGULAR = 0.001
        Dgain_ANGULAR = 0.03

        # parameters for motor commands
        command_offset = 0.0  # command offset for robot rotation
        motor_command_L = 0  # for left motor
        motor_command_R = 0  # for right motor
        # cycle length (seconds) to get acceleration of the rover
        dt = 1.0 / self.ODOM_RATE
        # accumulated error for integral control
        self.err_linear_vel += cmd_linear - self.linear_vel
        self.err_angular_vel += cmd_angular - self.angular_vel

        # PD control for linear velocity
        # if cmd_linear == 0:
        #     motor_command_L = 0
        #     self.err_linear_vel = 0
        # else:
        motor_command_L = Pgain_LINEAR * (cmd_linear - self.linear_vel) - \
            Dgain_LINEAR * (self.linear_vel - self.past_linear_vel) / \
            dt + Igain_LINEAR * self.err_linear_vel
        motor_command_R = -1 * motor_command_L

        # if cmd_linear < 0:
        #     cmd_angular = -1 * cmd_angular
        #     self.err_angular_vel = 0
        # PD control for angular velocity
        # if cmd_angular == 0:
        #     command_offset = 0
        #     self.err_angular_vel = 0
        # else:
        command_offset = Pgain_ANGULAR * \
            (cmd_angular - self.angular_vel) - Dgain_ANGULAR * \
            (self.angular_vel - self.past_angular_vel) / dt + \
            Igain_ANGULAR * self.err_angular_vel

        # print(Pgain_LINEAR * (cmd_linear - self.linear_vel),
        #       Dgain_LINEAR * (self.linear_vel - self.past_linear_vel) / dt,
        #       Igain_LINEAR * self.err_linear_vel)
        # print(Pgain_ANGULAR * (cmd_angular - self.angular_vel),
        #       Dgain_ANGULAR * (self.angular_vel - self.past_angular_vel) / dt,
        #       Igain_ANGULAR * self.err_angular_vel)
        motor_command_L = motor_command_L - command_offset
        motor_command_R = motor_command_R - command_offset

        # regulate pwm command
        motor_command_L = self.regulate_command(motor_command_L)
        motor_command_R = self.regulate_command(motor_command_R)
        self.publish_command(motor_command_L, motor_command_R)

        self.last_time = time.time()

    def publish_command(self, command_L, command_R):
        self.motor_command.data[0] = command_L
        self.motor_command.data[1] = command_R
        self.pub_motor_commands.publish(self.motor_command)
        pass

    # pwm command regulator
    def regulate_command(self, command):
        # maximum output for motor driver : vnh5019. The maximum value is 400
        # according to the driver library, but regulated up to 300 for motor
        # protection drived at 14.8V, higher than nominal voltage : 12V
        MAXIMUM_OUTPUT = 300

        # regulate
        if command > MAXIMUM_OUTPUT:
            command = MAXIMUM_OUTPUT
        elif command < -1 * MAXIMUM_OUTPUT:
            command = -1 * MAXIMUM_OUTPUT

        return command

    def handler(signal, frame):
        global cont
        cont = False


if __name__ == '__main__':
    # make class
    Velocity_Controller()
