#!/usr/bin/env python
# -*- coding: utf-8 -*-


# code to send PWM command and receive hardware information from arduino or other MCU
import rospy
import serial
import time
import signal
import tf
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from wheel_odometry.msg import Encoder_2wheel

cont = True


class Arduino_Interface():
    def __init__(self):
        global cont
        # initialize node
        rospy.init_node('arduino_interface')

        # publisher for encoders information
        self.pub_encoder = rospy.Publisher(
            'encoder_2wheel', Encoder_2wheel, queue_size=1)
        self.encoder_data = Encoder_2wheel()

        # publisher for imu data
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        self.imu_data = Imu()
        self.imu_data.header.frame_id = 'map'

        # subscriber for pwm data
        rospy.Subscriber('motor_commands', Int16MultiArray,
                         self.callback_update_command)
        # subscriber for pwm data
        rospy.Subscriber('operation_mode', Int8,
                         self.callback_update_operationmode)

        # wait until the arduino gets ready
        # time.sleep(3)
        rospy.loginfo("started communication between Arduino!")

        # parameters
        self.left_motor_command = 0
        self.right_motor_command = 0
        self.operation_mode = 0  # 0:disabled, 1:teleop, 2:teleop_turbo, 3:autonomous, 4:error
  
        # 2 encoders, 4 postures(@quaternion), 3 accelerometers(x,y,z), 3 gyros(roll, pitch,yaw), 2 battery voltages
        self.NUM_OF_RECEIVED_DATA = 14

        # start serial communication with arduino
        self.serial = serial.Serial('/dev/MEGA#1', 230400)
        time.sleep(1)  # wait until opening serial port completes
        self.serial.reset_input_buffer()

        rate = rospy.Rate(100)
        while cont:
            try:
                # send motor command
                self.send_data(self.left_motor_command,
                               self.right_motor_command)

                # wait until rovor information arrives
                while self.serial.inWaiting() < self.NUM_OF_RECEIVED_DATA * 4:  # 4 bytes per each data
                    pass

                # receive rover information
                self.receive_data()

            except KeyboardInterrupt:
                cont = False

            rate.sleep()

        self.serial.close()
        rospy.signal_shutdown('finished!')
        rospy.spin()

    # function to send command to arduino
    def send_data(self, command_L, command_R):
        # shift command to ensure they are positive number(include 0)
        OFFSET = 300  # big enough to offset minimum_output:-300
        command_L = command_L + OFFSET
        command_R = command_R + OFFSET

        # divide command by single byte
        command_L_high, command_L_low = self.divide_command(command_L)
        command_R_high, command_R_low = self.divide_command(command_R)

        # generate a command as a series of characters
        integrated_command = ['H', chr(self.operation_mode), chr(command_L_high), chr(
            command_L_low), chr(command_R_high), chr(command_R_low)]

        # send
        self.serial.reset_input_buffer()  # flush buffer
        self.serial.write(integrated_command)  # send

    # function to divide command with 2 bytes into 1 each(high/low)
    def divide_command(self, command):
        command_high = command >> 8
        command_low = command & 0x00ff
        return command_high, command_low

    # receive process
    def receive_data(self):
        received_data = [0.0] * self.NUM_OF_RECEIVED_DATA
        reset_flag = False

        for i in range(self.NUM_OF_RECEIVED_DATA):  # read 8 data line by line
            received_data[i] = self.serial.readline()
            received_data[i] = received_data[i].replace('\r\n', '')

            # TODO:procedure when arduino is reset >> turn reset_flag to True
            if received_data[i] == '*************':
                reset_flag = True

            # convert each received data (from string to float)
            received_data[i] = float(received_data[i])

        # exceptional process
        if reset_flag == True:
            pass  # todo:process when arduino is reset

        else:
            # print(received_data)

            # get and publish encoder info for wheel odometry
            self.encoder_data.left_encoder = received_data[0]
            self.encoder_data.right_encoder = received_data[1]
            self.pub_encoder.publish(self.encoder_data)

            # get posture angle information:x,y,z,w
            self.imu_data.orientation.x = received_data[2]
            self.imu_data.orientation.y = received_data[3]
            self.imu_data.orientation.z = received_data[4]
            self.imu_data.orientation.w = received_data[5]
            # get accelerometer data : x,y,z
            self.imu_data.linear_acceleration.x = received_data[6]
            self.imu_data.linear_acceleration.y = received_data[7]
            self.imu_data.linear_acceleration.z = received_data[8]
            # get gyro data ; x, y, z(roll,pitch,yaw)
            self.imu_data.angular_velocity.x = received_data[9]
            self.imu_data.angular_velocity.y = received_data[10]
            self.imu_data.angular_velocity.z = received_data[11]
            # publish imu data
            self.imu_pub.publish(self.imu_data)

            # euler = tf.transformations.euler_from_quaternion(
            #     (received_data[2], received_data[3], received_data[4], received_data[5]))
            # print(euler)

            self.check_battery_voltage(received_data[12], 1)
            self.check_battery_voltage(received_data[13], 2)

        self.serial.reset_input_buffer()  # flush buffer

    def check_battery_voltage(self, voltage, num):
        THE_NUM_OF_CELLS = 4.0  # cells of Lipo battery
        voltage = voltage / THE_NUM_OF_CELLS
        if voltage > 1.0:  # only when battery is connected, at least
            if voltage < 3.6:  # lower than 14.4V at 4S-Lipo
                rospy.logwarn("voltage of battery #" + str(num) +
                              " is low as " + str(voltage) + "[V/cell]")

    def callback_update_command(self, command):
        # print(pwm_command.data[0], pwm_command.data[1])
        self.left_motor_command = command.data[0]
        self.right_motor_command = command.data[1]

    def callback_update_operationmode(self, mode):
        self.operation_mode = mode.data

    def handler(self, signal, frame):
        global cont
        cont = False


if __name__ == '__main__':
    # make instance from the class:Arduino_Interface
    Arduino_Interface()
