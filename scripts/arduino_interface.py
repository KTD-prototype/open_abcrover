# code to send PWM command and receive hardware information from arduino or other MCU

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial

serial = serial.Serial('/dev/MEGA#1', 230400)


# function to send command to arduino
def send_data(pwm_L, pwm_R):
    # generate a command as a series of characters
    command = ['H', chr(pwm_L), chr(pwm_R)]
    serial.reset_input_buffer()
    serial.write(command)  # send


def receive_data():
    NUMBER_OF_DATA = 8  # 2 encoders, 4 quaternions, 2 battery voltages
    received_data = [0.0] * NUMBER_OF_DATA
    reset_flag = False

    for i in range(NUMBER_OF_DATA):  # read 8 data line by line
        received_data[i] = serial.readline()
        received_data[i] = received_data[i].replace('\r\n', '')
        # TODO:procedure when arduino is reset >> turn reset_flag to True
        received_data[i] = float(received_data[i])

    if reset_flag == False:
        # get and publish encoder info for wheel odometry
        encoders.left_encoder = received_data[0]
        encoders.right_encoder = received_data[1]
        pub_encoders.publish(encoders)

        # get and publish posture angle information:x,y,z,w
        imu.orientation.x = received_data[2]
        imu.orientation.y = received_data[3]
        imu.orientation.z = received_data[4]
        imu.orientation.w = received_data[5]
        imu_pub.publish(imu)

        check_battery_voltage(received_data[6], 1)
        check_battery_voltage(received_data[7], 2)


def check_battery_voltage(voltage, num):
    THE_NUM_OF_CELLS = 4.0  # cells of Lipo battery
    voltage = voltage / THE_NUM_OF_CELLS
    if voltage > 1.0:  # only when battery is connected, at least
        if voltage < 3.375:  # lower than 13.5V at 4S-Lipo
            rospy.logfatal("voltage of battery #" + str(num) +
                           " is low as " + str(voltage) + "[V/cell]")


if __name__ == '__main__':
    while serial.inWaiting() < 16:
        pass
