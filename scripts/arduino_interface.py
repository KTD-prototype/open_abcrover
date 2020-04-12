#!/usr/bin/env python
# -*- coding: utf-8 -*-


# code to send PWM command and receive hardware information from arduino or other MCU
import rospy
import serial
import time
import signal
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Imu
from wheel_odometry.msg import Encoder_2wheel

cont = True
g_pwm_L = 0
g_pwm_R = 0
G_NUM_OF_RECEIVE_DATA = 8  # 2 encoders, 4 quaternions, 2 battery voltages


serial = serial.Serial('/dev/MEGA#1', 230400)


# function to send command to arduino
def send_data(pwm_L, pwm_R):
    # print(pwm_L, pwm_R)

    # shift pwm command : from -127 - 127 to 0 - 254
    pwm_L = shift_pwm(pwm_L)
    pwm_R = shift_pwm(pwm_R)

    # generate a command as a series of characters
    command = ['H', chr(pwm_L), chr(pwm_R)]
    serial.reset_input_buffer()
    serial.write(command)  # send
    # print(pwm_L, pwm_R)


# shift pwm command : from -127 - 127 to 0 - 254
def shift_pwm(pwm_data):
    pwm_data = pwm_data + 127
    return pwm_data


def receive_data():
    global G_NUM_OF_RECEIVE_DATA
    received_data = [0.0] * G_NUM_OF_RECEIVE_DATA
    reset_flag = False

    for i in range(G_NUM_OF_RECEIVE_DATA):  # read 8 data line by line
        received_data[i] = serial.readline()
        received_data[i] = received_data[i].replace('\r\n', '')

        # TODO:procedure when arduino is reset >> turn reset_flag to True
        if received_data[i] == '*************':
            reset_flag = True

        received_data[i] = float(received_data[i])

    if reset_flag == True:
        pass  # todo:process when arduino is reset

    else:
        # get and publish encoder info for wheel odometry
        print(received_data)
        encoders_data.left_encoder = received_data[0]
        encoders_data.right_encoder = received_data[1]
        pub_encoders.publish(encoders_data)

        # get and publish posture angle information:x,y,z,w
        imu_data.orientation.x = received_data[2]
        imu_data.orientation.y = received_data[3]
        imu_data.orientation.z = received_data[4]
        imu_data.orientation.w = received_data[5]
        imu_pub.publish(imu_data)

        check_battery_voltage(received_data[6], 1)
        check_battery_voltage(received_data[7], 2)


def check_battery_voltage(voltage, num):
    THE_NUM_OF_CELLS = 4.0  # cells of Lipo battery
    voltage = voltage / THE_NUM_OF_CELLS
    if voltage > 1.0:  # only when battery is connected, at least
        if voltage < 3.375:  # lower than 13.5V at 4S-Lipo
            rospy.logfatal("voltage of battery #" + str(num) +
                           " is low as " + str(voltage) + "[V/cell]")


def callback_update_pwm(pwm_command):
    global g_pwm_L, g_pwm_R
    # print(pwm_command.data[0], pwm_command.data[1])
    g_pwm_L = pwm_command.data[0]
    g_pwm_R = pwm_command.data[1]


def handler(signal, frame):
    global cont
    cont = False


def arduino_interface_main():
    global cont, G_NUM_OF_RECEIVE_DATA

    rate = rospy.Rate(100)
    while cont:
        try:
            send_data(g_pwm_L, g_pwm_R)
            while serial.inWaiting() < G_NUM_OF_RECEIVE_DATA * 3:
                # todo : at first I thought that waiting data should be more than 4bytes/data
                #        but it doesn't work
                pass
            receive_data()

        except KeyboardInterrupt:
            cont = False

        rate.sleep()

    serial.close()
    rospy.signal_shutdown('finished!')
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('arduino_interface')

    # publisher for encoders information
    pub_encoders = rospy.Publisher(
        'encoder_2wheel', Encoder_2wheel, queue_size=1)
    encoders_data = Encoder_2wheel()

    # publisher for imu data
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
    imu_data = Imu()
    imu_data.header.frame_id = 'map'

    # subscriber for pwm data
    rospy.Subscriber('pwm_commands', Int8MultiArray, callback_update_pwm)

    # wait until arduino gets ready
    time.sleep(5)
    print("started!")

    # signal handler for KeyboardInterrupt
    signal.signal(signal.SIGINT, handler)

    # main function
    arduino_interface_main()
