# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node

#!/usr/bin/env python
# -*- coding: utf-8 -*-


def velocity_controller_main():
    global cont
    rospy.init_node('velocity_controller')

    # publisher for pwm command
    pub_pwm_commands = rospy.Publisher(
        'pwm_commands', Int8MultiArray, queue_size=1)

    # subscriber for wheel odometry
    rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                     callback_update_odometry, queue_size=1)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    velocity_controller_main()
