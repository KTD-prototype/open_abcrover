# node for velocity control based on wheel odometry data and send velocity command to arduino_interface node

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# global paramters for velocity control
# current velocity
g_linearvel = 0.0
g_angularvel = 0.0
# past velocity at last loop
g_past_linearvel = 0.0
g_past_angularvel = 0.0


def callback_update_odometry(odometry):
    global g_linearvel, g_angularvel
    g_linearvel = odometry.twist.twist.linear.x
    g_angularvel = odometry.twist.twist.angular.z


def velocity_controller_main():
    global cont
    rospy.init_node('velocity_controller')

    # publisher for pwm command
    pub_pwm_commands = rospy.Publisher(
        'pwm_commands', Int8MultiArray, queue_size=1)

    # subscriber for wheel odometry
    rospy.Subscriber('wheel_odometry_2wheel', Odometry,
                     callback_update_odometry, queue_size=1)

    rate = rospy.Rate(100)
    while cont:
        try:
            velocity_control(g_linearvel, g_past_linearvel,
                             g_angularvel, g_past_angularvel)

        except KeyboardInterrupt:
            cont = False

        rate.sleep()
    rospy.signal_shutdown('finished!')
    rospy.spin()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    velocity_controller_main()
