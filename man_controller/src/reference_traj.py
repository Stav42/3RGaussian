#!/usr/bin/env python3

import rospy
from man_controller.msg import Traj

def talker():
    # Initialize the publishers for position, velocity, and acceleration
    pos_pub = rospy.Publisher('/position_reference', Traj, queue_size=10)
    vel_pub = rospy.Publisher('/velocity_reference', Traj, queue_size=10)
    acc_pub = rospy.Publisher('/acceleration_reference', Traj, queue_size=10)

    # Initialize the node
    rospy.init_node('reference_node', anonymous=True)

    rate = rospy.Rate(10) # 10 Hz

    try:
        with open('/Users/stav.42/ws/src/man_controller/eom/src/position.txt', 'r') as pos_file, \
             open('/Users/stav.42/ws/src/man_controller/eom/src/velocity.txt', 'r') as vel_file, \
             open('/Users/stav.42/ws/src/man_controller/eom/src/acceleration.txt', 'r') as acc_file:
            
            for pos_line, vel_line, acc_line in zip(pos_file, vel_file, acc_file):
                pos_nums = map(float, pos_line.split())
                vel_nums = map(float, vel_line.split())
                acc_nums = map(float, acc_line.split())
                
                # if len(pos_nums) == len(vel_nums) == len(acc_nums) == 4:  # Ensure there are 4 numbers on each line
                pos_msg = Traj(*pos_nums)
                vel_msg = Traj(*vel_nums)
                acc_msg = Traj(*acc_nums)

                print("Publishing files")
                pos_pub.publish(pos_msg)
                vel_pub.publish(vel_msg)
                acc_pub.publish(acc_msg)
                print("Published")

                rate.sleep()
                # else:
                # rospy.logerr('One or more lines do not contain exactly four numbers')
    except IOError as e:
        rospy.logerr('Error opening files: %s', str(e))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
