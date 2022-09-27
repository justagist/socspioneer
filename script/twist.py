#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.init_node('Mover', anonymous=True)
    # rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = 0.1
        pub.publish( base_data )
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass 