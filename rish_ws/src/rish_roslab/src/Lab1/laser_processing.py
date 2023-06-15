#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

def callback(msg):
   
    # rospy.loginfo("Ranges: %s", msg.ranges)
    short=min(msg.ranges)
    print(short)
    long=max(msg.ranges)
    print(long)
    pub1.publish(short)
    pub2.publish(long)
    

def listener():
    rospy.init_node('laser_scanner')
    
    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin() 

if __name__ == '__main__':
    try:
        pub1=rospy.Publisher("/closest_point",Float64,queue_size=10)
        pub2=rospy.Publisher("/farthest_point",Float64,queue_size=10)
        listener()
    except rospy.ROSInterruptException:
        pass
