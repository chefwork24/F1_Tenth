#!/usr/bin/env python3
import rospy
import numpy as np 
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class Safety(object):
  
    def __init__(self):
        rospy.Subscriber("/scan",LaserScan,self.scan_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)
        self.pub1=rospy.Publisher("/brake",AckermannDriveStamped,queue_size=10)
        self.pub2=rospy.Publisher("/brake_bool",Bool,queue_size=10)

        

    def odom_callback(self, odom_msg):
        self.speed=odom_msg.twist.twist.linear.x
        self.angular_speed=odom_msg.twist.twist.angular.z
        print(self.speed,self.angular_speed)

    def scan_callback(self, scan_msg):
        bmsg=Bool()
        contsig=AckermannDriveStamped()
        if self.speed!=0:
            index=(scan_msg.ranges.index(min(scan_msg.ranges)))
            self.theta=((index)/1080)*2*np.pi

            self.TTC=abs((min(scan_msg.ranges))/(self.speed*np.cos(self.theta)))
            if math.isinf(self.TTC):
                TTC=10000
            else:
                pass
            
        else:
            self.TTC=10000
        if self.TTC<0.3:

            contsig.drive.speed=0
            bmsg=True
            print("Brakes applied")
        self.pub1.publish(contsig)
        self.pub2.publish(bmsg)
        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin() 
if __name__ == '__main__':
    main()