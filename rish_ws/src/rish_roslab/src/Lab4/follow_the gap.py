#!/usr/bin/env python

import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub =rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback)
        self.drive_pub =rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=10)
    
   
        
        
    def find_max_gap(self, free_space_ranges):
        
        lst=free_space_ranges
        longest_start = -1
        longest_end = -1
        current_start = -1
        longest_gap_length = 0
        current_gap_length = 0

        for i in range(self.min_index,self.max_index+1):
            num = lst[i]
            if num != 0:
                if current_start == -1:
                    current_start = i
                current_gap_length += 1
                current_end = i

                if current_gap_length > longest_gap_length:
                    longest_gap_length = current_gap_length
                    longest_start = current_start
                    longest_end = current_end
            else:
                current_start = -1
                current_gap_length = 0

        return longest_start, longest_end



    def find_best_point(self, start_i, end_i, ranges):
        
        max_index=ranges.index(max(ranges[start_i:end_i+1]))


        return max_index

    def lidar_callback(self, data):
        
        ranges=list(data.ranges)
        min_angle=-50/180*math.pi
        max_angle=50/180*math.pi
        self.min_index = int(math.floor((min_angle - data.angle_min) / data.angle_increment))
        self.max_index = int(math.ceil((max_angle - data.angle_min) / data.angle_increment))
        
        
        for i in range(self.min_index,self.max_index+1):
            if ranges[i] is math.isinf or ranges[i] is math.isnan:
                ranges[i]=0
            elif ranges[i]>=data.range_max:
                ranges[i]=data.range_max
        for i in range(self.min_index,self.max_index+1):
            ranges[i]=(ranges[i-5]+ranges[i-4]+ranges[i-3]+ranges[i-2]+ranges[i-1]+ranges[i]+ranges[i+1]+ranges[i+2]+ranges[i+5]+ranges[i+4]+ranges[i+3])/11
        closest_dist=math.inf
        closest_index=self.min_index
        for i in range((self.min_index),(self.max_index+1)):
            if ranges[i]<=closest_dist:
                closest_dist=ranges[i]
                closest_index=i
            else:
                continue 
        # print(closest_index) 
        radius=150
        for i in range(closest_index-radius,closest_index+radius+1):
            ranges[i]=0
        # print(ranges.count(0))

        index=self.find_max_gap(ranges)
        # print(index)
        start=index[0]
        end=index[1]
        max=0
        index_to_go=0
        for i in range(start,end+1):
            if ranges[i]>=max:
                max=ranges[i]
                index_to_go=i
            else:
                continue
        # print(index_to_go)

        angle =data.angle_min+(index_to_go)*data.angle_increment

        if angle>0.3:
            angle=0.3
        elif angle<-0.3:
            angle=-0.3
        else:
            angle=angle
        
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = angle

        if abs(angle) > 20.0 / 180.0 *math.pi:
            ackermann_drive_result.drive.speed = 0.4
        elif abs(angle) > 10.0 / 180.0 *math.pi:
            ackermann_drive_result.drive.speed = 1
        else:
            ackermann_drive_result.drive.speed = 1.4

        self.drive_pub.publish(ackermann_drive_result)
        


        
         
        
            

       

def main():
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main()
