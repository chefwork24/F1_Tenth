#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math

class WallFollow:
    def __init__(self):
        rospy.init_node("wall_follower")
        self.pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1000)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.prev_error = 0.0
        self.prev_time = rospy.Time.now().to_sec()
        self.integral = 0.0
        self.velocity = 0.0

    def callback(self, lidar_info):
        b_indx = int((90.0 / 180.0 * math.pi - lidar_info.angle_min) / lidar_info.angle_increment)
        b_angle = 90.0 / 180.0 * math.pi
        a_angle = 45.0 / 180.0 * math.pi
        a_indx = int((45.0 / 180.0 * math.pi - lidar_info.angle_min) / lidar_info.angle_increment)

        a_range = 0.0
        b_range = 0.0

        if not math.isinf(lidar_info.ranges[a_indx]) and not math.isnan(lidar_info.ranges[a_indx]):
            a_range = lidar_info.ranges[a_indx]
        else:
            a_range = 100.0

        if not math.isinf(lidar_info.ranges[b_indx]) and not math.isnan(lidar_info.ranges[b_indx]):
            b_range = lidar_info.ranges[b_indx]
        else:
            b_range = 100.0

        alpha = math.atan((a_range * math.cos(b_angle - a_angle) - b_range) / (a_range * math.sin(b_angle - a_angle)))
        d_t = b_range * math.cos(alpha)
        d_t1 = d_t + 1.0 * math.sin(alpha)
        error = 1.2 - d_t1

        self.pid_control(error)

    def pid_control(self, error):
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.prev_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        steering_angle = -1.0 * (1.0 * error + 0.001 * derivative + 0.005 * self.integral)
        self.prev_time = current_time
        self.prev_error = error

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = rospy.Time.now()
        ackermann_msg.header.frame_id = "laser"
        ackermann_msg.drive.steering_angle = steering_angle

        if abs(steering_angle) > 20.0 * math.pi / 180.0:
            ackermann_msg.drive.speed = 0.5
            self.velocity = 0.5
        elif abs(steering_angle) > 10.0 * math.pi / 180.0:
            ackermann_msg.drive.speed = 1.0
            self.velocity = 1.0
        else:
            ackermann_msg.drive.speed = 1.5
            self.velocity = 1.5

        self.pub.publish(ackermann_msg)

if __name__ == "__main__":
    wall_follower = WallFollow()
    rospy.spin()
