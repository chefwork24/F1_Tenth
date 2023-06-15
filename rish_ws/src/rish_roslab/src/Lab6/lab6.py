#!/usr/bin/env python
import rospy 
import math
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

l=1.2
Kp=1
class SubscribeAndPublish:
    def __init__(self):
        self.pub1=rospy.Publisher("/drive",AckermannDriveStamped,queue_size=1000)
        self.pub2=rospy.Publisher("/env_viz",Marker,queue_size=1000)
        self.sub=rospy.Subscriber("/odom",Odometry,self.callback)
        self.marker=Marker()
        self.angle=0
        self.x_current=0
        self.y_current=0
        self.heading_current=0
        self.current_indx=0
        self.flag=False
        self.xes=[]
        self.yes=[]
       
        with open("/home/rishwanth/data.csv","r") as f:
            content=f.readlines()
            for x in content:
                data=x.split(",")
                if len(data)>=3:
                    x=float(data[0])
                    y=float(data[1])
                    head=float(data[2])
                    self.xes.append(x)
                    self.yes.append(y)
                     
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time()
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
    def callback(self, odometry_info):
        self.x_current = odometry_info.pose.pose.position.x
        self.y_current = odometry_info.pose.pose.position.y
        siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z +
                          odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y +
                                 odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z)
        self.heading_current = math.atan2(siny_cosp, cosy_cosp)

        if not self.flag:
            min_distance = float("inf")
            for i in range(len(self.xes)):
                distance = math.sqrt((self.xes[i] - self.x_current) ** 2 + (self.yes[i] - self.y_current) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    self.current_indx = i
            self.flag = True

        while len(self.xes) > 0 and len(self.yes) > 0 and math.sqrt((self.xes[self.current_indx] - self.x_current) ** 2 +
                (self.yes[self.current_indx] - self.y_current) ** 2) <l:
            self.current_indx += 1
            if self.current_indx >= len(self.xes):
                self.current_indx = 0

        if len(self.xes) > 0 and len(self.yes) > 0:
            real_distance = math.sqrt((self.xes[self.current_indx] - self.x_current) ** 2 +
                                      (self.yes[self.current_indx] - self.y_current) ** 2)
            lookahead_angle = math.atan2(self.yes[self.current_indx] - self.y_current,
                                         self.xes[self.current_indx] - self.x_current)
            del_y = real_distance * math.sin(lookahead_angle - self.heading_current)
            self.angle = Kp* 2.00 * del_y / (1.2)**2.5  #just for normalising the angle of rotation very specific to the current bot 1.2 just not crashing so fine 

            points = Point()
            points.x = self.xes[self.current_indx]
            points.y = self.yes[self.current_indx]
            points.z = 0.0
            self.marker.points = [points]

            self.reactive_control()
            self.pub2.publish(self.marker) 
    def reactive_control(self):
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = self.angle

        if abs(self.angle) > 20.0 / 180.0 *math.pi:
            ackermann_drive_result.drive.speed = 3
        elif abs(self.angle) > 10.0 / 180.0 *math.pi:
            ackermann_drive_result.drive.speed = 3
        else:
            ackermann_drive_result.drive.speed =6.2

        self.pub1.publish(ackermann_drive_result)

def main():
    rospy.init_node('pure_pursuit')
    SAPObject = SubscribeAndPublish()
    rospy.spin()

if __name__=="__main__":
    main()