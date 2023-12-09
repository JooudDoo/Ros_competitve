# обработка развилки
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def avoid_walls(follow_trace, img):
    message = Twist()
    check = 1
    #follow_trace.get_logger().info(f"LIdar : {len(follow_trace.lidar_data.ranges)}")
    if follow_trace.lidar_data.ranges[90]>0.8:
        '''
        for i in range(len(follow_trace.lidar_data.ranges)):
            if(follow_trace.lidar_data.ranges[i] < follow_trace.lidar_data.range_min and follow_trace.lidar_data.ranges[i] > follow_trace.lidar_data.range_max):
                continue

            if(follow_trace.lidar_data.ranges[i] < 1.0):
                check = 0
                break
        '''
        #if(check):
        follow_trace.start_avoid = 1
        message.linear.x = 0.0
        message.angular.z = -0.5
    else:
        if follow_trace.start_avoid:
            message.linear.x = follow_trace._linear_speed
            message.angular.z = 0.0

    if follow_trace.start_avoid: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
