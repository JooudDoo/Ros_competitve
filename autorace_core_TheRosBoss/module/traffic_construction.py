# обработка развилки
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def avoid_walls(follow_trace, img):
    message = Twist()
    check = 1

    for i in range(len(follow_trace.lidar_data.ranges)):
        if(follow_trace.lidar_data.ranges[i] < follow_trace.lidar_data.range_min and follow_trace.lidar_data.ranges[i] > follow_trace.lidar_data.range_max):
            continue

        if(follow_trace.lidar_data.ranges[i] < 1.0):
            check = 0
            break

    if(check):
        message.linear.x = follow_trace._linear_speed
    else:
        message.linear.x = 0.0
        
    follow_trace._robot_cmd_vel_pub.publish(message)   
