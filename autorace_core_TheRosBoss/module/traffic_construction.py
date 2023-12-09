# обработка развилки
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from module.config import (
    OFFSET_BTW_CENTERS, 
    TASK_LEVEL,
    DEBUG_LEVEL, 
    LINES_H_RATIO,
    MAXIMUM_ANGLUAR_SPEED_CAP,
    MAX_LINIEAR_SPEED,

    FOLLOW_ROAD_MODE,
    WHITE_MODE_CONSTANT,
    YELLOW_MODE_CONSTANT,
    FOLLOW_ROAD_CROP_HALF,
    )

def avoid_walls(follow_trace, img):
    message = Twist()
    
    perspective = follow_trace._warpPerspective(img)
    perspective_h, persective_w, _ = perspective.shape

    hLevelLine = int(perspective_h*LINES_H_RATIO)

    # получаем координаты края желтой линии и белой
    endYellow = follow_trace._find_yellow_line(perspective, hLevelLine)

    #follow_trace.get_logger().info(f"LIdar : {len(follow_trace.lidar_data.ranges)}")
    follow_trace.get_logger().info(f"LIdar ищет: {follow_trace.lidar_data.ranges[0]}, {follow_trace.lidar_data.ranges[15]},{follow_trace.lidar_data.ranges[345]}")
    if  2.0>follow_trace.lidar_data.ranges[0]>0.1 and (2.0>follow_trace.lidar_data.ranges[15]>0.1 and 2.0>follow_trace.lidar_data.ranges[345]>0.1):
        follow_trace.get_logger().info(f"LIdar нашел: {follow_trace.lidar_data.ranges[0]}")
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
        message.angular.z = 0.5
    else:
        if follow_trace.start_avoid:
            message.linear.x = follow_trace._linear_speed
            message.angular.z = 0.0

    if follow_trace.start_avoid: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
