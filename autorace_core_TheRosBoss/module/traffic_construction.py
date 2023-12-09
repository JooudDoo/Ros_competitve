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

def check_yellow_color(follow_trace, perspectiveImg_, middle_h = None):
    if FOLLOW_ROAD_CROP_HALF:
        h_, w_, _ = perspectiveImg_.shape
        perspectiveImg = perspectiveImg_[:, :w_//2, :]
    else:
        perspectiveImg = perspectiveImg_

    h, w, _ = perspectiveImg.shape
    if middle_h is None:
        middle_h = int(h * LINES_H_RATIO)

    yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
    yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

    middle_row = yellow_mask[middle_h]
    try:
        first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
        follow_trace._yellow_prevs.append(first_notYellow)
    except:  # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
        first_notYellow = sum(
            follow_trace._yellow_prevs)//len(follow_trace._yellow_prevs)
        follow_trace.point_status = False

    if DEBUG_LEVEL >= 3:
        # рисуем линию откуда идет конец желтой полосы
        a = cv2.rectangle(yellow_mask, (first_notYellow+5,
                            middle_h), (w, middle_h), 255, 10)
        cv2.imshow("img_y", a)
        cv2.waitKey(1)

    return first_notYellow

def avoid_walls(follow_trace, img):
    message = Twist()
    
    perspective = follow_trace._warpPerspective(img)
    perspective_h, persective_w, _ = perspective.shape

    hLevelLine = int(perspective_h*LINES_H_RATIO)

    # получаем координаты края желтой линии и белой
    yellow_mask = check_yellow_color(follow_trace, perspective, hLevelLine)
    top_half = yellow_mask[:len(yellow_mask[0])//2,:]
    down_half = yellow_mask[len(yellow_mask[0])//2:,:]

    scan_data=follow_trace.lidar_data.ranges
    front = min(scan_data[0:10]+scan_data[349:359])
    left = min(scan_data[40:80])
    right = min(scan_data[260:300])

    #follow_trace.get_logger().info(f"LIdar : {len(follow_trace.lidar_data.ranges)}")
    follow_trace.get_logger().info(f"LIdar ищет")
    #if  2.0>follow_trace.lidar_data.ranges[0]>0.1 and (2.0>follow_trace.lidar_data.ranges[15]>0.1 and 2.0>follow_trace.lidar_data.ranges[345]>0.1):
    if front < 0.5:

        follow_trace.get_logger().info(f"LIdar нашел, поворачиваем отсюда")
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
    elif cv2.countNonZero(top_half) <= cv2.countNonZero(down_half) and follow_trace.start_avoid:
        message.linear.x = 0.0
        message.angular.z = -0.5
    else:
        if follow_trace.start_avoid:
            message.linear.x = follow_trace._linear_speed
            message.angular.z = 0.0

    if follow_trace.start_avoid: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
