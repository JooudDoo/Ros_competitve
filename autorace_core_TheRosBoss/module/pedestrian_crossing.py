# обработка миссии с обходом препятствий
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

from module.logger import log_info

### Уровни avoidance ###
# 0 - не встретили еще ни разу препятствий, либо уже прошли миссию
# 1 - встретили первое препятствие


def check_yellow_color(follow_trace, perspectiveImg_, middle_h = None):
    h_, w_, _ = perspectiveImg_.shape
    perspectiveImg = perspectiveImg_[:, :w_//2, :]

    h, w, _ = perspectiveImg.shape
    if middle_h is None:
        middle_h = int(h * LINES_H_RATIO)

    yellow_mask = cv2.inRange(perspectiveImg, (85, 140, 170), (100,190,225))
    yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

    #cv2.imshow("img_ye", yellow_mask)
    #cv2.waitKey(1)
    return yellow_mask

def stop_crosswalk(follow_trace, img):
    
    # ищем лежачий
    perspective = follow_trace._warpPerspective(img)
    perspective_h, persective_w, _ = perspective.shape

    hLevelLine = int(perspective_h*LINES_H_RATIO)

    yellow_mask = check_yellow_color(follow_trace, perspective, hLevelLine)

    # получаем данные с лидара
    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:10]+scan_data[349:359])
    left = min(scan_data[40:80])
    right = min(scan_data[260:300])


    log_info(follow_trace, f"Avoidance level: {follow_trace.avoidance}", debug_level=3, allow_repeat=True)

    # если человек идет и доехали до лежачего
    if front < 0.5 and cv2.countNonZero(yellow_mask) < 10:
        log_info(follow_trace, "Человек пересекает дорогу", debug_level=1)

        message = Twist()
        message.linear.x = 0.0
        message.angular.z = 0.0
        
        follow_trace._robot_cmd_vel_pub.publish(message) 
        follow_trace.avoidance = 1

    # если человек прошел через дорогу
    elif follow_trace.avoidance == 1:
        log_info(follow_trace, "Едем в туннель", debug_level=1)
        follow_trace.TASK_LEVEL = 5
        follow_trace.avoidance = 0

    else:
        log_info(follow_trace, "Никого нет, едем", debug_level=1)
        follow_trace.avoidance = 0
