# обработка миссии с обходом препятствий

import cv2
import numpy as np

from geometry_msgs.msg import Twist

from module.config import (
    LINES_H_RATIO,
    )

from module.logger import log_info

# поиск желтого цвета
def check_yellow_color(follow_trace, perspectiveImg_, middle_h = None):
    h_, w_, _ = perspectiveImg_.shape
    perspectiveImg = perspectiveImg_[:, :w_//2, :]

    h, w, _ = perspectiveImg.shape
    if middle_h is None:
        middle_h = int(h * LINES_H_RATIO)

    yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
    yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

    return yellow_mask

# поиск белого цвета
def check_white_color(follow_trace, perspectiveImg_, middle_h = None):
        h_, w_, _ = perspectiveImg_.shape
        perspectiveImg = perspectiveImg_[:, w_//2:, :]

        h, w, _ = perspectiveImg.shape
        if middle_h is None:
            middle_h = int(h * LINES_H_RATIO)

        white_mask = cv2.inRange(
            perspectiveImg, (250, 250, 250), (255, 255, 255))

        return white_mask


### Уровни avoidance ###
# 0 - не встретили еще ни разу препятствий, либо уже прошли миссию
# 1 - встретили первое препятствие
# 1.5 - отвернулись от первого препятствия и доехали до желтой линии
# 2 - развернулись в другую сторону от желтой линии и продолжаем путь

# обход препятствий
def avoid_walls(follow_trace, img):
    message = Twist()
    
    # получаем вид дороги с перспективы, ищем на нем желтый и белый цвет
    perspective = follow_trace._warpPerspective(img)
    perspective_h, persective_w, _ = perspective.shape

    hLevelLine = int(perspective_h*LINES_H_RATIO)

    yellow_mask = check_yellow_color(follow_trace, perspective, hLevelLine)
    top_half = yellow_mask[:int(len(yellow_mask[0])/1.38),:]
    down_half = yellow_mask[int(len(yellow_mask[0])/1.38):,:]

    white_mask = check_white_color(follow_trace, perspective, hLevelLine)

    # получаем данные с лидара
    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:10]+scan_data[349:359])
    left = min(scan_data[40:80])
    right = min(scan_data[260:300])

    log_info(follow_trace, message=f" Avoidance : {follow_trace.avoidance}", debug_level=3)

    # если обнаружено препятствие впереди
    if front < 0.5:
        log_info(follow_trace, message=f"Обнаружено препятствие впереди, поворачиваем", debug_level=1)

        # если встретили препятствие первый раз, переключаем режим
        if(follow_trace.avoidance < 1):
            follow_trace.avoidance = 1
        message.linear.x = 0.0

        # если встретили первый раз, то поворачиваем налево, второй - направо
        if 1.5 <= follow_trace.avoidance <= 2:
            message.angular.z = -0.5
        else:
            message.angular.z = 0.5

    # если успешно отвернулись от первого препятствия и доехали до желтой линии, не даем выйти роботу за ее пределы и меняем режим
    elif cv2.countNonZero(top_half) < cv2.countNonZero(down_half) and (1 <= follow_trace.avoidance <= 1.5) :
        log_info(follow_trace, message=f"Найдена желтая линия", debug_level=1)

        message.linear.x = 0.0
        message.angular.z = -0.5
        follow_trace.avoidance = 1.5

    # если отвернулись от желтой линии
    elif follow_trace.avoidance == 1.5 and cv2.countNonZero(top_half) == 0 and cv2.countNonZero(down_half)==0:
        follow_trace.avoidance = 2

    # если препятствий не обнаружно
    else:
        if follow_trace.avoidance:
            down = white_mask[int(len(white_mask[0])/2):,:]
            top = white_mask[:int(len(white_mask[0])/2),:]

            # если уже прошли препятствия и доехали до белой линии - миссия окончена
            if follow_trace.avoidance == 2 and (cv2.countNonZero(down) > cv2.countNonZero(top)):
                log_info(follow_trace, message=f"Миссия выполнена", debug_level=1)
                follow_trace.avoidance = 0
                follow_trace.TASK_LEVEL = 2.5

            # едем прямо
            log_info(follow_trace, message=f"Препятствий не обнаружено", debug_level=1)
            message.linear.x = follow_trace._linear_speed
            message.angular.z = 0.0

    # отправляем данные о скоростях
    if follow_trace.avoidance: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
