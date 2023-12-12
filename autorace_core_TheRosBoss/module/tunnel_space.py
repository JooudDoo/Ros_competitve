# Данный модуль предназначен для обработки места на парковке
# если машина препаркована слева, то движемся на право иначе налево
# останавливаемся в парковочном месте на опредеелнное время и возвращаемся на полосу

import numpy as np
from geometry_msgs.msg import Twist
from module.logger import log_info
import time


def go_tunnel_space(follow_trace, img):
    message = Twist()

    # получаем данные с лидара
    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:1]+scan_data[358:359])
    left = min(scan_data[60:70])
    right = min(scan_data[280:290])
    angle = follow_trace.get_angle()

    if (abs(angle) > 1):
        log_info(follow_trace, message=f"угол", debug_level=1)
        follow_trace.empty = 1

    if left < 0.3 and right < 0.3 and follow_trace.empty == 1:
        log_info(follow_trace, message=f"заехали в тунель", debug_level=1)
        follow_trace.avoidance = 1

           # пооврачиваем
    if follow_trace.avoidance == 1:
        log_info(follow_trace, message=f"поворот 1", debug_level=1)
        message.linear.x = follow_trace._linear_speed
        if abs(angle) > 0.50:
            message.angular.z = 1.0
        else:
            message.angular.z = 0.0
            if front < 0.55:
                log_info(follow_trace, message=f"поворот направо", debug_level=1)
                follow_trace.avoidance = 2  

    # вдоль 2 стенки
    if follow_trace.avoidance == 2:
        log_info(follow_trace, message=f"поворот 2", debug_level=1)
        message.linear.x = follow_trace._linear_speed

        print('angle 2 ', abs(angle))
        if abs(angle) < 0.95:
            message.angular.z = -1.0
        else:
            message.angular.z = 0.0
            print('front 2', front)
            if front < 0.4 and abs(angle) > 1.0:
                message.angular.z = 2.0
                log_info(follow_trace, message=f"поворот 3", debug_level=1)
                follow_trace.avoidance = 3

    if follow_trace.avoidance == 3:
        print('front 3 ', abs(angle), front)
        if abs(angle) > 0.50:
            log_info(follow_trace, message=f"финиш", debug_level=1)
            message.angular.z = 0.0
            follow_trace.avoidance = 0
        else: 
            log_info(follow_trace, message=f"поворот 3", debug_level=1)
            message.angular.z = 2.0

    # if follow_trace.avoidance == 3:
    #     follow_trace.avoidance = 0

    # отправляем данные о скоростях
    if follow_trace.avoidance:
        follow_trace._robot_cmd_vel_pub.publish(message)