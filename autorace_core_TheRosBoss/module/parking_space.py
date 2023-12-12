# Данный модуль предназначен для обработки места на парковке
# если машина препаркована слева, то движемся на право иначе налево
# останавливаемся в парковочном месте на опредеелнное время и возвращаемся на полосу

import numpy as np
from geometry_msgs.msg import Twist
from module.logger import log_info
import time

def parking(follow_trace, img):
    message = Twist()

    # получаем данные с лидара
    scan_data = follow_trace.lidar_data.ranges
    left = min(scan_data[50:80])
    right = min(scan_data[270:300])
    log_info(follow_trace, message=f"производим поворот на парковочное место", debug_level=1)

    # узнаем повернули ли мы
    yaw = follow_trace.pose.pose.pose.orientation.z

    # определяем где наше парковочное место
    if left < 0.5 and yaw < 0.8 and follow_trace.parking_status == 0:
        log_info(follow_trace, message=f"машина слева <-, паркуемся вправо", debug_level=1)
        follow_trace.parking_status = 1
    if right < 0.5 and yaw < 0.8 and follow_trace.parking_status == 0:
        log_info(follow_trace, message=f"машина справа ->, паркуемся влево", debug_level=1)
        follow_trace.parking_status = 2

    # поворачиваем на парковку
    if follow_trace.parking_status == 1:
        log_info(follow_trace, message=f"Заезжаю на равую парковку", debug_level=1)
        message.angular.z = -2.0
        message.linear.x = 0.35
        if (yaw > 0.90):
            follow_trace.parking_status = 3
            message.angular.z = 0.0
            message.linear.x = 0.0
    if follow_trace.parking_status == 2:
        log_info(follow_trace, message=f"Заезжаю на левую парковку", debug_level=1)
        message.angular.z = 2.0
        message.linear.x = 0.3
        if (yaw < 0.35):
            follow_trace.parking_status = 4
            message.angular.z = 0.0
            message.linear.x = 0.0

    # выезжаем из парковки
    if follow_trace.parking_status == 5:
        log_info(follow_trace, message=f"Выезжаю на правую парковку", debug_level=1)
        message.angular.z = -2.4
        message.linear.x = -0.13
        if right <= 0.6:
            follow_trace.parking_status = 7
            message.angular.z = 0.0
            message.linear.x = follow_trace._linear_speed
    if follow_trace.parking_status == 6:
        log_info(follow_trace, message=f"Выезжаю на левую парковку", debug_level=1)
        message.angular.z = 2.0
        message.linear.x = -0.13
        if left <= 0.4:
            log_info(follow_trace, message=f"погнали дальше", debug_level=1)
            follow_trace.parking_status = 7
            message.angular.z = 0.0
            message.linear.x = follow_trace._linear_speed

    # отправляем данные о скоростях
    if follow_trace.parking_status: 
        follow_trace._robot_cmd_vel_pub.publish(message)   
        # стоим на парковке
        if follow_trace.parking_status == 3: 
            log_info(follow_trace, message=f"стоим", debug_level=1)
            time.sleep(3.0)
            log_info(follow_trace, message=f"выезжаем", debug_level=1)
            follow_trace.parking_status = 5
        
        if follow_trace.parking_status == 4:
            log_info(follow_trace, message=f"стоим", debug_level=1)
            time.sleep(3.0)
            log_info(follow_trace, message=f"выезжаем", debug_level=1)
            follow_trace.parking_status = 6 
        
        # заканчиваем, выезжаем
        if follow_trace.parking_status == 7:
            time.sleep(3.0)
            follow_trace.parking_status = 0
            follow_trace.TASK_LEVEL = 3.5
            log_info(follow_trace, message=f"Миссия выполнена", debug_level=1)
