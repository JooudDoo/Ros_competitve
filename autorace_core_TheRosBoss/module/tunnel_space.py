# данный модуль отвечает за проез в тунель

import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from module.logger import log_info

def go_tunnel_space(follow_trace, img):
    message = Twist()

    # получаем данные с лидара
    scan_data = follow_trace.lidar_data.ranges
    front = min(scan_data[0:3]+scan_data[356:359])
    angle = follow_trace.get_angle()

    # заехали в тунель
    if follow_trace.avoidance == 0:
        log_info(follow_trace, message=f"заехали в тунель", debug_level=1)
        follow_trace.avoidance = 1
        follow_trace.angle = 0.50
        if abs(angle) > 2.1:
            follow_trace.angle = 2.1

    # вдоль первой стены
    if follow_trace.avoidance == 1:
        message.linear.x = follow_trace._linear_speed
        # print('front', front)
        if front < 0.55:
            log_info(follow_trace, message=f"поворот направо", debug_level=1)
            follow_trace.tunnel_started = time.time()
            follow_trace.avoidance = 2  

    # вдоль второй стены
    if follow_trace.avoidance == 2:
        message.linear.x = follow_trace._linear_speed
        message.angular.z = 2.0
        print('angle', abs(angle))
        if abs(angle) < follow_trace.angle: #0.50:
            print(' ============== ', abs(angle), '  ', follow_trace.angle)
            message.angular.z = 0.0
            # print('time ', time.time() - follow_trace.tunnel_started)
            if (time.time() - follow_trace.tunnel_started) > 14.5:
                message.linear.x = 0.0
                follow_trace.avoidance = 3
    
    if follow_trace.avoidance == 3:
        follow_trace.tunnel_started = time.time()
        follow_trace.avoidance = 4

    if follow_trace.avoidance == 4:
        log_info(follow_trace, message=f"===== Миссия выполнена =====", debug_level=1)
        message.linear.x = 0.0
        # что бы точно остановились
        if (time.time() - follow_trace.tunnel_started) > 1.0:
            log_info(follow_trace, message=f"!!!!===== Финиш =====!!!!", debug_level=1)
            msg = String()
            msg.data = "TheRosBoss"
            follow_trace._sign_finish.publish(msg)
        
    #        # пооврачиваем
    # if follow_trace.avoidance == 1:
    #     log_info(follow_trace, message=f"поворот 1", debug_level=1)
    #     message.linear.x = follow_trace._linear_speed
    #     if abs(angle) > 0.50:
    #         message.angular.z = 1.0
    #     else:
    #         message.angular.z = 0.0
    #         if front < 0.55:
    #             log_info(follow_trace, message=f"поворот направо", debug_level=1)
    #             follow_trace.avoidance = 2  

    # # вдоль 2 стенки
    # if follow_trace.avoidance == 2:
    #     log_info(follow_trace, message=f"поворот 2", debug_level=1)
    #     message.linear.x = follow_trace._linear_speed

    #     print('angle 2 ', abs(angle))
    #     if abs(angle) < 0.95:
    #         message.angular.z = -1.0
    #     else:
    #         message.angular.z = 0.0
    #         print('front 2', front)
    #         if front < 0.4 and abs(angle) > 1.0:
    #             message.angular.z = 2.0
    #             log_info(follow_trace, message=f"поворот 3", debug_level=1)
    #             follow_trace.avoidance = 3

    # if follow_trace.avoidance == 3:
    #     print('front 3 ', abs(angle), front)
    #     if abs(angle) > 0.50:
    #         log_info(follow_trace, message=f"финиш", debug_level=1)
    #         message.angular.z = 0.0
    #         follow_trace.avoidance = 0
    #     else: 
    #         log_info(follow_trace, message=f"поворот 3", debug_level=1)
    #         message.angular.z = 2.0

    # if follow_trace.avoidance == 3:
    #     follow_trace.avoidance = 0

    # отправляем данные о скоростях
    if follow_trace.avoidance:
        follow_trace._robot_cmd_vel_pub.publish(message)