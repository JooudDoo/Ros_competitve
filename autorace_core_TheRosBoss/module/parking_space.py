# Данный модуль предназначен для обработки места на парковке
# если машина препаркована слева, то движемся на право иначе налево
# останавливаемся в парковочном месте на опредеелнное время и возвращаемся на полосу

import numpy as np
from geometry_msgs.msg import Twist

def parking(follow_trace, ranges):
    # Проверка областей (лево, право)
    right_area = ranges[:len(ranges)//2]
    left_area = ranges[len(ranges)//2:]

    # Вычисляю среднию в каждой области (растояние)
    avg_distance_right = sum(right_area) / len(right_area)
    avg_distance_left = sum(left_area) / len(left_area)

    # Управление движением в зависимости от расположения объекта
    if avg_distance_right < avg_distance_left:
        # Обнаружен объект справа, двигайтесь влево
        move_left(follow_trace)
        print('-----------------------------------------------')
    else:
        # Обнаружен объект слева или нет объекта вообще, двигайтесь вправо
        move_right(follow_trace)
        print('+++++++++++++++++++++++++++++++++++++++++++++++')


# движение и парковка влево
def move_left(follow_trace):
    twist_cmd = Twist()
    twist_cmd.linear.x = 0.0  # Остановить движение вперед
    twist_cmd.angular.z = 0.5  # Установить угловую скорость для поворота влево
    follow_trace.publish_twist_command(follow_trace, twist_cmd)

# движение и парковка вправо
def move_right(follow_trace):
    twist_cmd = Twist()
    twist_cmd.linear.x = 0.0  # Остановить движение вперед
    twist_cmd.angular.z = -0.5  # Установить угловую скорость для поворота вправо
    follow_trace.publish_twist_command(follow_trace, twist_cmd)

def publish_twist_command(follow_trace, twist_cmd):
    follow_trace._robot_cmd_vel_pub.publish(twist_cmd)