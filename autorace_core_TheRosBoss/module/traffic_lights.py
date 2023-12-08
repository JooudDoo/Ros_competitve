# тут у нас обработка светофора

import cv2
import numpy as np

def check_green_color(img):
    # Определение зеленого цвета в HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 40, 40])  # Нижняя граница диапазона зеленого цвета в HSV
    upper_green = np.array([80, 255, 255])  # Верхняя граница диапазона зеленого цвета в HSV
    green_mask = cv2.inRange(hsv_img, lower_green, upper_green)

    # Проверка, есть ли зеленый цвет на изображении
    return cv2.countNonZero(green_mask) > 0

def check_traffic_lights(follow_trace, img):
    is_green_present = check_green_color(img)

    if is_green_present:
        follow_trace.get_logger().info("ПЕДАЛЬ В ПОЛ !!!")
        follow_trace.STATUS_CAR = 1
        follow_trace.TASK_LEVEL = 1
    else:
        follow_trace.get_logger().info(":ЖДЕМ ЗЕЛЕНЫЙ ЦВЕТ")