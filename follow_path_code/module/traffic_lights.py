# Данный модуль предназначен для обработки светофора.
# Как только мы при старте видим зеленый цвет, мы меняем статус машинки 
# на движение и меняем счетчик наших заданий на вперед

import cv2
import numpy as np

# Проверка зеленого цвета на экране
def check_green_color(img):
    """
    Проверка зеленого цвета на экране
        > img - картинка с камеры
    """
    # Определение зеленого цвета в HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Нижняя граница диапазона зеленого цвета в HSV
    lower_green = np.array([40, 40, 40])
    # Верхняя граница диапазона зеленого цвета в HSV
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
    # Проверка, есть ли зеленый цвет на изображении
    return cv2.countNonZero(green_mask) > 0

# Проверка готовности к движению, если был найден зеленый цвет, то меняем статусы
def check_traffic_lights(follow_trace, img):
    """
    Проверка готовности к движению, если был найден зеленый цвет, то меняем статусы
        > follow_trace - основной класс логики обработки и передвижения,

        > img - картинка с камеры
    """
    is_green_present = check_green_color(img)

    if is_green_present:
        follow_trace.get_logger().info("ПЕДАЛЬ В ПОЛ !!!")
        follow_trace.STATUS_CAR = 1
        follow_trace.TASK_LEVEL = 1
    else:
        follow_trace.get_logger().info(":ЖДЕМ ЗЕЛЕНЫЙ ЦВЕТ")