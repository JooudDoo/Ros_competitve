# обработка развилки
import cv2
import numpy as np

from module.logger import log_info

def check_blue_color(img):
    # Определение синего цвета в HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([100, 40, 40])  # Нижняя граница диапазона зеленого цвета в HSV
    upper_green = np.array([255, 255, 255])  # Верхняя граница диапазона зеленого цвета в HSV
    blue_mask = cv2.inRange(hsv_img, lower_green, upper_green)

    return blue_mask 

def check_direction(follow_trace, img):
    # Определение направления поворота на знаке
    angle = follow_trace.get_angle()
    if abs(angle) >= 2.80:
        blue_mask = check_blue_color(img)
        #follow_trace.get_logger().info(f"Angle: {follow_trace.get_angle()}")
        left_half = blue_mask[:,:250]
        right_half = blue_mask[:,250:]

        # cv2.imshow("mask_l", left_half)
        # cv2.imshow("mask_r", right_half)
        # cv2.waitKey(1)

        if cv2.countNonZero(left_half) >= cv2.countNonZero(right_half):
            log_info(follow_trace, "[Перекресток] Поворот налево", debug_level=0)
            follow_trace.MAIN_LINE = "YELLOW"
        else:
            log_info(follow_trace, "[Перекресток] Поворот направо", debug_level=0)
            follow_trace.MAIN_LINE = "WHITE"
        follow_trace.TASK_LEVEL = 1.5
