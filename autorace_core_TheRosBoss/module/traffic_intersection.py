# обработка развилки
import cv2
import numpy as np

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
        left_half = blue_mask[:,:400]
        right_half = blue_mask[:,400:]

        #cv2.imshow("mask", left_half)
        #cv2.waitKey(1)

        if cv2.countNonZero(left_half) >= cv2.countNonZero(right_half):
            follow_trace.get_logger().info(":ПОВОРОТ НАЛЕВО")
            follow_trace.MAIN_LINE = "YELLOW"
        else:
            follow_trace.get_logger().info(":ПОВОРОТ НАПРАВО")
            follow_trace.MAIN_LINE = "WHITE"
        follow_trace.TASK_LEVEL = 2
