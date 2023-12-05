from typing import Literal
from collections import deque

# модули
# конфигуратор
from module.config import OFFSET_BTW_CENTERS, DEBUG_LEVEL
# обработка светофора
from module.traffic_lights import check_traffic_lights

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import cv2
import math
import numpy as np

# Если потерял линию то стараться повернуть к ней?
# или наоборот держаться той линии что осталась, но на каком-то растоянии? (среднем за предыдущие время от этой линии)
# Что-то сделать со скоростями, PID регулятор?
class Follow_Trace_Node(Node):

    def __init__(self, linear_speed=0.1, angular_speed=0.2, linear_slow_speed=None):
        super().__init__("Follow_Trace_Node")

        self._pose_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback, 10)
        self._robot_Ccamera_sub = self.create_subscription(
            Image, "/color/image", self._callback_Ccamera, 3)
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cv_bridge = CvBridge()

        self._linear_speed = linear_speed
        self.angular_speed = angular_speed
        self._linear_slow_speed = linear_slow_speed

        if self._linear_slow_speed is None:
            self._linear_slow_speed = self._linear_speed / 5

        self.__yellow_prevs = deque(maxlen=10)
        self.__white_prevs = deque(maxlen=10)
        self.__yellow_prevs.append(0)
        self.__white_prevs.append(0)

        self.pose = Odometry()

        self.Kp = self.declare_parameter('Kp', value=2.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value
        self.Ki = self.declare_parameter('Ki', value=0.1, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value
        self.Kd = self.declare_parameter('Kd', value=0.1, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value

        self.old_e = 0
        self.E = 0

        self.STATUS_CAR = 0
        self.TASK_LEVEL = 0

    # Обратный вызов для получения данных о положении
    def pose_callback(self, data):
        self.pose = data

    # Получение угла поворота из данных о положении
    def get_angle(self):
        quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                      self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        return euler[2]

    # Преобразование перспективы изображения
    def __warpPerspective(self, cvImg):
        h, w, _ = cvImg.shape
        top_x_offset = 50

        pts1 = np.float32(
            [[0, 480], [w, 480], [top_x_offset, 300], [w-top_x_offset, 300]])
        result_img_width = np.int32(abs(pts1[0][0] - pts1[1][0]))  # 760
        result_img_height = np.int32(abs(pts1[0][1] - pts1[2][0]))  # 300

        pts2 = np.float32([[0, 0], [result_img_width, 0], [0, result_img_height], [
                          result_img_width, result_img_height]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(
            cvImg, M, (result_img_width, result_img_height))

        if (DEBUG_LEVEL >= 2):
            for pt in pts1:
                cvImg = cv2.rectangle(cvImg, np.int32(
                    pt), np.int32(pt), (255, 0, 0), 5)
            cv2.imshow("orig", cvImg)

        return cv2.flip(dst, 0)

    # Поиск желтой линии на изображении
    def _find_yellow_line(self, perspectiveImg):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2
        # middle_h = middle_h + h // 2

        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

        middle_row = yellow_mask[middle_h]
        try:
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
            self.__yellow_prevs.append(first_notYellow)
        except:  # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_notYellow = sum(
                self.__yellow_prevs)//len(self.__yellow_prevs)

        if DEBUG_LEVEL >= 3:
            # рисуем линию откуда идет конец желтой полосы
            a = cv2.rectangle(yellow_mask, (first_notYellow+5,
                              middle_h), (w, middle_h), 255, 10)
            cv2.imshow("img_y", a)
            cv2.waitKey(1)

        return (first_notYellow, middle_h)

    # Поиск белой линии на изображении
    def _find_white_line(self, perspectiveImg):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2
        # middle_h = middle_h + h // 2

        white_mask = cv2.inRange(
            perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.__white_prevs.append(first_white)
        except:  # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_white = sum(self.__white_prevs)//len(self.__white_prevs)

        if DEBUG_LEVEL >= 3:
            # рисуем линию откуда идет конец белой полосы
            a = cv2.rectangle(white_mask, (first_white-5,
                              middle_h), (0, middle_h), 255, 10)
            cv2.imshow("img_w", a)
            cv2.waitKey(1)

        return (first_white, middle_h)

    # Расчет новой угловой скорости с использованием PID-регулятора
    def _compute_PID(self, target):
        # расчет новой угловой скорости с помощью PID-регулятора
        err = target
        e = np.arctan2(np.sin(err), np.cos(err))

        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e

        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        w = np.arctan2(np.sin(w), np.cos(w))

        self.E = self.E + e
        self.old_e = e
        return w

    # Обратный вызов для обработки данных с камеры
    def _callback_Ccamera(self, msg: Image):
        emptyTwist = Twist()
        emptyTwist.linear.x = self._linear_speed

        # обрабатываем изо с камеры
        cvImg = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

        # получаем изо перед колесами
        persective = self.__warpPerspective(cvImg)

        # получаем координаты края желтой линии и белой
        endYellow, hYellow = self._find_yellow_line(persective)
        startWhite, hWhite = self._find_white_line(persective)

        h, w, _ = persective.shape

        middle_btw_lines = (startWhite + endYellow) // 2

        center_crds = (w//2, hYellow)
        lines_center_crds = (middle_btw_lines, hYellow)

        # центр справа - положительно, центр слева - отрицательно
        direction = center_crds[0] - lines_center_crds[0]

        # обработка первой таски, светофор
        if self.TASK_LEVEL == 0:
            check_traffic_lights(self, persective)

        # Выравниваем наш корабль
        # если центры расходятся больше чем нужно
        if (abs(direction) > OFFSET_BTW_CENTERS):
            angle_to_goal = math.atan2(
                direction, 215)
            # if DEBUG_LEVEL >= 1:
                # self.get_logger().info(
                #     f"Rotating dist: {abs(direction)}")
                # self.get_logger().info(f"Angle Error: {angle_to_goal}")
            angular_v = self._compute_PID(angle_to_goal)
            emptyTwist.angular.z = angular_v
            # self.get_logger().info(f"Angle Speed: {angular_v}")
            # self.get_logger().info("----------------------------")

            emptyTwist.linear.x = self._linear_slow_speed

        if DEBUG_LEVEL >= 1:
            # рисуем точки
            persective_drawed = cv2.rectangle(
                persective, center_crds, center_crds, (0, 255, 0), 5)  # Центр изо
            persective_drawed = cv2.rectangle(
                persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 5)  # центр точки между линиями
            # по сути пытаемся соединить центр изо с центром между линиями, т.е. поставить синюю точку на зеленую
            cv2.imshow("img", persective_drawed)
            cv2.waitKey(1)

        # изменение управление машинкой
        if DEBUG_LEVEL < 4 and self.STATUS_CAR == 1:
            self._robot_cmd_vel_pub.publish(emptyTwist)


def main():
    rclpy.init()
    FTN = Follow_Trace_Node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()
