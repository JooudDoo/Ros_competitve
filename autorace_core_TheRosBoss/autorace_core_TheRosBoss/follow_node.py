from typing import Literal
from collections import deque

# модули
# конфигуратор
from module.config import (
    OFFSET_BTW_CENTERS,
    # TASK_LEVEL,
    DEBUG_LEVEL,
    LINES_H_RATIO,
    MAXIMUM_ANGLUAR_SPEED_CAP,
    MAX_LINIEAR_SPEED,
    ANALOG_CAP_MODE,

    LINE_HISTORY_SIZE,
    WHITE_MODE_CONSTANT,
    YELLOW_MODE_CONSTANT,
    FOLLOW_ROAD_CROP_HALF,
)

# обработка светофора
from module.traffic_lights import check_traffic_lights
# обработка поворота
from module.traffic_intersection import check_direction
# обработка стен
from module.traffic_construction import avoid_walls
# обработка парковочного места
from module.parking_space import parking
# обработка пешеходного перехода
from module.pedestrian_crossing import stop_crosswalk
# обработка логов
from module.logger import log_info
# обработка туннеля
from module.tunnel_space import go_tunnel_space

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import LaserScan

import cv2
import math
import numpy as np


class Follow_Trace_Node(Node):

    def __init__(self, linear_speed=MAX_LINIEAR_SPEED):
        """
        Основная логика
        > point_status - статус центровой точки дороги (True - актуально, False - устарела),

        > pose - позиция машинки

        > STATUS_CAR - статус машинки (0 - стоп, 1 - ехать),

        > TASK_LEVEL - уровень задания которое нужно выполнить (0: светофор, 1: движение, 2: перекресток, 
                                                                3: движение, 4: обход преград, 5: парковка)
        """
        super().__init__("Follow_Trace_Node")

        # статус центровой точки дороги (True - актуально, False - устарела)
        self.point_status = True

        self._pose_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback, 10)
        self._robot_Ccamera_sub = self.create_subscription(
            Image, "/color/image", self._callback_Ccamera, 3)
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._sign_subscriber = self.create_subscription(
            String, '/sign', self._change_task, 1)
        self._cv_bridge = CvBridge()

        self._linear_speed = linear_speed
        self._lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self._yellow_prevs = deque(maxlen=LINE_HISTORY_SIZE)
        self.__white_prevs = deque(maxlen=LINE_HISTORY_SIZE)
        self._yellow_prevs.append(0)
        self.__white_prevs.append(0)

        self.pose = Odometry()
        self.lidar_data = LaserScan()

        self.Kp = self.declare_parameter('Kp', value=3.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value
        self.Ki = self.declare_parameter('Ki', value=1, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value
        self.Kd = self.declare_parameter('Kd', value=0.25, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE)).get_parameter_value().double_value

        self.avoidance = 0
        self.old_e = 0
        self.E = 0
        self.parking_status = 0

        self.STATUS_CAR = 0
        self.TASK_LEVEL = 0 

        self.MAIN_LINE = "YELLOW"

    # Обратный вызов для получения данных о положении
    def pose_callback(self, data):
        self.pose = data

    # Переключение миссий в результате детекции знаков
    def _change_task(self, msg):
        task_name = msg.data

        log_info(self, f"Смена задания на {task_name}",
                 debug_level=1, msg_id=0, allow_repeat=True)

        if task_name == "PedestrianCrossing":
            self.TASK_LEVEL = 4
        elif task_name == "TrafficConstruction":
            self.TASK_LEVEL = 2
        elif task_name == "TrafficIntersection":
            self.TASK_LEVEL = 1
        elif task_name == "TrafficParking":
            self.TASK_LEVEL = 3
        elif task_name == "Tunnel":
            self.TASK_LEVEL = 5
        elif task_name in ["YELLOW", "WHITE"]:
            self.MAIN_LINE = task_name

    # Получение угла поворота из данных о положении
    def get_angle(self):
        quaternion = (self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                      self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        return euler[2]

    # Преобразование перспективы изображения
    def _warpPerspective(self, cvImg):
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
    def _find_yellow_line(self, perspectiveImg_, middle_h=None):
        if FOLLOW_ROAD_CROP_HALF:
            h_, w_, _ = perspectiveImg_.shape
            perspectiveImg = perspectiveImg_[:, :w_//2, :]
        else:
            perspectiveImg = perspectiveImg_

        h, w, _ = perspectiveImg.shape
        if middle_h is None:
            middle_h = int(h * LINES_H_RATIO)

        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

        middle_row = yellow_mask[middle_h]
        try:
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
            self._yellow_prevs.append(first_notYellow)
        except:  # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_notYellow = sum(
                self._yellow_prevs)//len(self._yellow_prevs)
            self.point_status = False

        if DEBUG_LEVEL >= 3:
            # рисуем линию откуда идет конец желтой полосы
            a = cv2.rectangle(yellow_mask, (first_notYellow+5,
                              middle_h), (w, middle_h), 255, 10)
            cv2.imshow("img_y", a)
            cv2.waitKey(1)

        return first_notYellow

    # Поиск белой линии на изображении
    def _find_white_line(self, perspectiveImg_, middle_h=None):
        fix_part = 0  # значения для исправления обрезки пополам

        if FOLLOW_ROAD_CROP_HALF:
            h_, w_, _ = perspectiveImg_.shape
            perspectiveImg = perspectiveImg_[:, w_//2:, :]
            fix_part = w_//2
        else:
            perspectiveImg = perspectiveImg_

        h, w, _ = perspectiveImg.shape
        if middle_h is None:
            middle_h = int(h * LINES_H_RATIO)

        white_mask = cv2.inRange(
            perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.__white_prevs.append(first_white)
        except:  # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_white = sum(self.__white_prevs)//len(self.__white_prevs)
            self.point_status = False

        if DEBUG_LEVEL >= 3:
            # рисуем линию откуда идет конец белой полосы
            a = cv2.rectangle(white_mask, (first_white-5,
                              middle_h), (0, middle_h), 255, 10)
            cv2.imshow("img_w", a)
            cv2.waitKey(1)

        return first_white + fix_part

    # Расчет новой угловой скорости с использованием PID-регулятора
    def _compute_PID(self, target):
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

    # Обработка данных лидара
    def lidar_callback(self, data):
        # Обработка данных лидара
        self.lidar_data = data

    # Обратный вызов для обработки данных с камеры
    def _callback_Ccamera(self, msg: Image):

        self.point_status = True

        emptyTwist = Twist()
        emptyTwist.linear.x = self._linear_speed

        # обрабатываем изо с камеры
        cvImg = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

        # получаем изо перед колесами
        perspective = self._warpPerspective(cvImg)
        perspective_h, persective_w, _ = perspective.shape

        hLevelLine = int(perspective_h*LINES_H_RATIO)

        # получаем координаты края желтой линии и белой
        if self.MAIN_LINE == "WHITE":
            endYellow = WHITE_MODE_CONSTANT
        else:
            endYellow = self._find_yellow_line(perspective, hLevelLine)
        if self.MAIN_LINE == "YELLOW":
            startWhite = YELLOW_MODE_CONSTANT
        else:
            startWhite = self._find_white_line(perspective, hLevelLine)

        middle_btw_lines = (startWhite + endYellow) // 2

        center_crds = (persective_w//2, hLevelLine)
        lines_center_crds = (middle_btw_lines, hLevelLine)

        # центр справа - положительно, центр слева - отрицательно
        direction = center_crds[0] - lines_center_crds[0]

        # обработка первой таски, светофор
        if self.TASK_LEVEL == 0:
            check_traffic_lights(self, cvImg)

        if self.TASK_LEVEL == 1:
            # self.get_logger().info(f"Angle: {self.get_angle()}")
            check_direction(self, cvImg)

        if self.TASK_LEVEL == 2:
            self.MAIN_LINE = "WHITE"
            avoid_walls(self, cvImg)

        if self.TASK_LEVEL == 3:
            self.MAIN_LINE = "YELLOW"
            parking(self, cvImg)

        if self.TASK_LEVEL == 4:
            self.MAIN_LINE = "YELLOW"
            stop_crosswalk(self, cvImg)

        if self.TASK_LEVEL == 5:
            go_tunnel_space(self, cvImg)

        # self.get_logger().info(f"Task Level: {self.TASK_LEVEL}")
        # Выравниваем наш корабль
        # если центры расходятся больше чем нужно
        if (abs(direction) > OFFSET_BTW_CENTERS):
            angle_to_goal = math.atan2(
                direction, 215)
            # if DEBUG_LEVEL >= 1:
            # self.get_logger().info(
            #    f"Rotating dist: {abs(direction)}")
            # self.get_logger().info(f"Angle Error: {angle_to_goal}")
            angular_v = self._compute_PID(angle_to_goal)
            emptyTwist.angular.z = angular_v
            # self.get_logger().info(f"Angle Speed: {angular_v}")
            # self.get_logger().info("----------------------------")

            if ANALOG_CAP_MODE:
                angular_v *= 3/4
            emptyTwist.linear.x = abs(
                self._linear_speed * (MAXIMUM_ANGLUAR_SPEED_CAP - abs(angular_v)))

        if DEBUG_LEVEL >= 1:
            # рисуем точки
            persective_drawed = cv2.rectangle(
                perspective, center_crds, center_crds, (0, 255, 0), 5)  # Центр изо
            if self.point_status:
                persective_drawed = cv2.rectangle(
                    persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 5)  # центр точки между линиями
            else:
                persective_drawed = cv2.rectangle(
                    persective_drawed, lines_center_crds, lines_center_crds, (99, 99, 88), 5)  # центр точки между линиями
            # по сути пытаемся соединить центр изо с центром между линиями, т.е. поставить синюю точку на зеленую
            cv2.imshow("img", persective_drawed)
            cv2.waitKey(1)

        # изменение управление машинкой
        if DEBUG_LEVEL < 4 and self.STATUS_CAR == 1 and self.avoidance == 0 and self.parking_status == 0:
            self._robot_cmd_vel_pub.publish(emptyTwist)


def main():
    rclpy.init()
    FTN = Follow_Trace_Node()
    rclpy.spin(FTN)
    FTN.destroy_node()
    rclpy.shutdown()
