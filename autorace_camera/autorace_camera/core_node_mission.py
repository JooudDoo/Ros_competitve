import rclpy
from rclpy.node import Node
import subprocess, random
import time, os
from nav_msgs.msg import Odometry
import xml.etree.ElementTree as ET


class ControlMission(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.sub_odom = self.create_subscription(Odometry, '/robot/odom', self.getOdom, 1)
        self.traffic_state = 1
        self.loadMissionModel()
        self.setTraffic()
        self.controlMission()

    def getOdom(self, msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        # down_bar
        if abs(pose_x + 1.4) < 0.15 and abs(pose_y - 1.25) < 0.05 and self.traffic_state == 5:
            self.traffic_state = 6

        # up_bar
        elif abs(pose_x + 1.3) < 0.15 and (pose_y - 1.25) < 0.05 and self.traffic_state == 7:
            self.traffic_state = 8
            self.current_time = time.time()

    def loadMissionModel(self):
        model_dir_path = os.environ.get("GZ_SIM_RESOURCE_PATH").split(":")[-1]

        red_light_path = model_dir_path + '/traffic_light/red.sdf'
        with open(red_light_path, 'r') as rlm:
            self.red_light_model = rlm.read().replace("\n", "")

        yellow_light_path = model_dir_path + '/traffic_light/yellow.sdf'
        with open(yellow_light_path, 'r') as ylm:
            self.yellow_light_model = ylm.read().replace("\n", "")

        green_light_path = model_dir_path + '/traffic_light/green.sdf'
        with open(green_light_path, 'r') as glm:
            self.green_light_model = glm.read().replace("\n", "")

        traffic_left_path = model_dir_path + '/intersection/left.sdf'
        with open(traffic_left_path, 'r') as tlm:
            self.traffic_left_model = tlm.read().replace("\n", "")

        traffic_right_path = model_dir_path + '/intersection/right.sdf'
        with open(traffic_right_path, 'r') as trm:
            self.traffic_right_model = trm.read().replace("\n", "")

        suv_path = model_dir_path + '/suv/suv.sdf'
        with open(suv_path, 'r') as suv:
            self.suv_model = suv.read().replace("\n", "")

        # up_bar_path = model_dir_path + '/traffic_bar_up/model.sdf'
        # up_bar_model = open(up_bar_path, 'r')
        # self.up_bar_model = up_bar_model.read()

        # down_bar_path = model_dir_path + '/traffic_bar_down/model.sdf'
        # down_bar_model = open(down_bar_path, 'r')
        # self.down_bar_model = down_bar_model.read()

    def setTraffic(self):
        
        # Find the specific element and change its values
        parking_stop = random.random()
        x = 0.8 if parking_stop < 0.5 else 0.23
        y = 0.8
        z = 0.05
        modified_suv_model = self.suv_model.replace('0 0 0 0 0 -1.57079632679', f'{x} {y} {z} 0 0 0')

        command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{modified_suv_model}"']

        p = subprocess.run(command)

    def controlMission(self):
        while rclpy.ok():
            if self.traffic_state == 1:  # turn on red light
                
                command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.red_light_model}"']

                p = subprocess.run(command)
                self.traffic_state = 2
                self.current_time = time.time()

            elif self.traffic_state == 2:
                if abs(self.current_time - time.time()) > random.uniform(1, 3):  # turn on yellow light after 1-3s.
                    command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.yellow_light_model}"']

                    p = subprocess.run(command)

                    arg = ["gz", "service", "-s", "/world/course/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", 'name: "traffic_light_red" type: MODEL']

                    p = subprocess.run(arg)
                    self.traffic_state = 3
                    self.current_time = time.time()

            elif self.traffic_state == 3:
                if abs(self.current_time - time.time()) > random.uniform(4, 7):  # turn on green light after 4-7s.
                    command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.green_light_model}"']

                    p = subprocess.run(command)

                    arg = ["gz", "service", "-s", "/world/course/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", 'name: "traffic_light_yellow" type: MODEL']

                    p = subprocess.run(arg)
                    self.traffic_state = 4

            elif self.traffic_state == 4: # intersections
                intersection_direction = random.random()

                if intersection_direction < 0.5:
                    command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.traffic_left_model}"']

                    p = subprocess.run(command)

                else:
                    command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.traffic_right_model}"']

                    p = subprocess.run(command)

                self.traffic_state = 5

            # elif self.traffic_state == 6:  # bar down.
            #     rospy.wait_for_service('gazebo/spawn_sdf_model')
            #     spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            #     spawn_model_prox('up_bar', self.up_bar_model, "robotos_name_space",
            #                      self.initial_pose, "world")
            #     del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            #     del_model_prox('down_bar')
            #     self.traffic_state = 7

            # elif self.traffic_state == 8:  # bar up
            #     if abs(self.current_time - time.time()) > 10:
            #         rospy.wait_for_service('gazebo/spawn_sdf_model')
            #         spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            #         spawn_model_prox('down_bar', self.down_bar_model, "robotos_name_space",
            #                          self.initial_pose, "world")
            #         del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            #         del_model_prox('up_bar')
            #         rospy.signal_shutdown('shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = ControlMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
