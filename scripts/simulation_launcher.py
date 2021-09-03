#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
import traceback
from random import random, randint
from math import pi


class RobotLauncher:
    def __init__(self):
        self.num_detectors = rospy.get_param('~num_detectors')
        self.num_agents = rospy.get_param('~num_agents')
        self.agent_launch_file = rospy.get_param('~agent_launch_file')
        self.detector_launch_file = rospy.get_param('~detector_launch_file')
        self.package = rospkg.get_package_name(self.agent_launch_file)
        self.random_pose = rospy.get_param('~random_pose')
        self.random_costmap_dimensions = rospy.get_param('~random_costmap_dimensions')
        self.resolution = rospy.get_param('~costmap_resolution')
        self.args = list()
        self.robots_names = list()

    def set_ros_params(self):
        rospy.set_param('~robots_names', self.robots_names)

    def launch_robots(self):
        while self.num_detectors > 0:
            self.args.append((self.detector_launch_file, self.configure_robot('detector', self.num_detectors)))
            self.num_detectors -= 1
        while self.num_agents > 0:
            self.args.append((self.agent_launch_file, self.configure_robot('agent', self.num_agents)))
            self.num_agents -= 1
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, self.args)
        parent.start()
        # parent.shutdown()

    def configure_robot(self, robot_type, robot_num):
        robot_ns = str(robot_type) + str(robot_num)
        self.robots_names.append(robot_ns)
        if self.random_pose:
            x, y, z, roll, pitch, yaw = RobotLauncher.get_random_pose()
        else:
            x, y, z, roll, pitch, yaw = rospy.get_param('~' + robot_ns + '/coordinates')
        if self.random_costmap_dimensions:
            width, height = RobotLauncher.get_random_costmap_dimensions()
        else:
            width, height = rospy.get_param('~' + robot_ns + '/costmap')
        args = ['robot_ns:=' + str(robot_ns), 'robot_type:=' + str(robot_type), 'x:=' + str(x), 'y:=' + str(y),
                'z:=' + str(z), 'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw),
                'width:=' + str(width), 'height:=' + str(height), 'resolution:=' + str(self.resolution)]
        return args

    @staticmethod
    def get_random_pose():
        x = randint(-6, 6) * random()
        y = randint(-6, 6) * random()
        z = 0
        roll = 0
        pitch = 0
        yaw = pi / 4 * randint(-4, 4)
        return x, y, z, roll, pitch, yaw

    @staticmethod
    def get_random_costmap_dimensions():
        width = randint(-10, 10) * random()
        height = randint(-10, 10) * random()
        return width, height


if __name__ == "__main__":

    try:
        rospy.init_node('robot_launcher', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo('[robot_launcher]: Node started')
        robot_launcher = RobotLauncher()
        robot_launcher.launch_robots()
        robot_launcher.set_ros_params()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[robot_launcher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
