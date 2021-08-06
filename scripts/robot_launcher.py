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
        self.num_robots = rospy.get_param('~num_robots')
        self.detector_launch_file = rospy.get_param('~detector_launch_file')
        self.robot_launch_file = rospy.get_param('~robot_launch_file')
        self.package = rospkg.get_package_name(self.detector_launch_file)
        self.args = list()

    def launch_robots(self):
        count = 0
        while self.num_detectors > 0:
            self.args.append((self.detector_launch_file, self.configure_robot('detector', self.num_detectors)))
            self.num_detectors -= 1
            count += 1
        while self.num_robots > 0:
            self.args.append((self.robot_launch_file, self.configure_robot('robot', self.num_robots)))
            self.num_robots -= 1
            count += 1
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, self.args)
        parent.start()

    @staticmethod
    def configure_robot(robot_type, robot_num):
        # print(rospy.get_param('~d' + str(self.num_detectors)))
        namespace = str(robot_type) + str(robot_num)
        x, y, z, roll, pitch, yaw = RobotLauncher.create_random_pose()
        args = ['robot_ns:=' + str(namespace), 'x:=' + str(x), 'y:=' + str(y), 'z:=' + str(z),
                'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw)]
        return args

    @staticmethod
    def create_random_pose():
        x = randint(-6, 6) * random()
        y = randint(-6, 6) * random()
        z = 0
        roll = 0
        pitch = 0
        yaw = pi / 4 * randint(-4, 4)
        return x, y, z, roll, pitch, yaw


if __name__ == "__main__":

    try:
        rospy.init_node('robot_launcher', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo('[robot_launcher]: Node started')
        robot_launcher = RobotLauncher()
        robot_launcher.launch_robots()

    except Exception as e:
        rospy.logfatal('[robot_launcher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
