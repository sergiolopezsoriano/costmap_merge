#!/usr/bin/env python

import rospy
import roslaunch
from rospkg import RosPack
import traceback
from random import random, randint
from math import pi


class SimulationLauncher:
    def __init__(self):
        self.robot_types = rospy.get_param('~robot_types')
        self.num_robots = rospy.get_param('~num_robots')
        self.path = RosPack().get_path('costmap_merge')
        self.launch_file = self.path + '/launch/agent.launch'
        self.detector_file = rospy.get_param('~detector_file')
        self.listener_file = rospy.get_param('~listener_file')
        self.random_pose = rospy.get_param('~random_pose')
        self.random_costmap_dimensions = rospy.get_param('~random_costmap_dimensions')
        self.resolution = rospy.get_param('~costmap_resolution')
        self.use_sim_time = rospy.get_param('/use_sim_time')
        self.args = list()
        self.robots_names = list()

    def launch_robots(self):
        for index, robot_type in enumerate(self.robot_types):
            if not self.num_robots[index]:
                continue
            for k in range(self.num_robots[index]):
                self.args.append((self.launch_file, self.configure_robot(robot_type, k + 1)))
        rospy.set_param('/robots_names', self.robots_names)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, self.args)
        parent.start()
        # parent.shutdown()

    def configure_robot(self, robot_type, robot_num):
        robot_ns = robot_type + '_' + str(robot_num)
        self.robots_names.append(robot_ns)
        if self.random_pose:
            x, y, z, roll, pitch, yaw = self.get_random_pose()
        else:
            x, y, z, roll, pitch, yaw = rospy.get_param('~' + robot_ns + '/coordinates')
        if self.random_costmap_dimensions:
            width, height = self.get_random_costmap_dimensions()
        else:
            width, height = rospy.get_param('~' + robot_ns + '/costmap')
        can_detect = rospy.get_param('~' + robot_ns + '/can_detect')
        args = ['robot_ns:=' + robot_ns, 'robot_type:=' + robot_type, 'x:=' + str(x), 'y:=' + str(y),
                'z:=' + str(z), 'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw),
                'width:=' + str(width), 'height:=' + str(height), 'resolution:=' + str(self.resolution),
                'sim:=' + str(self.use_sim_time), 'can_detect:=' + str(can_detect),
                'detector_file:=' + self.detector_file, 'listener_file:=' + self.listener_file]
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
        rospy.init_node('simulation_launcher', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo('[simulation_launcher]: Node started')
        robot_launcher = SimulationLauncher()
        robot_launcher.launch_robots()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[simulation_launcher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
