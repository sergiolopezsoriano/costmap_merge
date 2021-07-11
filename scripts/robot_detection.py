#!/usr/bin/env python

import rospy
import traceback
import robot as rb
import math
from costmap_merge.srv import RobotDetected, RobotDetectedResponse
import cn_lib
import numpy as np


class RobotDetection:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.robots_names = rospy.get_param('~robots_names')
        self.min_detector_distance = rospy.get_param('~min_detector_distance')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = rb.OdomNode(namespace)
            self.robots[namespace].pose = cn_lib.get_map_to_odom_transform(namespace)
        # Iinitializes the proxy for the robot_detection service
        rospy.wait_for_service('/' + self.namespace + '/robot_detection_service')
        self.robot_detection_proxy = rospy.ServiceProxy('/' + self.namespace + '/robot_detection_service',
                                                        RobotDetected)

    def detect_robots(self):
        """ AI object identification """
        for robot in self.robots:
            yaw = cn_lib.get_yaw_from_orientation(self.robots[robot].pose.pose.orientation)
            self.robots[robot].x = self.robots[robot].pose.pose.position.x + self.robots[
                robot].odom.pose.pose.position.x * np.cos(yaw) - self.robots[
                                       robot].odom.pose.pose.position.y * np.sin(yaw)
            self.robots[robot].y = self.robots[robot].pose.pose.position.y + self.robots[
                robot].odom.pose.pose.position.y * np.cos(yaw) + self.robots[
                                       robot].odom.pose.pose.position.x * np.sin(yaw)
        for robot in self.robots:
            if robot != self.namespace:
                d = math.sqrt((self.robots[robot].x - self.robots[self.namespace].x) ** 2 + (
                            self.robots[robot].y - self.robots[self.namespace].y) ** 2)
                # rospy.loginfo('[robot_detection]: distance detector_' + str(self.namespace) + ' - ' + str(
                #     robot) + ' = ' + str(d))
                if d < self.min_detector_distance:
                    response = self.robot_detection_proxy(robot)
                    # rospy.loginfo('[robot_detection]: connected = ' + str(response))


if __name__ == "__main__":

    try:
        rospy.init_node('robot_detection', log_level=rospy.INFO)
        rospy.loginfo('[robot_detection]: Node started')
        rd = RobotDetection()
        rospy.sleep(5)
        while not rospy.is_shutdown():
            rd.detect_robots()
            rospy.sleep(5)


    except Exception as e:
        rospy.logfatal('[robot_detection]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
