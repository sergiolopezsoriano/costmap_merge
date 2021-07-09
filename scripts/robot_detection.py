#!/usr/bin/env python

import rospy
import traceback
import robot as rb
import math
from costmap_merge.srv import RobotDetected, RobotDetectedResponse


class RobotDetection:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('~robot_type')
        self.robots_names = rospy.get_param('~robots_names')
        self.min_detector_distance = rospy.get_param('~min_detector_distance')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = rb.OdomNode(namespace)

    def detect_robots(self):
        """ AI object identification """
        for robot in self.robots:
            if robot != self.namespace:
                d = math.sqrt(
                    (self.robots[robot].odom.pose.pose.x - self.robots[self.namespace].odom.pose.pose.x) ** 2 + (
                                self.robots[robot].odom.pose.pose.y - self.robots[
                            self.namespace].odom.pose.pose.y) ** 2)
                if d < self.min_detector_distance:
                    rospy.wait_for_service('/' + self.namespace + '/robot_communication_service')
                    rospy.ServiceProxy('/' + robot + '/robot_detection', RobotDetected)


if __name__ == "__main__":

    try:
        rospy.init_node('robot_detection', log_level=rospy.INFO)
        rospy.loginfo('[robot_detection]: Node started')
        rd = RobotDetection()
        while not rospy.is_shutdown():
            rd.detect_robots()


    except Exception as e:
        rospy.logfatal('[robot_detection]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
