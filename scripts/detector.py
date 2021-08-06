#!/usr/bin/env python

import rospy
import traceback
from nodes import OdomNode
import math
from costmap_merge.srv import Handshake
from costmap_merge.msg import UpdateRobots
from helpers import PoseHelper
import numpy as np


class Detector:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('/' + str(self.namespace) + '/robot_type')
        self.robots_names = rospy.get_param('/' + str(self.namespace) + '/robots_names')
        self.min_detector_distance = rospy.get_param('/' + str(self.namespace) + '/min_detector_distance')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = OdomNode(namespace)
            coordinates = rospy.get_param('/' + str(self.namespace) + '/coordinates')
            print(str(self.namespace) + ': ' + str(coordinates))
            self.robots[namespace].set_start_pose('/gazebo_world', rospy.Time.now(), coordinates)
        # Iinitializes the proxies for the robot_detection service
        self.robot_detection_proxies = dict()
        # Publishes all known robot poses in the detector frame.
        # It also publishes the detector pose in all known robots frames
        self.poses = []
        self.transform_publisher = rospy.Publisher('/' + self.namespace + '/detected_robots_topic', UpdateRobots, queue_size=10)

    def set_robots(self):
        for robot in self.robots:
            yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
            x = self.robots[robot].start.pose.position.x + self.robots[
                robot].odom.pose.pose.position.x * np.cos(yaw) - self.robots[robot].odom.pose.pose.position.y * np.sin(
                yaw)
            y = self.robots[robot].start.pose.position.y + self.robots[
                robot].odom.pose.pose.position.y * np.cos(yaw) + self.robots[robot].odom.pose.pose.position.x * np.sin(
                yaw)
            self.robots[robot].set_transformed_pose('/map', rospy.Time.now(),
                                                    [x, y, 0, 0, 0, yaw + PoseHelper.get_yaw_from_orientation(
                                                        self.robots[robot].odom.pose.pose.orientation)])

    def detect_robots(self):
        """ Returns the namespace of the detected robot. Detection and Location are based on sharing a common map frame.
        This is only possible in simulation. It needs to be modified for real time operation"""
        self.set_robots()
        for robot in self.robots:
            if robot != self.namespace:
                d = math.sqrt((self.robots[robot].transformed.pose.position.x - self.robots[
                    self.namespace].transformed.pose.position.x) ** 2 + (
                                          self.robots[robot].transformed.pose.position.y - self.robots[
                                      self.namespace].transformed.pose.position.y) ** 2)
                if d < self.min_detector_distance:
                    self.locate_robot(robot)

    def locate_robot(self, robot):
        """ AI robot location must provide pose_D_D, pose_R_D and alpha """
        pose_D_D = self.robots[self.namespace].transformed
        pose_R_D = self.robots[robot].transformed
        alpha = math.atan2((self.robots[robot].transformed.pose.position.y - self.robots[
            self.namespace].transformed.pose.position.y) / (
                                       self.robots[robot].transformed.pose.position.x - self.robots[
                                   self.namespace].transformed.transformed.pose.position.x))
        self.call_robot(robot, pose_D_D, pose_R_D, alpha)

    def call_robot(self, robot, pose_D_D, pose_R_D, alpha):
        if self.robot_detection_proxies[robot] is None:
            rospy.wait_for_service('/' + robot + '/detection_handshake_service')
            self.robot_detection_proxies[robot] = rospy.ServiceProxy('/' + robot + '/detection_handshake_service', Handshake)
        response = self.robot_detection_proxies[robot](self.namespace, pose_D_D, pose_R_D, alpha)
        rospy.loginfo('[detector]: response from ' + robot + ' = ' + str(response))
        msg = UpdateRobots()
        msg.pose = response.pose_R_D
        msg.origin = response.robot_ns
        msg.destination = response.detector_ns
        self.poses.append(msg)
        msg.pose = response.pose_D_R
        msg.destination = response.robot_ns
        msg.origin = response.detector_ns
        self.poses.append(msg)

    def publish_transforms(self, response):
        self.transform_publisher.publish()


if __name__ == "__main__":

    try:
        rospy.init_node('detector', log_level=rospy.INFO)
        rospy.loginfo('[detector]: Node started')
        detector = Detector()
        rospy.sleep(1)
        while not rospy.is_shutdown():
            # detector.detect_robots()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[detector]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
