#!/usr/bin/env python

import rospy
import traceback
import robot as rb
import math
from costmap_merge.srv import Handshake, HandshakeResponse
from helpers import TransformHelper, PoseHelper
import numpy as np


class HandshakeNode:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('~robot_type')
        self.robots_names = rospy.get_param('~robots_names')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = rb.OdomNode(namespace)
        # Service to send the pose and frame transforms
        self.detector_finder_service = rospy.Service('handshake_service', Handshake, self.find_detector)

    def update_detector(self, robot):
        yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
        x = self.robots[robot].start.pose.position.x + self.robots[
            robot].odom.pose.pose.position.x * np.cos(yaw) - self.robots[robot].odom.pose.pose.position.y * np.sin(
            yaw)
        y = self.robots[robot].start.pose.position.y + self.robots[
            robot].odom.pose.pose.position.y * np.cos(yaw) + self.robots[robot].odom.pose.pose.position.x * np.sin(
            yaw)
        self.robots[robot].set_transformed('/map', x, y, yaw + PoseHelper.get_yaw_from_orientation(
            self.robots[robot].odom.pose.pose.orientation))

    def find_detector(self, msg):
        self.update_detector(msg.detector_ns)
        pose_R_D = msg.pose_R_D
        pose_R_D.pose.orientation = int
        pose_D_R = self.find_detector_in_costmap(msg.detector_ns)
        pose_R_R = PoseHelper.get_pose_from_odom(self.robots[self.namespace].odom)
        beta = math.atan2((self.robots[msg.detector_ns].transformed.pose.position.y - self.robots[
            self.namespace].transformed.pose.position.y) / (
                                       self.robots[msg.detector_ns].transformed.pose.position.x - self.robots[
                                   self.namespace].transformed.transformed.pose.position.x))
        return HandshakeResponse(pose_R_D, pose_D_R, pose_R_R, beta)

    def find_detector_in_costmap(self, namespace):
        pose_D_R = PoseHelper.get_pose_from_odom(self.robots[namespace].transformed)
        return pose_D_R


if __name__ == "__main__":

    try:
        rospy.init_node('handshake_node', log_level=rospy.INFO)
        rospy.loginfo('[handshake_node]: Node started')
        hn = HandshakeNode()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[handshake_node]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
