#!/usr/bin/env python

import rospy
import traceback
from nodes import OdomNode
import math
from costmap_merge.srv import Handshake1, Handshake2, Handshake1Response
from helpers import TransformHelper, PoseHelper


class DetectionHandshake:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('/' + str(self.namespace) + '/robot_type')
        self.robots_names = rospy.get_param('/simulation_launcher/robots_names')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for robot in self.robots_names:
            self.robots[robot] = OdomNode(robot)
            coordinates = rospy.get_param('/simulation_launcher/' + str(self.namespace) + '/coordinates')
            self.robots[robot].set_start_pose('/map', rospy.Time.now(), coordinates)
        # Service to receive the robot poses in the detector's frame
        rospy.Service('handshake1_service', Handshake1, self.detector_call)
        # Initializes the proxies dictionary to communicate with the detection_manager node
        self.handshake2_proxies = dict()

    def detector_call(self, msg):
        """ The detector calls providing its position and angle relative to this robot. """
        pose_D_R = self.find_detector_in_costmap(msg.detector_ns)
        pose_R_R = PoseHelper.get_pose_from_odom(self.robots[self.namespace].odom)
        self.calculate_transformed_odom(self.namespace)
        print("detector: " + str(msg.detector_ns) + " " + str(self.robots[msg.detector_ns].transformed_odom))
        print("robot: " + str(self.namespace) + " " + str(self.robots[self.namespace].transformed_odom))
        beta = math.atan2(self.robots[msg.detector_ns].transformed_odom.pose.position.y - self.robots[
            self.namespace].transformed_odom.pose.position.y,
                          self.robots[msg.detector_ns].transformed_odom.pose.position.x - self.robots[
                              self.namespace].transformed_odom.pose.position.x)
        self.handshake2(msg.detector_ns, msg.pose_D_D, msg.pose_R_D, msg.alpha, self.namespace, pose_D_R, pose_R_R,
                        beta)
        return Handshake1Response()

    def find_detector_in_costmap(self, robot):
        """ This function emulates the search of the detector in the robot's costmap. First we calculate the detector's
                pose in the virtual map frame and then we translate these coordinates to the robot's odom frame"""
        self.calculate_transformed_odom(robot)
        return self.get_pose_D_R(robot)

    def calculate_transformed_odom(self, robot):
        """ Calculates the robot's odometry in the map frame. The result is saved in the robot's transformed_odom """
        yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
        x = self.robots[robot].start.pose.position.x + self.robots[robot].odom.pose.pose.position.x * math.cos(yaw) - \
            self.robots[robot].odom.pose.pose.position.y * math.sin(yaw)
        y = self.robots[robot].start.pose.position.y + self.robots[robot].odom.pose.pose.position.y * math.cos(yaw) + \
            self.robots[robot].odom.pose.pose.position.x * math.sin(yaw)
        transformed_yaw = yaw + PoseHelper.get_yaw_from_orientation(self.robots[robot].odom.pose.pose.orientation)
        self.robots[robot].set_transformed_odom('/map', rospy.Time.now(), [x, y, 0, 0, 0, transformed_yaw])

    def get_pose_D_R(self, robot):
        """ The robot is found in the costmap using a pattern recognition technique. Here we emulate this behavior by
                calculating the detector's coordinates in the robot/odom frame. Keep in mind that we don't know the robot's
                orientation, therefore we set it to zero """
        x = self.robots[robot].transformed_odom.pose.position.x - self.robots[self.namespace].start.pose.position.x
        y = self.robots[robot].transformed_odom.pose.position.y - self.robots[self.namespace].start.pose.position.y
        x, y = PoseHelper.rotate([self.robots[self.namespace].start.pose.position.x,
                                  self.robots[self.namespace].start.pose.position.y], [x, y],
                                 PoseHelper.get_yaw_from_orientation(
                                     self.robots[self.namespace].start.pose.orientation))
        pose_D_R = PoseHelper.set_2D_pose(str(self.namespace) + '/odom', rospy.Time.now(), [x, y, 0, 0, 0, 0])
        return pose_D_R

    def handshake2(self, detector_ns, pose_D_D, pose_R_D, alpha, robot_ns, pose_D_R, pose_R_R, beta):
        """ Sends the detector and robot poses in the detector frame to the detection_manager node in the Detector """
        pose_D_R, pose_R_D = TransformHelper.get_poses_transform(pose_D_D, pose_R_D, pose_R_R, pose_D_R, alpha, beta)
        try:
            self.handshake2_proxies[detector_ns](detector_ns, pose_D_D, pose_R_D, alpha, robot_ns, pose_D_R, pose_R_R,
                                                 beta)
        except KeyError:
            rospy.wait_for_service('/' + detector_ns + '/handshake2_service')
            self.handshake2_proxies[detector_ns] = rospy.ServiceProxy('/' + detector_ns + '/handshake2_service',
                                                                      Handshake2)
            self.handshake2_proxies[detector_ns](detector_ns, pose_D_D, pose_R_D, alpha, robot_ns, pose_D_R, pose_R_R,
                                                 beta)


if __name__ == "__main__":

    try:
        rospy.init_node('detection_handshake', log_level=rospy.INFO)
        rospy.loginfo('[detection_handshake]: Node started')
        detection_handshake = DetectionHandshake()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[detection_handshake]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)