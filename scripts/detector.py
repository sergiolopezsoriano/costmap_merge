#!/usr/bin/env python

import rospy
import traceback
from nodes import OdomNode
import math
from costmap_merge.srv import Handshake1
from helpers import PoseHelper


class Detector:
    def __init__(self):
        """ The map frame is used along the handshake nodes (detector.py and detection_handshake.py) in order to have a
        common frame where the calculation of distances and relationships between poses are possible. """
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('/' + str(self.namespace) + '/robot_type')
        self.robots_names = rospy.get_param('/simulation_launcher/robots_names')
        self.min_detection_distance = rospy.get_param('/' + str(self.namespace) + '/min_detection_distance')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for robot in self.robots_names:
            self.robots[robot] = OdomNode(robot)
            coordinates = rospy.get_param('/simulation_launcher/' + str(robot) + '/coordinates')
            self.robots[robot].set_start_pose('/map', rospy.Time.now(), coordinates)
        # Initializes the proxies dictionary to communicate with the detection_handshake node
        self.handshake1_proxies = dict()

    def set_robots(self):
        """ At this stage, we don't know other robots start.pose, therefore we wouldn't be able to calculate the
        transformed_odom pose. This information is only used to detect robots in a circle of radius
        self.min_detection_distance """
        for robot in self.robots:
            yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
            x = self.robots[robot].start.pose.position.x + self.robots[robot].odom.pose.pose.position.x * math.cos(
                yaw) - self.robots[robot].odom.pose.pose.position.y * math.sin(yaw)
            y = self.robots[robot].start.pose.position.y + self.robots[robot].odom.pose.pose.position.y * math.cos(
                yaw) + self.robots[robot].odom.pose.pose.position.x * math.sin(yaw)
            print("TRANSFORMED COORDINATES " + str(robot) + ": " + str(x) + " , " + str(y))
            transformed_yaw = yaw + PoseHelper.get_yaw_from_orientation(self.robots[robot].odom.pose.pose.orientation)
            self.robots[robot].set_transformed_odom('/map', rospy.Time.now(), [x, y, 0, 0, 0, transformed_yaw])

    def detect_robots(self):
        """ Detection and Location are based on sharing the common frame /map.
        This is only possible in simulation. It needs to be modified for real time operation """
        self.set_robots()
        for robot in self.robots:
            if robot != self.namespace:
                d = math.sqrt((self.robots[robot].transformed_odom.pose.position.x - self.robots[
                    self.namespace].transformed_odom.pose.position.x) ** 2 + (
                                          self.robots[robot].transformed_odom.pose.position.y - self.robots[
                                      self.namespace].transformed_odom.pose.position.y) ** 2)
                print("DISTANCE BETWEEN ROBOTS: " + str(d))
                if d < self.min_detection_distance:
                    self.locate_robot(robot)

    def locate_robot(self, robot):
        """ At this point, AI gives us pose_R_D and alpha in the detector's odom frame.
         However, we don't actually know the robot's orientation yet. Alpha calculation is simulated by using the
         robots cartesian coordinates in the emulated map frame. """
        pose_D_D = PoseHelper.get_pose_from_odom(self.robots[self.namespace].odom)
        pose_R_D = self.get_pose_R_D(robot)
        alpha = math.atan2(self.robots[robot].transformed_odom.pose.position.y - self.robots[
            self.namespace].transformed_odom.pose.position.y,
                           self.robots[robot].transformed_odom.pose.position.x - self.robots[
                               self.namespace].transformed_odom.pose.position.x)
        self.handshake1(robot, pose_D_D, pose_R_D, alpha)

    def get_pose_R_D(self, robot):
        """ This is given directly by the AI. However, in this simulation without visual information we need to emulate
         such behavior. Thus, the robot's coordinates in the detector/odom frame are calculated here. Keep in mind that
         we still don't know the robot's orientation, therefore we set its yaw to zero """
        x = self.robots[robot].transformed_odom.pose.position.x - self.robots[self.namespace].start.pose.position.x
        y = self.robots[robot].transformed_odom.pose.position.y - self.robots[self.namespace].start.pose.position.y
        pose_R_D = PoseHelper.set_2D_pose(str(self.namespace) + '/odom', rospy.Time.now(), [x, y, 0, 0, 0, 0])
        return pose_R_D

    def handshake1(self, robot, pose_D_D, pose_R_D, alpha):
        """ Sends the detector and robot poses in the detector frame to the detected robot """
        try:
            self.handshake1_proxies[robot](self.namespace, pose_D_D, pose_R_D, alpha)
        except KeyError:
            rospy.wait_for_service('/' + robot + '/handshake1_service')
            self.handshake1_proxies[robot] = rospy.ServiceProxy('/' + robot + '/handshake1_service', Handshake1)
            self.handshake1_proxies[robot](self.namespace, pose_D_D, pose_R_D, alpha)
        rospy.loginfo('[detector]: Handshake1 message sent to ' + robot)



if __name__ == "__main__":

    try:
        rospy.init_node('detector', log_level=rospy.INFO)
        rospy.loginfo('[detector]: Node started')
        detector = Detector()
        rospy.sleep(1)
        while not rospy.is_shutdown():
            detector.detect_robots()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[detector]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
