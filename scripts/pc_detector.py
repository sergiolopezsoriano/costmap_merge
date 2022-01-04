#!/usr/bin/env python

import rospy
import traceback
import math
from costmap_merge.srv import Handshake1, Handshake1Request, RobotIDs
from helpers import PoseHelper
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import tf2_ros


class PCDetector:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.tf_prefix = rospy.get_param('/' + self.namespace + '/tf_prefix')
        rospy.loginfo(self.tf_prefix)
        self.sim = rospy.get_param('/use_sim_time')
        if self.sim:
            self.robots_names = rospy.get_param('/robots_names')
        else:
            self.robots_names = rospy.get_param('robots').keys()
        # Odometry
        self.odom = Odometry()
        self.odom_ready = False
        rospy.Subscriber('/' + self.namespace + '/odom', Odometry, self.cb_odom, queue_size=1)
        while not self.odom_ready:
            rospy.sleep(1)
        # Initializes the proxies dictionary to communicate with the detection_handshake node
        self.handshake1_proxies = dict()
        # List with the detected robots
        self.detected_robots = list()
        # Subscriber to the darknet topic publishing the location of detected robots
        rospy.Subscriber('/' + self.namespace + '/darknet_ros_3d/markers', MarkerArray, self.cb_darknet, queue_size=1)
        self.marker_array = MarkerArray()
        self.detections = list()
        # Service proxy to identify robots using RFID
        rospy.wait_for_service('/' + self.namespace + '/robot_identification_srv')
        self.detection_check_proxy = rospy.ServiceProxy('/' + self.namespace + '/robot_identification_srv', RobotIDs)

    def cb_odom(self, msg):
        self.odom = msg
        if not self.odom_ready:
            self.odom_ready = True

    def cb_darknet(self, msg):
        self.marker_array = msg
        if not self.marker_array.markers:
            return
        distance = list()
        for marker in self.marker_array.markers:
            distance.append(self.get_distance_to_detection(marker))
        sorted_dist = distance
        sorted_dist.sort()
        sorted_indexes = [distance.index(dist) for dist in sorted_dist]
        print(sorted_indexes)
        sorted_robots = self.detection_check_proxy(len(sorted_indexes))
        print(sorted_robots.sorted_detections)
        count = 0
        for robot in sorted_robots.sorted_detections:
            if robot not in self.detected_robots:
                self.detected_robots.append(robot)
            else:
                self.detected_robots.remove(robot)
            self.locate_robot(robot, count)
            count += 1

    @staticmethod
    def get_distance_to_detection(marker):
        return math.sqrt(marker.pose.position.x ** 2 + marker.pose.position.y ** 2)

    def locate_robot(self, robot, count):
        pose_D_D = PoseHelper.get_pose_from_odom(self.odom)
        pose_R_D = self.get_pose_R_D(count)
        alpha = math.atan2(pose_R_D.pose.position.y - pose_D_D.pose.position.y,
                           pose_R_D.pose.position.x - pose_D_D.pose.position.x)
        self.handshake1(robot, pose_D_D, pose_R_D, alpha)

    def get_pose_R_D(self, count):
        x = self.marker_array.markers[count].pose.position.x + self.odom.pose.pose.position.x
        y = self.marker_array.markers[count].pose.position.y + self.odom.pose.pose.position.y
        pose_R_D = PoseHelper.set_2D_pose(self.tf_prefix + '/odom', rospy.Time.now(), [x, y, 0, 0, 0, 0])
        return pose_R_D

    def handshake1(self, robot, pose_D_D, pose_R_D, alpha):
        """ Sends the detector and robot poses in the detector frame to the detected robot """
        try:
            self.handshake1_proxies[robot](self.namespace, pose_D_D, pose_R_D, alpha)
        except KeyError:
            rospy.wait_for_service('/' + robot + '/handshake1_service')
            self.handshake1_proxies[robot] = rospy.ServiceProxy('/' + robot + '/handshake1_service', Handshake1)
            self.handshake1_proxies[robot](self.namespace, pose_D_D, pose_R_D, alpha)


if __name__ == "__main__":

    try:
        rospy.init_node('pc_detector', log_level=rospy.INFO)
        rospy.logdebug('[pc_detector]: Node started')
        pc_detector = PCDetector()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[pc_detector]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
