#!/usr/bin/env python

import rospy
import traceback
import math
from costmap_merge.srv import Handshake1, Handshake2, Handshake1Response, RobotIDs
from helpers import TransformHelper, PoseHelper
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry


class PCListener:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.tf_prefix = rospy.get_param('/' + self.namespace + '/tf_prefix')
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
        # Service to receive the robot poses in the detector's frame
        rospy.Service('handshake1_service', Handshake1, self.cb_handshake1)
        # Initializes the proxies dictionary to communicate with the detection_manager node
        self.handshake2_proxies = dict()
        # Subscriber to the darknet topic publishing the location of detected robots
        rospy.Subscriber('/' + self.namespace + '/darknet_ros_3d/markers', MarkerArray, self.cb_darknet, queue_size=1)
        self.marker_array = MarkerArray()
        self.marker_array_buffer = MarkerArray()
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

    def cb_handshake1(self, msg):
        if self.marker_array:
            self.marker_array_buffer = self.marker_array
            # if self.marker_array_buffer.markers[0].header.stamp < rospy.Time.now() - rospy.Duration(1):
            #     return Handshake1Response()
            print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
            pose_D_R = self.rfid_detection(msg.detector_ns)
            if not pose_D_R:
                return Handshake1Response()
            print('BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB')
            pose_R_R = PoseHelper.get_pose_from_odom(self.odom)
            print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC')
            beta = math.atan2(pose_D_R.pose.position.y - pose_R_R.pose.position.y,
                              pose_D_R.pose.position.x - pose_R_R.pose.position.x)
            print('DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD')
            if pose_D_R:
                self.handshake2(msg.detector_ns, msg.pose_R_D, pose_R_R, msg.alpha, beta)
                print('EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE')
            print('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF')
        return Handshake1Response()

    @staticmethod
    def get_distance_to_detection(marker):
        return math.sqrt(marker.pose.position.x ** 2 + marker.pose.position.y ** 2)

    def rfid_detection(self, detector):
        distance = list()
        for marker in self.marker_array_buffer.markers:
            distance.append(self.get_distance_to_detection(marker))
        sorted_dist = distance
        sorted_dist.sort()
        sorted_indexes = [distance.index(dist) for dist in sorted_dist]
        sorted_robots = self.detection_check_proxy(len(sorted_indexes))
        count = 0
        for robot in sorted_robots.sorted_detections:
            if robot == detector:
                return self.get_pose_D_R(count)
            count += 1

    def get_pose_D_R(self, count):
        x = self.marker_array_buffer.markers[count].pose.position.x + self.odom.pose.pose.position.x
        y = self.marker_array_buffer.markers[count].pose.position.y + self.odom.pose.pose.position.y
        pose_D_R = PoseHelper.set_2D_pose(self.tf_prefix + '/odom', rospy.Time.now(), [x, y, 0, 0, 0, 0])
        return pose_D_R

    def handshake2(self, detector_ns, pose_R_D, pose_R_R, alpha, beta):
        """ Sends the detector and robot poses in the detector frame to the detection_manager node in the Detector """
        pose_R_D = TransformHelper.get_pose_orientation(pose_R_D, pose_R_R, alpha, beta)
        try:
            self.handshake2_proxies[detector_ns](self.namespace, pose_R_D, pose_R_R, alpha, beta)
        except KeyError:
            rospy.wait_for_service('/' + detector_ns + '/handshake2_service')
            self.handshake2_proxies[detector_ns] = rospy.ServiceProxy('/' + detector_ns + '/handshake2_service',
                                                                      Handshake2)
            self.handshake2_proxies[detector_ns](self.namespace, pose_R_D, pose_R_R, alpha, beta)


if __name__ == "__main__":

    try:
        rospy.init_node('pc_listener', log_level=rospy.INFO)
        rospy.logdebug('[pc_listener]: Node started')
        pc_listener = PCListener()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[pc_listener]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
