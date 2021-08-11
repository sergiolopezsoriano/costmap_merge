#!/usr/bin/env python

import rospy
import traceback
from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import RobotName
from helpers import TransformHelper
import tf2_ros


class DetectionManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service receiving poses and angles for transforms computation
        rospy.Service('handshake2_service', Handshake2, self.cb_detection_manager)
        # Publishes all known robot poses in the detector frame.
        self.poses = dict()
        self.robot_list_publisher = rospy.Publisher('/detected_robots_topic', RobotName, queue_size=10)
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.StaticTransformBroadcaster()
        # List of detected robots
        self.detected_robots = list()

    def cb_detection_manager(self, msg):
        if msg.robot_ns not in self.detected_robots:
            self.detected_robots.append(msg.robot_ns)
        t = TransformHelper.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)
        self.br.sendTransform(t)
        return Handshake2Response()

    def publish_robots_list(self):
        for robot in self.detected_robots:
            self.robot_list_publisher.publish(robot)


if __name__ == "__main__":

    try:
        rospy.init_node('detection_manager', log_level=rospy.INFO)
        rospy.loginfo('[detection_manager]: Node started')
        manager = DetectionManager()
        while not rospy.is_shutdown():
            if manager.detected_robots:
                manager.publish_robots_list()
            rospy.sleep(10)

    except Exception as e:
        rospy.logfatal('[detection_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
