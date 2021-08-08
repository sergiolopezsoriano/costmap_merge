#!/usr/bin/env python

import rospy
import traceback
from nodes import OdomNode
from costmap_merge.srv import Handshake2
from costmap_merge.msg import UpdateRobots
from std_srvs.srv import EmptyResponse
from helpers import TransformHelper, PoseHelper
import tf


class DetectionManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service receiving poses and angles for transforms computation
        rospy.Service('handshake2_service', Handshake2, self.cb_detection_manager)
        # Publishes all known robot poses in the detector frame.
        self.poses = dict()
        self.robot_pose_publisher = rospy.Publisher('/' + self.namespace + '/detected_robots_topic', UpdateRobots,
                                                    queue_size=10)

    @staticmethod
    def cb_detection_manager(msg):
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        # t = TransformHelper.get_pose_transform(msg)
        return EmptyResponse()

        # msg = UpdateRobots()
        # msg.pose = response.pose_R_D
        # msg.robot_ns = response.robot_ns
        # msg.detector_ns = response.detector_ns
        # self.poses[response.robot_ns] = msg

    def publish_robots_poses(self, poses):
        self.robot_pose_publisher.publish(poses)


if __name__ == "__main__":

    try:
        rospy.init_node('detection_manager', log_level=rospy.INFO)
        rospy.loginfo('[detection_manager]: Node started')
        manager = DetectionManager()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[detection_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
