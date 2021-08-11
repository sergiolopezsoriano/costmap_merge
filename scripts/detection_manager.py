#!/usr/bin/env python

import rospy
import traceback
from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import UpdateRobots
from helpers import TransformHelper, PoseHelper
import tf2_ros


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
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # Dictionary of Flags to decide whether to broadcast a transform or not depending on the robot discovery and the
        # elapsed time since the last detection
        self.flags = dict()
        # List of detected robots
        self.robots = list()

    def cb_detection_manager(self, msg):
        self.robots.append(msg.robot_ns)
        self.flags[msg.robot_ns] = True
        t = TransformHelper.get_frame_transform(msg.pose_D_R, msg.pose_D_D, msg.alpha, msg.beta)
        self.br.sendTransform(t)
        self.poses[msg.robot_ns] = msg.pose_R_D
        return Handshake2Response()

    def publish_robots_poses(self, poses):
        self.robot_pose_publisher.publish(poses)


if __name__ == "__main__":

    try:
        rospy.init_node('detection_manager', log_level=rospy.INFO)
        rospy.loginfo('[detection_manager]: Node started')
        manager = DetectionManager()
        while not rospy.is_shutdown():
            if manager.robots:
                for rb in manager.robots:
                    pass
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[detection_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
