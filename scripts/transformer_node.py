#!/usr/bin/env python

import traceback
import rospy
from costmap_merge.srv import Transform, TransformResponse
from helpers import TransformHelper as th
import tf2_ros
from geometry_msgs.msg import TransformStamped
from threading import Thread
import signal


class TransformerNode(Thread):
    def __init__(self):
        super(TransformerNode, self).__init__()
        # # Getting ROS parameters
        # self.namespace = rospy.get_namespace().strip('/')
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # The transform to be published
        self.t = TransformStamped()
        # Flags
        self.transform_received = False
        # Service to communicate with the transform managers
        rospy.Service('transformer_node_service', Transform, self.cb_action)
        # manage keyboard interrupt due to open processes after exiting
        signal.signal(signal.SIGINT, self.signal_handler)

    def cb_action(self, msg):
        self.set_transform(msg)
        if msg.action == 'START':
            self.start()
        return TransformResponse()

    def set_transform(self, msg):
        self.transform_received = True
        self.t = th.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)

    def run(self):
        while self.transform_received:
            self.br.sendTransform(self.t)
            rospy.sleep(0.5)

    @staticmethod
    def signal_handler():
        rospy.signal_shutdown('')


if __name__ == "__main__":

    try:
        rospy.init_node('transformer_node', log_level=rospy.INFO)
        node = TransformerNode()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[transformer_node]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
