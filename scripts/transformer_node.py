#!/usr/bin/env python

import traceback
import rospy
from costmap_merge.srv import TransformerData, TransformerDataResponse
from helpers import TransformHelper as th
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TransformerNode:
    def __init__(self):
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # The transform to be published
        self.t = TransformStamped()
        # Flags
        self.transform_received = False
        # Service to communicate with the transform managers
        rospy.Service('/transformer_node_service', TransformerData, self.cb_action)

    def cb_action(self, msg):
        self.set_transform(msg)
        if msg.action == 'START':
            self.send_transform()
        return TransformerDataResponse()

    def set_transform(self, msg):
        self.transform_received = True
        self.t = th.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)

    def send_transform(self):
        while self.transform_received:
            self.br.sendTransform(self.t)
            rospy.sleep(0.5)


if __name__ == "__main__":

    try:
        rospy.init_node('transformer_node', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo('[transformer_node]: Node started')
        node = TransformerNode()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[transformer_node]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
