#!/usr/bin/env python

import traceback
import rospy
import tf2_ros


class FrameLinker:

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        # Server for creating and updating robots
        self.service = rospy.Service('link_to_map_frame', RobotUpdate, self.cb_update_robot)

    def transform_map_to_odom(self, transform):
        self.br.sendTransform(transform)


if __name__ == "__main__":

    try:
        rospy.init_node('transform_publisher', log_level=rospy.INFO)
        rospy.loginfo('[transform_publisher]: Node started')
        fl = FrameLinker()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[transform_publisher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
