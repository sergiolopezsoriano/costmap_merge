#!/usr/bin/env python

import rospy
import traceback
import robot as rb
import math
from costmap_merge.srv import Transforms
from helpers import TransformHelper, PoseHelper
import numpy as np


class TransformBroadcaster:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.robots_names = rospy.get_param('~robots_names')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = rb.OdomNode(namespace)
            self.robots[namespace].pose = TransformHelper.get_map_to_odom_transform(namespace)
        # Service receiving poses and angles for transforms computation
        self.transform_calculator_service = rospy.Service('transform_calculator_service', Transforms,
                                                          self.cb_transform_calculator())

    def cb_transform_calculator(self, msg):
        TransformHelper.get_pose_transform()
        return True


if __name__ == "__main__":

    try:
        rospy.init_node('transform_broadcaster', log_level=rospy.INFO)
        rospy.loginfo('[transform_broadcaster]: Node started')
        tb = TransformBroadcaster()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[transform_broadcaster]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
