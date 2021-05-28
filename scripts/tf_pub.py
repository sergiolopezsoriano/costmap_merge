#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
from geometry_msgs.msg import Pose
import time


class TFPub:

    def __init__(self):
        self.br = tf.TransformBroadcaster()

    def transform_map_to_odom(self):
        self.br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         '/robot3_tf/odom',
                         '/map')

if __name__ == "__main__":

    try:
        rospy.init_node('transform_publisher', log_level=rospy.INFO)
        rospy.loginfo('[transform_publisher]: Node started')
        tp = TFPub()
        while not rospy.is_shutdown():
            tp.transform_map_to_odom()

    except Exception as e:
        rospy.logfatal('[transform_publisher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
