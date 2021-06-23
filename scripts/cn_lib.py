#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math


def get_yaw_from_orientation(orientation):
    return tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) + math.sin(angle) * (py - oy)
    qy = oy - math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def get_map_to_odom_transform(namespace):
    pose_tf = PoseStamped()
    time = rospy.Time(0)
    tl = tf.TransformListener()
    tl.waitForTransform('/map', '/' + namespace + '_tf/odom', time, rospy.Duration(1.0))
    (position, orientation) = tl.lookupTransform('/map', '/' + namespace + '_tf/odom', time)
    pose_tf.pose.position.x = position[0]
    pose_tf.pose.position.y = position[1]
    pose_tf.pose.position.z = position[2]
    pose_tf.pose.orientation.x = orientation[0]
    pose_tf.pose.orientation.y = orientation[1]
    pose_tf.pose.orientation.z = orientation[2]
    pose_tf.pose.orientation.w = orientation[3]
    pose_tf.header.stamp = time
    return pose_tf
