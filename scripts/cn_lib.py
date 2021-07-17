#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
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
    pose_tf.header.stamp = rospy.Time.now()
    return pose_tf


def get_pose2to1_transform(pose1, pose2):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = '/map'
    t.child_frame_id = pose2.header.frame_id
    t.transform.translation.x = pose2.pose.position.x - pose1.pose.position.x
    t.transform.translation.y = pose2.pose.position.y - pose1.pose.position.y
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, get_yaw_from_orientation(
        pose2.pose.orientation) - get_yaw_from_orientation(pose1.pose.orientation))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t
