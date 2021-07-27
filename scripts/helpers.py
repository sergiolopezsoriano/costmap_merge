#!/usr/bin/env python

import rospy
import tf
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import math


class TransformHelper:

    def __init__(self):
        self.br = TransformBroadcaster()

    def publish_transform(self, transform):
        self.br.sendTransform(transform)

    @staticmethod
    def get_pose_transforms(pose_D_D, pose_R_D, pose_R_R, pose_D_R, alpha, beta):
        gamma = math.pi + alpha - beta
        yaw_D_D = PoseHelper.get_yaw_from_orientation(pose_D_D.pose.orientation)
        yaw_R_R = PoseHelper.get_yaw_from_orientation(pose_R_R.pose.orientation)
        yaw_R_D = yaw_R_R + gamma
        yaw_D_R = yaw_D_D - gamma
        rotation_R_D = yaw_D_D - yaw_R_D
        rotation_D_R = yaw_D_R - yaw_R_R
        tDR = TransformStamped()
        tDR.header.stamp = rospy.Time.now()
        tDR.header.frame_id = pose_D_D.header.frame_id  # origin
        tDR.child_frame_id = pose_R_D.header.frame_id  # destination
        tDR.transform.translation.x = pose_R_D.pose.position.x - pose_D_D.pose.position.x
        tDR.transform.translation.y = pose_R_D.pose.position.y - pose_D_D.pose.position.y
        tDR.transform.translation.z = pose_R_D.pose.position.z - pose_D_D.pose.position.z
        tDR.transform.rotation = PoseHelper.get_orientation_from_yaw(rotation_R_D)
        tRD = TransformStamped()
        tRD.header.stamp = rospy.Time.now()
        tRD.header.frame_id = pose_R_R.header.frame_id  # origin
        tRD.child_frame_id = pose_D_R.header.frame_id  # destination
        tRD.transform.translation.x = pose_D_R.pose.position.x - pose_R_R.pose.position.x
        tRD.transform.translation.y = pose_D_R.pose.position.y - pose_R_R.pose.position.y
        tRD.transform.translation.z = pose_D_R.pose.position.z - pose_R_R.pose.position.z
        tRD.transform.rotation = PoseHelper.get_orientation_from_yaw(rotation_D_R)
        return tDR, pose_D_R, tRD, pose_R_D

    @staticmethod
    def get_frame_transform(pose_R_D, pose_R_R, alpha, beta):
        gamma = math.pi + alpha - beta
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = pose_R_D.header.frame_id  # origin
        t.child_frame_id = pose_R_R.header.frame_id  # destination
        t.transform.translation.x = pose_R_R.pose.position.x - pose_R_D.pose.position.x * math.cos(
            gamma) + pose_R_D.pose.position.y * math.sin(gamma)
        t.transform.translation.y = pose_R_R.pose.position.y - pose_R_D.pose.position.y
        t.transform.translation.z = pose_R_R.pose.position.z - pose_R_D.pose.position.z
        t.transform.rotation = PoseHelper.get_orientation_from_yaw(gamma)
        return t

    @staticmethod
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


class PoseHelper:
    def __init__(self):
        pass

    @staticmethod
    def get_yaw_from_orientation(orientation):
        return tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

    @staticmethod
    def get_orientation_from_yaw(yaw):
        orientation = Quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        return orientation

    @staticmethod
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

    def get_2D_pose(self, namespace, x, y, z, yaw, ts):
        pose = PoseStamped()
        pose.header.frame_id = namespace + '/odom'
        pose.header.stamp = ts
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self.get_orientation_from_yaw(yaw)
        return pose

    @staticmethod
    def get_pose_from_odom(self, odom):
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        return pose
