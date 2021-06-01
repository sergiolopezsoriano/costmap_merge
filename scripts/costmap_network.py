#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from costmap_merge.srv import RobotLocation, RobotLocationResponse
import robot as rb


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


class CostmapNetwork:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.type = rospy.get_param('~robot_type')
        self.simulation = rospy.get_param('~simulation')
        # Server for creating and updating robots
        self.service = rospy.Service('update_robots', RobotLocation, self.cb_update_robot)
        # Costmap information
        self.occupancy_range = 100
        # Creating and starting the costmap_node
        self.robots = dict()
        if self.type == 'detector':
            self.robots[self.namespace] = rb.Detector(self.namespace, self.type)
        else:
            self.robots[self.namespace] = rb.CostmapNode(self.namespace, self.type)
        # TODO: The transform must be received from the AI and the Find_robot_in_costmap scripts every time it's
        #  detected by the detector
        if self.simulation:
            self.robots[self.namespace].pose = get_map_to_odom_transform(self.namespace)
        self.robots[self.namespace].start()

    def cb_update_robot(self, msg):
        if not (msg.namespace in self.robots):
            self.robots[msg.namespace] = rb.Robot(msg.namespace, msg.type)
        # TODO: To be used outside simulations
        self.robots[msg.namespace].pose.header.frame_id = str(msg.namespace) + '/odom'
        self.robots[msg.namespace].pose.header.stamp = msg.ts
        self.robots[msg.namespace].pose.pose.position.x = msg.x
        self.robots[msg.namespace].pose.pose.position.y = msg.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.yaw)
        self.robots[msg.namespace].pose.pose.orientation.x = quaternion[0]
        self.robots[msg.namespace].pose.pose.orientation.y = quaternion[1]
        self.robots[msg.namespace].pose.pose.orientation.z = quaternion[2]
        self.robots[msg.namespace].pose.pose.orientation.w = quaternion[3]
        return RobotLocationResponse(msg.namespace, msg.type, msg.x, msg.y, msg.yaw, msg.ts)

    def build_global_costmap(self):
        # Building the merged_global_costmap area
        global_costmap = OccupancyGrid()
        while not (self.robots[self.namespace].global_ready and self.robots[self.namespace].local_ready):
            rospy.sleep(1)
        global_costmap.info.resolution = self.robots[self.namespace].global_costmap.info.resolution
        x = []
        y = []
        for robot in self.robots:
            yaw = rb.get_yaw_from_orientation(self.robots[robot].pose.pose.orientation)
            if abs(yaw) <= np.pi / 4 or abs(yaw) > 3 * np.pi / 4:
                if abs(yaw) <= np.pi / 4:
                    sign = 1
                else:
                    sign = -1
                x.append(self.robots[robot].pose.pose.position.x + self.robots[robot].odom.pose.pose.position.x * sign)
                y.append(self.robots[robot].pose.pose.position.y + self.robots[robot].odom.pose.pose.position.y * sign)
            if np.pi / 4 < abs(yaw) <= 3 * np.pi / 4 or np.pi / 4 < abs(yaw) <= 3 * np.pi / 4:
                if np.pi / 4 < abs(yaw) <= 3 * np.pi / 4:
                    sign_x = -1  # -np.sin(yaw)
                    sign_y = 1  # np.sin(yaw)
                elif np.pi / 4 < abs(yaw) <= 3 * np.pi / 4:
                    sign_x = 1
                    sign_y = -1
                x.append(
                    self.robots[robot].pose.pose.position.x + self.robots[robot].odom.pose.pose.position.y * sign_x)
                y.append(
                    self.robots[robot].pose.pose.position.y + self.robots[robot].odom.pose.pose.position.x * sign_y)
        x_min = min(x)
        x_max = max(x)
        y_min = min(y)
        y_max = max(y)
        # rospy.logdebug('[costmap_network-' + str(self.namespace) + '] x_min: ' + str(x_min))
        # rospy.logdebug('[costmap_network-' + str(self.namespace) + '] y_min: ' + str(y_min))
        global_costmap.info.origin.position.x = sum(x) / len(self.robots)
        global_costmap.info.origin.position.y = sum(y) / len(self.robots)
        global_costmap.info.width = int(
            (x_max - x_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.width
        global_costmap.info.height = int(
            (y_max - y_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.height

        # Filling the global global costmap with random numbers
        # global_costmap.data = np.random.random_integers(self.occupancy_range, size=(
        #             global_costmap.info.width * global_costmap.info.height)).tolist()

        # zero padding
        global_costmap.data = np.zeros(int(global_costmap.info.width * global_costmap.info.height)).tolist()

        # Adding all robots global_costmaps to the merged_global_costmap
        for robot in self.robots:
            # Taking into account the starting angle
            yaw = rb.get_yaw_from_orientation(self.robots[robot].pose.pose.orientation)
            if abs(yaw) <= np.pi / 4 or abs(yaw) > 3 * np.pi / 4:
                if abs(yaw) <= np.pi / 4:
                    sign = 1
                    data = self.robots[robot].global_costmap.data
                else:
                    sign = -1
                    data = self.robots[robot].global_costmap.data[::-1]
                pos_x = int((self.robots[robot].pose.pose.position.x + self.robots[
                    robot].odom.pose.pose.position.x * sign - x_min) / global_costmap.info.resolution)
                pos_y = int((self.robots[robot].pose.pose.position.y + self.robots[
                    robot].odom.pose.pose.position.y * sign - y_min) / global_costmap.info.resolution)
            elif np.pi / 4 < abs(yaw) <= 3 * np.pi / 4:
                if np.pi / 4 < abs(yaw) <= 2 * np.pi / 4:
                    sign_x = -1
                    sign_y = 1
                    data = self.robots[robot].global_costmap.data
                elif 2 * np.pi / 4 < abs(yaw) <= 3 * np.pi / 4:
                    sign_x = 1
                    sign_y = -1
                    data = self.robots[robot].global_costmap.data[::-1]
                data = np.reshape(data, (self.robots[self.namespace].global_costmap.info.width,
                                         self.robots[self.namespace].global_costmap.info.height))
                data = np.rot90(data, 3)
                data = np.reshape(data, (self.robots[self.namespace].global_costmap.info.width *
                                         self.robots[self.namespace].global_costmap.info.height)).tolist()
                pos_x = int((self.robots[robot].pose.pose.position.x + self.robots[
                    robot].odom.pose.pose.position.y * sign_x - x_min) / global_costmap.info.resolution)
                pos_y = int((self.robots[robot].pose.pose.position.y + self.robots[
                    robot].odom.pose.pose.position.x * sign_y - y_min) / global_costmap.info.resolution)

            for row in range(self.robots[robot].global_costmap.info.height):
                row_start = row * self.robots[robot].global_costmap.info.width
                global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x:
                                    (row + pos_y) * global_costmap.info.width + pos_x +
                                    self.robots[robot].global_costmap.info.width] = \
                    data[row_start:row_start + self.robots[robot].global_costmap.info.width]

            ####################### overlapping costmaps ########################
            # for row in range(self.robots[robot].global_costmap.info.height):
            #     for col in range(self.robots[robot].global_costmap.info.width):
            #         if data[row * self.robots[robot].global_costmap.info.width + col] > \
            #                 global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x + col]:
            #             global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x + col] = \
            #                 data[row * self.robots[robot].global_costmap.info.width + col]

        if self.simulation:
            global_costmap.info.origin.position.x = x_min - self.robots[
                self.namespace].global_costmap.info.width / 2 * global_costmap.info.resolution
            global_costmap.info.origin.position.y = y_min - self.robots[
                self.namespace].global_costmap.info.height / 2 * global_costmap.info.resolution
        else:
            global_costmap.info.origin.position.x = x_min + (
                        x_max - x_min) / 2 - global_costmap.info.width / 2 * global_costmap.info.resolution
            global_costmap.info.origin.position.y = y_min + (
                        y_max - y_min) / 2 - global_costmap.info.height / 2 * global_costmap.info.resolution
        return global_costmap


if __name__ == "__main__":

    try:
        rospy.init_node('costmap_network', log_level=rospy.INFO)
        rospy.loginfo('[costmap_network]: Node started')
        cn = CostmapNetwork()
        # preventing building the map before receiving any costmap
        while not (cn.robots[cn.namespace].global_ready and cn.robots[cn.namespace].local_ready):
            rospy.sleep(1)
        while not rospy.is_shutdown():
            cn.robots[cn.namespace].merged_global_costmap = cn.build_global_costmap()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[costmap_network]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
