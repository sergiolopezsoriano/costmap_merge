#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from costmap_merge.srv import RobotLocation, RobotLocationResponse
import thread


class Robot:
    def __init__(self, robot_ns, robot_type='', robot_pose=PoseStamped()):
        # Initialization
        self.robot_type = robot_type
        self.robot_ns = robot_ns
        self.pose = robot_pose  # in the frame of the Robot running the costmap_merger node
        # Variables
        self.odom = Odometry()  # in this Robot frame, of course!
        self.global_costmap = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        # Flags to wait until both costmaps are received
        self.global_ready = False
        self.local_ready = False
        # Odometry subscriber
        rospy.Subscriber('/' + self.robot_ns + '/odom', Odometry, self.cb_odom, queue_size=1)
        # Costmaps subscribers
        rospy.Subscriber('/' + self.robot_ns + '/global_costmap', OccupancyGrid, self.cb_global_costmap, queue_size=1)
        rospy.Subscriber('/' + self.robot_ns + '/local_costmap', OccupancyGrid, self.cb_local_costmap, queue_size=1)
        # Proxy for the Locator-Pawn handshake
        # rospy.wait_for_service('/' + self.robot_ns + '/location_handshake')
        self.location_handshake_proxy = rospy.ServiceProxy('/' + self.robot_ns + '/location_handshake', RobotLocation)

    def cb_global_costmap(self, msg):
        self.global_ready = True
        self.global_costmap = msg

    def cb_local_costmap(self, msg):
        self.local_ready = True
        self.local_costmap = msg

    def cb_odom(self, msg):
        self.odom = msg

    def set_pose(self, ps):
        self.pose = ps


class Merger:

    def __init__(self):
        # Flag for simulation
        self.simulation = True
        # Transformations
        self.tl = tf.TransformListener()
        # getting ros params
        self.namespace = rospy.get_namespace().strip('/')
        self.robot_type = rospy.get_param('~robot_type')
        # Pawns are informed by the Locator and send back the response
        self.s = rospy.Service('location_handshake', RobotLocation, self.cb_location_handshake)
        # Dictionary containing all the robots found so far
        self.robots = dict()
        # robot definition
        self.robots[self.namespace] = Robot(self.namespace, self.robot_type)
        # TODO: The transform must be received from the AI and the Find_robot_in_costmap scripts
        if self.simulation:
            self.robots[self.namespace].pose = self.get_transform_from_map()
        # Costmap information
        self.occupancy_range = 100
        # Global costmap
        self.merged_global_costmap = OccupancyGrid()
        # Costmap publishers
        self.local_publisher = rospy.Publisher('/' + self.namespace + '/move_base/local_costmap/costmap',
                                               OccupancyGrid, queue_size=1)
        self.global_publisher = rospy.Publisher('/' + self.namespace + '/move_base/global_costmap/costmap',
                                                OccupancyGrid, queue_size=1)

    def cb_location_handshake(self, msg):
        # rospy.loginfo('[costmap_merger-' + str(self.namespace) + ']: Locator calling')
        if msg.namespace in self.robots:
            self.update_robots(msg)
        else:
            self.update_robots(msg, True)
        return RobotLocationResponse(self.namespace, self.robot_type,
                                     self.robots[self.namespace].pose.pose.position.x,
                                     self.robots[self.namespace].pose.pose.position.y,
                                     self.get_yaw_from_quaternion(self.namespace),
                                     self.robots[self.namespace].pose.header.stamp)

    def update_robots(self, msg, new=False):
        ps = PoseStamped()
        ps.header.frame_id = str(msg.namespace) + '/odom'
        ps.header.stamp = msg.ts
        ps.pose.position.x = msg.x
        ps.pose.position.y = msg.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.yaw)
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]
        if new:
            self.robots[msg.namespace] = Robot(msg.namespace, msg.type, ps)
        else:
            self.robots[msg.namespace].set_pose(ps)

    def talk_to_robot(self, robot_ns):
        # rospy.loginfo('[costmap_merger-' + str(self.namespace) + ']: Talking to Pawn ' + str(robot_ns))
        try:
            response = self.robots[robot_ns].location_handshake_proxy(self.namespace, self.robot_type,
                                                                      self.robots[self.namespace].pose.pose.position.x,
                                                                      self.robots[self.namespace].pose.pose.position.y,
                                                                      self.get_yaw_from_quaternion(self.namespace),
                                                                      self.robots[self.namespace].pose.header.stamp)
            if self.robots[response.namespace]:
                self.update_robots(response)
            else:
                self.update_robots(response, True)
        except rospy.ServiceException:
            rospy.logfatal("[costmap_merger]: Service proxy failed")

    def get_yaw_from_quaternion(self, robot_ns):
        quaternion = [0] * 4
        quaternion[0] = self.robots[robot_ns].pose.pose.orientation.x
        quaternion[1] = self.robots[robot_ns].pose.pose.orientation.y
        quaternion[2] = self.robots[robot_ns].pose.pose.orientation.z
        quaternion[3] = self.robots[robot_ns].pose.pose.orientation.w
        return tf.transformations.euler_from_quaternion(quaternion)[2]

    def get_transform_from_map(self):
        pose_tf = PoseStamped()
        time = rospy.Time(0)
        try:
            self.tl.waitForTransform('/map', '/' + self.namespace + '_tf/odom', time, rospy.Duration(1.0))
            (position, orientation) = \
                self.tl.lookupTransform('/map', '/' + self.namespace + '_tf/odom', time)
            pose_tf.pose.position.x = position[0]
            pose_tf.pose.position.y = position[1]
            pose_tf.pose.position.z = position[2]
            pose_tf.pose.orientation.x = orientation[0]
            pose_tf.pose.orientation.y = orientation[1]
            pose_tf.pose.orientation.z = orientation[2]
            pose_tf.pose.orientation.w = orientation[3]
            pose_tf.header.stamp = time
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal('Error in get_transform_from_map')
        return pose_tf

    def build_global_costmap(self):
        # Building the merged_global_costmap area
        global_costmap = OccupancyGrid()
        global_costmap.info.resolution = self.robots[self.namespace].global_costmap.info.resolution
        x = []
        y = []
        for robot in self.robots:
            yaw = self.get_yaw_from_quaternion(robot)
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
        # rospy.loginfo('[costmap_merger-' + str(self.namespace) + '] x_min: ' + str(x_min))
        # rospy.loginfo('[costmap_merger-' + str(self.namespace) + '] y_min: ' + str(y_min))
        global_costmap.info.origin.position.x = sum(x) / len(self.robots)
        global_costmap.info.origin.position.y = sum(y) / len(self.robots)
        global_costmap.info.width = int(
            (x_max - x_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.width
        global_costmap.info.height = int(
            (y_max - y_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.height

        # Filling the global global costmap with random numbers
        global_costmap.data = np.random.random_integers(self.occupancy_range, size=(
                    global_costmap.info.width * global_costmap.info.height)).tolist()

        # zero padding
        # global_costmap.data = np.zeros(int(global_costmap.info.width * global_costmap.info.height)).tolist()

        # Adding all robots global_costmaps to the merged_global_costmap
        for robot in self.robots:
            # Taking into account the starting angle
            yaw = self.get_yaw_from_quaternion(robot)
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
        self.merged_global_costmap = global_costmap

    def publish_robot_costmaps(self):
        self.global_publisher.publish(self.merged_global_costmap)
        # self.local_publisher.publish(self.merged_global_costmap)
        self.local_publisher.publish(self.robots[self.namespace].local_costmap)


if __name__ == "__main__":

    try:
        robot_names = ['robot1', 'robot2', 'robot3']  # TODO: The robot_ns must be given by the AI node!!!
        rospy.init_node('costmap_merger', log_level=rospy.INFO)
        rospy.loginfo('[costmap_merger]: Node started')
        cm = Merger()
        rospy.sleep(5)  # Waiting for the ROS subscribers, publishers and servers to initialize
        if cm.robot_type == 'locator':
            for name in robot_names:
                if name != cm.namespace:
                    cm.robots[name] = Robot(name)
        while not rospy.is_shutdown():
            if cm.robots[cm.namespace].local_ready and cm.robots[cm.namespace].global_ready:
                if cm.robot_type == 'locator':
                    for name in robot_names:
                        if name != cm.namespace:
                            cm.talk_to_robot(name)
                rospy.sleep(1)
                cm.build_global_costmap()
                cm.publish_robot_costmaps()

    except Exception as e:
        rospy.logfatal('[costmap_merger]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
