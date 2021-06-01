#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
from geometry_msgs.msg import PoseStamped
from costmap_merge.srv import RobotLocation, RobotLocationResponse
from costmap_merge.msg import RobotDetected
from threading import Thread, Lock


def get_yaw_from_orientation(orientation):
    quaternion = [0] * 4
    quaternion[0] = orientation.x
    quaternion[1] = orientation.y
    quaternion[2] = orientation.z
    quaternion[3] = orientation.w
    return tf.transformations.euler_from_quaternion(quaternion)[2]


class Robot(object):
    def __init__(self, namespace, robot_type):
        # getting ros params
        self.namespace = namespace
        self.type = robot_type
        # Poses and costmaps
        self.pose = PoseStamped()
        self.odom = Odometry()  # in this Robot frame, of course!
        self.global_costmap = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        self.merged_global_costmap = OccupancyGrid()
        # Flags to prevent publishing before both costmaps are received
        self.global_ready = False
        self.local_ready = False
        # Odometry subscriber
        rospy.Subscriber('/' + self.namespace + '/odom', Odometry, self.cb_odom, queue_size=1)
        # Costmaps subscribers and publishers
        rospy.Subscriber('/' + self.namespace + '/global_costmap', OccupancyGrid, self.cb_global_costmap, queue_size=1)
        rospy.Subscriber('/' + self.namespace + '/local_costmap', OccupancyGrid, self.cb_local_costmap, queue_size=1)

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


class CostmapNode(Robot, Thread):
    def __init__(self, namespace, robot_type):
        Thread.__init__(self)
        super(CostmapNode, self).__init__(namespace, robot_type)
        # Service for the Detector-Robot communication
        self.service = rospy.Service('robot_communication_service', RobotLocation, self.cb_robot_communication)
        # Proxy to update costmap_network robots
        rospy.wait_for_service('/' + self.namespace + '/update_robots')
        self.update_robots_proxy = rospy.ServiceProxy('/' + self.namespace + '/update_robots', RobotLocation)
        # Lock to avoid concurrent calls with the update_robots_proxy
        self.lock_update_robots = Lock()
        # Flag for simulation
        self.simulation = rospy.get_param('~simulation')
        # Costmap Publishers
        self.local_publisher = rospy.Publisher('/' + self.namespace + '/move_base/local_costmap/costmap',
                                               OccupancyGrid, queue_size=1)
        self.global_publisher = rospy.Publisher('/' + self.namespace + '/move_base/global_costmap/costmap',
                                                OccupancyGrid, queue_size=1)
        # Namespaces of the known detectors
        self.detectors = []
        self.last_detector = ''

    def run(self):
        while not rospy.is_shutdown():
            self.check_last_detector()
            self.publish_costmap()

    def check_last_detector(self):
        if self.last_detector not in self.detectors and self.last_detector != '':
            self.detectors.append(self.last_detector)
            # Adding subscribers to receive the robots detected by the known detectors
            rospy.Subscriber('/' + self.last_detector + '/detected_robots', RobotDetected, self.cb_detected_robots,
                             queue_size=10)

    def publish_costmap(self):
        if self.global_ready and self.local_ready:
            self.global_publisher.publish(self.merged_global_costmap)
            self.local_publisher.publish(self.local_costmap)

    def cb_robot_communication(self, msg):
        rospy.loginfo('[' + str(self.type) + '-' + str(self.namespace) + ']: Detector-' + str(msg.namespace) + ' calling')
        # Creating or updating robots in the costmap_network
        self.update_robots(msg)
        # Adding detector
        self.last_detector = msg.namespace
        # Replying to the locator
        return RobotLocationResponse(self.namespace, self.type, self.pose.pose.position.x, self.pose.pose.position.y,
                                     get_yaw_from_orientation(self.pose.pose.orientation), self.pose.header.stamp)

    def cb_detected_robots(self, msg):
        self.update_robots(msg)

    def update_robots(self, msg):
        self.lock_update_robots.acquire()
        try:
            self.update_robots_proxy(msg.namespace, msg.type, msg.x, msg.y, msg.yaw, msg.ts)
        finally:
            self.lock_update_robots.release()


class Detector(CostmapNode):
    def __init__(self, namespace, robot_type):
        super(Detector, self).__init__(namespace, robot_type)
        # Loads the known robot names and initializes the proxies to talk to the robots
        self.robots_names = rospy.get_param('~robots_names')
        self.proxies = dict()
        for namespace in self.robots_names:
            if namespace != self.namespace:
                rospy.wait_for_service('/' + namespace + '/robot_communication_service')
                self.proxies[namespace] = rospy.ServiceProxy('/' + namespace + '/robot_communication_service',
                                                             RobotLocation)
        # Publishes the detected robots
        self.robot_publisher = rospy.Publisher('/' + self.namespace + '/detected_robots', RobotDetected, queue_size=10)

    def run(self):
        # Detector method code
        #
        #######################
        while not rospy.is_shutdown():
            self.check_last_detector()
            self.publish_costmap()
            # If the AI node detects a robot, it starts the communication
            for namespace in self.robots_names:
                if namespace != self.namespace:
                    self.talk_to_robot(namespace)
            rospy.sleep(5)

    def talk_to_robot(self, namespace):
        rospy.loginfo('[' + str(self.type) + '-' + str(self.namespace) + ']: Talking to ' + str(namespace))
        response = self.proxies[namespace](self.namespace, self.type, self.pose.pose.position.x,
                                           self.pose.pose.position.y,
                                           get_yaw_from_orientation(self.pose.pose.orientation),
                                           self.pose.header.stamp)
        self.update_robots(response)
        self.robot_publisher.publish(response)
