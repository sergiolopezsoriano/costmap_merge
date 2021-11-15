#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from robots import OdomRobot
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from costmap_merge.srv import PCHandshake1, PCHandshake1Request, PCHandshake2
from helpers import PoseHelper


class PCDetector:
    def __init__(self):
        self.namespace = rospy.get_namespace().strip('/')
        # Robots names
        self.sim = rospy.get_param('/use_sim_time')
        if self.sim:
            self.robots_names = rospy.get_param('/robots_names')
        else:
            self.robots_names = rospy.get_param('robots').keys()
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for robot in self.robots_names:
            self.robots[robot] = OdomRobot(robot)
            if self.sim:
                coordinates = rospy.get_param('/simulation_launcher/' + robot + '/coordinates')
                self.robots[robot].set_start_from_coordinates('/map', rospy.Time.now(), coordinates)
            else:
                coordinates = rospy.get_param('/' + self.namespace + '/robots/' + robot + '/coordinates')
                if robot == self.namespace:
                    self.robots[robot].set_start_from_coordinates('/map', rospy.Time.now(), coordinates)
        # Initializes the proxies dictionary to communicate with the detection_handshake node
        self.request_proxies = dict()
        self.response_proxies = dict()
        # List with the detected robots
        self.detected_robots = list()
        # Subscriber to the darknet topic publishing the location of detected robots
        rospy.Subscriber('/' + self.namespace + '/darknet_ros_3d/markers', MarkerArray, self.cb_detection, queue_size=1)
        self.marker = MarkerArray()
        self.detections = list()
        # Service proxy to identify robots using RFID
        rospy.wait_for_service('detection_check_service')
        self.detection_check_proxy = rospy.ServiceProxy('detection_check_service', Trigger)
        # flags to change between detector and listener
        self.request_response_flag = True

    def cb_detection(self, msg):
        self.marker = msg

    @staticmethod
    def get_distance_to_detection(marker):
        return math.sqrt(marker.pose.position.x ** 2 + marker.pose.position.y ** 2)

    def update_detections(self, msg=PCHandshake1Request):
        distance = list()
        for marker in self.marker.markers:
            distance.append(self.get_distance_to_detection(marker))
        sorted_dist = distance.sort(key=lambda x: x)
        indexes = [distance.index(dist) for dist in sorted_dist]
        for index in indexes:
            transform = self.get_transform(index)
            robot = self.detection_check(distance[index])
            transform.child_frame_id = '/' + robot + '_tf/odom'
            if robot and self.request_response_flag:
                self.cm_request(robot, transform)
            elif not self.request_response_flag:
                self.cm_response(msg, transform)
            else:
                pass

    def detection_check(self, distance):
        rospy.wait_for_service('detection_check_service')
        robot = self.detection_check_proxy()
        if robot not in self.detected_robots:
            self.detected_robots.append(robot)
        return robot

    def get_transform(self, index):
        p2 = PoseStamped()
        p2.header.frame_id = rospy.get_param('/' + self.namespace + '_tf/odom')
        p2.header.stamp = rospy.Time.now()
        p2.pose.position.x = self.marker.markers[index].pose.position.x + \
            self.robots[self.namespace].odom.pose.pose.position.x
        p2.pose.position.y = self.marker.markers[index].pose.position.y + \
            self.robots[self.namespace].odom.pose.pose.position.y
        p2.pose.position.z = self.marker.markers[index].pose.position.z + \
            self.robots[self.namespace].odom.pose.pose.position.z
        p1 = PoseHelper.get_pose_from_odom(self.robots[self.namespace].odom)
        alpha = math.atan2(p2.pose.position.y - p1.pose.position.y,
                           p2.pose.position.x - p1.pose.position.x)
        t = TransformStamped()
        t.header.frame_id = p1.header.frame_id
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = p2.pose.position.x - p1.pose.position.x
        t.transform.translation.y = p2.pose.position.y - p1.pose.position.y
        t.transform.translation.z = p2.pose.position.z - p1.pose.position.z
        t.transform.rotation = PoseHelper.get_orientation_from_yaw(0, 0, alpha)
        return t

        # if robot in self.detected_robots:
        #     self.detected_robots.remove(robot)

    def cm_request(self, robot, transform):
        """ Sends the detector and robot poses in the detector frame to the detected robot """
        try:
            self.request_proxies[robot](self.namespace, transform)
        except KeyError:
            rospy.wait_for_service('/' + robot + '/cm_request_service')
            self.request_proxies[robot] = rospy.ServiceProxy('/' + robot + '/cm_request_service', CM_Request)
            self.request_proxies[robot](self.namespace, transform)

    def cb_request(self, msg):
        self.update_detections(msg)
        return CM_Request

    def cm_response(self, msg, t2):
        t2 = self.get_transform()
        try:
            self.response_proxies[msg.detector_ns](self.namespace, msg.translate(), t2)
        except KeyError:
            rospy.wait_for_service('/' + msg.detector_ns + '/cm_response_service')
            self.response_proxies[msg.detector_ns] = rospy.ServiceProxy('/' + msg.detector_ns + '/cm_response_service',
                                                                      CM_Response)
            self.response_proxies[msg.detector_ns](self.namespace, t1, t2)

    def calculate_transformed_odom(self, robot):
        """ Calculates the robot's odometry in the map frame. The result is saved in the robot's transformed_odom """
        yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
        x = self.robots[robot].start.pose.position.x + self.robots[robot].odom.pose.pose.position.x * math.cos(yaw) - \
            self.robots[robot].odom.pose.pose.position.y * math.sin(yaw)
        y = self.robots[robot].start.pose.position.y + self.robots[robot].odom.pose.pose.position.y * math.cos(yaw) + \
            self.robots[robot].odom.pose.pose.position.x * math.sin(yaw)
        transformed_yaw = yaw + PoseHelper.get_yaw_from_orientation(self.robots[robot].odom.pose.pose.orientation)
        self.robots[robot].set_transformed_odom('/map', rospy.Time.now(), [x, y, 0, 0, 0, transformed_yaw])


if __name__ == "__main__":

    try:
        rospy.init_node('pc_detector', log_level=rospy.INFO)
        pcd = PCDetector()
        pcd.calculate_transformed_odom(pcd.namespace)
        """ Check when the robot is not detected for a long time """
        while not rospy.is_shutdown():
            if pcd.request_response_flag:
                pcd.update_detections()
            rospy.sleep(1)

    except:
        pass
