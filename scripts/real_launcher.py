#!/usr/bin/env python

import rospy
import roslaunch
import traceback


class RobotLauncher:
    def __init__(self):
        self.robot_ns = rospy.get_param('~robot_ns')
        self.robot_type = rospy.get_param(self.robot_ns + '/robots/' + self.robot_ns + '/robot_type')
        self.launch_file = rospy.get_param('~agent_launch_file')
        self.detector_file = rospy.get_param('~detector_file')
        self.listener_file = rospy.get_param('~listener_file')
        self.use_sim_time = rospy.get_param('/use_sim_time')

    def launch_robot(self):
        file_args = [(self.launch_file, self.configure_robot())]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, file_args)
        parent.start()
        # parent.shutdown()

    def configure_robot(self):
        x, y, z, roll, pitch, yaw = rospy.get_param(self.robot_ns + '/robots/' + self.robot_ns + '/coordinates')
        can_detect = rospy.get_param(self.robot_ns + '/robots/' + self.robot_ns + '/can_detect')
        args = ['robot_ns:=' + str(self.robot_ns), 'robot_type:=' + str(self.robot_type), 'x:=' + str(x),
                'y:=' + str(y), 'z:=' + str(z), 'roll:=' + str(roll), 'pitch:=' + str(pitch), 'yaw:=' + str(yaw),
                'sim:=' + str(self.use_sim_time), 'can_detect:=' + str(can_detect),
                'detector_file:=' + self.detector_file, 'listener_file:=' + self.listener_file]
        return args


if __name__ == "__main__":

    try:
        rospy.init_node('robot_launcher', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo('[robot_launcher]: Node started')
        robot_launcher = RobotLauncher()
        robot_launcher.launch_robot()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[robot_launcher]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
