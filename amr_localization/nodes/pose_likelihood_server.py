#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'pose_likelihood_server'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose2D
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodResponse, GetNearestOccupiedPointOnBeam, GetNearestOccupiedPointOnBeamRequest, SwitchRanger


class PoseLikelihoodServerNode:
    """
    This is a port of the AMR Python PoseLikelihoodServerNode
    """
    ROBOT_FRAME_ID = '/base_link'
    ALLOWED_MISMATCH_NUM = 4
    def __init__(self):

        rospy.init_node(NODE)
        # Wait until SwitchRanger service (and hence stage node) becomes available.
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...');
        rospy.wait_for_service('/switch_ranger')
        try:
            switch_ranger = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
            # Make sure that the hokuyo laser is available and enable them (aka switch on range scanner)
            switch_ranger('scan_front', True)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        # Laser scanner variables
        self._scan_ranges = None
        self._scan_range_max = None
        self._scan_range_min = None
        self._scan_angle_increment = None
        self._scan_sigma = 0.7
        self._scan_poses_world_frame = []
        self._scan_angle_min = None
        # transformation from robot frame to laser scanner frame
        self._tf_matrix_robot_to_scan = None
        # yaw of front laser scan in robot frame
        self._scan_front_yaw_robot_frame = None
        # frame ids
        self._scan_front_header = None
        self._world_frame_id = None

        """
            Expose GetMultiplePoseLikelihood Service here,
            subscribe for /scan_front,
            create client for /occupancy_query_server/get_nearest_occupied_point_on_beam service

            http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        """
        self._pose_likelihood_server = rospy.Service(
                                            '/pose_likelihood_server/get_pose_likelihood',
                                            GetMultiplePoseLikelihood,
                                            self._pose_likelihood_callback)

        #TODO: check queue_size
        self._scan_front_subscriber = rospy.Subscriber('/scan_front',
                                                       LaserScan,
                                                       self._scan_front_callback,
                                                       queue_size=5)

        try:
            self._get_nearest_occupied_client = rospy.ServiceProxy(
                                                '/occupancy_query_server/get_nearest_occupied_point_on_beam',
                                                GetNearestOccupiedPointOnBeam)
        except rospy.ServiceException, e:
            rospy.logerr("Service call to get_nearest_occupied_point_on_beam failed: %s", e)

        self._tf = tf.TransformListener()

        rospy.loginfo('Started [pose_likelihood_server] node.')


    def _pose_likelihood_callback(self, multi_pose_srv):
        """ return likelihood for a cell group """
        response = []
        for pose_stamped_msg in multi_pose_srv.poses:
            # calculate response
            response.append(self._calculate_single_pose_likelihood(pose_stamped_msg))

        return GetMultiplePoseLikelihoodResponse(response)


    def _calculate_single_pose_likelihood(self, pose_stamped_msg):
        ''' Run here instead of _init_ to have access to _scan_front_header'''
        # get world frame id
        if self._world_frame_id is None:
            self._world_frame_id = pose_stamped_msg.header.frame_id
        # tf matrix from world fram to robot
        translation = (pose_stamped_msg.pose.position.x,
                       pose_stamped_msg.pose.position.y,
                       pose_stamped_msg.pose.position.z)
        orientation = (pose_stamped_msg.pose.orientation.x,
                       pose_stamped_msg.pose.orientation.y,
                       pose_stamped_msg.pose.orientation.z,
                       pose_stamped_msg.pose.orientation.w)
        tf_matrix_world_to_robot = tf.TransformerROS().fromTranslationRotation(translation, orientation)
        robot_yaw = tf.transformations.euler_from_quaternion(orientation)[2]
        # get tf matrix from robot to laser scanner if not calculated
        if self._tf_matrix_robot_to_scan is None:
            self._cal_scan_poses_world_frame()
        # Calculate robot to world frame tf matrix
        tf_matrix_world_to_scan = np.dot(tf_matrix_world_to_robot, self._tf_matrix_robot_to_scan)

        for i in range(len(self._scan_ranges)):
            beam = Pose2D()
            position = np.dot(tf_matrix_world_to_scan, np.array([0, 0, 0, 1]))
            beam.x = position[0]
            beam.y = position[1]
            beam.theta = (robot_yaw + self._scan_front_yaw_robot_frame
                                    + self._scan_angle_min
                                    + i * self._scan_angle_increment)
            # adjust large angles
            if beam.theta > math.pi * 2:
                beam.theta = beam.theta - math.pi * 2
            # Keep beam length to 12
            if len(self._scan_poses_world_frame) < i + 1:
                self._scan_poses_world_frame.append(beam)
            else:
                self._scan_poses_world_frame[i] = beam

        req = GetNearestOccupiedPointOnBeamRequest()
        req.beams = self._scan_poses_world_frame
        req.threshold = 1
        response = self._get_nearest_occupied_client(req)
        return self._cal_likelihood(self._scan_ranges, response.distances)


    def _cal_likelihood(self, scanner_ranges, map_ranges):
        '''
        Calculate likelihood from comparing occupied points from map
        and laser ranges
        '''
        if len(scanner_ranges) != len(map_ranges):
            rospy.logerr("Length mismatch between laser ranges and map distances")
        weight = 1.0
        mismatch_count = 0
        # the sum of all the weight would cause most of the map to be red,
        # the average creates gradients redder toward the robot location,
        # but doesn't represent the mismatches case very well
        for i in range(len(scanner_ranges)):
            # eliminate large ranges from map_ranges
            map_range = map_ranges[i]
            if map_range > self._scan_range_max:
                map_range = self._scan_range_max
            if map_range < self._scan_range_min:
                map_range = self._scan_range_min
            difference = scanner_ranges[i] - map_range
            if difference < 2 * self._scan_sigma:
                weight = weight * (math.exp(-(difference)**2 / (2 * self._scan_sigma**2)))
            else:
                mismatch_count = mismatch_count + 1
            # limit number of mismatches
            if mismatch_count > self.ALLOWED_MISMATCH_NUM:
                return 0.0
        return weight


    def _cal_scan_poses_world_frame(self):
        '''
        Calculate transform from world frame to laser scanner frame
        '''
        try:
            time = self._tf.getLatestCommonTime(self.ROBOT_FRAME_ID, self._scan_front_header.frame_id)
            position, quaternion = self._tf.lookupTransform(
                                         self.ROBOT_FRAME_ID,
                                         self._scan_front_header.frame_id,
                                         time)
            self._tf_matrix_robot_to_scan = tf.TransformerROS().fromTranslationRotation(position, quaternion)
            self._scan_front_yaw_robot_frame = tf.transformations.euler_from_quaternion(quaternion)[2]

        except tf.Exception, e:
            rospy.logerr("Error calculating transform: %s", e)


    def _scan_front_callback(self, scan_msg):
        ''' Record laser scan data '''
        self._scan_range_max = scan_msg.range_max
        self._scan_range_min = scan_msg.range_min
        self._scan_angle_increment = scan_msg.angle_increment
        self._scan_ranges = scan_msg.ranges
        if self._scan_front_header is None:
            self._scan_front_header = scan_msg.header
        if self._scan_angle_min is None:
            self._scan_angle_min = scan_msg.angle_min


    """
    ============================== YOUR CODE HERE ==============================
    Instructions:   implemenent the pose likelihood server node including a
                    constructor which should create all needed servers, clients,
                    and subscribers, and appropriate callback functions.
                    GetNearestOccupiedPointOnBeam service allows to query
                    multiple beams in one service request. Use this feature to
                    simulate all the laser beams with one service call, otherwise
                    the time spent on communication with the server will be too
                    long.

    Hint: refer to the sources of the previous assignments or to the ROS
          tutorials to see examples of how to create servers, clients, and
          subscribers.

    Hint: in the laser callback it is enough to just store the incoming laser
          readings in a class member variable so that they could be accessed
          later while processing a service request.

    Hint: the GetNearestOccupiedPointOnBeam service may return arbitrary large
          distance, do not forget to clamp it to [0..max_range] interval.


    Look at the tf library capabilities, you might need it to find transform
    from the /base_link to /base_laser_front_link.
    Here's an example how to use the transform lookup:

        time = self._tf.getLatestCommonTime(frame_id, other_frame_id)
        position, quaternion = self._tf.lookupTransform(frame_id,
                                                        other_frame_id,
                                                        time)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        x, y, yaw = position[0], position[1], yaw

    You might need other functions for transforming routine, you can find
    a brief api description
    http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
    """


if __name__ == '__main__':
    w = PoseLikelihoodServerNode()
    rospy.spin()

