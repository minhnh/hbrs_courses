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
        self._ranges = None
        self._range_max = None
        self._range_min = None
        self._angle_increment = None
        self._laser_sigma = 0.7
        # transformation from robot frame to laser scanner frame
        self._tf_matrix_robot_to_scan = None
        self._scan_poses_world_frame = []
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
        '''
        multiple pose likelihood srv
            # Robot pose to test
            geometry_msgs/PoseStamped[] poses
            ---
            # Likelihood of the pose (in the 0 to 1 range)
            float32[] likelihoods
        PoseStamped message:
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        '''
        response = []
        for pose in multi_pose_srv.poses:
            # calculate response
            response.append(self._calculate_single_likelihood(pose))

        return GetMultiplePoseLikelihoodResponse(response)


    def _calculate_single_likelihood(self, pose):
        ''' Run here instead of _init_ to have access to _scan_front_header'''
        # get world frame id
        if self._world_frame_id is None:
            self._world_frame_id = pose.header.frame_id
        # get tf matrix from robot to laser scanner
        self._cal_scan_poses_world_frame()
        # Calculate robot to world frame tf matrix
        #tf_matrix_world_to_robot = tf.TransformerROS().transformPose(
        #                                                    self._world_frame_id,
        #                                                    pose.pose.orientation)
        #tf_matrix_world_to_scan = np.dot(tf_matrix_world_to_robot, self._tf_matrix_robot_to_scan)
        req = GetNearestOccupiedPointOnBeamRequest()
        req.beams = self._scan_poses_world_frame
        req.threshold = 1
        response = self._get_nearest_occupied_client(req)
        rospy.loginfo(len(response.distances))
        return 0.5


    def _cal_scan_poses_world_frame(self):
        '''
        Calculate transform from world frame to laser scanner frame
        '''
        try:
            time = self._tf.getLatestCommonTime(self._world_frame_id, self._scan_front_header.frame_id)
            position, quaternion = self._tf.lookupTransform(
                                         self._world_frame_id,
                                         self._scan_front_header.frame_id,
                                         time)
            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            x, y, yaw = position[0], position[1], yaw
            for i in range(len(self._ranges)):
                beam = Pose2D()
                beam.x = x
                beam.y = y
                beam.theta = yaw + i * self._angle_increment
                # Keep beam length to 12
                if len(self._scan_poses_world_frame) < i + 1:
                    self._scan_poses_world_frame.append(beam)
                else:
                    self._scan_poses_world_frame[i] = beam
            #self._tf_matrix_robot_to_scan = tf.TransformerROS.fromTranslationRotation(position, quaternion)
        except tf.Exception, e:
            rospy.logerr("Error calculating transform: %s", e)


    def _scan_front_callback(self, scan_msg):
        ''' Record laser scan data '''
        '''
        LaserScan message content:
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            float32 angle_min
            float32 angle_max
            float32 angle_increment
            float32 time_increment
            float32 scan_time
            float32 range_min
            float32 range_max
            float32[] ranges
            float32[] intensities
        '''
        #TODO Normalize ranges?
        self._range_max = scan_msg.range_max
        self._range_min = scan_msg.range_min
        self._angle_increment = scan_msg.angle_increment
        self._ranges = scan_msg.ranges
        if self._scan_front_header is None:
            self._scan_front_header = scan_msg.header

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

