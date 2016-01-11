#!/usr/bin/env python

PACKAGE = 'amr_exploration'
NODE = 'explorer'

import random
import rospy
import actionlib
import tf
from math import pi
from geometry_msgs.msg import Pose2D
from amr_msgs.msg import Ranges, ExecutePathAction, ExecutePathGoal, PathExecutionFailure, Frontiers
from nav_msgs.msg import MapMetaData, OccupancyGrid
from amr_srvs.srv import PlanPathRequest, PlanPath, PlanPathResponse

class ExplorerNode:
    
    
    def __init__(self):
        
        rospy.init_node(NODE)
        """
            Publishers
        """
        self._path_failures_publisher = rospy.Publisher('path_planner/path_execution_failures',
                                                        PathExecutionFailure,
                                                        queue_size=10)
        """
            Subscribers
        """
        self._frontier_subscriber = rospy.Subscriber('frontier_centroids',
                                                     Frontiers,
                                                     self._frontier_callback,
                                                     queue_size=1)
        
        self._path_plan_client = rospy.ServiceProxy('path_planner/plan_path',
                                                    PlanPath)
        self._execute_path_client = actionlib.SimpleActionClient('path_executor/execute_path',
                                                                 ExecutePathAction)
        self._tf = tf.TransformListener()
        self._last_map_publication = rospy.Time.now()
        rospy.loginfo('Started [explorer] node.')
        self._targets = None
        self._world_is_explored = False
        
    
    def explore(self):
        while not rospy.is_shutdown() and not self._world_is_explored:
            if self._targets is None:
                rospy.logwarn('No frontier information acquired, waiting')
                rospy.sleep(2.0)
                continue
            """
            Your code here
            
            Centroids of the frontier clusters are stored in self._targets
            
            Check Frontiers essage for more info.
            
            * Choose a target
            * Plan a path to it
            * Execute the path
            .. etc
            """
            pass
        rospy.loginfo('World is explored, exiting.')
        
    
    def _get_current_pose(self):
        try:
            position, quaternion = self._tf.lookupTransform("odom",
                                                            "base_link",
                                                            rospy.Time.now())
        except Exception as ex:
            rospy.logwarn('Unable to lookup the base transform '
                          'Reason: {}.'.format(ex.message))
            return None
        pose = Pose2D()
        pose.x, pose.y = position[0], position[1]
        pose.theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return pose
    
    
    def _frontier_callback(self, frontiers_msg):
        rospy.loginfo("EXPLORER: FRONTIER CALLBACK")
        self._targets = frontiers_msg.centroids
        pass


if __name__ == '__main__':
    n = ExplorerNode()
    n.explore()