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
    _NO_TARGET_THRESHOLD = 5

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
        self._execute_path_client.wait_for_server()
        self._tf = tf.TransformListener()
        self._last_map_publication = rospy.Time.now()
        rospy.loginfo('Started [explorer] node.')
        self._targets = None
        self._sizes = None
        self._world_is_explored = False
        self._no_target_counter = 0


    def explore(self):
        while not rospy.is_shutdown() and not self._world_is_explored:
            if self._targets is None:
                rospy.logwarn(self.__class__.__name__ + ': No frontier info, waiting')
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
            if len(self._sizes) < 1:
                rospy.logwarn(self.__class__.__name__ + ': No frontier info, going forward')
                # Move forward
                current_pose = None
                while current_pose is None:
                    current_pose = self._get_current_pose()
                current_pose.x = current_pose.x + 1.0
                self._targets = [current_pose]
                self._sizes = [0.0]
                continue

            rospy.loginfo("targets: %d, sizes: %d", len(self._targets), len(self._sizes))

            #?
            if self._no_target_counter > self._NO_TARGET_THRESHOLD:
                self._world_is_explored = True
            #?

            while len(self._targets) > 0:
                # Setup service request to path_plan
                path_plan_req = PlanPathRequest()
                # Choose largest cluster
                target_index = min(xrange(len(self._sizes)), key=self._sizes.__getitem__)
                target = self._targets[target_index]
                # start at current pose
                start = self._get_current_pose()
                # Check None poses
                if start is None:
                    rospy.logerr(self.__class__.__name__ + ": _get_current_pose() return None")
                    continue
                elif target is None:
                    rospy.logerr(self.__class__.__name__ + ": None target")
                    continue
                else:
                    path_plan_req.start = start
                    path_plan_req.end = target

                # Send request
                try:
                    path_plan_resp = self._path_plan_client(path_plan_req)
                    path_goal = ExecutePathGoal()
                    path_goal.path = path_plan_resp.path
                    path_goal.skip_unreachable = True
                    self._execute_path_client.send_goal(path_goal)
                except rospy.ServiceException as e:
                    rospy.logerr(self.__class__.__name__ + ': Path plan service failed: ' + str(e))

                # Remove arrived target
                self._targets.remove(self._targets[target_index])
                self._sizes.remove(self._sizes[target_index])

        rospy.loginfo(self.__class__.__name__ + ': World is explored, exiting.')


    def _get_current_pose(self):
        try:
            time = self._tf.getLatestCommonTime("odom", "base_link")
            position, quaternion = self._tf.lookupTransform("odom",
                                                            "base_link",
                                                            time)
        except Exception as ex:
            rospy.logwarn(self.__class__.__name__ + ': Unable to lookup base transform '
                          'Reason: ' + str(ex))
            return None
        pose = Pose2D()
        pose.x, pose.y = position[0], position[1]
        pose.theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return pose


    def _frontier_callback(self, frontiers_msg):
        rospy.loginfo(self.__class__.__name__ + ": FRONTIER CALLBACK")
        self._targets = frontiers_msg.centroids
        self._sizes = frontiers_msg.sizes
        # Increment counter if frontier unable to find any centroids, else reset
        if len(self._targets) == 0:
            self._no_target_counter = self._no_target_counter + 1
        else:
            self._no_target_counter = 0
        pass


if __name__ == '__main__':
    n = ExplorerNode()
    n.explore()