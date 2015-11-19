#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult
from actionlib_msgs.msg import GoalStatus

class PathExecutor:

    goal_index = 0
    reached_all_nodes = True

    def __init__(self):
        rospy.loginfo('__init__ start')

        # create a node
        rospy.init_node(NODE)

        # subscribers & servers
        self.path_server = SimpleActionServer('/path_executor/execute_path', ExecutePathAction, self.handle_path, False)
        self.path_server.start()

        # publishers & clients
        self.visualization_publisher = rospy.Publisher('/path_executor/current_path', Path)

        # get parameters from launch file
        self.use_obstacle_avoidance = rospy.get_param('use_obstacle_avoidance', False)
        if self.use_obstacle_avoidance == False:
            self.goal_client = SimpleActionClient('/motion_controller/move_to', MoveToAction)
        else:
            self.goal_client = SimpleActionClient('/bug2/move_to', MoveToAction)


        self.goal_client.wait_for_server()

        # other fields
        self.goal_index = 0
        self.executePathGoal = None
        self.executePathResult = ExecutePathResult()

        rospy.loginfo('__init__ end')

    def handle_path(self, paramExecutePathGoal):
        rospy.loginfo('handle_goal')
        rospy.loginfo('paramExecutePathGoal size')
        rospy.loginfo(len(paramExecutePathGoal.path.poses))

        self.goal_index = 0
        self.executePathGoal = paramExecutePathGoal
        self.executePathResult = ExecutePathResult()

        self.visualization_publisher.publish(self.executePathGoal.path)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if not self.path_server.is_active():
                return

            if self.path_server.is_preempt_requested():
                rospy.loginfo('aborting... preempt requested')
                self.path_server.set_aborted(self.executePathResult, 'aborting... preempt requested')

                # clean up
                self.goal_client.cancel_all_goals()

                # say goodbye!
                self.path_server.set_preempted()
                break

            if self.goal_index < len(self.executePathGoal.path.poses):
                rospy.loginfo('working on goal: ' + str(self.goal_index))

                moveto_goal = MoveToGoal()
                moveto_goal.target_pose = self.executePathGoal.path.poses[self.goal_index]
                self.goal_client.send_goal(moveto_goal, done_cb=self.handle_goal)
            else:
                rospy.loginfo('covered_all_path')
                if(self.reached_all_nodes == True):
                    self.path_server.set_succeeded(self.executePathResult, 'succeeded... covered_all_path and reached_all_nodes')
                else:
                    self.path_server.set_succeeded(self.executePathResult, 'failed... covered_all_path BUT couldnt reach all nodes')
                return

            rate.sleep()
        self.path_server.set_aborted(self.executePathResult, 'Aborted. The node has been killed.')


    def handle_goal(self, state, result):

        feedback = ExecutePathFeedback()
        feedback.pose = self.executePathGoal.path.poses[self.goal_index]

        # state is GoalStatus message as shown here:
        # http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if state == GoalStatus.SUCCEEDED:
            self.executePathResult.visited.append(True)
            feedback.reached = True
        else:
            self.executePathResult.visited.append(False)
            feedback.reached = False
            self.reached_all_nodes = False # global flag to highlight that at least one node was not reachable


        self.path_server.publish_feedback(feedback)

        self.goal_index = self.goal_index + 1



if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
