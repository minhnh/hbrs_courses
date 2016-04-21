#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The state machine contains three states:
    * findWall:     initial state - drives until a wall is detected
    * alignWall     aligning state - used to align at convex corners or walls in front of robot
    * followWall    following state - used to follow a straight wall, robust against sensor noise, curls around concave corners

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import rospy
import roslib
roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from math import log
from math import cos
from math import sin
from math import radians
from math import atan2
from math import sqrt
from types import MethodType
from geometry_msgs.msg import Twist

__all__ = ['construct']

# Indices of velocity and sonar pose arrays
_X_LINEAR = 0
_Y_LINEAR = 1
_Z_ANGULAR = 2


def search(ud):
    '''
    Go toward closest wall
    '''
    if ud.min_all < ud.clearance:
        return 'wall_approached'
    x_vel = ud.max_forward_velocity * cos(ud.sonar_pose[ud.min_all_index][_Z_ANGULAR])
    y_vel = ud.max_forward_velocity * sin(ud.sonar_pose[ud.min_all_index][_Z_ANGULAR])
    ud.velocity = (x_vel, y_vel, 0)


def align_angular_only(ud):
    '''
    Spin until at a reasonable angle with the wall
    '''
    if ud.min_all > 2 * ud.clearance:
        return 'lost'
    if (ud.side_mean <= (ud.youbot_diag - ud.youbot_width) / 2 + ud.clearance):
        return 'aligned'
    ud.velocity = (0, 0, ud.default_rotational_speed)


def align_forward(ud):
    '''
    Move forward while keeping distance to the wall
    '''
    if ud.min_all > 2 * ud.clearance:
        return 'lost'
    if ud.in_corner or ud.front_distance <= ud.clearance:
        return 'concave_detected'

    # Ensure rotational_speed less than default speed
    #if ud.side_back_sonar - ud.side_front_sonar > ud.side_sonar_distance:
    if ud.convex_detected:
        return 'convex_detected'
    # Calculate rotational speed depending on how the robot is oriented
    # with respect to the wall
    rotational_speed = (atan2(ud.side_back_sonar - ud.side_front_sonar,
                              ud.side_sonar_distance)
                        * ud.default_rotational_speed)
    # Calculate linear speed in y to keep distance from wall. Set maximum
    # y-axis speed to the same with max_forward_velocity
    linear_y_speed = (ud.side_mean - ud.clearance) / ud.clearance
    if linear_y_speed > ud.max_forward_velocity:
        linear_y_speed = ud.max_forward_velocity
    ud.velocity = (0.3, linear_y_speed, rotational_speed)


def convex(ud):
    '''
    Rotate until out of convex
    '''
    if ud.min_all > 2 * ud.clearance:
        return 'lost'
    if not ud.convex_detected:
        return 'convex_passed'
    ud.velocity = (ud.max_forward_velocity, 0, -ud.default_rotational_speed)


def concave(ud):
    '''
    Rotate until out of concave
    '''
    if ud.min_all > 2 * ud.clearance:
        return 'lost'
    if ud.convex_detected:
        return 'convex_detected'

    if not ud.in_corner and (ud.sonar_10 > ud.sonar_30 and
                             ud.sonar_30 > ud.sonar_50 and
                             ud.sonar_50 > ud.side_mean):
        return 'concave_passed'
    ud.velocity = (0, 0, ud.default_rotational_speed)


def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback.
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """
    range_values = []
    for value in ranges:
        range_values.append(value.range)
    # Adjust for sonar's pose
    front_distance_2 = range_values[2] * cos(self.userdata.sonar_pose[2][_Z_ANGULAR])
    front_distance_3 = range_values[3] * cos(self.userdata.sonar_pose[3][_Z_ANGULAR])
    front_distance_4 = range_values[4] * cos(self.userdata.sonar_pose[4][_Z_ANGULAR])
    front_distance_5 = range_values[5] * cos(self.userdata.sonar_pose[5][_Z_ANGULAR])
    # Store min of all sonars for search state
    self.userdata.min_all = min(range_values)
    self.userdata.min_all_index = min(xrange(len(range_values)),
                                      key=range_values.__getitem__)
    # Store min front distace to avoid problems with doors
    self.userdata.front_distance = min(front_distance_3, front_distance_4,
                                       front_distance_2, front_distance_5)
    # front distance to allow space for spinning in corners
    if  ( front_distance_2 < self.userdata.front_distance and
          front_distance_5 < self.userdata.front_distance and
          self.userdata.front_distance < ( self.userdata.youbot_width / 2 +
                                           self.userdata.clearance )):
        self.userdata.in_corner = True
    else:
        self.userdata.in_corner = False

    self.userdata.side_front_sonar = range_values[self.userdata.sonar_side_front_index]
    self.userdata.side_back_sonar = range_values[self.userdata.sonar_side_back_index]
    self.userdata.side_mean = (self.userdata.side_front_sonar + self.userdata.side_back_sonar) / 2
    # Store min side distance to avoid problems at sharp convexes
    side_min = min(self.userdata.side_front_sonar, self.userdata.side_back_sonar)
    # Store front sonar at 3 angles of the turning side
    self.userdata.sonar_10 = range_values[self.userdata.sonar_10_index]
    self.userdata.sonar_30 = range_values[self.userdata.sonar_30_index]
    self.userdata.sonar_50 = range_values[self.userdata.sonar_50_index]
    # Convex condition is when sonar at 50 degrees is farther from the wall than
    # side sonar
    if self.userdata.sonar_50 * sin(radians(50)) > side_min + self.userdata.clearance:
        self.userdata.convex_detected = True
    else:
        self.userdata.convex_detected = False


def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.x = self.userdata.velocity[0]
    twist.linear.y = self.userdata.velocity[1] * (-self.userdata.direction)
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = self.userdata.velocity[2] * self.userdata.direction

    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client. self.userdata.direction sets a velocity sign depending on the mode.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    if self.userdata.mode == 1:
        self.userdata.direction = 1
        self.userdata.sonar_10_index = 4
        self.userdata.sonar_30_index = 5
        self.userdata.sonar_50_index = 6
        self.userdata.sonar_side_front_index = 7
        self.userdata.sonar_side_back_index = 8
    else:
        self.userdata.direction = -1
        self.userdata.sonar_10_index = 3
        self.userdata.sonar_30_index = 2
        self.userdata.sonar_50_index = 1
        self.userdata.sonar_side_front_index = 0
        self.userdata.sonar_side_back_index = 15
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.3
    sm.userdata.ranges = None
    sm.userdata.max_forward_velocity = 0.3
    sm.userdata.default_rotational_speed = 0.5
    sm.userdata.direction = 1

    # Newly defined state variables
    # Data acquired from amr_stage_worlds/models/sonars.inc
    sm.userdata.sonar_pose = [[ 0.075,  0.130, radians(  90) ],  # 0
                              [ 0.115,  0.115, radians(  50) ],  # 1
                              [ 0.150,  0.080, radians(  30) ],  # 2
                              [ 0.170,  0.025, radians(  10) ],  # 3
                              [ 0.170, -0.025, radians( -10) ],  # 4
                              [ 0.150, -0.080, radians( -30) ],  # 5
                              [ 0.115, -0.115, radians( -50) ],  # 6
                              [ 0.075, -0.130, radians( -90) ],  # 7
                              [-0.155, -0.130, radians( -90) ],  # 8
                              [-0.195, -0.115, radians(-130) ],  # 9
                              [-0.230, -0.080, radians(-150) ],  # 10
                              [-0.250, -0.025, radians(-170) ],  # 11
                              [-0.250,  0.025, radians( 170) ],  # 12
                              [-0.230,  0.080, radians( 150) ],  # 13
                              [-0.195,  0.115, radians( 130) ],  # 14
                              [-0.155,  0.130, radians(  90) ]]  # 15
    sm.userdata.youbot_length = 0.58
    sm.userdata.youbot_width = 0.38
    sm.userdata.youbot_diag = sqrt(sm.userdata.youbot_length**2 +
                                   sm.userdata.youbot_width**2)
    sm.userdata.in_corner = False
    sm.userdata.convex_detected = False
    sm.userdata.side_front_sonar = 0
    sm.userdata.side_back_sonar = 0
    sm.userdata.side_mean = 0
    sm.userdata.sonar_10 = 0
    sm.userdata.sonar_30 = 0
    sm.userdata.sonar_50 = 0
    # Min of all sonar for searching state
    sm.userdata.min_all = 0
    sm.userdata.min_all_index = 0
    # Indices of sonar readings dependent on mode
    sm.userdata.sonar_10_index = 4
    sm.userdata.sonar_30_index = 5
    sm.userdata.sonar_50_index = 6
    sm.userdata.sonar_side_front_index = 7
    sm.userdata.sonar_side_back_index = 8
    # Assuming 2 pairs of side sonar are mounted mirrored to each other, as
    # indicated in above data
    sm.userdata.side_sonar_distance = (
            sm.userdata.sonar_pose[sm.userdata.sonar_side_front_index][_X_LINEAR] -
            sm.userdata.sonar_pose[sm.userdata.sonar_side_back_index ][_Y_LINEAR])

    with sm:
        smach.StateMachine.add('SEARCH',
                               PreemptableState(search,
                                                input_keys=['in_corner',
                                                            'front_distance',
                                                            'clearance',
                                                            'max_forward_velocity',
                                                            'min_all',
                                                            'min_all_index',
                                                            'sonar_pose'],
                                                output_keys=['velocity'],
                                                outcomes=['wall_approached']),
                               transitions={'wall_approached' : 'ALIGN_ANGULAR_ONLY'})
        smach.StateMachine.add('ALIGN_ANGULAR_ONLY',
                               PreemptableState(align_angular_only,
                                                input_keys=['side_front_sonar',
                                                            'side_back_sonar',
                                                            'youbot_width',
                                                            'youbot_diag',
                                                            'front_distance',
                                                            'side_mean',
                                                            'clearance',
                                                            'default_rotational_speed',
                                                            'min_all'],
                                                output_keys=['velocity'],
                                                outcomes=['aligned',
                                                          'concave_detected',
                                                          'lost']),
                               transitions={'aligned' : 'ALIGN_FORWARD',
                                            'concave_detected' : 'CONCAVE',
                                            'lost' : 'SEARCH'})
        smach.StateMachine.add('ALIGN_FORWARD',
                               PreemptableState(align_forward,
                                                input_keys=['side_back_sonar',
                                                            'side_front_sonar',
                                                            'side_sonar_distance',
                                                            'side_mean',
                                                            'in_corner',
                                                            'convex_detected',
                                                            'default_rotational_speed',
                                                            'max_forward_velocity',
                                                            'front_distance',
                                                            'clearance',
                                                            'min_all'],
                                                output_keys=['velocity'],
                                                outcomes=['convex_detected',
                                                          'concave_detected',
                                                          'lost']),
                               transitions={'convex_detected' : 'CONVEX',
                                            'concave_detected': 'CONCAVE',
                                            'lost' : 'SEARCH'})
        smach.StateMachine.add('CONVEX',
                               PreemptableState(convex,
                                                input_keys=['convex_detected',
                                                            'default_rotational_speed',
                                                            'max_forward_velocity',
                                                            'clearance',
                                                            'min_all'],
                                                output_keys=['velocity'],
                                                outcomes=['convex_passed',
                                                          'lost']),
                               transitions={'convex_passed': 'ALIGN_FORWARD',
                                            'lost' : 'SEARCH'})
        smach.StateMachine.add('CONCAVE',
                               PreemptableState(concave,
                                                input_keys=['in_corner',
                                                            'convex_detected',
                                                            'clearance',
                                                            'default_rotational_speed',
                                                            'front_distance',
                                                            'sonar_10',
                                                            'sonar_30',
                                                            'sonar_50',
                                                            'side_mean',
                                                            'min_all'],
                                                output_keys=['velocity'],
                                                outcomes=['concave_passed',
                                                          'convex_detected',
                                                          'lost']),
                               transitions={'concave_passed' : 'ALIGN_FORWARD',
                                            'convex_detected' : 'CONVEX',
                                            'lost' : 'SEARCH'})
    return sm
