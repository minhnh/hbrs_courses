#!/usr/bin/env python


#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'

from amr_navigation import omni_velocity_controller as ovc
import numpy as np
import math
import rospy

_X_INDEX = 0
_Y_INDEX = 1
_LEFT_MODE = -1
_RIGHT_MODE = 1

class BugBrain:

    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        self._goal_x = goal_x
        self._goal_y = goal_y
        self._goal_x_from_bot = 0
        self._goal_y_from_bot = 0
        self._start_coords = None
        # Store where youbot leave and back to wall following
        self._leave_wall_coords = []
        self._skip_wall_leave = []
        self._begin_wall_following_coords = []
        # Record walls between youbot and goal
        self._vertical_walls = []
        self._horizontal_walls = []
        if side == 1:
            self._direction = 1
        else:
            self._direction = -1


    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        # Store where youbot starts following wall again after leaving wall
        for coord in self._begin_wall_following_coords:
            if abs(x - coord[_X_INDEX]) < self.TOLERANCE and abs(y - coord[_Y_INDEX]) < self.TOLERANCE:
                return
        if len(self._leave_wall_coords) > len(self._begin_wall_following_coords):
            self._begin_wall_following_coords.append([x, y])


    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        # compute and store necessary variables
        x_wall_new = x
        y_wall_new = y
        for coord in self._leave_wall_coords:
            if abs(x - coord[_X_INDEX]) < self.TOLERANCE and abs(y - coord[_Y_INDEX]) < self.TOLERANCE:
                return

        if abs(x - self._goal_x) < self.TOLERANCE:
            y_wall_new = y - self._direction * self.TOLERANCE * math.copysign(1, math.cos(theta))

        if abs(y - self._goal_y) < self.TOLERANCE:
            x_wall_new = x + self._direction * self.TOLERANCE * math.copysign(1, math.sin(theta))
            # Check if similar wall already in list
        self._leave_wall_coords.append([x_wall_new, y_wall_new])
        self._skip_wall_leave.append(False)


    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        if len(self._vertical_walls) < 2 or len(self._horizontal_walls) < 2:
            return False
        if ( abs(x - self._start_coords[_X_INDEX]) < self.TOLERANCE and
             abs(y - self._start_coords[_Y_INDEX]) < self.TOLERANCE):
            if len(self._leave_wall_coords) == 0:
                return True
            # If there's a 'leave spot' not skipped, keep trying
            for v in self._skip_wall_leave:
                if v == False:
                    return False
            return True
        return False


    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        if abs(x - self._goal_x) > self.TOLERANCE and abs(y - self._goal_y) > self.TOLERANCE:
            return False

        if len(self._leave_wall_coords) == 0 and self._start_coords is None:
            self._start_coords = [x, y]

        # If close to location of begining following wall, return False
        if len(self._begin_wall_following_coords) > 0:
            follow_wall_coord = self._begin_wall_following_coords[-1]
            if      abs(x - follow_wall_coord[_X_INDEX]) < self.TOLERANCE and\
                    abs(y - follow_wall_coord[_Y_INDEX]) < self.TOLERANCE:
                return False

        self.find_walls(x, y, theta)

        if abs(x - self._goal_x) < self.TOLERANCE:
            if y > self._goal_y:
                # Return False if there's a wall above goal below youbot
                for y_wall in self._vertical_walls:
                    if y_wall > self._goal_y and y_wall < y:
                        return False
            else:
                # Return False if there's a wall below goal above youbot
                for y_wall in self._vertical_walls:
                    if y_wall < self._goal_y and y_wall > y:
                        return False

            # Check if have left wall here, comparing y
            for i, v in enumerate(self._leave_wall_coords):
                leave_wall_range = [v[_Y_INDEX], self._begin_wall_following_coords[i][_Y_INDEX]]
                if min(leave_wall_range) < y and max(leave_wall_range) > y:
                    self._skip_wall_leave[i] = True
                    return False

        elif abs(y - self._goal_y) < self.TOLERANCE:
            if x > self._goal_x:
                # Return False if there's a wall right of goal left of youbot
                for x_wall in self._horizontal_walls:
                    if x_wall > self._goal_x and x_wall < x:
                        return False
            else:
                # Return False if there's a wall left of goal right of youbot
                for x_wall in self._horizontal_walls:
                    if x_wall < self._goal_x and x_wall > x:
                        return False

            # Check if have left wall here, comparing x
            for i, v in enumerate(self._leave_wall_coords):
                leave_wall_range = [v[_X_INDEX], self._begin_wall_following_coords[i][_X_INDEX]]
                if min(leave_wall_range) < x and max(leave_wall_range) > x:
                    self._skip_wall_leave[i] = True
                    return False

        return True


    def find_walls(self, x, y, theta):
        """ Find and append wall coordinates to wall lists """

        if abs(x - self._goal_x) > self.TOLERANCE and abs(y - self._goal_y) > self.TOLERANCE:
            return

        transform_AG = ovc.get_inverse_transform2D(theta, x, y)
        target_coord_A = np.dot(transform_AG, np.array([self._goal_x, self._goal_y,
                                                        0, 1]))
        self._goal_x_from_bot = target_coord_A[_X_INDEX]
        self._goal_y_from_bot = target_coord_A[_Y_INDEX]

        # If goal is in positive y when right following or negative y when left
        # following in youbot's frame, there's no wall between youbot and wall
        if self._goal_y_from_bot * self._direction > 0:
            return

        if abs(x - self._goal_x) < self.TOLERANCE:

            y_wall_new = y - self._direction * self.TOLERANCE * math.copysign(1, math.cos(theta))
            # Check if similar wall already in list
            for y_wall in self._vertical_walls:
                if abs(y_wall_new - y_wall) < self.TOLERANCE:
                    return
            self._vertical_walls.append(y_wall_new)

        if abs(y - self._goal_y) < self.TOLERANCE:
            x_wall_new = x + self._direction * self.TOLERANCE * math.copysign(1, math.sin(theta))
            # Check if similar wall already in list
            for x_wall in self._horizontal_walls:
                if abs(x_wall_new - x_wall) < self.TOLERANCE:
                    return
            # Record new wall
            self._horizontal_walls.append(x_wall_new)

#==============================================================================
