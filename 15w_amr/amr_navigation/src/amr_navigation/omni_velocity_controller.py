#!/usr/bin/env python

PACKAGE = 'amr_navigation'
import rospy
import numpy as np
import math
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance

class OmniVelocityController(VelocityController):

    # Index in coordinate arrays
    X_INDEX = 0
    Y_INDEX = 1

    def __init__(self, l_max_vel, l_max_acc, l_tolerance,
                       a_max_vel, a_max_acc, a_tolerance,
                       controller_frequency):
        self._controller_period = 1 / controller_frequency

        self._l_max_vel = l_max_vel
        self._l_max_acc = l_max_acc
        self._l_tolerance = l_tolerance

        self._a_max_vel = a_max_vel
        self._a_max_acc = a_max_acc
        self._a_tolerance = a_tolerance

        # Use equation 2*a*d = v_final^2 - v_initial^2 to find minimum
        # decelerate distance. Adjust for tolerance
        self._a_min_dist_dec = self._a_max_vel * self._a_max_vel / 2 / self._a_max_acc + self._a_tolerance
        self._l_min_dist_dec = self._l_max_vel * self._l_max_vel / 2 / self._l_max_acc + self._l_tolerance

        # Store last linear speed and angular velocity for deceleration
        self._l_last_speed = None
        self._a_last_vel = None


    """
    ========================= YOUR CODE HERE =========================

    Instructions: put here all the functions that are necessary to
    implement the VelocityController interface. You may
    use the DiffVelocityController as an example.

    Implement the constructor to accept all the necessry parameters
    and implement compute_velocity() method

    You are free to write any helper functions or classes you might
    need.

    ==================================================================

    """
    def compute_velocity(self, actual_pose):
        # Compute remaining distances, return if close to target
        linear_dist = get_distance(self._target_pose, actual_pose)
        angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)
        if (    abs(linear_dist) < self._l_tolerance and
                abs(angular_dist) < self._a_tolerance   ):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        # Find transform matrix of world frame in agent frame
        # This is needed for perfect straight line travel. Linear velocity
        # to target is a constant deceleration movement, which may not be
        # apparent in values posted to /cmd_vel
        transform_AW = get_inverse_transform2D(actual_pose.theta,
                                               actual_pose.x,
                                               actual_pose.y)
        target_coord_A = np.dot(transform_AW, np.array([self._target_pose.x,
                                                        self._target_pose.y,
                                                        0, 1]))
        # Compute linear and angular velocity
        velocity_linear = self.compute_linear_velocity(target_coord_A[self.X_INDEX],
                                                target_coord_A[self.Y_INDEX],
                                                linear_dist)

        velocity_angular = self.compute_angular_velocity(angular_dist)
        #rospy.loginfo("Velocity angular: " + str(velocity_angular))

        return Velocity(velocity_linear[self.X_INDEX],
                        velocity_linear[self.Y_INDEX],
                        velocity_angular)

    def compute_linear_velocity(self, target_X_A, target_Y_A, linear_dist):
        '''
        Calculate linear velocity from linear distance and coordinate
        of targets from the agent's frame.
        '''
        velocity = [0, 0]
        if (linear_dist < self._l_tolerance):
            self._linear_complete = True
            return velocity

        if (abs(target_X_A) > self._l_min_dist_dec and
            abs(target_Y_A) > self._l_min_dist_dec):

            if (abs(target_X_A) > abs(target_Y_A)):
                velocity[self.X_INDEX] = math.copysign(self._l_max_vel, target_X_A)
                velocity[self.Y_INDEX] = velocity[self.X_INDEX] / target_X_A * target_Y_A
            else:
                velocity[self.Y_INDEX] = math.copysign(self._l_max_vel, target_Y_A)
                velocity[self.X_INDEX] = velocity[self.Y_INDEX] / target_Y_A * target_X_A

            current_speed = math.sqrt(velocity[0] * velocity[0] +
                                      velocity[1] * velocity[1])
        else:
            # Handle issue where initial velocity is maxed out at close distance
            if (self._l_last_speed == None):
                self._l_last_speed = math.sqrt(abs(2 * self._l_max_acc * linear_dist))

            acceleration = self._l_last_speed * self._l_last_speed / 2 / linear_dist
            #acceleration = self._l_max_acc

            # Handle issue where last_speed is too small
            if (self._l_last_speed > acceleration * self._controller_period):
                current_speed = self._l_last_speed - acceleration * self._controller_period
            else:
                current_speed = acceleration * self._controller_period

            # Handle movement only in Y direction (when x < tolerance)
            if (abs(target_X_A) > self._l_tolerance):
                target_angle = math.atan2(target_Y_A, target_X_A)
                velocity[self.Y_INDEX] = current_speed * math.sin(target_angle)
                velocity[self.X_INDEX] = current_speed * math.cos(target_angle)
            else:
                velocity[self.Y_INDEX] = math.copysign(current_speed, target_Y_A)

        # Store linear speed
        self._l_last_speed = current_speed

        return velocity

    def compute_angular_velocity(self, angular_dist):
        '''
        Compute angular velocity based on the current angular distance
        '''
        if abs(angular_dist) < self._a_tolerance:
            self._angular_complete = True
            return 0

        if abs(angular_dist) > self._a_min_dist_dec:
            velocity_angular = math.copysign(self._a_max_vel, angular_dist)
        else:
            # Handle small initial angles
            if self._a_last_vel == None:
                self._a_last_vel = math.sqrt(abs(2 * self._a_max_acc * angular_dist))
                self._a_last_vel = math.copysign(self._a_last_vel, angular_dist)
            # acceleration is negative of angular_dist
            acceleration = - self._a_last_vel * self._a_last_vel / 2 / angular_dist
            #acceleration = math.copysign(self._a_max_acc, -angular_dist)
            velocity_angular = self._a_last_vel + acceleration * self._controller_period
            velocity_angular = math.copysign(velocity_angular, angular_dist)

        self._a_last_vel = velocity_angular

        return velocity_angular

def get_rotation_matrix(alpha, beta, gamma):
    """
    Returns angular rotation matrix given three rotation angles
    following the Euler's method
    """
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    cg = np.cos(gamma)
    sg = np.sin(gamma)

    rotation_matrix = np.array([[ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg ],
                                [sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg ],
                                [  -sb,            cb*sg,            cb*cg ]])
    return rotation_matrix

def get_inverse_transform2D(alpha, x, y):
    """
    Returns inverted frame transform from the regular rotation and
    translation matrices
    """
    rotation_matrix = get_rotation_matrix(alpha, 0, 0)
    rotation_matrix_tp = np.transpose(rotation_matrix)
    pa_borg = np.array([x, y, 0])
    pb_aorg = -np.dot(rotation_matrix_tp, np.reshape(pa_borg, (3,1)))

    result = np.append(rotation_matrix_tp, pb_aorg, axis=1)
    result = np.append(result, [[0, 0, 0, 1]], axis=0)

    return result
