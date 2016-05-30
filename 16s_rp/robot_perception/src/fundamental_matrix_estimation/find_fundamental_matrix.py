#!/usr/bin/env python3

import numpy as np
from homography_estimation import util


def perform_normalized_eight_points(x, x_tick):
    """
    :param x:
    :param x_tick:
    """
    # Normalize x and x_tick
    (x_norm_matrix, x_norm) = util.normalize(x)
    (x_tick_norm_matrix, x_tick_norm) = util.normalize(x_tick)
    # DLT algorithm
    f_norm = perform_eight_points(x_norm, x_tick_norm)
    # Denormalize H_norm
    fundamental_matrix = np.dot(np.dot(np.linalg.inv(x_norm_matrix), f_norm), x_norm_matrix)
    return fundamental_matrix


def perform_eight_points(x, x_tick):
    pass


def construct_A_i_matrix():
    pass
