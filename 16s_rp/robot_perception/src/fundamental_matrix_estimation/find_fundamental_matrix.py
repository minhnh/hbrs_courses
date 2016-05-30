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
    fundamental_matrix = np.dot(np.transpose(x_tick_norm_matrix), np.dot(f_norm, x_norm_matrix))
    # Divide by H[2][2] for correct scaling? Not sure why this worked
    fundamental_matrix = fundamental_matrix/fundamental_matrix[2][2]
    return fundamental_matrix


def perform_eight_points(x, x_tick):
    # Dimension check
    if x.shape != x_tick.shape:
        print("Input arrays must have same dimensions")
        return None

    # Construct matrix A in equation A.f = 0
    matrix_a = None
    for i in range(len(x)):
        a_i = construct_a_i_matrix(x[i], x_tick[i])
        # Append to A
        matrix_a = a_i if matrix_a is None else np.append(matrix_a, a_i, axis=0)

    # SVD decomposition, F is the eigenvector corresponding to the
    # smallest singular value
    u, s, v = np.linalg.svd(matrix_a, full_matrices=False)
    fundamental_matrix = v[np.argmin(s)].reshape(3, 3)
    # Replace F_hat with F_hat_tick such that det(F_hat_tick) = 0 (section 11.1.1)
    u, s, v = np.linalg.svd(fundamental_matrix, full_matrices=False)
    s[2] = 0.0
    s = np.diag(s)
    fundamental_matrix = np.dot(u, np.dot(s, np.transpose(v)))

    return fundamental_matrix


def construct_a_i_matrix(single_x, single_x_tick):
    a_i = np.array([single_x_tick[0]*single_x[0],
                    single_x_tick[0]*single_x[1],
                    single_x_tick[0],
                    single_x_tick[1]*single_x[0],
                    single_x_tick[1]*single_x[1],
                    single_x_tick[1],
                    single_x[0],
                    single_x[1],
                    1])
    a_i = a_i.reshape(1, len(a_i))
    return a_i
