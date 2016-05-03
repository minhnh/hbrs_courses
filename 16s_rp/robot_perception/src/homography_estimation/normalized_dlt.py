#!/usr/bin/env python3

import numpy as np
from homography_estimation import util


def get_homography(x, x_tick):
    """
    Perform normalized DLT algorithm

    :param x: coordinates of points in original image
    :param x_tick: coordinates of similar points in transformed image
    :return: homography matrix
    """
    # Normalize x and x_tick
    (x_similar_transform, x_norm) = util.normalize(x)
    (x_tick_similar_transform, x_tick_norm) = util.normalize(x_tick)
    # DLT algorithm
    h_norm = get_homography_unnormalized(x_norm, x_tick_norm)
    # Denormalize H_norm
    homography = np.dot(np.dot(np.linalg.inv(x_tick_similar_transform), h_norm), x_similar_transform)
    return homography


def get_homography_unnormalized(x, x_tick):
    """
    Perform DLT algorithm
    """
    # Dimension check
    if x.shape != x_tick.shape:
        print("Input arrays must have same dimensions")
        return None

    # Construct matrix A in equation A.H = 0
    A = None
    for i in range(len(x)):
        A_i = construct_A_i_matrix(x[i], x_tick[i])
        # Append to A
        if A is None:
            A = A_i
        else:
            A = np.append(A, A_i, axis=0)

    # SVD decomposition, H is the eigenvector corresponding to the
    # smallest singular value
    u, s, v = np.linalg.svd(A, full_matrices=False)
    homography = v[np.argmin(s)].reshape(3, 3)
    # Divide by H[2][2] for correct scaling? Not sure why this worked
    homography = homography/homography[2][2]

    return homography


def construct_A_i_matrix(x, x_tick):
    zeros = np.zeros(3)
    # First row of A_i [0  -w'x  y'x]
    A_i_0 = zeros
    A_i_0 = np.append(A_i_0, - x_tick[2]*x)
    A_i_0 = np.append(A_i_0,   x_tick[1]*x)
    A_i_0 = A_i_0.reshape(1, len(A_i_0))
    # Second row of A_i [w'x  0  -x'x]
    A_i_1 = x_tick[2]*x
    A_i_1 = np.append(A_i_1,   zeros)
    A_i_1 = np.append(A_i_1, - x_tick[0]*x)
    A_i_1 = A_i_1.reshape(1, len(A_i_1))

    return np.append(A_i_0, A_i_1, axis=0)
