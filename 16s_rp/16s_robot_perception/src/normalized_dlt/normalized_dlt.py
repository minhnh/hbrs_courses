#!/usr/bin/env python

import numpy as np


def dlt_normalized(x, x_tick):
    """
    Perform normalized DLT algorithm

    @param: homogeneous coordinates of similar points in 2 images, in 2D
            numpy array format
    @return: transform matrix
    """
    # Normalize x and x_tick
    (x_similar_transform, x_norm) = normalize(x)
    (x_tick_similar_transform, x_tick_norm) = normalize(x_tick)
    # DLT algorithm
    h_norm = dlt(x_norm, x_tick_norm)
    # Denormalize H_norm
    H = np.dot(np.dot(np.linalg.inv(x_tick_similar_transform), h_norm), x_similar_transform)
    return H


def dlt(x, x_tick):
    """
    Perform DLT algorithm
    """
    # Dimension check
    if x.shape != x_tick.shape:
        print("Input arrays must have same dimensions")
        return None

    # Construct matrix A in equation A.H = 0
    A = None
    zeros = np.zeros(3)
    for i in range(len(x)):
        # First row of A_i [0  -w'x  y'x]
        A_i_0 = zeros
        A_i_0 = np.append(A_i_0, - x_tick[i][2]*x[i])
        A_i_0 = np.append(A_i_0,   x_tick[i][1]*x[i])
        A_i_0 = A_i_0.reshape(1, len(A_i_0))
        # Second row of A_i [w'x  0  -x'x]
        A_i_1 = x_tick[i][2]*x[i]
        A_i_1 = np.append(A_i_1,   zeros)
        A_i_1 = np.append(A_i_1, - x_tick[i][0]*x[i])
        A_i_1 = A_i_1.reshape(1, len(A_i_1))
        # Append to A
        if A is None:
            A = A_i_0
        else:
            A = np.append(A, A_i_0, axis=0)
        A = np.append(A, A_i_1, axis=0)

    # SVD decomposition, H is the eigenvector corresponding to the
    # smallest singular value
    u, s, v = np.linalg.svd(A, full_matrices=False)
    H = v[np.argmin(s)].reshape(3, 3)
    # Divide by H[2][2] for correct scaling? Not sure why this worked
    H = H/H[2][2]

    return H


def normalize(x):
    # Calculate centroid of points and translate points to origin
    centroid = np.mean(x, axis=0)
    x_normalized = x - centroid
    # Calculate the average distance to origin
    scale_factor = np.mean(np.sqrt(np.sum(x_normalized**2, axis=1)))
    # Scale factor to achieve average distance sqrt(2)
    scale_factor = np.sqrt(2) / scale_factor
    x_normalized = x_normalized * scale_factor
    x_normalized = make_homog(x_normalized)
    # Similarity transform matrix, no rotation, in the form:
    # | s 0 t_x |
    # | 0 s t_y |
    # | 0 0   1 |
    similarity_transform = np.array([[scale_factor, 0, - scale_factor * centroid[0]],
                                     [0, scale_factor, - scale_factor * centroid[1]],
                                     [0, 0, 1]])

    return similarity_transform, x_normalized


def make_homog(x):
    return np.append(x, np.ones((len(x), 1)), axis=1)