#!/usr/bin/env python

import numpy as np


def normalize(x):
    """
    Normalize a matrix of 2D coordinates to have mean at the origin and average
    distance from origin of sqrt(2)
    """
    # Calculate centroid of points and translate points to origin
    centroid = np.mean(x, axis=0)
    x_normalized = x - centroid
    # Calculate the average distance to origin
    scale_factor = np.mean(np.sqrt(np.sum(x_normalized**2, axis=1)))
    # Scale factor to achieve average distance sqrt(2)
    scale_factor = np.sqrt(2) / scale_factor
    x_normalized = x_normalized * scale_factor
    x_normalized = make_homogeneous(x_normalized)
    # Similarity transform matrix, no rotation, in the form:
    # | s 0 t_x |
    # | 0 s t_y |
    # | 0 0   1 |
    similarity_transform = np.array([[scale_factor, 0, - scale_factor * centroid[0]],
                                     [0, scale_factor, - scale_factor * centroid[1]],
                                     [0, 0, 1]])

    return similarity_transform, x_normalized


def make_homogeneous(x):
    return np.append(x, np.ones((len(x), 1)), axis=1)
