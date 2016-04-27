#!/usr/bin/env python3

import matplotlib.pyplot as plt
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
    """Append 1's to a 2D array to make homogeneous"""
    return np.append(x, np.ones((len(x), 1)), axis=1)


def transform_image(image, homography):
    """
    Transform an image using a given homography

    :param image: original image to be transformed
    :param homography: homography matrix of the transformation
    :return: transformed image
    """
    x_range, y_range = image.shape
    image_reconstruct = np.zeros((x_range, y_range))

    for x in range(x_range):
        for y in range(y_range):
            x_transformed, y_transformed, w = np.dot(homography, [x, y, 1.])
            x_transformed = int(np.round(x_transformed))
            y_transformed = int(np.round(y_transformed))
            if x_transformed < x_range and y_transformed < y_range:
                image_reconstruct[x_transformed, y_transformed] = image[x, y]

    return image_reconstruct


def plot_lines_on_image(img, x, x_tick):
    """Draw lines (2D) from coordinates in x to coordinates in x_tick"""
    plt.imshow(img, cmap='gray')
    for i in range(len(x)):
        plt.plot(x[i], x_tick[i], color='r', linestyle='-', linewidth=1)


def euclidean_squared(x, y):
    """Squared euclidean distance between 2 1D arrays"""
    return np.sum(np.subtract(x, y)**2)
