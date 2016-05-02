import random
import numpy as np
from homography_estimation import normalized_dlt, util


class RANSAC:
    def __init__(self, error_threshold, p_outlier, p=0.99, sample_size_min=4):
        """
        Init function for RANSAC

        :param error_threshold:
        :param p_outlier: probability of observing an outlier in the samples
        :param p: probability that at least one of the sets of random samples does not include an outlier
        :param sample_size_min: minimum sample size
        """
        self._error_threshold = error_threshold
        self._p_outlier = p_outlier
        self._p = p
        self._sample_size_min = sample_size_min

    def get_homography(self, x, x_tick):
        """
        Perform RANSAC algorithm with DLT

        :param x:
        :param x_tick:
        """
        if x.shape != x_tick.shape:
            print("ransac: input matrices need to have the same dimensions.")
            return None
        if self._sample_size_min < 4:
            print("ransac: Need at least 4 samples for homography calculation.")
            return None

        iteration_num = self._calculate_iteration_num()
        sample_num = len(x)
        inlier_num_threshold = int((1 - self._p_outlier) * sample_num * 0.8)

        for i in range(iteration_num*2):
            # Pick and fit sample of minimum size
            random_indices = self._get_random_indices(sample_num)
            homography = normalized_dlt.get_homography(x[random_indices], x_tick[random_indices])
            # Find inliers and check if enough found
            inlier_indices = self._get_inlier_indices(x, x_tick, homography, sample_num)

            if len(inlier_indices) > inlier_num_threshold:
                return inlier_indices, normalized_dlt.get_homography(x[inlier_indices], x_tick[inlier_indices])

        print("ransac: failed to find sufficient inliers")
        return None, None

    def _get_inlier_indices(self, x, x_tick, homography, sample_num):
        """Returns indices of inliers according to the homography model"""
        inlier_indices = []
        for j in range(sample_num):
            if (util.calculate_transfer_error_symmetric(x[j], x_tick[j], homography)) < self._error_threshold:
                inlier_indices.append(j)
        return inlier_indices

    def _calculate_iteration_num(self):
        """
        N = log10(1-p)/log10(1 - (1 - p_outlier)^sample_size_min)
        """
        iteration_num = np.log10(1 - self._p) / np.log10(1 - np.power(1 - self._p_outlier, self._sample_size_min))

        return int(iteration_num)

    def _get_random_indices(self, total_num_sample):
        if self._sample_size_min > total_num_sample:
            return None

        random.seed()
        random_indices = []
        for i in range(self._sample_size_min):
            random_indices.append(random.randint(0, total_num_sample - 1))

        return random_indices
