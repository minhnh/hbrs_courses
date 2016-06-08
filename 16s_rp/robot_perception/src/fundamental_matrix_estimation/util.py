#!/usr/bin/env python3
# Illustration from http://docs.opencv.org/3.1.0/da/de9/tutorial_py_epipolar_geometry.html#gsc.tab=0
import cv2
import numpy as np
from matplotlib import pyplot as plt


def drawlines(img1, img2, lines, pts1, pts2):
    '''
    img1 - image on which we draw the epilines for the points in img2
    lines - corresponding epilines '''
    r, c = img1.shape
    img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    pts1 = pts1.astype(int)
    pts2 = pts2.astype(int)
    for r, pt1, pt2 in zip(lines, pts1, pts2):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        x0, y0 = map(int, [0, -r[2]/r[1]])
        x1, y1 = map(int, [c, -(r[2]+r[0]*c)/r[1]])
        img1 = cv2.line(img1, (x0,y0), (x1, y1), color, 1)
        img1 = cv2.circle(img1, tuple(pt1), 5, color, -1)
        img2 = cv2.circle(img2, tuple(pt2), 5, color, -1)
    return img1, img2


def draw_epilines(img1, img2, points_img1, points_img2, fundamental_matrix):
    # Find epilines corresponding to points in right image (second image) and
    # drawing its lines on left image
    lines1 = cv2.computeCorrespondEpilines(points_img2.reshape(-1, 1, 2), 2, fundamental_matrix)
    lines1 = lines1.reshape(-1, 3)
    img5, img6 = drawlines(img1, img2, lines1, points_img1, points_img2)

    # Find epilines corresponding to points in left image (first image) and
    # drawing its lines on right image
    lines2 = cv2.computeCorrespondEpilines(points_img1.reshape(-1, 1, 2), 1, fundamental_matrix)
    lines2 = lines2.reshape(-1, 3)
    img3, img4 = drawlines(img2, img1, lines2, points_img2, points_img1)

    plt.figure(figsize=(20, 20))
    plt.subplot(211), plt.imshow(img5)
    plt.subplot(212), plt.imshow(img3)
    plt.show()
