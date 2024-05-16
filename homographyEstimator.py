"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/7/24
"""

import numpy as np
import cv2


class HomographyEstimator:
    def __init__(self, ref_img, mask=None):
        self.mask = mask
        self.sift = cv2.SIFT_create()
        ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
        self.ref_kp, self.ref_desc = self.sift.detectAndCompute(ref_gray, mask=self.mask)

        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.frame_kp = None
        self.frame_desc = None
        self.points = np.array([])

    def get_points(self):
        return self.points

    def estimate(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_kp, self.frame_desc = self.sift.detectAndCompute(frame, mask=self.mask)
        matches = self.flann.knnMatch(self.ref_desc, self.frame_desc, k=2)
        img_1 = cv2.drawKeypoints(frame_gray, self.frame_kp, frame_gray)
        # cv2.imshow("kp", img_1)
        good = []
        for m, n in matches:
            if m.distance < 0.8 * n.distance:
                good.append(m)

        query_pts = np.float32([self.ref_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        self.points = np.float32([self.frame_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        matrix = None
        if len(self.points) > 4:
            matrix, mask = cv2.findHomography(query_pts, self.points, cv2.RANSAC, 5.0)
        return matrix
