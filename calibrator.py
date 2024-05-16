import numpy as np
import cv2


class Calibrator:
    def __init__(self, width, height, realsense=True):
        self.w = width
        self.h = height

        if realsense:
            fx = 379.344
            fy = 378.837
            cx = 316.72
            cy = 248.074
            self.distortion_coeff = np.array([-0.0558231, 0.0665041, 0.000208511, -0.000802755, -0.0209751])
        else:
            if width == 640 and height == 360:
                # For 640 x 360
                fx = 362.52653535
                fy = 362.25998752
                cx = 318.37194831
                cy = 164.16965647
                k1 = -3.63861378e-01
                k2 = 1.90949424e-01
                k3 = -6.15328607e-02
                p1 = -2.30130885e-04
                p2 = 1.81061807e-04
                self.distortion_coeff = np.array([k1, k2, p1, p2, k3])
            elif width == 1280 and height == 720:
                # For 1280 x 720
                fx = 725.1829323
                fy = 724.62218134
                cx = 636.48541667
                cy = 330.34250723
                k1 = -3.63674122e-01
                k2 = 1.90752803e-01
                k3 = -6.13239547e-02
                p1 = -3.78403414e-04
                p2 = 2.98197866e-04
                self.distortion_coeff = np.array([k1, k2, p1, p2, k3])
            else:
                raise Exception("No calibration parameters for the resolution found")

        self.calibration_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

        self.center = (cx, cy)
        self.f = (fx, fy)

    def calibrate(self, img):
        return cv2.undistort(img, self.calibration_matrix, self.distortion_coeff)
