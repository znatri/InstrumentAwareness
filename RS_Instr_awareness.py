"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/7/24
"""

import time
from realsense import RealSenseCamera
import numpy as np
import cv2
from multiprocessing import Process, Manager
from xarm.wrapper import XArmAPI
from calibrator import Calibrator
from homographyEstimator import HomographyEstimator

ref_img = cv2.imread("ref.png")
h_estimator = HomographyEstimator(ref_img)

# Mask Pick
mask = np.ones(ref_img.shape[:2], dtype="uint8") * 255
mask = cv2.rectangle(mask, (360, 170), (640, 360), color=(0, 0, 0), thickness=-1)
mask = cv2.rectangle(mask, (580, 0), (640, 480), color=(0, 0, 0), thickness=-1)
ref_img = cv2.bitwise_and(ref_img, ref_img, mask=mask)


def decompose_homography(h, A, A_inv):
    f = np.array([A[0, 0], A[1, 1]])
    c = np.array([A[0, 2], A[1, 2], -f.mean()])
    h = h.T
    h1 = h[0]
    h2 = h[1]
    h3 = h[2]
    _l = 1 / np.linalg.norm(np.dot(A_inv, h1))
    _t = _l * np.dot(A_inv, h3).flatten()
    _t += c
    _t /= 2
    # change scale
    _t[0] *= 10 / 9
    return _t


W = 640
H = 480
calibrator = Calibrator(W, H, True)
cam = RealSenseCamera(W, H, calibrator)

K = calibrator.calibration_matrix
K_inv = np.linalg.inv(K)

# P control
CALIB_TARGET = np.array((0, 0, 200))  # Const : set physically when ref img is shot
target = np.array((0, 0, 182))  # 182
kp = np.array([0.01, 0, 0.01])


def get_traj_pt(_i, freq=0.75, amp=25, phase=0):
    return amp * np.sin(2 * np.pi * freq * _i + phase) + 4


def move_arm(stop_arg, err_arg):
    arm = XArmAPI('192.168.1.204')
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    initial_pose = [-450, 230, 230, -90, 0, 180]
    pose = np.array(initial_pose, dtype=float)
    arm.set_position(*initial_pose, speed=100, mvacc=300, wait=True, is_radian=False)
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(1)
    init_time = time.time()
    while not stop_arg.value:
        now = time.time() - init_time
        P = kp * err_arg.value
        pose[0] += P[0]
        pose[1] += P[2]
        pose[2] = initial_pose[2] + get_traj_pt(now)
        # arm.set_servo_cartesian(pose)
        # print(now, pose)
        end_time = time.time() - now - init_time
        diff = np.maximum(0, 0.005 - end_time)
        time.sleep(diff)

    arm.set_mode(0)
    arm.set_state(0)
    arm.set_position(*initial_pose, speed=100, mvacc=300, wait=True, is_radian=False)


manager = Manager()
STOP = manager.Value('STOP', False)
err = manager.Value('err', np.array([0, 0, 0]))

# if __name__ == '__main__':
p = Process(target=move_arm, args=(STOP, err))
p.start()
while True:
    frame = cam.read_frame()
    if frame is None:
        continue
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    M = h_estimator.estimate(frame)
    if M is not None:
        gtr_pos = decompose_homography(M, K, K_inv)
        print(gtr_pos)
        err.value = target - (CALIB_TARGET + gtr_pos)
        M = np.linalg.inv(M)
        dst = cv2.warpPerspective(frame, M, (W, H))
        cv2.imshow("output", dst)

    if cv2.waitKey(1) == ord('q'):
        break

STOP.value = True
cv2.destroyAllWindows()
cam.stop()
p.join()
