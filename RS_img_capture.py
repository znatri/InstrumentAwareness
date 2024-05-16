"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/16/24
"""

import numpy as np
import cv2
from calibrator import Calibrator
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False

for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break

if not found_rgb:
    print("No RGB sensor found")
    exit(0)

W = 640
H = 480
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
config = pipeline.start(config)
profile = config.get_stream(rs.stream.color)

calibrator = Calibrator(W, H)

i = 0
while True:
    frames = pipeline.wait_for_frames()
    frame = frames.get_color_frame()
    if not frame:
        continue
    frame = np.asanyarray(frame.get_data())
    frame = calibrator.calibrate(frame)
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('s'):
        print("img save")
        cv2.imwrite("ref.png", frame)
        i = i + 1

pipeline.stop()
cv2.destroyAllWindows()
