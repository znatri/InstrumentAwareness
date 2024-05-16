"""
Author: Raghavasimhan Sankaranarayanan
Date Created: 5/7/24
"""

import pyrealsense2 as rs
import numpy as np


class RealSenseCamera:
    def __init__(self, width, height, calibrator=None):
        self.calibrator = calibrator
        self.pipeline = rs.pipeline()
        config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False

        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break

        if not found_rgb:
            Exception("No RGB sensor found")

        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        config = self.pipeline.start(config)
        profile = config.get_stream(rs.stream.color)

        for i in range(30):
            frames = self.pipeline.wait_for_frames()
            frame = frames.get_color_frame()
            if not frame:
                continue

    def read_frame(self, undistort=True):
        frames = self.pipeline.wait_for_frames()
        frame = frames.get_color_frame()
        if frame is None:
            return None
        frame = np.asanyarray(frame.get_data())
        if undistort and self.calibrator is not None:
            frame = self.calibrator.calibrate(frame)
        return frame

    def stop(self):
        self.pipeline.stop()
