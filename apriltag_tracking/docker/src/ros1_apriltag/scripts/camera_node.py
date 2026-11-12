#!/usr/bin/env python3
"""
ROS 1 Noetic camera publisher.
Opens the first external (USB) V4L2 camera it finds and publishes:
  - /camera/image_raw/compressed (sensor_msgs/CompressedImage)
  - /camera/camera_info          (sensor_msgs/CameraInfo)

If no camera is present, or the camera is unplugged, it keeps rescanning
until one shows up.
"""

import glob
import os
import re

import cv2
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo


def list_video_indices():
    """All /dev/videoN indices, lowest first."""
    indices = []
    for path in glob.glob("/dev/video*"):
        m = re.fullmatch(r"/dev/video(\d+)", path)
        if m:
            indices.append(int(m.group(1)))
    return sorted(indices)


def is_external(index):
    """True if the video device hangs off the USB bus."""
    sysfs = os.path.realpath(f"/sys/class/video4linux/video{index}/device")
    return "/usb" in sysfs


def device_name(index):
    try:
        with open(f"/sys/class/video4linux/video{index}/name") as f:
            return f.read().strip()
    except OSError:
        return "unknown"


class CameraNode:
    def __init__(self):
        rospy.init_node("camera")

        self.device = str(rospy.get_param("~device", "auto"))
        self.width = int(rospy.get_param("~width", 1280))
        self.height = int(rospy.get_param("~height", 720))
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.quality = int(rospy.get_param("~jpeg_quality", 75))
        self.frame_id = rospy.get_param("~frame_id", "camera_optical_frame")

        self.pub_image = rospy.Publisher("/camera/image_raw/compressed", CompressedImage, queue_size=1)
        self.pub_info = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1, latch=True)

    def try_open(self, index):
        """Open /dev/video<index>; return the capture only if it delivers frames."""
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        # MJPG lets USB webcams reach higher resolutions at full frame rate
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = cap.read()
        if not ok or frame is None:
            # e.g. the metadata-only node that UVC cameras expose alongside the real one
            cap.release()
            return None
        return cap

    def open_camera(self):
        """Return (capture, index) for the first usable external camera, or (None, None)."""
        if self.device != "auto":
            m = re.search(r"(\d+)$", self.device)
            candidates = [int(m.group(1))] if m else []
        else:
            candidates = [i for i in list_video_indices() if is_external(i)]

        for index in candidates:
            cap = self.try_open(index)
            if cap is not None:
                return cap, index
        return None, None

    def make_camera_info(self, width, height):
        # Approximate pinhole intrinsics (no calibration file)
        f = width * 0.8
        cx, cy = width / 2.0, height / 2.0
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def stream(self, cap, index):
        """Publish frames until the camera stops delivering them."""
        info = None
        last_info = rospy.Time(0)
        failures = 0
        jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]

        while not rospy.is_shutdown():
            ok, frame = cap.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 30:
                    rospy.logwarn(f"[Camera] Lost /dev/video{index}, rescanning...")
                    return
                rospy.sleep(0.01)
                continue
            failures = 0

            stamp = rospy.Time.now()
            if info is None:
                h, w = frame.shape[:2]
                info = self.make_camera_info(w, h)
                rospy.loginfo(f"[Camera] Streaming /dev/video{index} at {w}x{h} -> /camera/image_raw/compressed")

            ok, encoded = cv2.imencode(".jpg", frame, jpeg_params)
            if not ok:
                continue

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self.pub_image.publish(msg)

            if (stamp - last_info).to_sec() > 2.0:
                info.header.stamp = stamp
                self.pub_info.publish(info)
                last_info = stamp

    def run(self):
        while not rospy.is_shutdown():
            cap, index = self.open_camera()
            if cap is None:
                rospy.logwarn_throttle(10.0, "[Camera] No external camera found, waiting for one to be plugged in...")
                rospy.sleep(1.0)
                continue

            rospy.loginfo(f"[Camera] Using external camera /dev/video{index} ({device_name(index)})")
            try:
                self.stream(cap, index)
            finally:
                cap.release()
            rospy.sleep(1.0)


if __name__ == "__main__":
    try:
        CameraNode().run()
    except rospy.ROSInterruptException:
        pass
