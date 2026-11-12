#!/usr/bin/env python3
"""
ROS 1 Noetic Node for AprilTag Detection.
Subscribes to camera frames (raw or compressed), detects AprilTags,
publishes annotated image streams, PoseArray, TF transforms, and JSON summaries.
"""

import math
import json
import numpy as np
import cv2

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
import tf
import tf.transformations

# Import detector backend
HAS_PUPIL = False
try:
    from pupil_apriltags import Detector as PupilDetector
    HAS_PUPIL = True
except ImportError:
    HAS_PUPIL = False


class Ros1AprilTagNode:
    def __init__(self):
        rospy.init_node("apriltag_detector", anonymous=False)

        # Parameters
        self.tag_family = rospy.get_param("~tag_family", "tag36h11")
        self.tag_size = float(rospy.get_param("~tag_size", 0.10))
        self.camera_frame = rospy.get_param("~camera_frame", "camera_optical_frame")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        rospy.loginfo(f"[AprilTag] Initializing with family '{self.tag_family}', size {self.tag_size}m")

        # Initialize detector
        if HAS_PUPIL:
            self.detector = PupilDetector(
                families=self.tag_family,
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            self.backend = "pupil-apriltags"
        else:
            dict_map = {
                "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
                "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
                "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
            }
            d_id = dict_map.get(self.tag_family, cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(d_id)
            if hasattr(cv2.aruco, "DetectorParameters_create"):
                self.aruco_params = cv2.aruco.DetectorParameters_create()
            else:
                self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

            if hasattr(cv2.aruco, "ArucoDetector"):
                self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            else:
                self.detector = None
            self.backend = "cv2.aruco"

        rospy.loginfo(f"[AprilTag] Using backend: {self.backend}")

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        if self.publish_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()

        # Publishers
        self.pub_annotated = rospy.Publisher("/tag_detections_image", Image, queue_size=1)
        self.pub_annotated_comp = rospy.Publisher("/tag_detections_image/compressed", CompressedImage, queue_size=1)
        self.pub_poses = rospy.Publisher("/tag_poses", PoseArray, queue_size=5)
        self.pub_json = rospy.Publisher("/tag_detections_json", String, queue_size=5)

        # Subscribers
        self.sub_info = rospy.Subscriber("/camera/camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_raw = rospy.Subscriber("/camera/image_raw", Image, self.cb_raw_image, queue_size=1, buff_size=2**24)
        self.sub_comp = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, self.cb_compressed_image, queue_size=1, buff_size=2**24)

        rospy.loginfo("[AprilTag] Node started, waiting for camera images on /camera/image_raw or /camera/image_raw/compressed...")

    def cb_camera_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            if len(msg.D) >= 4:
                self.dist_coeffs = np.array(msg.D[:4], dtype=np.float64).reshape(-1, 1)
            rospy.loginfo(f"[AprilTag] Camera calibration received: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}")

    def cb_raw_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_frame(cv_image, msg.header.stamp)
        except Exception as e:
            rospy.logerr(f"Raw image processing error: {e}")

    def cb_compressed_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_frame(cv_image, msg.header.stamp)
        except Exception as e:
            rospy.logerr(f"Compressed image processing error: {e}")

    def draw_axes(self, img, rvec, tvec, length=0.05):
        pts = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]]).reshape(-1, 3)
        imgpts, _ = cv2.projectPoints(pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        origin = tuple(imgpts[0])
        cv2.line(img, origin, tuple(imgpts[1]), (0, 0, 255), 3)  # Red X
        cv2.line(img, origin, tuple(imgpts[2]), (0, 255, 0), 3)  # Green Y
        cv2.line(img, origin, tuple(imgpts[3]), (255, 100, 0), 3)  # Blue Z

    def process_frame(self, frame, stamp):
        h, w = frame.shape[:2]
        if self.camera_matrix is None:
            fl = w * 0.8
            self.camera_matrix = np.array([
                [fl, 0, w / 2.0],
                [0, fl, h / 2.0],
                [0, 0, 1]
            ], dtype=np.float64)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = []

        if self.backend == "pupil-apriltags":
            camera_params = (
                self.camera_matrix[0, 0],
                self.camera_matrix[1, 1],
                self.camera_matrix[0, 2],
                self.camera_matrix[1, 2]
            )
            raw_dets = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=self.tag_size
            )
            for d in raw_dets:
                R = d.pose_R
                rvec, _ = cv2.Rodrigues(R)
                tvec = d.pose_t.reshape(3, 1)
                detections.append({
                    "id": int(d.tag_id),
                    "center": d.center,
                    "corners": np.array(d.corners, dtype=np.float32),
                    "rvec": rvec,
                    "tvec": tvec,
                    "R": R
                })
        else:
            if hasattr(cv2.aruco, "ArucoDetector") and self.detector is not None:
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None:
                s = self.tag_size / 2.0
                obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
                for i, tag_id in enumerate(ids.flatten()):
                    tag_corners = corners[i][0]
                    ret, rvec, tvec = cv2.solvePnP(obj_pts, tag_corners, self.camera_matrix, self.dist_coeffs)
                    if ret:
                        R, _ = cv2.Rodrigues(rvec)
                        detections.append({
                            "id": int(tag_id),
                            "center": np.mean(tag_corners, axis=0),
                            "corners": tag_corners,
                            "rvec": rvec,
                            "tvec": tvec,
                            "R": R
                        })

        # Build ROS messages
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = stamp
        pose_array_msg.header.frame_id = self.camera_frame

        json_summary = []

        # Annotate image
        annotated = frame.copy()
        for d in detections:
            corners = d["corners"].astype(np.int32)
            cv2.polylines(annotated, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            for idx, pt in enumerate(corners):
                cv2.circle(annotated, tuple(pt), 4, (0, 0, 255) if idx == 0 else (255, 255, 0), -1)

            # Draw 3D axes
            self.draw_axes(annotated, d["rvec"], d["tvec"], length=self.tag_size * 0.75)

            # 4x4 transform matrix for quaternion conversion
            T = np.eye(4)
            T[:3, :3] = d["R"]
            quat = tf.transformations.quaternion_from_matrix(T)
            t = d["tvec"].flatten()

            # Broadcast TF
            if self.publish_tf:
                self.tf_broadcaster.sendTransform(
                    (t[0], t[1], t[2]),
                    quat,
                    stamp,
                    f"tag_{d['id']}",
                    self.camera_frame
                )

            # Pose message
            p = Pose()
            p.position = Point(x=t[0], y=t[1], z=t[2])
            p.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            pose_array_msg.poses.append(p)

            # Text label
            dist = float(np.linalg.norm(t))
            cx, cy = int(d["center"][0]), int(d["center"][1])
            label = f"ID:{d['id']} Dist:{dist:.2f}m"
            cv2.putText(annotated, label, (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(annotated, label, (cx - 40, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

            json_summary.append({
                "id": d["id"],
                "translation_xyz": [float(t[0]), float(t[1]), float(t[2])],
                "quaternion_xyzw": [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
                "distance_m": dist
            })

        # Draw HUD info
        cv2.putText(annotated, f"ROS 1 Noetic AprilTags: {len(detections)}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 200), 2)

        # Publish pose array & JSON
        self.pub_poses.publish(pose_array_msg)
        if json_summary:
            str_msg = String(data=json.dumps({"tags": json_summary}))
            self.pub_json.publish(str_msg)

        # Publish annotated images
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.camera_frame
            self.pub_annotated.publish(img_msg)

            comp_msg = CompressedImage()
            comp_msg.header.stamp = stamp
            comp_msg.header.frame_id = self.camera_frame
            comp_msg.format = "jpeg"
            _, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            comp_msg.data = encoded.tobytes()
            self.pub_annotated_comp.publish(comp_msg)
        except Exception as e:
            rospy.logerr(f"Publish image error: {e}")


def main():
    try:
        node = Ros1AprilTagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
