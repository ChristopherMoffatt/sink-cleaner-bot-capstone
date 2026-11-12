#!/usr/bin/env python3
"""Spin in place to find an AprilTag, then drive toward it and stop.

States:
  SEARCH   rotate in place until the target tag is seen; give up after ~1 turn.
  CONFIRM  hold still until the tag is seen several times in a row.
  APPROACH steer toward the tag and drive forward until stop_distance_m.
  DONE     reached the tag; robot stopped, node idles.
  FAILED   no tag after a full turn; robot stopped, node idles.

Tag data comes from the apriltag_tracking container on /tag_detections_json,
in camera_optical_frame (x right, y down, z forward). The camera faces forward
on the robot's centreline, so the robot steers straight from the camera
reading and no camera TF is needed.

Velocity goes to /cmd_vel, the lowest-priority twist_mux input, so holding L1
on the PS4 controller always overrides it. LiDAR guard: no rotation if
anything is inside the spin circle, no forward motion if anything is in the
corridor just ahead. The node stops and exits on /e_stop True, and stops if
/e_stop goes quiet (e-stop node not running).
"""
import json
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

SEARCH, CONFIRM, APPROACH, DONE, FAILED = "SEARCH", "CONFIRM", "APPROACH", "DONE", "FAILED"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LaserGuard:
    """Keeps the latest scan of one LiDAR as (x, y) points in base_link."""

    def __init__(self, topic, frame, tf_buffer):
        self.frame = frame
        self.tf_buffer = tf_buffer
        self.offset = None  # (x, y, yaw) of the laser in base_link
        self.points = []
        self.stamp = None
        rospy.Subscriber(topic, LaserScan, self.cb, queue_size=1)

    def cb(self, scan):
        if self.offset is None:
            try:
                t = self.tf_buffer.lookup_transform("base_link", self.frame, rospy.Time(0),
                                                    rospy.Duration(0.5)).transform
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                return
            self.offset = (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))
        ox, oy, oyaw = self.offset
        pts = []
        a = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max and math.isfinite(r) and r > 0.05:
                ang = oyaw + a
                pts.append((ox + r * math.cos(ang), oy + r * math.sin(ang)))
            a += scan.angle_increment
        self.points = pts
        self.stamp = rospy.Time.now()


class TagApproachNode:
    def __init__(self):
        rospy.init_node("tag_approach")
        p = rospy.get_param
        self.target_id = int(p("~target_id", 0))              # -1 = any tag
        # The detector computes distance from the tag size it assumes. Distance
        # scales linearly with tag size, so rescale to the real printed size.
        tag_size = float(p("~tag_size_m", 0.2032))
        detector_tag_size = float(p("~detector_tag_size_m", 0.10))
        self.tag_scale = tag_size / detector_tag_size
        self.stop_distance = float(p("~stop_distance_m", 0.6))
        self.distance_tol = float(p("~distance_tolerance_m", 0.03))
        self.bearing_tol = float(p("~bearing_tolerance_rad", 0.05))
        self.search_speed = float(p("~search_angular_speed", 0.25))
        self.search_turns = float(p("~search_max_turns", 1.1))
        self.confirm_count = int(p("~confirm_detections", 3))
        self.confirm_timeout = float(p("~confirm_timeout_s", 1.5))
        self.tag_lost_timeout = float(p("~tag_lost_timeout_s", 1.0))
        self.max_linear = float(p("~max_linear_speed", 0.15))
        self.min_linear = float(p("~min_linear_speed", 0.03))
        self.max_angular = float(p("~max_angular_speed", 0.4))
        self.k_linear = float(p("~k_linear", 0.5))
        self.k_angular = float(p("~k_angular", 0.8))
        self.turn_first_bearing = float(p("~turn_first_bearing_rad", 0.35))
        self.half_length = float(p("~robot_half_length_m", 0.48))
        self.half_width = float(p("~robot_half_width_m", 0.40))
        self.front_clearance = float(p("~front_clearance_m", 0.25))
        self.spin_clearance = float(p("~spin_clearance_m", 0.10))
        self.scan_timeout = float(p("~scan_timeout_s", 0.5))
        self.estop_timeout = float(p("~estop_heartbeat_timeout_s", 0.5))
        cmd_topic = p("~cmd_vel_topic", "/cmd_vel")

        self.state = SEARCH
        self.tag = None            # (x, z) in camera_optical_frame
        self.tag_time = None
        self.tag_seq = 0           # count of target detections received
        self.confirm_start_seq = 0
        self.state_since = rospy.Time.now()
        self.yaw = None
        self.last_yaw = None
        self.turned = 0.0
        self.estop_time = None
        self.estop_seen_true = False
        self.blocked_reason = ""
        self.blocked_logged = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(tf_buffer)
        self.lasers = [LaserGuard("/front/scan", "front_laser", tf_buffer),
                       LaserGuard("/rear/scan", "rear_laser", tf_buffer)]
        rospy.Subscriber("/tag_detections_json", String, self.tag_cb, queue_size=1)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/e_stop", Bool, self.estop_cb, queue_size=1)
        rospy.on_shutdown(self.stop)

        rospy.loginfo("Tag approach: target id %s, stop at %.2f m, publishing to %s",
                      "any" if self.target_id < 0 else self.target_id, self.stop_distance, cmd_topic)
        rospy.loginfo("Tag size %.4f m, detector assumes %.4f m: scaling distances by %.3f",
                      tag_size, detector_tag_size, self.tag_scale)

    # ---------- callbacks ----------
    def tag_cb(self, msg):
        try:
            tags = json.loads(msg.data).get("tags", [])
        except ValueError:
            return
        best = None
        for t in tags:
            if self.target_id >= 0 and int(t.get("id", -1)) != self.target_id:
                continue
            x, _, z = [v * self.tag_scale for v in t["translation_xyz"]]
            if z <= 0:
                continue
            if best is None or z < best[1]:
                best = (x, z)
        if best is not None:
            self.tag = best
            self.tag_time = rospy.Time.now()
            self.tag_seq += 1

    def odom_cb(self, msg):
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def estop_cb(self, msg):
        self.estop_time = rospy.Time.now()
        if msg.data and not self.estop_seen_true:
            self.estop_seen_true = True
            self.stop()
            rospy.logerr("E-stop is latched; tag approach shutting down.")
            rospy.signal_shutdown("e-stop")

    # ---------- helpers ----------
    def stop(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, s, note=""):
        if s != self.state:
            rospy.loginfo("State %s -> %s %s", self.state, s, note)
            self.state = s
            self.state_since = rospy.Time.now()
            self.state_pub.publish(String(data=s))

    def tag_fresh(self):
        return (self.tag_time is not None and
                (rospy.Time.now() - self.tag_time).to_sec() < self.tag_lost_timeout)

    def scans_fresh(self):
        now = rospy.Time.now()
        return all(l.stamp is not None and (now - l.stamp).to_sec() < self.scan_timeout
                   for l in self.lasers)

    def spin_blocked(self):
        """True if anything is inside the circle the footprint sweeps when spinning."""
        radius = math.hypot(self.half_length, self.half_width) + self.spin_clearance
        for l in self.lasers:
            for x, y in l.points:
                if abs(x) <= self.half_length and abs(y) <= self.half_width:
                    continue  # point on the robot itself
                if math.hypot(x, y) < radius:
                    return True, math.hypot(x, y)
        return False, None

    def front_blocked(self):
        """True if anything is in the corridor just ahead of the front bumper."""
        y_lim = self.half_width + 0.05
        x_lim = self.half_length + self.front_clearance
        for x, y in self.lasers[0].points:
            if self.half_length < x < x_lim and abs(y) < y_lim:
                return True, x - self.half_length
        return False, None

    def warn_blocked(self, reason):
        # Log when a hold starts, then at most every 3 s while it lasts.
        now = rospy.Time.now()
        if not self.blocked_reason or (now - self.blocked_logged).to_sec() > 3.0:
            rospy.logwarn("Holding still: %s", reason)
            self.blocked_logged = now
        self.blocked_reason = reason

    # ---------- main loop ----------
    def step(self):
        now = rospy.Time.now()
        if self.estop_time is None or (now - self.estop_time).to_sec() > self.estop_timeout:
            self.stop()
            rospy.logwarn_throttle(2.0, "No /e_stop heartbeat. Is drain_safety estop.launch running?")
            return
        if not self.scans_fresh():
            self.stop()
            rospy.logwarn_throttle(2.0, "LiDAR scans are stale or missing; holding still.")
            return

        cmd = Twist()
        if self.state == SEARCH:
            if self.tag_fresh():
                self.stop()
                self.confirm_start_seq = self.tag_seq
                self.set_state(CONFIRM)
                return
            blocked, d = self.spin_blocked()
            if blocked:
                self.warn_blocked("obstacle %.2f m from centre, too close to spin" % d)
                self.stop()
                return
            self.blocked_reason = ""
            if self.yaw is not None:
                if self.last_yaw is not None:
                    dy = math.atan2(math.sin(self.yaw - self.last_yaw), math.cos(self.yaw - self.last_yaw))
                    self.turned += abs(dy)
                self.last_yaw = self.yaw
            if self.turned > self.search_turns * 2.0 * math.pi:
                self.stop()
                self.set_state(FAILED, "(no tag after %.1f turns)" % self.search_turns)
                return
            cmd.angular.z = self.search_speed

        elif self.state == CONFIRM:
            if self.tag_seq - self.confirm_start_seq >= self.confirm_count:
                self.set_state(APPROACH, "(tag at x=%.2f z=%.2f)" % self.tag)
            elif (now - self.state_since).to_sec() > self.confirm_timeout:
                self.last_yaw = None
                self.set_state(SEARCH, "(tag not confirmed)")
            self.stop()
            return

        elif self.state == APPROACH:
            if not self.tag_fresh():
                self.stop()
                self.turned = 0.0
                self.last_yaw = None
                self.set_state(SEARCH, "(tag lost)")
                return
            x, z = self.tag
            bearing = math.atan2(x, z)    # positive = tag to the right
            dist_err = z - self.stop_distance
            if dist_err <= self.distance_tol and abs(bearing) <= self.bearing_tol:
                self.stop()
                self.set_state(DONE, "(tag %.2f m ahead)" % z)
                return
            cmd.angular.z = clamp(-self.k_angular * bearing, -self.max_angular, self.max_angular)
            if dist_err > self.distance_tol:
                # Turn toward the tag first; only drive forward once it is roughly centred.
                scale = max(0.0, 1.0 - abs(bearing) / self.turn_first_bearing)
                speed = clamp(self.k_linear * dist_err, self.min_linear, self.max_linear)
                cmd.linear.x = speed * scale
            if cmd.linear.x > 0.0:
                blocked, d = self.front_blocked()
                if blocked:
                    self.warn_blocked("obstacle %.2f m ahead of the bumper" % d)
                    cmd.linear.x = 0.0
                else:
                    self.blocked_reason = ""
            if abs(cmd.angular.z) > 0.0:
                blocked, d = self.spin_blocked()
                if blocked and cmd.linear.x == 0.0:
                    # Turning in place near an obstacle could clip a corner.
                    self.warn_blocked("obstacle %.2f m from centre, too close to turn" % d)
                    cmd.angular.z = 0.0

        else:  # DONE or FAILED: stay stopped
            return

        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(20)
        self.state_pub.publish(String(data=self.state))
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    TagApproachNode().run()
