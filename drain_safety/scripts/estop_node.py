#!/usr/bin/env python3
"""PS4 software e-stop for the Ridgeback.

Pressing the e-stop button (circle) latches the stop:
  * publishes True on /e_stop, which is a priority-255 lock in twist_mux.
    This blocks every drive command, including the PS4 sticks.
  * stops the Ewellix lift.
  * shuts down the autonomy nodes listed in ~kill_nodes.

Holding the reset buttons (Share + Options) for ~reset_hold_s releases it.
Autonomy is not restarted; relaunch it by hand.

The stop also latches if the controller stops publishing for ~joy_timeout_s
(for example, a Bluetooth dropout), because the manual override is gone then.

/e_stop is published at 10 Hz whether latched or not, so autonomy nodes can
treat it as a heartbeat and halt if this node dies. If this node dies while
latched, twist_mux keeps the lock (its timeout is 0), so it fails safe.

This is a software stop. It does not replace the robot's physical e-stop.
"""
import rospy
import rosnode
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty


class EStopNode:
    def __init__(self):
        rospy.init_node("estop")
        self.joy_topic = rospy.get_param("~joy_topic", "/bluetooth_teleop/joy")
        self.estop_button = int(rospy.get_param("~estop_button", 1))          # PS4 circle
        self.reset_buttons = [int(b) for b in rospy.get_param("~reset_buttons", [6, 7])]  # Share + Options
        self.reset_hold_s = float(rospy.get_param("~reset_hold_s", 1.0))
        self.joy_timeout_s = float(rospy.get_param("~joy_timeout_s", 1.0))
        self.kill_nodes = list(rospy.get_param("~kill_nodes", ["/tag_approach"]))

        self.latched = False
        self.reason = ""
        self.last_joy_time = None
        self.reset_pressed_since = None

        self.estop_pub = rospy.Publisher("/e_stop", Bool, queue_size=1, latch=True)
        self.lift_stop_pub = rospy.Publisher("/ewellix_tlt/actions/stop", Empty, queue_size=1)
        rospy.Subscriber(self.joy_topic, Joy, self.joy_cb, queue_size=1)

        rospy.loginfo("E-stop ready: button %d latches, buttons %s held %.1f s reset. Watching %s.",
                      self.estop_button, self.reset_buttons, self.reset_hold_s, self.joy_topic)

    def joy_cb(self, msg):
        self.last_joy_time = rospy.Time.now()
        buttons = msg.buttons
        needed = [self.estop_button] + self.reset_buttons
        if max(needed) >= len(buttons):
            rospy.logwarn_throttle(5.0, "Joy message has only %d buttons; check the button indices.",
                                   len(buttons))
            return

        if buttons[self.estop_button]:
            self.reset_pressed_since = None
            if not self.latched:
                self.engage("e-stop button pressed")
            return

        if self.latched and all(buttons[b] for b in self.reset_buttons):
            if self.reset_pressed_since is None:
                self.reset_pressed_since = rospy.Time.now()
            elif (rospy.Time.now() - self.reset_pressed_since).to_sec() >= self.reset_hold_s:
                self.release()
        else:
            self.reset_pressed_since = None

    def engage(self, reason):
        self.latched = True
        self.reason = reason
        self.estop_pub.publish(Bool(data=True))
        self.lift_stop_pub.publish(Empty())
        rospy.logerr("E-STOP LATCHED (%s). All base motion is blocked. "
                     "Hold Share + Options to release.", reason)
        self.shutdown_autonomy()

    def shutdown_autonomy(self):
        try:
            running = set(rosnode.get_node_names())
        except rosnode.ROSNodeIOException as e:
            rospy.logerr("Could not list nodes to shut down: %s", e)
            return
        targets = [n for n in self.kill_nodes if n in running]
        if targets:
            ok, failed = rosnode.kill_nodes(targets)
            rospy.logwarn("Shut down autonomy nodes: %s%s", ok,
                          " (failed: %s)" % failed if failed else "")

    def release(self):
        self.latched = False
        self.reset_pressed_since = None
        self.estop_pub.publish(Bool(data=False))
        rospy.logwarn("E-stop released. Manual driving works again; relaunch autonomy to resume.")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            joy_stale = (self.last_joy_time is None or
                         (now - self.last_joy_time).to_sec() > self.joy_timeout_s)
            if joy_stale and not self.latched:
                self.engage("no PS4 controller messages on %s for %.1f s"
                            % (self.joy_topic, self.joy_timeout_s))
            if self.latched:
                self.lift_stop_pub.publish(Empty())
            self.estop_pub.publish(Bool(data=self.latched))
            rate.sleep()


if __name__ == "__main__":
    node = EStopNode()
    # Give the controller a moment to report before the dropout check can latch.
    rospy.sleep(node.joy_timeout_s)
    node.run()
