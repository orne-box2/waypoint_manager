#!/usr/bin/python3
"""Record in memory; persist only through RViz's Waypoint Save.

There is no goal property in the existing server. A pause is an intermediate
stop=true point; a goal is the final stop=true point and ends this session.
Both remain editable with the existing RViz property editor.
"""

import copy
import math
import threading
import time
import uuid

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from waypoint_manager_msgs.msg import Property, Route, WaypointStamped, Waypoints


class JoyWaypointRecorder:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        # User-reported layout: down=0, right=1, left=2, up=3 (unused).
        # Verify physical positions on /joy; -1 disables an assignment.
        self.buttons = {
            "normal": rospy.get_param("~register_button", 0),
            "goal": rospy.get_param("~goal_button", 1),
            "pause": rospy.get_param("~pause_button", 2),
        }
        for kind, index in self.buttons.items():
            if type(index) is not int or index < -1:
                raise ValueError("%s button must be an integer >= -1" % kind)
        enabled = [index for index in self.buttons.values() if index >= 0]
        if len(enabled) != len(set(enabled)) or not enabled:
            raise ValueError("Assign at least one button, with distinct indices")
        self.cooldown_sec = self.read_seconds("cooldown_sec", 1.0)
        self.wait_timeout = self.read_seconds("wait_new_id_timeout", 5.0, positive=True)
        self.max_tf_age = self.read_seconds("max_tf_age", 1.0, positive=True)
        self.joy_timeout = self.read_seconds("joy_reconnect_timeout", 2.0)
        self.condition = threading.Condition()
        self.latest_waypoints = None
        self.latest_route = None
        self.waypoints_frame = None
        self.route_frame = None
        self.prev_buttons = None
        self.last_joy_time = None
        self.last_register_time = float("-inf")
        self.busy = False
        self.goal_id = None
        self.incomplete_id = None

        # Legacy settings must never re-enable saving or navigation controls.
        for name in ("auto_save", "save_button", "start_button", "save_service",
                     "reset_route_service", "switch_cancel_service"):
            if rospy.has_param("~" + name):
                rospy.logwarn("Ignoring legacy ~%s: recording only; save from RViz", name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.update_pub = rospy.Publisher(
            rospy.get_param("~update_waypoint_topic", "/waypoint_manager/waypoint/update"),
            WaypointStamped, queue_size=1)
        self.route_append_pub = rospy.Publisher(
            rospy.get_param("~append_route_topic", "/waypoint_manager/route/append"),
            String, queue_size=1)
        self.waypoints_sub = rospy.Subscriber(
            rospy.get_param("~waypoints_topic", "/waypoint_manager/waypoints"),
            Waypoints, self.waypoints_callback, queue_size=1)
        self.route_sub = rospy.Subscriber(
            rospy.get_param("~route_topic", "/waypoint_manager/route"),
            Route, self.route_callback, queue_size=1)
        self.joy_sub = rospy.Subscriber(
            rospy.get_param("~joy_topic", "/joy"), Joy, self.joy_callback, queue_size=10)
        rospy.loginfo("Recorder buttons: %s (-1: disabled)", self.buttons)
        for kind, index in self.buttons.items():
            if index == -1:
                rospy.logwarn("%s recording is disabled; assign its verified Joy button index", kind)
        rospy.loginfo("Goal = final stop point; pause = intermediate stop point. "
                      "Release buttons before recording. Save only with RViz Waypoint Save.")

    @staticmethod
    def read_seconds(name, default, positive=False):
        value = rospy.get_param("~" + name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0 or (positive and value == 0)):
            raise ValueError("Invalid ~%s duration" % name)
        return float(value)

    def waypoints_callback(self, msg):
        with self.condition:
            self.latest_waypoints = {wp.identity: wp for wp in msg.waypoints}
            self.waypoints_frame = msg.info.header.frame_id
            self.condition.notify_all()

    def route_callback(self, msg):
        with self.condition:
            self.latest_route = list(msg.identities)
            self.route_frame = msg.header.frame_id
            self.condition.notify_all()

    def joy_callback(self, msg):
        now = time.monotonic()
        buttons = list(msg.buttons)
        with self.condition:
            previous = self.prev_buttons
            reconnect = (self.last_joy_time is not None and self.joy_timeout > 0
                         and now - self.last_joy_time > self.joy_timeout)
            # Update edges even for rejected requests, before any registration.
            self.prev_buttons = buttons
            self.last_joy_time = now
            if any(index >= len(buttons) for index in self.buttons.values() if index >= 0):
                rospy.logwarn_throttle(5.0, "Recorder button index outside /joy buttons array")
                return
            if previous is None or len(previous) != len(buttons) or reconnect:
                rospy.loginfo("Joy initialized/reconnected; release and press to record")
                return
            pressed = [kind for kind, index in self.buttons.items()
                       if index >= 0 and buttons[index] == 1]
            edges = [kind for kind in pressed if previous[self.buttons[kind]] != 1]
            if not edges:
                return
            if len(pressed) != 1:
                rospy.logwarn("Multiple recording buttons pressed; release and choose one")
                return
            if self.goal_id is not None:
                rospy.logwarn("Goal %s completes this session. Edit/save in RViz; "
                              "restart recorder to begin another session", self.goal_id)
                return
            if self.incomplete_id is not None:
                rospy.logerr("Incomplete registration %s: inspect waypoint and route in RViz "
                             "before restarting recorder; no automatic retry", self.incomplete_id)
                return
            if self.busy or now - self.last_register_time < self.cooldown_sec:
                rospy.logwarn("Recorder busy or cooling down; release and press again")
                return
            if self.latest_waypoints is None or self.latest_route is None:
                rospy.logwarn("Waiting for the initial waypoint list and route")
                return
            if self.waypoints_frame != self.map_frame or self.route_frame != self.map_frame:
                rospy.logerr("Waypoint/route frame differs from ~map_frame; registration rejected")
                return
            if not self.connected():
                return
            # Nonblocking TF lookup captures the pose at button reception.
            # Wait for server acknowledgements in a worker so releases aren't lost.
            pose = self.get_current_pose()
            if pose is None:
                return
            self.busy = True
            self.last_register_time = now
            route_before = list(self.latest_route)
        threading.Thread(target=self.register_current_pose_and_append_route,
                         args=(edges[0], pose, route_before), daemon=True).start()

    def connected(self):
        if (self.update_pub.get_num_connections() != 1
                or self.route_append_pub.get_num_connections() != 1):
            rospy.logwarn("Recording needs exactly one waypoint server on update and route/append")
            return False
        return True

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn("Cannot record without current TF: %s", exc)
            return None
        age = (rospy.Time.now() - transform.header.stamp).to_sec()
        # Static TF has a zero stamp. Dynamic TF must be current.
        if transform.header.stamp != rospy.Time(0) and (age < 0 or age > self.max_tf_age):
            rospy.logwarn("Rejected stale/future TF (age %.3f s)", age)
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = transform.header.stamp
        translation = transform.transform.translation
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
        return pose

    def wait_for(self, predicate):
        deadline = time.monotonic() + self.wait_timeout
        with self.condition:
            while not rospy.is_shutdown():
                if predicate():
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(min(remaining, 0.1))
        return False

    def registered(self, waypoint_id, kind):
        waypoint = (self.latest_waypoints or {}).get(waypoint_id)
        if waypoint is None or self.waypoints_frame != self.map_frame:
            return False
        properties = {prop.name: prop.data for prop in waypoint.properties}
        return kind == "normal" or properties.get("stop") == "true"

    def register_current_pose_and_append_route(self, kind, pose, route_before):
        # Existing updateGoalPose creates missing IDs and merges properties.
        # A unique ID on this API avoids confusing concurrent RViz additions
        # with our own point; no new service or saved property is introduced.
        waypoint_id = "registed_joy_" + uuid.uuid4().hex
        request = WaypointStamped()
        request.header = copy.deepcopy(pose.header)
        request.waypoint.identity = waypoint_id
        request.waypoint.pose = copy.deepcopy(pose.pose)
        if kind in ("pause", "goal"):
            request.waypoint.properties = [Property(name="stop", data="true")]
        sent = False
        try:
            with self.condition:
                if not self.connected() or self.latest_route != route_before:
                    rospy.logwarn("Server/route changed before registration; press again")
                    return
                sent = True
                self.update_pub.publish(request)
            if not self.wait_for(lambda: self.registered(waypoint_id, kind)):
                raise RuntimeError("waypoint/property acknowledgement timed out")
            with self.condition:
                if not self.connected() or self.latest_route != route_before:
                    raise RuntimeError("server/route changed; waypoint exists but was not appended")
                self.route_append_pub.publish(String(data=waypoint_id))
            expected_route = route_before + [waypoint_id]
            if not self.wait_for(lambda: self.latest_route == expected_route
                                 and self.route_frame == self.map_frame):
                raise RuntimeError("route acknowledgement timed out or route was edited concurrently")
            with self.condition:
                if kind == "goal":
                    self.goal_id = waypoint_id
            rospy.loginfo("Recorded %s waypoint %s and confirmed route. Not saved; "
                          "use RViz Waypoint Save after editing", kind, waypoint_id)
        except Exception as exc:
            if sent:
                with self.condition:
                    self.incomplete_id = waypoint_id
            rospy.logerr("Registration of %s incomplete: %s. Inspect RViz before retrying; "
                         "nothing was saved", waypoint_id, exc)
        finally:
            with self.condition:
                self.busy = False


if __name__ == "__main__":
    rospy.init_node("joy_waypoint_recorder")
    try:
        JoyWaypointRecorder()
    except ValueError as exc:
        rospy.logfatal("Invalid recorder configuration: %s", exc)
        raise SystemExit(1)
    rospy.spin()
