#!/usr/bin/env python3
import rospy
import tf2_ros

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from waypoint_manager_msgs.msg import Waypoints


class JoyWaypointRecorder:
    def __init__(self):
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")

        self.regist_pose_topic = rospy.get_param(
            "~regist_pose_topic",
            "/waypoint_manager/waypoint/regist_pose"
        )

        self.waypoints_topic = rospy.get_param(
            "~waypoints_topic",
            "/waypoint_manager/waypoints"
        )

        self.append_route_topic = rospy.get_param(
            "~append_route_topic",
            "/waypoint_manager/route/append"
        )

        # waypoint と route の両方を保存するサービス
        self.save_service_name = rospy.get_param(
            "~save_service",
            "/waypoint_manager/waypoint_server/save"
        )

        # 自律移動制御用サービス
        self.reset_route_service_name = rospy.get_param(
            "~reset_route_service",
            "/waypoint_manager/waypoint_server/reset_route"
        )

        self.switch_cancel_service_name = rospy.get_param(
            "~switch_cancel_service",
            "/waypoint_manager/waypoint_server/switch_cancel"
        )

        # joyボタン割り当て
        self.register_button = rospy.get_param("~register_button", 0)
        self.save_button = rospy.get_param("~save_button", 1)
        self.start_button = rospy.get_param("~start_button", 2)
        self.pause_button = rospy.get_param("~pause_button", 3)

        self.auto_save = rospy.get_param("~auto_save", False)

        self.cooldown_sec = rospy.get_param("~cooldown_sec", 1.0)
        self.wait_new_id_timeout = rospy.get_param("~wait_new_id_timeout", 3.0)

        self.last_register_time = rospy.Time(0)
        self.prev_buttons = []

        self.latest_waypoint_ids = set()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_pub = rospy.Publisher(
            self.regist_pose_topic,
            PoseStamped,
            queue_size=1
        )

        self.route_append_pub = rospy.Publisher(
            self.append_route_topic,
            String,
            queue_size=1
        )

        rospy.Subscriber(
            self.waypoints_topic,
            Waypoints,
            self.waypoints_callback,
            queue_size=1
        )

        rospy.Subscriber(
            self.joy_topic,
            Joy,
            self.joy_callback,
            queue_size=1
        )

        rospy.loginfo("joy_waypoint_recorder started")
        rospy.loginfo("register_button: %d", self.register_button)
        rospy.loginfo("save_button: %d", self.save_button)
        rospy.loginfo("start_button: %d", self.start_button)
        rospy.loginfo("pause_button: %d", self.pause_button)
        rospy.loginfo("auto_save: %s", str(self.auto_save))
        rospy.loginfo("regist_pose_topic: %s", self.regist_pose_topic)
        rospy.loginfo("append_route_topic: %s", self.append_route_topic)
        rospy.loginfo("save_service: %s", self.save_service_name)
        rospy.loginfo("reset_route_service: %s", self.reset_route_service_name)
        rospy.loginfo("switch_cancel_service: %s", self.switch_cancel_service_name)

    def waypoints_callback(self, msg):
        self.latest_waypoint_ids = set([wp.identity for wp in msg.waypoints])

    def is_pressed_edge(self, buttons, index):
        if index < 0:
            return False

        if index >= len(buttons):
            return False

        now_pressed = buttons[index] == 1

        if index >= len(self.prev_buttons):
            prev_pressed = False
        else:
            prev_pressed = self.prev_buttons[index] == 1

        return now_pressed and not prev_pressed

    def joy_callback(self, msg):
        if self.is_pressed_edge(msg.buttons, self.register_button):
            rospy.loginfo("REGISTER button pressed: index=%d", self.register_button)
            self.register_current_pose_and_append_route()

        if self.is_pressed_edge(msg.buttons, self.save_button):
            rospy.loginfo("SAVE button pressed: index=%d", self.save_button)
            self.save_all()

        if self.is_pressed_edge(msg.buttons, self.start_button):
            rospy.loginfo("START button pressed: index=%d", self.start_button)
            self.start_navigation()

        if self.is_pressed_edge(msg.buttons, self.pause_button):
            rospy.loginfo("PAUSE/RESUME button pressed: index=%d", self.pause_button)
            self.toggle_pause()

        self.prev_buttons = list(msg.buttons)

    def register_current_pose_and_append_route(self):
        now = rospy.Time.now()

        if (now - self.last_register_time).to_sec() < self.cooldown_sec:
            rospy.logwarn("Ignored register button because cooldown is active")
            return

        before_ids = set(self.latest_waypoint_ids)

        pose = self.get_current_pose()
        if pose is None:
            return

        self.wait_for_publisher_connection(
            self.pose_pub,
            "regist_pose_topic",
            timeout_sec=2.0
        )

        self.pose_pub.publish(pose)

        rospy.loginfo(
            "Published regist_pose: x=%.3f, y=%.3f, z=%.3f",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        )

        new_id = self.wait_for_new_waypoint_id(before_ids)

        if new_id is None:
            rospy.logerr("Failed to detect new waypoint ID. Route was not appended.")
            return

        self.append_route(new_id)

        self.last_register_time = now

        if self.auto_save:
            self.save_all()

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logerr(
                "Failed to get TF %s -> %s: %s",
                self.map_frame,
                self.robot_frame,
                str(e)
            )
            return None

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame

        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation

        return pose

    def wait_for_new_waypoint_id(self, before_ids):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            current_ids = set(self.latest_waypoint_ids)
            new_ids = list(current_ids - before_ids)

            if len(new_ids) > 0:
                new_id = sorted(new_ids)[-1]
                rospy.loginfo("Detected new waypoint ID: %s", new_id)
                return new_id

            if (rospy.Time.now() - start_time).to_sec() > self.wait_new_id_timeout:
                return None

            rate.sleep()

    def append_route(self, waypoint_id):
        msg = String()
        msg.data = waypoint_id

        self.wait_for_publisher_connection(
            self.route_append_pub,
            "append_route_topic",
            timeout_sec=2.0
        )

        self.route_append_pub.publish(msg)
        rospy.loginfo("Appended waypoint to route: %s", waypoint_id)

    def wait_for_publisher_connection(self, publisher, label, timeout_sec=2.0):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logwarn("No subscriber for %s yet", label)
                break
            rate.sleep()

    def call_trigger_service(self, service_name, label):
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            srv = rospy.ServiceProxy(service_name, Trigger)
            res = srv()

            # TriggerResponseを返す場合は中身を表示
            if hasattr(res, "success") and hasattr(res, "message"):
                rospy.loginfo(
                    "Called %s: %s, success=%s, message=%s",
                    label,
                    service_name,
                    str(res.success),
                    res.message
                )
            else:
                rospy.loginfo("Called %s: %s", label, service_name)

            return True

        except Exception as e:
            rospy.logerr(
                "Failed to call %s %s: %s",
                label,
                service_name,
                str(e)
            )
            return False

    def save_all(self):
        # /waypoint_manager/waypoint_server/save は waypoint と route の両方を保存する
        self.call_trigger_service(
            self.save_service_name,
            "save waypoints and route"
        )

    def start_navigation(self):
        # routeを先頭に戻す
        reset_ok = self.call_trigger_service(
            self.reset_route_service_name,
            "reset_route"
        )

        if not reset_ok:
            rospy.logerr("Start navigation canceled because reset_route failed.")
            return

        # waypoint_server側のcancel状態を切り替えて走行開始
        # 注意: switch_cancel はトグルなので、走行中に押すと停止側に切り替わる可能性がある
        self.call_trigger_service(
            self.switch_cancel_service_name,
            "switch_cancel start"
        )

    def toggle_pause(self):
        # 一時停止/再開をトグル
        self.call_trigger_service(
            self.switch_cancel_service_name,
            "switch_cancel pause/resume"
        )


if __name__ == "__main__":
    rospy.init_node("joy_waypoint_recorder")
    JoyWaypointRecorder()
    rospy.spin()


