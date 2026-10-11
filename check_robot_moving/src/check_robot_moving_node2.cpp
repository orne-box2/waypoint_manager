#include <limits>
#include <atomic>
#include <string>
#include <stdexcept>
#include <exception>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <time.h>

#include <std_msgs/Bool.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::Twist cmd_vel;

namespace {
    static std::atomic_bool recived_waypoint, stop_waypoint;
    static float default_goal_radius = 1;
    static float current_goal_radius = default_goal_radius;
    static Eigen::Vector2f current_position = Eigen::Vector2f::Zero();
    static Eigen::Vector2f old_current_position = Eigen::Vector2f::Zero();
    static time_t start_time, last_moving_time;
    static std::atomic_bool is_fst_flag, is_fst_waypoint_reached, is_reached_goal, is_to_prev_waypoint;
    static std::string old_id;
    static std::atomic_bool queue_active{false};
    static double delta_pose_dist = 0, pose_dist = 0;
    static float vel_x = 0;
    static float limit_delta_pose_dist = 1.0;
    static float limit_time = 20;
    static ros::Time last_cmd_vel_time; //add
    static const double cmd_vel_timeout_sec = 0.5; // cmd_velがこの秒数届かないと停止とみなす
    static int repeat_waypoint_counter = 0;
    static const int repeat_waypoint_threshold = 2;
    static int moving_confirm_count = 0;  // 動いていると判定された回数をカウント
    static int clear_count = 0;
    static const int moving_confirm_threshold = 50; // 連続で50回動いていたらリセットする

}
// グローバル変数として定義（関数の外に書く）
ros::Time last_mcl_pose_time = ros::Time(0);

void QueueActiveCallback(const std_msgs::Bool::ConstPtr &msg) {
    queue_active.store(msg->data);
}

/*void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    try {
        cmd_vel = *msg;
    } catch (const std::exception &) {
        ROS_WARN("Failed cmd_vel");
    }
}*/
void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    try {
        cmd_vel = *msg;
        last_cmd_vel_time = ros::Time::now();  // cmd_vel を受信した時刻を記録
    } catch (const std::exception &) {
        ROS_WARN("Failed cmd_vel");
    }
}

void waypointCallback(const waypoint_manager_msgs::Waypoint::ConstPtr &msg) {
    try {
        if (is_fst_flag) {
            old_id = msg->identity;
            is_fst_flag.store(false);
            start_time = time(NULL);
            return;
        }

        if (msg->identity != old_id) {
            start_time = time(NULL);
            is_fst_waypoint_reached.store(true);
        }

        recived_waypoint.store(true);
        old_id = msg->identity;
    } catch (const std::exception &) {
        recived_waypoint.store(false);
        ROS_WARN("Failed parse check_robot_moving_node");
    }
}

void MclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    last_mcl_pose_time = ros::Time::now();
    try {
        current_position.x() = msg->pose.pose.position.x;
        current_position.y() = msg->pose.pose.position.y;
        ros::Time current_time = ros::Time::now();
        
        double time_since_last_cmd_vel = (current_time - last_cmd_vel_time).toSec(); //add
        if (cmd_vel.linear.x == 0.0 || time_since_last_cmd_vel > cmd_vel_timeout_sec) {
         //   ROS_INFO("Robot is stopped.");
         //   ROS_INFO("time_since_last_cmd_vel: %f, cmd_vel_timeout_sec: %f", time_since_last_cmd_vel, cmd_vel_timeout_sec);
            delta_pose_dist = 0.0;
        } else {
         //   ROS_INFO("Robot is moving.");
         //   ROS_INFO("time_since_last_cmd_vel: %f, cmd_vel_timeout_sec: %f", time_since_last_cmd_vel, cmd_vel_timeout_sec);
            delta_pose_dist = 1.0;
        }
        //ROS_INFO("time_since_last_cmd_vel: %f, cmd_vel_timeout_sec: %f", time_since_last_cmd_vel, cmd_vel_timeout_sec);
        old_current_position = current_position;
    } catch (const std::exception &) {
  //      ROS_WARN("Failed mcl_pose");
    }
}

ros::Duration mcl_pose_timeout(1.0); // 1秒以上来てなければ無効

void checkMclPoseTimeout(const ros::TimerEvent&) {
    ros::Duration time_since_last_pose = ros::Time::now() - last_mcl_pose_time;
    if (time_since_last_pose > mcl_pose_timeout) {
//        ROS_WARN("MCL pose timeout! Robot is considered stopped.");
        delta_pose_dist = 0.0;
    } else {
        // 動いている前提のロジックで計算（例えば odom などを使う）
        delta_pose_dist = 1.0; // 仮に動いてるとする
    }
}

void IsReachedGoalCallback(const std_msgs::Bool::ConstPtr &msg) {
    try {
        is_reached_goal.store(msg->data);
    } catch (const std::exception &) {
     //   ROS_WARN("Failed is_reached_goal");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "check_robot_moving_node");
    ros::NodeHandle nh, private_nh("~");
    ros::Time last_mcl_pose_time; //add

    std::string goal_topic, waypoint_topic, is_reached_goal_topic;
    std::string mcl_pose_topic, cmd_vel_topic, clear_costmap_srv;
    std::string robot_base_frame, global_frame;

    float goal_check_frequency, wait_no_waypoint_time;

    recived_waypoint.store(false);
    stop_waypoint.store(false);
    is_fst_flag.store(true);
    is_fst_waypoint_reached.store(false);
    is_reached_goal.store(false);
    is_to_prev_waypoint.store(false);

    private_nh.param("goal_topic", goal_topic, std::string("move_base_simple/goal"));
    private_nh.param("waypoint", waypoint_topic, std::string("waypoint"));
    private_nh.param("is_reached_goal_topic", is_reached_goal_topic, std::string("waypoint/is_reached"));
    private_nh.param("robot_base_frame", robot_base_frame, std::string("base_link"));
    private_nh.param("global_frame", global_frame, std::string("map"));
    private_nh.param("goal_check_frequency", goal_check_frequency, static_cast<float>(1));
    private_nh.param("wait_no_waypoint_time", wait_no_waypoint_time, static_cast<float>(5.0));
    private_nh.param("default_goal_radius", default_goal_radius, static_cast<float>(1.0));
    private_nh.param("mcl_pose_topic", mcl_pose_topic, std::string("mcl_pose"));
    private_nh.param("cmd_vel_topic", cmd_vel_topic, std::string("icart_mini/cmd_vel"));
    private_nh.param("clear_costmap_srv", clear_costmap_srv, std::string("move_base/clear_costmaps"));
    private_nh.param("limit_time", limit_time, static_cast<float>(20.0));
    private_nh.param("limit_delta_pose_dist", limit_delta_pose_dist, static_cast<float>(0.01));

    ros::Rate loop_rate(5);

    auto waypoint_subscriber = nh.subscribe(waypoint_topic, 1, waypointCallback);
    auto mcl_pose_subscriber = nh.subscribe(mcl_pose_topic, 2, MclPoseCallback);
    auto cmd_vel_subscriber = nh.subscribe(cmd_vel_topic, 2, CmdVelCallback);
    auto is_reached_goal_subscriber = nh.subscribe<std_msgs::Bool>(is_reached_goal_topic, 1, IsReachedGoalCallback);
    auto queue_subscriber = nh.subscribe<std_msgs::Bool>("/queue_wait/active", 1, QueueActiveCallback);

    auto prev_waypoint_service = nh.serviceClient<std_srvs::Trigger>("waypoint_server/prev_waypoint");
    auto next_waypoint_service = nh.serviceClient<std_srvs::Trigger>("waypoint_server/next_waypoint");
    auto clear_costmap_service = private_nh.serviceClient<std_srvs::Empty>(clear_costmap_srv);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), checkMclPoseTimeout); //add

    ROS_INFO("Start check_robot_moving_node");

    while (ros::ok()) {
        ros::spinOnce();

        if (queue_active.load()) {
            last_moving_time = time(NULL);
            clear_count = 0;
            repeat_waypoint_counter = 0;
            moving_confirm_count = 0;
            is_to_prev_waypoint.store(false);
            loop_rate.sleep();
            continue;
        }

        if (!recived_waypoint.load()) {
       //     ROS_INFO("Waiting waypoint check_moving_node");
            ros::Duration(1.0).sleep();
            last_moving_time = time(NULL);
            continue;
        }

        if (is_reached_goal && is_to_prev_waypoint) {
            std_srvs::Empty data;
            clear_costmap_service.call(data);
            is_to_prev_waypoint.store(false);
         //   ROS_WARN("Clear Costmaps");
        }
        //delta_pose_dist = 0.0; //add
       /// ROS_INFO_STREAM("delta_pose_dist: " << delta_pose_dist);

        if (delta_pose_dist <= limit_delta_pose_dist && !is_reached_goal) {
            //ROS_INFO("time:%ld, stopped time:%ld\n", time(NULL) - start_time, time(NULL) - last_moving_time);
            moving_confirm_count = 0;
            //clear_count = 0;
            if (time(NULL) - last_moving_time >= 5) {
                clear_count += 1;
                if (is_fst_waypoint_reached && clear_count == 1) {
                    std_srvs::Trigger trigger;
                    std_srvs::Empty empty_service;
                    ROS_INFO("Costmap_clear!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    clear_costmap_service.call(empty_service);
                    //clear_count += 1;
                }
            }
            if (clear_count >= 20){
                clear_count = 0;
            }
            if (time(NULL) - last_moving_time >= limit_time) {
                if (is_fst_waypoint_reached) {
                    std_srvs::Trigger trigger;
                    std_srvs::Empty empty_service;

                    ROS_INFO("Recall!!!!!!!!!");
                    clear_costmap_service.call(empty_service);
                    if (prev_waypoint_service.call(trigger)) {
            //            ROS_INFO("PrevWaypoint call success");

                        // 次にNextWaypointを即座に呼ぶ
                        ros::Duration(1.0).sleep();  // ちょっと待ってから
                        ROS_INFO("Service call NextWaypoint()");
                        repeat_waypoint_counter++;
    //                    ROS_INFO("repeat_waypoint_counter = %d", repeat_waypoint_counter);
                        if (!next_waypoint_service.call(trigger)) {
                            ROS_WARN("Failed to call NextWaypoint");
                        }
                    } else {
                        ROS_WARN("Failed to call PrevWaypoint");
                    }

                    is_to_prev_waypoint.store(true);
                    last_moving_time = time(NULL);
                    //repeat_waypoint_counter++;
                    if (repeat_waypoint_counter >= repeat_waypoint_threshold) {
                    //    ROS_WARN("Repeated Prev→Next twice, force NextWaypoint");
            
                        if (next_waypoint_service.call(trigger)) {
      //                      ROS_INFO("Forced NextWaypoint call success");
                        } else {
                            ROS_WARN("Forced call failed");
                        }
            
                        repeat_waypoint_counter = 0;
                    }
                }
            }
        } else {
            last_moving_time = time(NULL);
            //repeat_waypoint_counter = 0;
            moving_confirm_count++;
           // ROS_INFO("moving_confirm_count: %d", moving_confirm_count);

            if (moving_confirm_count >= moving_confirm_threshold) {
                // 連続で動けている場合のみリセット
                repeat_waypoint_counter = 0;
            }
        }

        loop_rate.sleep();
    }

//    ROS_INFO("Finish waypoint_server_node");
    return 0;
}
