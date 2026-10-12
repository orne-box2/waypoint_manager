#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

// === グローバル変数 ===
ros::ServiceClient dynamic_client;
ros::Publisher front_stop_publisher;
void publishFrontStop(bool stopped)
{
    std_msgs::Bool state;
    state.data = stopped;
    front_stop_publisher.publish(state);
}
bool obstacle_detected = false;
ros::Time last_obstacle_time;
double current_vel_x = -1.0;
double current_vel_theta = 100.0;  // 初期値を無効値として設定
bool force_stop = false;
double normal_slow_vel = 0.5;
double force_slow_vel = 0.7;

// しきい値（パラメータで読み込む）
double slow_threshold = 8.0;
double stop_threshold = 5.0;
double normal_check_angle_deg = 5.0;
double force_check_angle_deg = 5.0;
double obstacle_clear_duration = 7.0;
double normal_slow_theta = 1.0;

// === front obstacle check ===
// 与えられたしきい値以下なら true（障害物あり）
bool checkFrontObstacle(const sensor_msgs::LaserScan::ConstPtr& scan, double threshold, int angle_deg)
{
    int center_index = (0.0 - scan->angle_min) / scan->angle_increment;
    int width = angle_deg / (scan->angle_increment * 180.0 / M_PI);

    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = center_index - width; i <= center_index + width; ++i)
    {
        if (i >= 0 && i < scan->ranges.size())
        {
            double d = scan->ranges[i];
            if (std::isfinite(d))
            {
                min_distance = std::min(min_distance, d);
            }
        }
    }

    return (min_distance < threshold);
}


void setSymmetricThetaVel(double max_theta)
{
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter param_max;
    dynamic_reconfigure::DoubleParameter param_min;
    dynamic_reconfigure::Config config;

    param_max.name = "max_vel_theta";
    param_max.value = max_theta;

    param_min.name = "min_vel_theta";
    param_min.value = -max_theta;

    config.doubles.push_back(param_max);
    config.doubles.push_back(param_min);
    req.config = config;

    if (dynamic_client.call(req, res))
    {
        ROS_INFO("Set symmetric theta velocity: max = %f, min = %f", max_theta, -max_theta);
    }
    else
    {
        ROS_ERROR("Failed to set symmetric theta velocity.");
    }
}




// === max_vel_x setter ===
void setMaxVelX(double value)
{
    if (value == current_vel_x)
        return;

    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    double_param.name = "max_vel_x";
    double_param.value = value;
    config.doubles.push_back(double_param);
    req.config = config;

    if (dynamic_client.call(req, res))
    {
        current_vel_x = value;
        ROS_INFO("Set max_vel_x to %f", value);
    }
    else
    {
        ROS_ERROR("Failed to set max_vel_x");
    }
}

// === LaserScan callback ===
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (force_stop)
    {
        bool detected = checkFrontObstacle(scan, stop_threshold, force_check_angle_deg);
        publishFrontStop(detected);

        if (detected)
        {
            setMaxVelX(0.0);  // 完全停止
            setSymmetricThetaVel(0.0);
        }
        else
        {
            setMaxVelX(force_slow_vel);
            setSymmetricThetaVel(normal_slow_theta);
        }
        return;
    }

    bool detected = checkFrontObstacle(scan, slow_threshold, normal_check_angle_deg);

    if (detected)
    {
        if (!obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();
            obstacle_detected = true;
            setMaxVelX(normal_slow_vel);
        }
    }
    else
    {
        if (obstacle_detected)
        {
            last_obstacle_time = ros::Time::now();
            obstacle_detected = false;
        }

        if ((ros::Time::now() - last_obstacle_time).toSec() > obstacle_clear_duration)
        {
            setMaxVelX(1.0);  // 通常速度へ戻す
            setSymmetricThetaVel(2.0);

        }
    }
}



// === /force_stop service callback ===
bool forceStopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    force_stop = true;
    publishFrontStop(true);
    setMaxVelX(0.0);
    res.success = true;
    res.message = "Force stop activated.";
    return true;
}

// === /force_resume service callback ===
bool forceResumeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    force_stop = false;
    publishFrontStop(false);
    res.success = true;
    res.message = "Force stop released. Resuming normal control.";
    return true;
}

// === main ===
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_front_obstacle_check");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // パラメータの読み込み
    pnh.param("slow_threshold", slow_threshold, 8.0);
    pnh.param("stop_threshold", stop_threshold, 5.0);
    pnh.param("normal_check_angle_deg", normal_check_angle_deg, 5.0);
    pnh.param("force_check_angle_deg", force_check_angle_deg, 5.0);
    pnh.param("normal_slow_vel", normal_slow_vel, 0.5);
    pnh.param("force_slow_vel", force_slow_vel, 0.7);
    pnh.param("obstacle_clear_duration", obstacle_clear_duration, 7.0);
    pnh.param("normal_slow_theta", normal_slow_theta, 1.0);

    dynamic_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/move_base/TrajectoryPlannerROS/set_parameters");

    front_stop_publisher = nh.advertise<std_msgs::Bool>("/queue_wait/front_stop", 1, true);
    publishFrontStop(false);

    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::ServiceServer stop_srv = nh.advertiseService("/force_stop", forceStopCallback);
    ros::ServiceServer resume_srv = nh.advertiseService("/force_resume", forceResumeCallback);

    ros::spin();
    return 0;
}
