#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class RobotClass
{
    public:
        RobotClass(ros::NodeHandle* nh);
        void                calculateOdom();
        void                checkToggleRobot();

    private:
        void                subLeft(const std_msgs::Float32::ConstPtr &msg);
        void                subRight(const std_msgs::Float32::ConstPtr &msg);
        ros::NodeHandle*    _nh;
        ros::Publisher      _pub_odom;
        ros::ServiceClient  _client_toggle_robot;
        ros::Subscriber     _sub_left;
        ros::Subscriber     _sub_right;

        struct Params
        {
            double          time_between_toggles;
            double          axle_track;
            double          gear_ratio;
            double          wheel_radius;
            std::string     topic_name_left_rpm;
            std::string     topic_name_right_rpm;
        }_params;

        struct Vel_m_s
        {
            double          left;
            double          right;
        }_vel_m_s;

        struct RobotPose
        {
            double x;
            double y;
            double theta;
        }_robot_pose;

        double _prev_timestamp;
        double _prev_timestamp_toggle;
};