#include <gmr_intro_poo/gmr_intro_poo.hpp>

RobotClass::RobotClass(ros::NodeHandle* nh)
{
    _nh = nh;
    _nh->param("nome_topico_left_rpm", _params.topic_name_left_rpm, std::string("/left_rpm"));
    _nh->param<std::string>("nome_topico_right_rpm", _params.topic_name_right_rpm, "/right_rpm");
    _nh->param("time_between_toggles", _params.time_between_toggles, -1.0);
    _nh->param("/axle_track", _params.axle_track, 1.0);
    _nh->param("/gear_ratio", _params.gear_ratio, 1.0);
    _nh->param("/wheel_radius", _params.wheel_radius, 1.0);
    
    _client_toggle_robot = _nh->serviceClient<std_srvs::Trigger>("/toggle_robot");
    _pub_odom = _nh->advertise<nav_msgs::Odometry>("/odom", 1);
    _sub_left = _nh->subscribe(_params.topic_name_left_rpm, 1, &RobotClass::subLeft, this);
    _sub_right = _nh->subscribe(_params.topic_name_right_rpm, 1, &RobotClass::subRight, this);

    _robot_pose = (const struct RobotPose) { 0 };
    _prev_timestamp = ros::Time::now().toSec();
    _prev_timestamp_toggle = _prev_timestamp;
    _vel_m_s.left = 0.0;
    _vel_m_s.right = 0.0;  
    ROS_INFO_STREAM("time_between_toggles " << _params.time_between_toggles);  
}
void RobotClass::checkToggleRobot()
{
    double cur_timestamp = ros::Time::now().toSec();
    std_srvs::Trigger srv;
    if(_params.time_between_toggles > 0  && cur_timestamp - _prev_timestamp_toggle > _params.time_between_toggles)
    {
        _prev_timestamp_toggle = cur_timestamp;
        if(_client_toggle_robot.call(srv))
        {
            ROS_WARN_STREAM("message: " << srv.response.message);
        }
        else 
        {
           ROS_ERROR("Falha em chamar o serviço");
        }
    }
}
void RobotClass::calculateOdom()
{
    ros::Time cur_timestamp = ros::Time::now();
    double vl = _vel_m_s.left;
    double vr = _vel_m_s.right;
    double dt, wz, vx;
    nav_msgs::Odometry odom;
    tf2::Quaternion odom_quat;

    dt = cur_timestamp.toSec() - _prev_timestamp;
    _prev_timestamp = cur_timestamp.toSec();
    vx = 0.5*(vr + vl); // Eq. 1                     
    wz = (vr - vl)/_params.axle_track; // Eq.2
    _robot_pose.theta += wz*dt; // Eq. 8
    _robot_pose.x += vx*std::cos(_robot_pose.theta)*dt; // Eq. 6
    _robot_pose.y += vx*std::sin(_robot_pose.theta)*dt; // Eq. 7

    odom.header.frame_id = "/map";
    odom.child_frame_id = "/base_link";
    odom_quat.setRPY(0,0, _robot_pose.theta);
    odom.twist.twist.angular.z = wz;
    odom.twist.twist.linear.x = vx;
    odom.pose.pose.position.x = _robot_pose.x;
    odom.pose.pose.position.y = _robot_pose.y;
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);
    _pub_odom.publish(odom);

    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "/map";
    odom_trans.child_frame_id = "/base_link";
    odom_trans.header.stamp = cur_timestamp;
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.rotation = tf2::toMsg(odom_quat);
    br.sendTransform(odom_trans);

}
void RobotClass::subLeft(const std_msgs::Float32::ConstPtr &msg)
{
    _vel_m_s.left = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    ROS_INFO_STREAM( "velocidade linear esquerda: " << _vel_m_s.left );
}
void RobotClass::subRight(const std_msgs::Float32::ConstPtr &msg)
{
    _vel_m_s.right  = msg->data/_params.gear_ratio*_params.wheel_radius*2*M_PI/60.0;
    ROS_INFO_STREAM( "velocidade linear direita: " << _vel_m_s.right );
}