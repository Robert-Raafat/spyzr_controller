#ifndef SIMPLE_CONTROLLER_H
#define SIMPLE_CONTROLLER_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


class SimpleController
{
public:
    SimpleController(const ros::NodeHandle &, double radius, double width_separation, double length_separation);
private:
    void velCallback(const geometry_msgs::Twist &);

    ros::NodeHandle nh_;
    ros::Subscriber vel_sub_;
    ros::Publisher right_front_cmd_pub_;
    ros::Publisher right_rear_cmd_pub_;
    ros::Publisher left_front_cmd_pub_;
    ros::Publisher left_rear_cmd_pub_;

    Eigen::Matrix<double, 4, 3> speed_conversion_;

};
#endif 
