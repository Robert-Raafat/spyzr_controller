#include "spyzr_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>

SimpleController::SimpleController(const ros::NodeHandle &nh, double radius, double width_separation, double length_separation): nh_(nh)
{


    radius = 0.04;
    width_separation = 0.2916;
    length_separation = 0.22;

    ROS_INFO_STREAM("Using wheel radius " << radius);
    ROS_INFO_STREAM("Using wheel width separation " << width_separation);
    ROS_INFO_STREAM("Using wheel length separation " << length_separation);

    right_front_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_front_right_controller/command", 10);
    right_rear_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_rear_right_controller/command", 10);
    left_front_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_front_left_controller/command", 10);
    left_rear_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_rear_left_controller/command", 10);

 
    vel_sub_ = nh_.subscribe("spyzr_controller/cmd_vel", 1000, &SimpleController::velCallback, this);
    //speed_conversion_ << radius/4, radius/4, radius/4, radius/4, -radius/4, radius/4, radius/4, -radius/4, -radius/(4*((width_separation/2)+(length_separation/2))), radius/(4*((width_separation/2)+(length_separation/2))), -radius/(4*((width_separation/2)+(length_separation/2))), radius/(4*((width_separation/2)+(length_separation/2)));
    //speed_conversion_ << (radius/4)*(1, 1, 1, 1, -1, 1, 1, -1, -1/((width_separation+length_separation)/2), 1/((width_separation+length_separation)/2), -1/((width_separation+length_separation)/2), 1/((width_separation+length_separation)/2));
    speed_conversion_ << 1 / radius, -1/radius, -(length_separation+width_separation)/(2*radius), 1/radius, 1/radius, (length_separation+width_separation)/(2*radius), 1/radius, 1/radius, -(length_separation+width_separation)/(2*radius), 1/radius, -1/radius, (length_separation+width_separation)/(2*radius);
    
    //speed_conversion_ << 1/radius, 1/radius, (length_separation+width_separation)/(2*radius), 1/radius, -1/radius, (length_separation+width_separation)/(2*radius), -1/radius, -1/radius, (length_separation+width_separation)/(2*radius), -1/radius, 1/radius, (length_separation+width_separation)/(2*radius);

    //speed_conversion_ << ((1/4)*(1, -1, -(length_separation+width_separation)/2, 1, 1, (length_separation+width_separation)/2, 1, 1, -(length_separation+width_separation)/2, 1, -1, (length_separation+width_separation)/2));
    ROS_INFO_STREAM("The conversion matrix is \n" << speed_conversion_);    
}       

void SimpleController::velCallback(const geometry_msgs::Twist &msg)
{

    Eigen::Vector3d robot_speed(msg.linear.x, msg.linear.y, msg.angular.z);
    Eigen::Vector4d wheel_speed = speed_conversion_* robot_speed;
    
    std_msgs::Float64 left_front_speed;
    std_msgs::Float64 right_front_speed;
    std_msgs::Float64 left_rear_speed;
    std_msgs::Float64 right_rear_speed;
    
    left_front_speed.data = wheel_speed.coeff(1);
    right_front_speed.data = wheel_speed.coeff(0);
    left_rear_speed.data = wheel_speed.coeff(3);
    right_rear_speed.data = wheel_speed.coeff(2);  

    ROS_INFO_STREAM("left_front_speed.data " << left_front_speed.data);
    ROS_INFO_STREAM("right_front_speed.data" << right_front_speed.data);
    ROS_INFO_STREAM("left_rear_speed.data " << left_rear_speed.data);
    ROS_INFO_STREAM("right_rear_speed.data" << right_rear_speed.data);



    right_front_cmd_pub_.publish(right_front_speed);
    right_rear_cmd_pub_.publish(right_rear_speed);
    left_front_cmd_pub_.publish(left_front_speed);
    left_rear_cmd_pub_.publish(right_rear_speed);
}
