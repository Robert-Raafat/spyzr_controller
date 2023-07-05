#include <ros/ros.h>
#include "spyzr_controller/simple_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double wheel_radius, wheel_width_separation, wheel_length_separation;
    pnh.getParam("wheel_radius", wheel_radius);
    pnh.getParam("wheel_width_separation", wheel_width_separation);
    pnh.getParam("wheel_length_separation", wheel_length_separation);

    SimpleController controller(nh, wheel_radius, wheel_width_separation, wheel_length_separation);
    
    ros::spin();
    
    return 0;
} 
