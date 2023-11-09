#include <ros/ros.h>
#include "wheel_odometry.h"

int main(int argc,char** argv)
{
    ros::init( argc, argv, "wheel_odometry");

    WheelOdometry odom;
    odom.run();
    ros::spin();
    return 0;
}