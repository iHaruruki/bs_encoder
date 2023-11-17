#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


#ifndef _INCLUDED_WHEEL_ODOMETRY_H_
#define _INCLUDED_WHEEL_ODOMETRY_H_


class WheelOdometry
{
private:
    ros::NodeHandle nh;
    nav_msgs::Odometry odom;
    tf::TransformBroadcaster tf_caster;
    ros::Timer timer;
    ros::Publisher pub_odom;

    ros::Time cur_stamp;
    ros::Time last_stamp;
    double d_time;
    double cur_x;
    double cur_y;
    double cur_th;
    double cur_v_x;
    double cur_v_th;

    double Lx, Ll;
    double d_Lx, d_Ll;
    double Lr_o, Ll_o;

    double track_width;
    double wheel_radius;

    void init();
    void update();
    void publish_odom();
    void publish_tf();
    double normalize_angle( double angle );
    void timer_callback( const ros::TimerEvent& e);

public:
    WheelOdometry();
    void run();
};

#endif
