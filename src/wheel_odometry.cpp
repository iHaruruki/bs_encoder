#include "wheel_odometry.h"


WheelOdometry:: WheelOdometry()
{
    init();
}

void WheelOdometry::init()
{
    cur_x = 0.0;
    cur_y = 0.0;
    cur_th = 0.0;
    cur_v_x = 0.0;
    cur_v_th = 0.0;
    Lr_o = 0.0;
    Ll_o = 0.0;
}


void WheelOdometry::run()
{
    ros::NodeHandle pnh("~");
    if ( pnh.getParam("/track_width", track_width) )
    {
        ROS_INFO("Odometry Node: Set track width: %lf", track_width);
    }
    else {
        track_width = 1.0;
        ROS_WARN("Odometry Node: Set DEFAULT value for track width: %lf", track_width);
    }

    if (pnh.getParam("/wheel_radius", wheel_radius))
    {
        ROS_INFO("Odometry Node: Set wheel radius: %lf", wheel_radius);
    }
    else {
        wheel_radius = 0.1;
        ROS_WARN("Odometry Node: Set DEFAULT value for wheel radius: %lf", wheel_radius);
    }


    timer = nh.createTimer ( ros::Duration(0.1), WheelOdometry::timer_callback );
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
}


void WheelOdometry::timer_callback(const ros::TimerEvent& e)
{
    cur_stamp = ros::Time::now();
    d_time = (cur_stamp - last_stamp).toSec(); // 微小時間
    last_stamp = cur_stamp;

    // エンコーダ値の取得と距離計算 : Ll とLr に積算距離 m] を代入する．-----------------------------------------------------------------------------------------------------
    Lr = 0;
    Ll = 0;

    d_Lr = Lr - Lr_o; // 右車輪の微小移動距離
    d_Ll = Ll - Ll_o; // 左車輪の微小移動距離
    Lr_o = Lr;
    Ll_o = Ll;

    update();
}

void WheelOdometry::update( )
{
    // 自己位置計算
    double d_L = (d_Lr + d_Ll) / 2.0; // 微小並進移動距離
    double d_theta = 0.0; // 微小旋回角度
    // 直進時
    if ( std::fabs( d_Lr - d_Ll ) < 0.000001 )
    {
        cur_x += d_L * std::cos( cur_th ); // x_i
        cur_y += d_L * std::sin( cur_th ); // y_i
    }

    // 旋回時
    else
    {
        d_theta = ( d_Lr - d_Ll ) / track_width; // 微小旋回角度
        double rho = d_L / d_theta; // 旋回半径
        double d_Lp = 2.0 * rho * std::sin( d_theta * 0.5 );
        cur_x += d_Lp * std::cos( cur_th + d_theta * 0.5 );     // x_i
        cur_y += d_Lp * std::sin( cur_th + d_theta * 0.5 );     // y_i
        cur_th = normalize_angle( cur_th + d_theta );           // theta_i
    }

    // 移動速度計算
    if ( d_time < 0.000001 )
    {
        cur_v_x = 0.0;
        cur_v_th = 0.0;
    }
    else
    {
        cur_v_x = d_L / d_time;
        cur_v_th = d_theta / d_time;
    }
    publish_odom();
    publish_tf();
}


void WheelOdometry::publish_odom()
{
    odom.header.stamp = cur_stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = " base_footprint "; // ロボットのベースフレーム．u rdf に合わせて修正

    odom.pose.pose.position.x = cur_x;
    odom.pose.pose.position.y = cur_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw( cur_th );

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = FLT_MAX;
    odom.pose.covariance[21] = FLT_MAX;
    odom.pose.covariance[28] = FLT_MAX;
    odom.pose.covariance[35] = 0.01;

    odom.twist.twist.linear.x = cur_v_x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = cur_v_th;

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[14] = FLT_MAX;
    odom.twist.covariance[21] = FLT_MAX;
    odom.twist.covariance[28] = FLT_MAX;
    odom.twist.covariance[35] = 0.01;


    pub_odom.publish(odom);
}


void WheelOdometry::publish_tf()
{
    geometry_msgs::TransformStamped tf_trans;
    tf_trans.header.stamp = odom.header.stamp;
    tf_trans.header.frame_id = odom.header.frame_id;
    tf_trans.child_frame_id = odom.child_frame_id;
    tf_trans.transform.translation.x = odom.pose.pose.position.x;
    tf_trans.transform.translation.y = odom.pose.pose.position.y;
    tf_trans.transform.translation.z = odom.pose.pose.position.z;
    tf_trans.transform.rotation = odom.pose.pose.orientation;

    tf_caster.sendTransform(tf_trans);
}

double WheelOdometry::normalize_angle (double angle)
{
    while ( angle > M_PI ) angle -= 2.0 * M_PI;
    while ( angle < M_PI ) angle += 2.0 * M_PI;
    return angle;
}