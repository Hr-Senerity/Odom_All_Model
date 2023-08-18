/*
 * @Description: Using the ROS-NOETIC version, an analysis of the wheel odometer for a double wheel differential car model
 * @Version: 1.0
 * @Author: Senerity
 * @Date: 2023-08-16
 * @LastEditors: Senerity
 * @LastEditTime: 2023-08-17 16:28:11
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

double wheel_radius = 1;  // vehicle diameter，The unit is m
double wheel_dist = 1;   // wheel base，The unit is m

/**
 * @description: This function inputs two wheel speeds and outputs odometer information
 * @param {double} wheel_l
 * @param {double} wheel_r
 * @param {double} time
 * @return {nav_mags::Odometry}  odom_num
 * @author: Senerity
 */

nav_msgs::Odometry odom_return_function(const double wheel_l, double wheel_r, double time) {
    nav_msgs::Odometry odom_num;
    odom_num.header.frame_id = "odom";

    // Convert the wheel speed to the line speed of each wheel
    // According to the formula, the unit is m/min, converted to m/s;

    double v_l = wheel_l * M_PI * wheel_radius / 60;
    double v_r = wheel_r * M_PI * wheel_radius / 60;

    // Convert the linear speed of each wheel to the speed of the car body
    double v = (v_l + v_r) / 2;
    double w = (v_r - v_l) / wheel_dist;
    double d = wheel_dist * (v_l + v_r) / 2 / (v_r - v_l);

    // Give the angle, pose
    double theta = w * time;
    double x = v * cos(theta) * time;
    double y = v * sin(theta) * time;

    // Pose update
    theta += theta;
    x += x;
    y += y;

    //Output calculated values as odometry msgs
    odom_num.twist.twist.linear.x = v;
    odom_num.twist.twist.angular.z = w;

    odom_num.pose.pose.position.x = x;
    odom_num.pose.pose.position.y = y;
    odom_num.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);

    return odom_num;
}

/**
 * @description: This function is a reserved low-level communication function, and the recommended output is two rounds of each wheel speed
 * @return {*}
 * @author: Senerity
 */
void get_new_data() {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    double wheel_a, wheel_b;

    ros::Rate loop_rate(100);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (ros::ok()) {
        current_time = ros::Time::now();
        double velDeltaTime = (current_time - last_time).toSec();

        get_new_data();
        nav_msgs::Odometry odom_out = odom_return_function(wheel_a, wheel_b, velDeltaTime);

        odom_pub.publish(odom_out);

        ros::spinOnce();
        loop_rate.sleep();
        last_time = current_time;
    }
    return 0;
}
