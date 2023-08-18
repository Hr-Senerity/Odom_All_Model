/*
 * @Description: This file is used for two-wheel differential model analysis odometer
 * @Version: 1.0
 * @Autor: Senerity
 * @Date: 2023-08-17
 * @LastEditors: Senerity
 * @LastEditTime: 2023-08-17
 */

#include <iostream>
#include <math.h>

double wheel_Radis = 1; //vehicle diameter，The unit is m
double wheel_dist = 1;//wheel base，The unit is m
double dist_time = 0.01; //Unit of time，Here is 0.01 seconds, that is, 100hz frequency calculation

//In the function, wheel_l and wheel_r are the speed of the wheel, the unit is rad/min
void double_wheel_odom_pose(double wheel_l, double wheel_r, double &x, double &y, double &yaw) {
    //Convert the wheel speed to the line speed of each wheel
    //According to the formula, the unit is m/min, converted to m/s;
    double v_l = wheel_l * M_PI * wheel_Radis / 60;
    double v_r = wheel_r * M_PI * wheel_Radis / 60;

    double v = (v_l + v_r) / 2;
    double w = (v_r - v_l) / wheel_dist;
    double d = wheel_dist * (v_l + v_r) / 2 / (v_r - v_l);

    //Convert the linear speed of each wheel to the speed of the car body
    v = M_PI * wheel_Radis * (wheel_l + wheel_r) / 2 / 60;
    w = M_PI * wheel_Radis * (wheel_l + wheel_r) / wheel_dist / 60;
    d = wheel_dist * (wheel_l + wheel_r) / 2 / (wheel_r - wheel_l);

    //Give the angle, pose
    double theta = w * dist_time;
    double x = v * cos(theta) * dist_time;
    double y = v * sin(theta) * dist_time;

    //Pose update
    theta += theta;
    x += x;
    y += y;
}