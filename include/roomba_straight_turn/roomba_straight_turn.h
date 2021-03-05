#ifndef ROOMBA_STRAIGHT_TURN_H
#define ROOMBA_STRAIGHT_TURN_H

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<roomba_500driver_meiji/RoombaCtrl.h>
#include<tf/tf.h>
#include<sensor_msgs/LaserScan.h>
#include<vector>
class RoombaStraightTurn
{
    public:
        RoombaStraightTurn();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void go_straight();
        void turn();
        void stop();

        int hz_;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_pose;
        ros::Subscriber sub_laser;
        ros::Publisher pub_cmd_vel;
        nav_msgs::Odometry current_pose;
        sensor_msgs::LaserScan laser;
};

#endif
