#include "roomba_straight_turn/roomba_straight_turn.h"

RoombaStraightTurn::RoombaStraightTurn():private_nh("~")
{
    private_nh.param("hz", hz_, {10});
    sub_pose = nh.subscribe("/roomba/odometry", 10, &RoombaStraightTurn::odometry_callback, this);

    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void RoombaStraightTurn::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = *msg;
}

void RoombaStraightTurn::go_straight()
{
    std::cout<<current_pose<<std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.linear.x = 0.1;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaStraightTurn::turn()
{
    std::cout << current_pose << std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.angular.z = 0.5;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaStraightTurn::process()
{
    ros::Rate loop_rate(hz_);
    double dist = 0;
    double bef_x = current_pose.pose.pose.position.x;
    while(ros::ok())
    {
        if (dist <= 0.5){
            go_straight();
            dist += std::abs(current_pose.pose.pose.position.x - bef_x);
            bef_x = current_pose.pose.pose.position.x;
        }
        else{
            turn();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roomba_straight_turn");
    RoombaStraightTurn roomba_straight_turn;

    roomba_straight_turn.process();
    return 0;
}
