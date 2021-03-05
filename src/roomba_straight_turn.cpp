#include "roomba_straight_turn/roomba_straight_turn.h"

RoombaStraightTurn::RoombaStraightTurn():private_nh("~")
{
    private_nh.param("hz", hz_, {50});
    sub_pose = nh.subscribe("/roomba/odometry", 10, &RoombaStraightTurn::odometry_callback, this);
    sub_laser = nh.subscribe("/scan", 10, &RoombaStraightTurn::laser_callback, this);


    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void RoombaStraightTurn::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = *msg;


}
void RoombaStraightTurn::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
}
void RoombaStraightTurn::go_straight()
{
    std::cout<<current_pose.pose.pose<<std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.linear.x = 0.1;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaStraightTurn::turn()
{
    std::cout << current_pose.pose.pose << std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.angular.z = 0.2;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaStraightTurn::process()
{
    ros::Rate loop_rate(hz_);
    double dist = 0;
    double bef_x = current_pose.pose.pose.position.x;
    double bef_r,bef_p,bef_y;//出力値
    double sum_y = 0;
    tf::Quaternion quat(current_pose.pose.pose.orientation.x,current_pose.pose.pose.orientation.y,current_pose.pose.pose.orientation.z,current_pose.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(bef_r, bef_p, bef_y);//クォータニオン→オイラー角
    while(ros::ok())
    {
        if (laser.ranges.size() > 1000){
            std::cout << "ranges" << laser.ranges[539] << std::endl;
        }
        if (dist <= 0.5){
            go_straight();
            dist += std::abs(current_pose.pose.pose.position.x - bef_x);
            bef_x = current_pose.pose.pose.position.x;
            sum_y = 0;
            tf::Quaternion quat(current_pose.pose.pose.orientation.x,current_pose.pose.pose.orientation.y,current_pose.pose.pose.orientation.z,current_pose.pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(bef_r, bef_p, bef_y);
            std::cout << "dist" << dist << std::endl;
        }
        else if(sum_y <= M_PI){
            double r, p, y;
            turn();
            tf::Quaternion quat(current_pose.pose.pose.orientation.x,current_pose.pose.pose.orientation.y,current_pose.pose.pose.orientation.z,current_pose.pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(r, p, y);
            double delta_y = y - bef_y;
            if (delta_y < 0 && y < 0 && bef_y > 0){
                delta_y += 2*M_PI;
            }
            sum_y += delta_y;


            std::cout << "yowyow" << sum_y << std::endl;
            bef_x = current_pose.pose.pose.position.x;
            bef_y = y;
        }
        else{
            dist = 0;
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
