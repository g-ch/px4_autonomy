#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

/* functions declaration */
void chatterCallback_local_pose(const geometry_msgs::PoseStamped &msg);
void chatterCallback_local_vel(const geometry_msgs::TwistStamped &msg);
void chatterCallback_mode(const mavros_msgs::State &msg);
void chatterCallback_cmd_pose(const px4_autonomy::Position &msg);
void chatterCallback_cmd_vel(const px4_autonomy::Velocity &msg);

/* global variaties */
Eigen::Vector3d pos(0,0,0);
Eigen::Vector3d vel(0,0,0);
Eigen::Quaterniond att;

Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
float yaw_sp = 0.0;
float yaw_rate_sp = 0.0;

/*values to feed watch dog*/
double pose_stamp = 0.0;
double vel_stamp = 0.0;
double pose_sp_stamp = 0.0;
double vel_sp_stamp = 0.0;
bool offboard_ready = false;

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "offboard_center");
    ros::NodeHandle nh;

    /* setting parameters definition */
    float coor_type;
    float toff_height;
    float max_vx;
    float max_vy;
    float max_vz;
    float max_yawrate;
    float pt_kp;
    float pt_ki;
    float pt_kd;

    /* read from settings.yaml */
    nh.getParam("/offboard_center/coor_type", coor_type);
    nh.getParam("/offboard_center/toff_height", toff_height);
    nh.getParam("/offboard_center/max_vx", max_vx);
    nh.getParam("/offboard_center/max_vy", max_vy);
    nh.getParam("/offboard_center/max_vz", max_vz);
    nh.getParam("/offboard_center/max_yawrate", max_yawrate);
    nh.getParam("/offboard_center/pt_kp", pt_kp);
    nh.getParam("/offboard_center/pt_ki", pt_ki);
    nh.getParam("/offboard_center/pt_kd", pt_kd);

    /* handle topics */
    ros::Subscriber local_pose_sub = nh.subscribe("/mavros/local_position/pose", 1,chatterCallback_local_pose);
    ros::Subscriber local_vel_sub = nh.subscribe("/mavros/local_position/velocity", 1,chatterCallback_local_vel);
    ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
    ros::Subscriber pose_sp_sub = nh.subscribe("/px4/cmd_pose", 1,chatterCallback_cmd_pose);
    ros::Subscriber vel_sp_sub = nh.subscribe("/px4/cmd_vel", 1,chatterCallback_cmd_vel);

    ros::Rate loop_rate(20);
    while(nh.ok())
    {
        
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}

void chatterCallback_local_pose(const geometry_msgs::PoseStamped &msg)
{
    pose_stamp = msg.header.stamp.toSec();

    pos(0) = msg.pose.position.x;
    pos(1) = msg.pose.position.y;
    pos(2) = msg.pose.position.z;

    att.x() = msg.pose.orientation.x;
    att.y() = msg.pose.orientation.y;
    att.z() = msg.pose.orientation.z;
    att.w() = msg.pose.orientation.w;
}

void chatterCallback_local_vel(const geometry_msgs::TwistStamped &msg)
{
    vel_stamp = msg.header.stamp.toSec();

    vel(0) = msg.twist.linear.x;
    vel(1) = msg.twist.linear.y;
    vel(2) = msg.twist.linear.z;    
}

void chatterCallback_mode(const mavros_msgs::State &msg)
{
    if(msg.mode=="OFFBOARD") offboard_ready=true;
    else offboard_ready=false;
}

void chatterCallback_cmd_pose(const px4_autonomy::Position &msg)
{
    pos_sp(0) = msg.x;
    pos_sp(1) = msg.y;
    pos_sp(2) = msg.z;
    yaw_sp = msg.yaw;
}

void chatterCallback_cmd_vel(const px4_autonomy::Velocity &msg)
{
    vel_sp(0) = msg.x;
    vel_sp(1) = msg.y;
    vel_sp(2) = msg.z;
    yaw_rate_sp = msg.yaw_rate;
}