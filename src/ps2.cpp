#include "ros/ros.h"
#include <iostream>
#include <mavros_msgs/State.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

#define VX_LIMIT 2.0
#define VY_LIMIT 2.0
#define VZ_LIMIT 2.0
#define YR_LIMIT 1.0
#define PI 3.14159265


using namespace std;

enum control_flag
{
    TAKE_OFF = 0,
    FLY = 1,
    LAND = 2
}ps2_flag;

/* FOR GAZABO OFFBOARD ENABLE */
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
///////////////////////////
float PI_2  = PI / 2.0;
int status;
float current_px = 0.0;
float current_py = 0.0;
float current_pz = 0.0;
float current_yaw = 0.0;

float current_vx = 0.0;
float current_vy = 0.0;

float cmd_vx = 0.0;
float cmd_vy = 0.0;
float cmd_vz = 0.0;
float cmd_yr = 0.0;

void chatterCallback_status(const std_msgs::UInt8 &msg)
{
	status =  msg.data;
}

void chatterCallback_pose(const px4_autonomy::Position &msg)
{
    current_px = msg.x;
    current_py = msg.y;
    current_pz = msg.z;
    current_yaw = msg.yaw;
}

void chatterCallback_vel(const px4_autonomy::Velocity &msg)
{
    current_vx = msg.x*sin(current_yaw + PI_2) + msg.y*cos(current_yaw + PI_2);
    current_vy = msg.x*cos(current_yaw + PI_2) - msg.y*sin(current_yaw + PI_2);
    //cout<<"v("<<current_vx<<","<<current_vy<<")"<<endl;
}

void chatterCallback_joy(const sensor_msgs::Joy::ConstPtr& joy)
{
    cmd_vx = VX_LIMIT * joy->axes[2];
    cmd_vy = VY_LIMIT * joy->axes[5];
    cmd_vz = VZ_LIMIT * joy->axes[1];
    cmd_yr = YR_LIMIT * joy->axes[0];
    if(joy->buttons[1] == 1 && joy->buttons[2] == 0)
    {
        ps2_flag == LAND;
    }
    else if(joy->buttons[1] == 0 && joy->buttons[2] == 1)
    {
        ps2_flag = TAKE_OFF;
    }
    else
        ps2_flag = FLY;
}

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "ps2");
    ros::NodeHandle nh;

     /* FOR GAZABO OFFBOARD ENABLE */
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //////////////////////////////


    ros::Subscriber status_sub = nh.subscribe("/px4/status", 1, chatterCallback_status);
    ros::Subscriber local_pose_sub = nh.subscribe("/px4/pose", 1,chatterCallback_pose);
    ros::Subscriber local_vel_sub = nh.subscribe("/px4/velocity", 1,chatterCallback_vel);
    ros::Subscriber joy_sub = nh.subscribe("/joy",1,chatterCallback_joy);

    ros::Publisher pose_pub = nh.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
    ros::Publisher vel_pub = nh.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
    ros::Publisher takeoff_pub = nh.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 

    px4_autonomy::Position pose;
    px4_autonomy::Velocity vel;
    px4_autonomy::Takeoff tf_val;

    ros::Rate loop_rate(20);


    bool record_bool = false; //just to test stage 3(counter between 600~700)
    float record_x = 0.0;
    float record_y = 0.0;

    while(nh.ok())
    {
    	/* FOR GAZABO OFFBOARD ENABLE */
    	if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        /////////////////////////////

        switch(status)
        {
            case 1:
            {
                if(ps2_flag == TAKE_OFF)
                {
                    tf_val.take_off = 1; //take off when status is 1(waiting take off orders.)
                    takeoff_pub.publish(tf_val);
                }
                break;
            }

            case 4:
            case 5:
            {
                if(ps2_flag == FLY)
                {
                    vel.header.stamp = ros::Time::now();
                    vel.x = -cmd_vx*sin(current_yaw) + cmd_vy*cos(current_yaw);
                    vel.y = cmd_vx*cos(current_yaw) + cmd_vy*sin(current_yaw);
                    vel.z = cmd_vz;
                    vel.yaw_rate = cmd_yr;
                    vel_pub.publish(vel);
                }
                else if(ps2_flag == LAND)
                {
                    tf_val.take_off = 2; //land
                    takeoff_pub.publish(tf_val);
                }
                else;

                break;
            }
        }


    	

    	ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
