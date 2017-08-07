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


using namespace std;

int status;
float current_px = 0.0;
float current_py = 0.0;
float current_pz = 0.0;

void chatterCallback_status(const std_msgs::UInt8 &msg)
{
	status =  msg.data;
}

void chatterCallback_local_pose(const geometry_msgs::PoseStamped &msg)
{
    current_px = msg.pose.position.x;
    current_py = msg.pose.position.y;
    current_pz = msg.pose.position.z;
}


int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "test1");
    ros::NodeHandle nh;


    ros::Subscriber status_sub = nh.subscribe("/px4/status", 1, chatterCallback_status);
    ros::Subscriber local_pose_sub = nh.subscribe("/mavros/local_position/pose", 1,chatterCallback_local_pose);

    ros::Publisher pose_pub = nh.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
    ros::Publisher vel_pub = nh.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
    ros::Publisher takeoff_pub = nh.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 

    px4_autonomy::Position pose;
    px4_autonomy::Velocity vel;
    px4_autonomy::Takeoff tf_val;

    ros::Rate loop_rate(20);

    int counter = 0;


    bool flight_over = false;
    bool record_bool = false; //just to test stage 3(counter between 600~700)
    float record_x = 0.0;
    float record_y = 0.0;

    while(nh.ok())
    {
        
        if(!flight_over)
        {
        	switch(status)
	    	{
	    		case 1:
	    		{
	    			tf_val.take_off = 1; //take off when status is 1(waiting take off orders.)
	    			takeoff_pub.publish(tf_val);
	    			ROS_INFO("Taking off");
	    			break;
	    		}

	    		case 4:
	    		case 5:
	    		{	
	    			counter ++;

	    			if(counter > 200)  //to test position control
	    			{
	    				tf_val.take_off = 2; //land
	    				takeoff_pub.publish(tf_val);
	    				flight_over = true;
	    				ROS_INFO("Landing");
	    			}
	    			
	    			break;
	    		}
	    	}
        }

    	

    	ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}