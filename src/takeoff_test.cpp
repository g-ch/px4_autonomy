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
float current_yaw = 0.0;

void chatterCallback_status(const std_msgs::UInt8 &msg)
{
	status =  msg.data;
}

void chatterCallback_px4_pose(const px4_autonomy::Position &msg)
{
    current_px = msg.x;
    current_py = msg.y;
    current_pz = msg.z;
    current_yaw = msg.yaw;
}
 
int wait(int &counter, const int &limitation)
{
	if(counter > limitation) return 1;
	else
	{
		counter ++;
		return 0;
	}
}

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "test1");
    ros::NodeHandle nh;


    ros::Subscriber status_sub = nh.subscribe("/px4/status", 1, chatterCallback_status);
    ros::Subscriber px4_pose_sub = nh.subscribe("/px4/pose", 1,chatterCallback_px4_pose);

    ros::Publisher pose_pub = nh.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
    ros::Publisher vel_pub = nh.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
    ros::Publisher takeoff_pub = nh.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 

    px4_autonomy::Position pose;
    px4_autonomy::Velocity vel;
    px4_autonomy::Takeoff tf_val;

    ros::Rate loop_rate(40);

    int counter = 0;


    bool flight_over = false;
    bool record_bool = false; //just to test stage 3(counter between 600~700)
    float record_x = 0.0;
    float record_y = 0.0;
    float record_yaw = 0.0;

    int wait_counter = 0;

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

	    			if(counter == 200 || counter == 600) record_bool = true;

	    			if(counter < 200)  //v
	    			{
	    				vel.header.stamp = ros::Time::now();
		    			vel.x = 0.2;
		    			vel.y = 0.0;
		    			vel.z = 0;
		    			vel.yaw_rate = 0.0;
		    			vel_pub.publish(vel);
	    			}
	    			else if(counter < 400)  //position control start from the position after velocity control
	    			{
	    				/*if(record_bool)
	    				{
	    					record_x = current_px;
	    					record_y = current_py;
	    					record_yaw = current_yaw;
	    					record_bool = false;
	    				}
	    				pose.header.stamp = ros::Time::now();
		    			pose.x = -(counter - 200)/200.f + record_x;
		    			pose.y = record_y;
		    			pose.z = 1.0;
		    			pose.yaw = record_yaw;
		    			pose_pub.publish(pose);*/
	    			}
	    			else if(counter < 600)  //v
	    			{
	    				vel.header.stamp = ros::Time::now();
		    			vel.x = 0.0;
		    			vel.y = 0.2;
		    			vel.z = 0;
		    			vel.yaw_rate = 0.0;
		    			vel_pub.publish(vel);
	    			}
	    			else if(counter < 700)
	    			{

	    			}
	    			else if(counter < 800)  //p
	    			{
	    				/*if(record_bool)
	    				{
	    					record_x = current_px;
	    					record_y = current_py;
	    					record_yaw = current_yaw;
	    					record_bool = false;
	    				}
	    				pose.header.stamp = ros::Time::now();
		    			pose.x = record_x;
		    			pose.y = -(counter - 600)/200.f + record_y;
		    			pose.z = 1.0;
		    			pose.yaw = record_yaw;
		    			pose_pub.publish(pose);*/
	    			}

	    			else if(counter > 1000)  //to test position control
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

        else 
        {
        	switch(status)
	    	{
	    		case 1:
	    		{
	    			if(wait(wait_counter, 120))
	    			{
	    				tf_val.take_off = 1; //take off when status is 1(waiting take off orders.)
	    				takeoff_pub.publish(tf_val);
	    				ROS_INFO("Taking off");	
	    			}
	    			break;
	    		}
	    		default:
	    		break;
	    	}
        }

    	

    	ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}