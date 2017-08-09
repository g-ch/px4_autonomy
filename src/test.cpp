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


/* FOR GAZABO OFFBOARD ENABLE */
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
///////////////////////////

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

        
        if(!flight_over)
        {
        	switch(status)
	    	{
	    		case 1:
	    		{
	    			tf_val.take_off = 1; //take off when status is 1(waiting take off orders.)
	    			takeoff_pub.publish(tf_val);
	    			break;
	    		}

	    		case 4:
	    		case 5:  
	    		{	
	    			counter ++;

	    			if(counter < 100)  //to test position control
	    			{
	    				vel.header.stamp = ros::Time::now();
		    			vel.x = 0.0;
		    			vel.y = 0.5;
		    			vel.z = 0;
		    			vel.yaw_rate = 0.0;
		    			vel_pub.publish(vel);
	    			}
	    			else if(counter < 300)
	    			{
	    				pose.header.stamp = ros::Time::now();
		    			pose.x = (counter-100.0)/20.f;
		    			pose.y = 5.0;
		    			pose.z = 3.0;
		    			pose.yaw = 0.0;
		    			pose_pub.publish(pose);
	    			}
	    			else if(counter < 400 && counter > 300)  //to test velocity control
	    			{
	    				vel.header.stamp = ros::Time::now();
		    			vel.x = 1.0;
		    			vel.y = 0.5;
		    			vel.z = 0;
		    			vel.yaw_rate = 0.0;
		    			vel_pub.publish(vel);
	    			}
	    			/*else if(counter < 600) //to test velocity control with position tracker, 
	    			{
	    				if(!record_bool)
	    				{
	    					record_x = current_px;
	    					record_y = current_py;
	    					record_bool = true;
	    				}

	    				pose.header.stamp = ros::Time::now();
		    			pose.x = record_x;
		    			pose.y = record_y+ ((float)counter-400.0)/100.0;
		    			pose.z = 0.6;
		    			pose.yaw = 0;
		    			pose_pub.publish(pose);

		    			vel.header.stamp = ros::Time::now();
		    			vel.x = 0.0;
		    			vel.y = 0.2;
		    			vel.z = 0;
		    			vel.yaw_rate = 0;
		    			vel_pub.publish(vel);
	    			}*/
	    			else if(counter < 700)  //hover 5 seconds
	    			{
	    				cout<<"hovering"<<endl;
	    			}
	    			else   //land, note flight_over must be set to true. 
	    				  //otherwise when landed, uav status would be 1 the loop will start again!
	    			{
	    				tf_val.take_off = 2; //land
	    				takeoff_pub.publish(tf_val);
	    				flight_over = true;
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