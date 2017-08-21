#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <std_msgs/UInt8.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf/message_filter.h"
#include "tf/transform_datatypes.h"

using namespace std;

#define LOOP_RATE 40
#define DOG_RATE 2
#define PI 3.1416
/* functions declaration */
void chatterCallback_local_pose(const geometry_msgs::PoseStamped &msg);
void chatterCallback_local_vel(const geometry_msgs::TwistStamped &msg);
void chatterCallback_mode(const mavros_msgs::State &msg);
void chatterCallback_cmd_pose(const px4_autonomy::Position &msg);
void chatterCallback_cmd_vel(const px4_autonomy::Velocity &msg);
void chatterCallback_cmd_takeoff(const px4_autonomy::Takeoff &msg);
int watch_dog(int &time_bone);
void set_straint_abs(float &v, float limit);
float pid_calculate(float &kp, float &ki, float &kd, float &error, float &error_last, float &error_acc);

/* global variables */
Eigen::Vector3d pos(0,0,0);
Eigen::Vector3d vel(0,0,0);
Eigen::Quaterniond att;
double yaw = 0.0;
double yaw_rate = 0.0;

Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
double yaw_sp = 0.0;
double yaw_rate_sp = 0.0;

bool take_off_flag = false;
bool land_flag = false; 

bool v_sp_flag = false;
bool p_sp_flag = false;

/* status: very important variable.
0: waiting for offboard mode 1: wait on ground 
2:take off processing 3:landing processing 
4: normal offboard flight 5:hovering */
int status = 0; 

/*values to feed watch dog*/
double pose_stamp = 0.0;
double vel_stamp = 0.0;
double pose_sp_stamp = 0.0;
double vel_sp_stamp = 0.0;

double pose_stamp_last = 0.0;
double vel_stamp_last = 0.0;
double pose_sp_stamp_last = 0.0;
double vel_sp_stamp_last = 0.0;

bool offboard_ready = false;
int dog_feed_times = 0; //to adjust loop rate and dog feeding rate
unsigned int offboard_flight_times = 0;

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "offboard_center");
    ros::NodeHandle nh;

    /* setting parameters definition */
    float coor_type;
    float control_type;
    float toff_height;
    float land_height;
    float max_vx;
    float max_vy;
    float max_vz;
    float max_yawrate;
    float pt_kp_xy;
    float pt_ki_xy;
    float pt_kd_xy;
    float pt_kp_z;
    float pt_ki_z;
    float pt_kd_z;
    float pt_kp_yaw;
    float pt_ki_yaw;
    float pt_kd_yaw;

    dog_feed_times = LOOP_RATE/DOG_RATE;
    float period =  1.f/(float)(LOOP_RATE); 

    /* read from settings.yaml */
    nh.getParam("/offboard_center/coor_type", coor_type);
    nh.getParam("/offboard_center/toff_height", toff_height);
    nh.getParam("/offboard_center/land_height", land_height);
    nh.getParam("/offboard_center/max_vx", max_vx);
    nh.getParam("/offboard_center/max_vy", max_vy);
    nh.getParam("/offboard_center/max_vz", max_vz);
    nh.getParam("/offboard_center/max_yawrate", max_yawrate);
    nh.getParam("/offboard_center/pt_kp_xy", pt_kp_xy);
    nh.getParam("/offboard_center/pt_ki_xy", pt_ki_xy);
    nh.getParam("/offboard_center/pt_kd_xy", pt_kd_xy);
    nh.getParam("/offboard_center/pt_kp_z", pt_kp_z);
    nh.getParam("/offboard_center/pt_ki_z", pt_ki_z);
    nh.getParam("/offboard_center/pt_kd_z", pt_kd_z);
    nh.getParam("/offboard_center/pt_kp_yaw", pt_kp_yaw);
    nh.getParam("/offboard_center/pt_ki_yaw", pt_ki_yaw);
    nh.getParam("/offboard_center/pt_kd_yaw", pt_kd_yaw);
    nh.getParam("/offboard_center/control_type", control_type);
    

    /* handle topics */
    ros::Subscriber local_pose_sub = nh.subscribe("/mavros/local_position/pose", 1,chatterCallback_local_pose);
    ros::Subscriber local_vel_sub = nh.subscribe("/mavros/local_position/velocity", 1,chatterCallback_local_vel);
    ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
    ros::Subscriber pose_sp_sub = nh.subscribe("/px4/cmd_pose", 1,chatterCallback_cmd_pose);
    ros::Subscriber vel_sp_sub = nh.subscribe("/px4/cmd_vel", 1,chatterCallback_cmd_vel);
    ros::Subscriber take_off_sub = nh.subscribe("/px4/cmd_takeoff", 1,chatterCallback_cmd_takeoff);

    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);  
    ros::Publisher pose_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    ros::Publisher status_pub = nh.advertise<std_msgs::UInt8>("/px4/status", 1); 
    ros::Publisher pose_pub = nh.advertise<px4_autonomy::Position>("/px4/pose", 1); 

    ros::Rate loop_rate(LOOP_RATE);
    int time_bone = 0;

    geometry_msgs::TwistStamped cmd_vel; //the velocity sent to mavros
    geometry_msgs::PoseStamped cmd_pose;  //the position sent to mavros
    std_msgs::UInt8 status_value;
    px4_autonomy::Position pose_cal;

    /*variable for position control type in px4*/
    float x_record = 0.0;
    float y_record = 0.0;
    float z_record = 0.0;
    float yaw_record = 0.0;
    float pitch_record = 0.0;
    float roll_record = 0.0;
    bool if_record = true;
    int control_state = 0;
    int control_state_last = 0;
    int land_counter = 0;
    float last_height = 0.0;
    float takeoff_z_set = 0.f;

    /* PID storage values */
    float x_error = 0.0;
    float x_error_1ast = 0.0;
    float x_error_acc = 0.0;
    float y_error = 0.0;
    float y_error_1ast = 0.0;
    float y_error_acc = 0.0;
    float z_error = 0.0;
    float z_error_1ast = 0.0;
    float z_error_acc = 0.0;
    float yaw_error = 0.0;
    float yaw_error_1ast = 0.0;
    float yaw_error_acc = 0.0;

    /* Variables for velocity setpoint in position  control mode*/
    float acc_x = 0.0;
    float acc_y = 0.0;
    float acc_z = 0.0;
    float acc_yaw = 0.0;




    /* Main loop */
    while(nh.ok())
    {

        /* Publish px4 pose */
        pose_cal.x = pos(0);
        pose_cal.y = pos(1);
        pose_cal.z = pos(2);
        pose_cal.yaw = yaw;
        pose_pub.publish(pose_cal);

        /* Velocity control type */
        if(control_type == 0.f)
        {
            switch(status)
            {
                case 0:  //waiting for offboard mode
                {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.linear.z = 0.0;
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;

                    vel_sp_pub.publish(cmd_vel);

                    if(offboard_ready) status = 1;
                    break;
                }


                case 1:  //wait on ground in offboard mode
                {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.linear.z = -0.5; //test
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;

                    vel_sp_pub.publish(cmd_vel);

                    if(take_off_flag) status = 2;
                    break;
                }


                case 2: //take off process
                {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;

                    cmd_vel.twist.linear.z = (toff_height - pos(2)) * 1.0 + 0.1;
                    if(cmd_vel.twist.linear.z > 0.5) cmd_vel.twist.linear.z = 0.5;
                    //cmd_vel.twist.linear.z = 1.0; //test

                    vel_sp_pub.publish(cmd_vel);

                    if(pos(2) > toff_height - 0.1 ) status = 5;

                    break;
                }


                case 3: //land process
                {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;

                    cmd_vel.twist.linear.z = (land_height - pos(2)) * 1.0 - 0.2;
                    if(cmd_vel.twist.linear.z < -0.5) cmd_vel.twist.linear.z = -0.5;

                    //cmd_vel.twist.linear.z = -1.0; //test

                    vel_sp_pub.publish(cmd_vel);

                    if(pos(2) < land_height) status = 1;

                    break;
                }


                case 4:
                {
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;

                    if(coor_type == 0.f) //enu coordinate
                    {
                        if(v_sp_flag && !p_sp_flag) //only velocity
                        {
                            cmd_vel.twist.linear.x = vel_sp(0);
                            cmd_vel.twist.linear.y = vel_sp(1);
                            cmd_vel.twist.linear.z = vel_sp(2);
                            cmd_vel.twist.angular.z = yaw_rate_sp;
                        }
                        else if(!v_sp_flag && p_sp_flag) //only position
                        {
                            x_error = pos_sp(0) - pos(0);
                            y_error = pos_sp(1) - pos(1);
                            z_error = pos_sp(2) - pos(2);
                            yaw_error = yaw_sp - yaw;

                            cmd_vel.twist.linear.x = pid_calculate(pt_kp_xy, pt_ki_xy, pt_kd_xy, x_error, x_error_1ast, x_error_acc);
                            cmd_vel.twist.linear.y = pid_calculate(pt_kp_xy, pt_ki_xy, pt_kd_xy, y_error, y_error_1ast, y_error_acc);
                            cmd_vel.twist.linear.z = pid_calculate(pt_kp_z, pt_ki_z, pt_kd_z, z_error, z_error_1ast, z_error_acc);
                            cmd_vel.twist.angular.z = pid_calculate(pt_kp_yaw, pt_ki_yaw, pt_kd_yaw, yaw_error, yaw_error_1ast, yaw_error_acc);
                        }
                        else if(v_sp_flag && p_sp_flag) //velocity with position tracker
                        {
                            x_error = pos_sp(0) - pos(0);
                            y_error = pos_sp(1) - pos(1);
                            z_error = pos_sp(2) - pos(2);
                            yaw_error = yaw_sp - yaw;

                            cmd_vel.twist.linear.x = vel_sp(0) + pid_calculate(pt_kp_xy, pt_ki_xy, pt_kd_xy, x_error, x_error_1ast, x_error_acc);
                            cmd_vel.twist.linear.y = vel_sp(1) + pid_calculate(pt_kp_xy, pt_ki_xy, pt_kd_xy, y_error, y_error_1ast, y_error_acc);
                            cmd_vel.twist.linear.z = vel_sp(2) + pid_calculate(pt_kp_z, pt_ki_z, pt_kd_z, z_error, z_error_1ast, z_error_acc);
                            cmd_vel.twist.angular.z = yaw_rate_sp + pid_calculate(pt_kp_yaw, pt_ki_yaw, pt_kd_yaw, yaw_error, yaw_error_1ast, yaw_error_acc);
                        }
                        else
                        {
                            /* no setpoints received, leave to handle by the watch dog */
                        }
                    }
                    else  // head coordinate
                    {
                        /*for now, just hover*/
                        ROS_INFO("Wrong coordinate type");
                        status = 5;
                    }

                    /*set_straint_abs(cmd_vel.twist.linear.x, max_vx);
                    set_straint_abs(cmd_vel.twist.linear.y, max_vy);
                    set_straint_abs(cmd_vel.twist.linear.z, max_vz);
                    set_straint_abs(cmd_vel.twist.angular.z, max_yawrate);*/

                    /* set constraints */
                    if(cmd_vel.twist.linear.x > max_vx) cmd_vel.twist.linear.x = max_vx;
                    else if(cmd_vel.twist.linear.x < -max_vx) cmd_vel.twist.linear.x = -max_vx;

                    if(cmd_vel.twist.linear.y > max_vy) cmd_vel.twist.linear.y = max_vy;
                    else if(cmd_vel.twist.linear.y < -max_vy) cmd_vel.twist.linear.y = -max_vy;

                    if(cmd_vel.twist.linear.z > max_vz) cmd_vel.twist.linear.z = max_vz;
                    else if(cmd_vel.twist.linear.z < -max_vz) cmd_vel.twist.linear.z = -max_vz;

                    if(cmd_vel.twist.angular.z > max_yawrate) cmd_vel.twist.angular.z = max_yawrate;
                    else if(cmd_vel.twist.angular.z < -max_yawrate) cmd_vel.twist.angular.z = -max_yawrate;

                    vel_sp_pub.publish(cmd_vel);

                    if(!v_sp_flag && !p_sp_flag) status = 5;
                    if(land_flag) status = 3;

                    break;
                }


                case 5:  //hover mode
                {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.linear.z = 0.0;
                    cmd_vel.twist.angular.x = 0.0;
                    cmd_vel.twist.angular.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;

                    vel_sp_pub.publish(cmd_vel);

                    if(v_sp_flag || p_sp_flag) status = 4;
                    if(land_flag) status = 3;

                    break;
                }

                default:
                {
                    ROS_INFO("What the hell is this status? It should not happen.");
                    break;
                }
            }

        }

        /*Position control type*/
        else if(control_type == 1.f)
        {
            switch(status)
            {
                case 0:  //waiting for offboard mode
                {
                    cmd_pose.pose.position.x = pos(0);
                    cmd_pose.pose.position.y = pos(1);
                    cmd_pose.pose.position.z = pos(2);
                    cmd_pose.pose.orientation.x = 0.0;
	                cmd_pose.pose.orientation.y = 0.0;
	                cmd_pose.pose.orientation.z = sin(yaw/2.f);
                    cmd_pose.pose.orientation.w = cos(yaw/2.f);
                    //tf::Quaternion cmd_q(yaw, pitch_record, roll_record);
                    //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);

                    pose_sp_pub.publish(cmd_pose);

                    if(offboard_ready)
                    {
                        status = 1;
                        if_record = true;
                    } 
                    break;
                }


                case 1:  //wait on ground in offboard mode
                {
                    if(if_record)
                    {
                        x_record = pos(0);
                        y_record = pos(1);
                        z_record = pos(2);
                        yaw_record = yaw;

                        cout<<"z_rec"<<z_record<<endl;
                        if_record = false;
                    }

                    cmd_pose.pose.position.x = pos(0);
                    cmd_pose.pose.position.y = pos(1);

                    if(pos(2) < land_height + 0.05)
                        cmd_pose.pose.position.z = pos(2) - 0.2;

                    else
                        cmd_pose.pose.position.z = z_record;

                    cmd_pose.pose.orientation.x = 0.0;
                    cmd_pose.pose.orientation.y = 0.0;
                    cmd_pose.pose.orientation.z = sin(yaw/2.f);
                    cmd_pose.pose.orientation.w = cos(yaw/2.f);
                    //tf::Quaternion cmd_q(yaw_record, pitch_record, roll_record);
                    //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);

                    pose_sp_pub.publish(cmd_pose);

                    if(take_off_flag)
                    {
                        status = 2;
                        if_record = true;
                    } 
                    break;
                }


                case 2: //take off process
                {
                    if(if_record)
                    {
                        x_record = pos(0);
                        y_record = pos(1);
                        z_record = pos(2);
                        yaw_record = yaw;

                        cout<<"z_rec"<<z_record<<endl;
                        takeoff_z_set = z_record;

                        if_record = false;
                    }

                    cmd_pose.pose.position.x = x_record;
                    cmd_pose.pose.position.y = y_record;

                    /*float add_height = (toff_height +0.1f - pos(2)) * 1.2f;
                    if(add_height > 0.8f) add_height = 0.8f;
                    else if(add_height < 0.4f) add_height = 0.4f;
                    cmd_pose.pose.position.z = pos(2) + add_height;*/

                    takeoff_z_set += 0.02;
                    cmd_pose.pose.position.z = takeoff_z_set;

                    if(cmd_pose.pose.position.z > toff_height) cmd_pose.pose.position.z = toff_height;

                    //tf::Quaternion cmd_q(yaw_record, pitch_record, roll_record);
                    //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);
                    cmd_pose.pose.orientation.x = 0.0;
                    cmd_pose.pose.orientation.y = 0.0;
                    cmd_pose.pose.orientation.z = sin(yaw_record/2.f);
                    cmd_pose.pose.orientation.w = cos(yaw_record/2.f);
           

                    if(pos(2) > toff_height - 0.1f )
                    {
                        cmd_pose.pose.position.z = pos(2);
                        status = 5;
                        takeoff_z_set = 0.f;
                        if_record = true;
                    } 

                    pose_sp_pub.publish(cmd_pose);

                    break;
                }


                case 3: //land process
                {

                    if(if_record)
                    {
                        x_record = pos(0);
                        y_record = pos(1);
                        z_record = pos(2);
                        yaw_record = yaw;

                        cout<<"z_rec"<<z_record<<endl;

                        if_record = false;
                    }

                    if(pos(2) < land_height + 0.05)
                    {
                         cmd_pose.pose.position.x = pos(0);
                         cmd_pose.pose.position.y = pos(1);
                    }
                    else
                    {
                        cmd_pose.pose.position.x = x_record;
                        cmd_pose.pose.position.y = y_record;
                    }

                    float dec_height = 0.f - (pos(2) + 0.1f)* 1.1f;
                    if(dec_height > -0.2f) dec_height = -0.2f;
                    else if(dec_height < -0.6f) dec_height = -0.6f;

                    cmd_pose.pose.position.z = pos(2) + dec_height;

                    //tf::Quaternion cmd_q(yaw_record, pitch_record, roll_record);
                    //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);
                    cmd_pose.pose.orientation.x = 0.0;
                    cmd_pose.pose.orientation.y = 0.0;
                    cmd_pose.pose.orientation.z = sin(yaw_record/2.f);
                    cmd_pose.pose.orientation.w = cos(yaw_record/2.f);

                    pose_sp_pub.publish(cmd_pose);


                    /*if(!(last_height > pos(2)))
                    {
                        land_counter ++;
                    } 
                    last_height = pos(2);
                        

                    if(land_counter > 5) //pos(2) < land_height  land_counter > 5) //)
                    {
                        status = 1;
                        if_record = true;
                        land_counter = 0;
                    } */

                    if(pos(2) < land_height) 
                    {
                        status = 1;
                        if_record = true;
                        land_counter = 0;
                    } 

                    break;
                }


                case 4:
                {
                    if(coor_type == 0.f) //enu coordinate
                    {
                        // control state:
                        // 0: no setpoint
                        // 1: vel setpoint
                        // 2: pose setpoint
                        // 3: both setpoint
                        
                        control_state = 0;
                        if(v_sp_flag) control_state += 1;
                        if(p_sp_flag) control_state += 2;

                        if(control_state != control_state_last) if_record = true; //for control mode change from position to velocity
                        control_state_last = control_state;

                        
                        if(control_state == 1) //only velocity
                        {
                            if(if_record)
                            {
                                x_record = pos(0);
                                y_record = pos(1);
                                z_record = pos(2);
                                yaw_record = yaw;

                                acc_x = 0.0;
                                acc_y = 0.0;
                                acc_z = 0.0;
                                acc_yaw = 0.0;

                                if_record = false;

                                cout<<"z_rec"<<z_record<<endl;
                            }

                            acc_x += vel_sp(0) * period; 
                            acc_y += vel_sp(1) * period;
                            acc_z += vel_sp(2) * period;
                            acc_yaw += yaw_rate_sp * period;

                            cmd_pose.pose.position.x = x_record + acc_x;
                            cmd_pose.pose.position.y = y_record + acc_y;
                            cmd_pose.pose.position.z = z_record + acc_z;
                            float temp_yaw = yaw_record + acc_yaw;

                            //tf::Quaternion cmd_q(temp_yaw, pitch_record, roll_record);
                            //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);
                            cmd_pose.pose.orientation.x = 0.0;
                            cmd_pose.pose.orientation.y = 0.0;
                            cmd_pose.pose.orientation.z = sin(temp_yaw/2.f);
                            cmd_pose.pose.orientation.w = cos(temp_yaw/2.f);

                            pose_sp_pub.publish(cmd_pose);
                            
                        }
                        else if(control_state == 2) //only position
                        {
                            cmd_pose.pose.position.x = pos_sp(0);
                            cmd_pose.pose.position.y = pos_sp(1);
                            cmd_pose.pose.position.z = pos_sp(2);

                            //tf::Quaternion cmd_q(yaw_sp, pitch_record, roll_record);
                            //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);
                            cmd_pose.pose.orientation.x = 0.0;
                            cmd_pose.pose.orientation.y = 0.0;
                            cmd_pose.pose.orientation.z = sin(yaw_sp/2.f);
                            cmd_pose.pose.orientation.w = cos(yaw_sp/2.f);

                            pose_sp_pub.publish(cmd_pose);
                        }
                        else if(control_state == 3) //velocity with position tracker
                        {
                            ROS_INFO("Can not handle both velocity and position setpoint in position control mode!");
                            //status = 5;
                        }
                        else
                        {
                            /* no setpoints received, leave to handle by the watch dog */
                            if_record = true;          
                        }
                    }
                    else  // head coordinate
                    {
                        /*for now, just hover*/
                        ROS_INFO("Wrong coordinate type");
                        status = 5;
                    }
                    
                    if(!v_sp_flag && !p_sp_flag) status = 5;
                    if(land_flag) status = 3;

                    break;
                }


                case 5:  //hover mode
                {
                    if(if_record)
                    {
                        x_record = pos(0);
                        y_record = pos(1);
                        z_record = pos(2);
                        yaw_record = yaw;

                        if_record = false;
                        cout<<"z_rec"<<z_record<<endl;
                    }

                    cmd_pose.pose.position.x = x_record;
                    cmd_pose.pose.position.y = y_record;
                    cmd_pose.pose.position.z = z_record;

                    //tf::Quaternion cmd_q(yaw_record, pitch_record, roll_record);
                    //tf::quaternionTFToMsg(cmd_q, cmd_pose.pose.orientation);

                    cmd_pose.pose.orientation.x = 0.0;
                    cmd_pose.pose.orientation.y = 0.0;
                    cmd_pose.pose.orientation.z = sin(yaw_record/2.f);
                    cmd_pose.pose.orientation.w = cos(yaw_record/2.f);

                    pose_sp_pub.publish(cmd_pose);

                    if((!v_sp_flag && p_sp_flag) || (v_sp_flag && !p_sp_flag))
                    {
                        status = 4;
                        if_record = true;
                    } 
                    if(land_flag) 
                    {
                        status = 3;
                        if_record = true;
                    }
                        

                    break;
                }
                default:
                {
                    ROS_INFO("What the hell is this status? It should not happen.");
                    break;
                }
            }   
        }
        else
        {
            ROS_INFO("Unknown control type!");
        }

        
        status_value.data = status;
        status_pub.publish(status_value);


        watch_dog(time_bone);
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}

/* PID tracker: give a velocity value according to position error*/
float pid_calculate(float &kp, float &ki, float &kd, float &error, float &error_last, float &error_acc)
{
    /* limit the error_acc to avoid too large values */
    set_straint_abs(error_acc, 10.0);

    /*calculate and store*/
    float value = kp*error + ki*error_acc + kd*(error-error_last);
    error_last = error;
    error_acc += error;

    return  value;
}

/* Straints */
void set_straint_abs(float &v, float limit)
{
    if(v > limit) v = limit;
    else if(v < -limit) v = -limit;
}


/* Watch dog is fed to detect connections with uav and set point giving mode*/
int watch_dog(int &time_bone)
{
    if(time_bone < dog_feed_times) time_bone ++;
    else
    {
        time_bone = 0;

        /* check values update */
        bool check1 = false;
        bool check4 = false; //check 2 & 3 are eaten by the dog, 2333
        bool check5 = false;

        if(offboard_ready) check1 = true;  //mode check
        else ROS_INFO("Please switch to offboard mode");

        if(pose_sp_stamp != pose_sp_stamp_last)  //position setpoint check
        {
            check4 = true;
            p_sp_flag = true;
        }
        else p_sp_flag = false;


        if(vel_sp_stamp != vel_sp_stamp_last) //velocity setpoint check
        {
            check5 = true;
            v_sp_flag = true;
        }
        else v_sp_flag = false;

        //ROS_INFO("flag: p %d, v %d", (int)p_sp_flag, (int)v_sp_flag);


        if(check1 && (check4||check5)) offboard_flight_times++; //clock feed, just to know the offboard time
        else if(!check1) //having troubles connecting to uav
        {
            status = 0;
            //if(pos(2) > 0.2) status = 5;
            //else status = 0;  
        }
        else if(status == 4)
        {
            ROS_INFO("Waiting for setpoints...");
        }


        /* update last state */
        pose_stamp_last = pose_stamp; 
        vel_stamp_last = vel_stamp;
        pose_sp_stamp_last = pose_sp_stamp;
        vel_sp_stamp_last = vel_sp_stamp;

    }
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

    //Eigen::Vector3d euler = att.toRotationMatrix().eulerAngles(2, 1, 0);  //test
    //yaw = euler(0);

   // double roll, pitch;
   // tf::Quaternion q;
   // tf::quaternionMsgToTF(msg.pose.orientation, q);
   // tf::Matrix3x3 m(q);
   // m.getRPY(roll, pitch, yaw);
   yaw = atan2(2*att.y()*att.x() - 2*att.z()*att.w(), -2*att.y()*att.y() - 2*att.w()*att.w() +1 ) + PI;
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
    pose_sp_stamp = msg.header.stamp.toSec();
    pos_sp(0) = msg.x;
    pos_sp(1) = msg.y;
    pos_sp(2) = msg.z;
    yaw_sp = msg.yaw;
}

void chatterCallback_cmd_vel(const px4_autonomy::Velocity &msg)
{
    vel_sp_stamp = msg.header.stamp.toSec();
    vel_sp(0) = msg.x;
    vel_sp(1) = msg.y;
    vel_sp(2) = msg.z;
    yaw_rate_sp = msg.yaw_rate;
}

void chatterCallback_cmd_takeoff(const px4_autonomy::Takeoff &msg)
{
    if(msg.take_off == 1)  //take off
    {
        take_off_flag = true;
        land_flag = false;
    }
    else if(msg.take_off == 2)  //land
    {
        take_off_flag = false;
        land_flag = true;
    }   
    else
    {
        take_off_flag = false;
        land_flag = false;
    }
}
