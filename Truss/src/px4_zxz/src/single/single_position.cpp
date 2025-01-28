/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>


static int cmdd;
static geometry_msgs::PoseStamped aim;
static geometry_msgs::PoseStamped offset;
void pose_suber(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped nmsg;
  nmsg=*msg;
}

void Cmd_suber(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cmdd=int(msg->pose.orientation.x);
    aim.pose.position.x = msg->pose.position.x + offset.pose.position.x;
    aim.pose.position.y = msg->pose.position.y + offset.pose.position.y;
    aim.pose.position.z = msg->pose.position.z + offset.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single");
    ros::NodeHandle nh;
    int UAVID;
    nh.param<int>("UAVID", UAVID, 1);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Subscriber swarm_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("swarm_cmd/main", 10, Cmd_suber);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);
    switch(UAVID){
        case 1:{
            ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("vrpn_client_node/UAV1/pose", 10, pose_suber);
            break;
        }
        case 2:{
            ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("vrpn_client_node/UAV2/pose", 10, pose_suber);
            break;
        }
        case 3:{
            ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("vrpn_client_node/UAV3/pose", 10, pose_suber);
            break;
        }
        case 4:{
            ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("vrpn_client_node/UAV4/pose", 10, pose_suber);
            break;
        }
    }

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    geometry_msgs::PoseStamped offset;

    switch(UAVID){
        case 1:{
            offset.pose.position.x = 0.3;
            offset.pose.position.y = 0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 2:{
            offset.pose.position.x = 0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 3:{
            offset.pose.position.x = -0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0;
            break;
        }
        case 4:{
            offset.pose.position.x = -0.3;
            offset.pose.position.y = 0.3;
            offset.pose.position.z = 0;
            break;
        }
    }


    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x  = 0;
    vel.twist.linear.y  = 0;
    vel.twist.linear.z  = 0;
    vel.twist.angular.x = 0;
    vel.twist.angular.y = 0;
    vel.twist.angular.z = 0;

    while(ros::ok()){
        if(cmdd>=9){

            local_pos_pub.publish(aim);
        }
        if(cmdd==4){
          vel.twist.linear.z  = -0.2;
          local_vel_pub.publish(vel);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


