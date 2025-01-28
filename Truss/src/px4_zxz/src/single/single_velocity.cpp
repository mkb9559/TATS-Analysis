/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
ros::Publisher optitrack_pose_pub;
double lx   = 0;
double ly   = 0;
double lz   = 0;
double lyaw = 0;
double ax   = 0.0;
double ay   = 9.5;
double az   = 1.2;
double ayaw = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_suber(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped nmsg;
  double qx,qy,qz,qw,siny,cosy;
  nmsg=*msg;
  nmsg.pose.position.x = -msg->pose.position.x;
  nmsg.pose.position.y = msg->pose.position.z;
  nmsg.pose.position.z = msg->pose.position.y;

  nmsg.pose.orientation.x = msg->pose.orientation.x;
  nmsg.pose.orientation.y = msg->pose.orientation.y;
  nmsg.pose.orientation.z = msg->pose.orientation.z;
  nmsg.pose.orientation.w = msg->pose.orientation.w;


  optitrack_pose_pub.publish(nmsg);

  lx=nmsg.pose.position.x;
  ly=nmsg.pose.position.y;
  lz=nmsg.pose.position.z;
  qx=nmsg.pose.orientation.x;
  qy=nmsg.pose.orientation.y;
  qz=nmsg.pose.orientation.z;
  qw=nmsg.pose.orientation.w;
  
  siny = 2*(qw*qz+qx*qy);
  cosy = 1-2*(qy*qy+qz*qz);
  lyaw = std::atan2(siny,cosy);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity");
    ros::NodeHandle nh;
    double eyaw,vx,vy;
    //double init_x,init_y;

    /*
    nh.getParam("init_x",init_x);
    nh.getParam("init_y",init_y);
*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber optitrack_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("vrpn_client_node/RigidBody/pose", 10, pose_suber);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel",10);
    optitrack_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped mpose;
    mpose.pose.position.x = 0;
    mpose.pose.position.y = 0;
    mpose.pose.position.z = 2;
    mpose.pose.orientation.x = 0.0;
    mpose.pose.orientation.y = 0.0;
    mpose.pose.orientation.z = 0.0;
    mpose.pose.orientation.w = -1;

    geometry_msgs::TwistStamped cmdd;
    cmdd.twist.linear.x  = 0;
    cmdd.twist.linear.y  = 0;
    cmdd.twist.linear.z  = 0;
    cmdd.twist.angular.x = 0;
    cmdd.twist.angular.y = 0;
    cmdd.twist.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_pos_pub.publish(mpose);
        local_vel_pub.publish(cmdd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
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
        cmdd.twist.linear.z = 0.5*(az-ly);
        cmdd.twist.angular.z = 1*(ayaw-lyaw);
        vx = -1*(ax-lx);
        vy = 1*(ay-lz);
        if(fabs(cmdd.twist.linear.z)<1)
        {
/*
cmyaw = 0;

cmdd.x+ -->> pos.y--

cmdd.y+ -->> pos.x++


*/


/*
            cmdd.twist.linear.x = 0.1;
            cmdd.twist.linear.y = 0;
*/

            cmdd.twist.linear.x = 0;
            //cmdd.twist.linear.x = 0;
            cmdd.twist.linear.y = 0;
            //cmdd.twist.linear.y = 3;

        }
        else
        {
            cmdd.twist.linear.x = 0;
            cmdd.twist.linear.y = 0;
        }

        cmdd.twist.linear.x = vx;
        cmdd.twist.linear.y = vy;

	ROS_INFO("pos_x:%.3lf, pos_y:%.3lf, pos_z:%.3lf, yaw:%.3lf",lx,ly,lz,lyaw);
	ROS_INFO("cmd_x:%.3lf, cmd_y:%.3lf, cmd_z:%.3lf, cmyaw:%.3lf",cmdd.twist.linear.x,cmdd.twist.linear.y,cmdd.twist.linear.z,cmdd.twist.angular.z);
        //local_pos_pub.publish(mpose);
        local_vel_pub.publish(cmdd);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


